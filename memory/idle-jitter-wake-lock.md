# 静止待机抖动：`set_base_velocity` 把整机永久锁在唤醒态（2026-10-14 结案）

> 2026-11-02四轮八驱复查另见[idle-jitter-swerve-20261102.md](idle-jitter-swerve-20261102.md)：当前没有持续目标写入。6.2高层set_joint_velocities同时改速度目标，不能用于保目标唤醒；L3重置还会丢颈部阻尼。不要直接套用本文旧根因。

用户："现在这个状态还是有一些抖动"，部位选**整机/机身**，时机选**静止待机也有**。

## 复现（先复现，别猜）

之前我测出"整定后所有 link 摆幅 0.000mm"、判定不抖 —— **那个结论是错的**：
当时读的是 `_grasp_link_T`（USD 变换），**静止时 USD 不回写**，读到的是冻住的值。
改读 `articulation._physics_view.get_link_transforms()`（物理真值）后，
真静止就有抖动：

| 段 | head_pitch 抖动 | link 抖动峰 | link 摆幅峰 |
|---|---|---|---|
| 真静止（什么都不下发） | 0.979 mrad | 201.3 µm | 1.215 mm |
| 夹爪半开待机 | 0.946 | 200.3 | 2.173 |
| 抓取完成后 | 0.748 | 194.9 | 3.769 |

50 个关节全在抖，|v| 峰到 1.8 rad/s —— 是**系统性**的，不是某个关节。

## 根因

**PhysX 里"睡着"就是完全静止，"醒着"就有 0.3~4 mrad 的宽带微振。**
而 `set_base_velocity` 一旦被调用过，`_base_cmd` 就**永久非空**，
`_update_base_velocity` 于是**每帧都写轮子位置目标** →
整个 articulation 永远睡不着 → 用户看到的"静止待机也在抖"。

实测对照（同场景，只改这一条）：

| 条件 | head_pitch | 轮子 | \|v\|峰 |
|---|---|---|---|
| 调过一次 `set_base_velocity(0,0)` | 4.006 mrad | 0.38 | 2.0 |
| 清掉 `_base_cmd` | **0.000** | 0.000 | 0.00 |
| 轮子刚度归零（同上仍写目标） | 3.401 | 0.37 | 2.0 |
| 轮子刚度 203（同上仍写目标） | 4.082 | 0.55 | 2.0 |

**与轮子刚度无关** —— 关键是"有没有在写目标"。

## 修法（`robot_manager.py`）

1. `_update_base_velocity`：速度已经是 0 就**彻底撒手**，不再写轮子目标，
   并清 `_base_cmd` / `_wheel_pos_target`。
2. `set_base_velocity`：**从静止转为驱动的那一次**显式唤醒
   （写一次 `set_joint_velocities`，因为睡着时写位置目标唤不醒它 ——
   实测不唤醒底盘只走 1.7mm/5s）。
   **不能每次调用都写**：客户端 10Hz 发指令，每帧把速度写回采样值会打断
   加速过程。

效果：**静止 0.979 mrad / 201µm / 1.215mm → 0.000 / 0.0µm / 0.000mm**。

## 没解决的（诚实记录）

**夹住物体时 / 抓取完成后**仍有微振：head_pitch ~0.6~0.8 mrad、
link 抖动 ~120~280 µm、手部（link#59/#60）z 向 2~3mm 的持续上下振。
**把方块放掉（张开夹爪）后立刻归零** —— 所以是"夹爪驱动 + 接触 + 软件 mimic
三者硬顶"造成的，正是 PhysX 官方 *Articulation Stability Guide* 点名的组合：

> "Gripping scenarios may exhibit mimic joint instability when finger joints are
> 'passively' actuated via a mimic joint connected to a driven joint... especially
> prone to instability when the driven joint is configured to be very stiff
> compared to the typically small masses and inertia of finger links."

## 已用数据排除的做法（别再试）

| 做法 | 结果 |
|---|---|
| 开 stabilization + 32 位置迭代 + `solveArticulationContactLast` | **差 4.4 倍**（head_pitch 2.343 vs 0.535 mrad，各 3 次）→ 已默认关，见 config.json `physics.enabled` |
| 轮子驱动刚度 0 / 203 / ζ=1 | 与抖动无关 |
| 夹爪 armature 扫 0 → 1e-2 | 无一致改善 |
| 夹爪跟随关节软化 40000 → 1000 | 无改善（略差） |
| 松开轮子驱动刚度 | **更糟**：轮子自由空转（位置极差 12.5 rad ≈ 两圈） |

**注意**：PhysX 场景级参数（solverType / 迭代数 / stabilization）**必须在
`World()` 之后、`world.reset()` 之前**写进 USD，运行时改不生效。
`_ensure_physics_scene()` 里写了但默认 `enabled=false`。
属性名是 `physxScene:minPositionIterationCount` / `maxPositionIterationCount`
（**没有** `solverPositionIterationCount`）；`physxScene:enableStabilization`
本机实测默认是 **false**（Isaac 的 `World` docstring 说默认开，不符）。

相关 [[rm65-grasp-failure-root-cause]]、[[neck-camera-damping]]。
