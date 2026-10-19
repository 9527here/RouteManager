# 夹爪 mimic 链与指令接口

**2026-10-08 修好**：夹爪"连杆脱节/手指不动"的根因是软件 mimic 死掉了，不是 URDF 或资产问题。

## URDF 事实（`humanoid_upper_body_v3_ctag2f120.urdf`）

每只手 14 个 gripper 关节 = 1 主动 + 11 跟随 + 2 个 fixed pad。
主动关节 `{side}_gripper_right_2_joint`，限位 **[0, 0.7] rad**（0=全闭，0.7=全开）。
mimic 链在 URDF 里是**链式**的，扁平化到主动关节后的倍率：

| 关节 | 倍率 | 说明 |
|---|---|---|
| `*_right_in` `*_right_3` `*_right_main` `*_right_slip` `*_left_2` `*_left_in` `*_left_slip` | **+1** | |
| `*_left_3` `*_left_main` | **−1** | 对称指 |
| `*_right_1` `*_left_1` | **−0.15** | 连杆机构小角度 |

`robot_manager._build_mimic_map()` 的命名规则（`left_main`/`left_3`→−1、`_1_joint`→−0.15、其余+1）与上表**完全一致**，是对的。

**2026-10-12 改成读 URDF**：命名规则只认人形关节名，单独加载官方
`assets/RM65-6FB-V/urdf/RM65-6FB-V_ctag2f120_isaac.urdf`（关节叫 `ctag_R2`/
`ctag_Left_in`）时一个都认不出来。现在 `load_urdf` 会解析 URDF 的 `<mimic>`
标签填 `_urdf_mimic`，`_build_mimic_map` 优先用它并**链式展开**到根主动关节
（`Left_in→L2→R2` 展开成倍率 1；`L3→R3→R2` 展开成 −1）。走 `load_robot` 加载
已转换 USD 时拿不到 URDF，自动退回命名规则，人形行为不变。
`set_gripper_position` 的主动关节也改成可解析：命名对不上且 mimic 表里只有
一个主动关节时直接用它（单臂场景）。

**单独看机械臂**：`config.json` 预设 `rm65_ctag2f120` → 官方 RM65+原厂 CT 夹爪，
`./scripts/load_scene.sh -n rm65_ctag2f120`（或直接 `load_urdf` 传 `fix_base=true`，
否则臂会倒）。19 DOF，对外只暴露 8 个（6 臂 + camera_rojoint + `ctag_R2`）。

## 驱动上限力矩：夹爪闭不下去的真正原因（2026-10-09）

资产里**主动关节的 `max_force` 只有 20 N·m，11 个跟随关节是 30 N·m** ——
主动关节反而比跟随关节弱，于是被跟随关节按各自目标"反锁"：空载指令全闭
（leader=0）实测只能到 **0.35 rad（开口≈41mm）** 就卡死，60mm 的物体根本夹不到。

`robot_manager._tune_gripper_drives()`（`_build_mimic_map()` 末尾自动调用）把 24 个
夹爪关节统一提到 **60 N·m**。改后空载扫描**完全线性可重复、往返重合、可闭到底**。

**症状识别**：夹爪"能动但闭不到底"、"同样指令不同时刻结果不一样"（26/33/88mm）、
`left_gripper_left_2_joint` 目标 +0.55 实际 −0.24 —— 都是这个。

## 实测开口宽度（pad 连杆世界坐标，USD Xformable 算的）

60 N·m 下实测（往返扫描完全重合，无迟滞）：

| 主动关节角 | 0.0 | 0.1 | 0.2 | 0.3 | 0.4 | 0.5 | 0.6 | 0.7 |
|---|---|---|---|---|---|---|---|---|
| 指垫**中心**距 mm | 2.68 | 15.59 | 28.49 | 41.27 | 53.80 | 65.95 | 77.58 | 88.49 |

**开口**（内表面间距）= 中心距 − 2mm（每片指垫厚 2mm）。表在
`gripper_model.py`（`LEADER_TO_SEP_MM`），**不要再用线性 ×0.7 近似** ——
中间段能差 5mm 以上，抓取时足以让指垫压根碰不到物体。

## 指垫中点随开口漂移 ~11mm

两指垫的**中点**（即 TCP）沿夹爪接近轴随开口移动：全开→全闭漂移
**11~12mm**（URDF 模型与仿真实测一致）。所以：

- **规划必须用"闭合态"的 `tcp_local`**（在目标闭合角下量），用张开态的会让
  物体落在指垫外侧（实测闭合后指垫离物体面还有 3~4mm）。
- 等效地，抓取时把张开态的 TCP 瞄在物体中心 **上方 11mm** 也行。

## 抓取配方（`scripts/pick_object.py`，实测 4/4 成功）

1. 先算目标中心距 = 物体宽 + 2mm(垫厚) − **过盈**，过盈取 **3mm**。
   （1~2mm 不够：连杆受载时指垫停在离物体面 ~0.5mm 处，建立不起夹持力。）
2. 把夹爪开到该闭合角，量出**闭合态** `tcp_local`。
3. 张开，用该 `tcp_local` 做 IK（TCP 落到物体中心、朝下、虎口对 ±y 面）。
4. 到抓取位 → 闭合到目标角 → 抬起。

## 资产里左手夹爪关节**没有限位**（2026-10-09 发现，未修）

`physics.usd` 里右手 6 个夹爪关节有限位（`right_2` [0, 40.107°] 等，单位是度），
但**左手 6 个全是 `±inf`**。URDF 里两侧都有 limit（左指是右指的镜像：
`left_3`/`left_main` 倍率 −1，其余同值）。

后果：左手四连杆可以被驱到物理行程之外，实测会出现**"主动关节到位但两指垫只分开
34mm"**的坏平衡态（`left_2` 目标 +0.55 实际 −0.24），夹爪卡死、物体夹不住；
而且**时好时坏**——同一段代码在干净状态下扫描完全正常，被物体压过之后就卡住，
靠反复下指令只能很慢地恢复。

**修法**：离线烘焙（`scripts/scene/bake_gripper_limits.py` 已写好，需权限执行）——
把右指的限位照搬到左指，`left_3`/`left_main` 取反并交换上下限。烘焙后需重启服务。

## 其它坑

- **VR 会话在跑但没接手柄时**不会抢写关节目标（实测开/关 VR 扫描结果一致）；
  但夹爪一旦卡进坏平衡态，要连跑几个完整开合周期才恢复。
- 运行时给 `/World/cube_c` 改 `xformOp:scale` 会让物理后端失效
  （"Failed to get DOF positions from backend"）→ 只能重启服务。
  改相机 prim 的变换是安全的。
- `/World/cams/tv_cam` 每帧被 VR 代码按 base_link 重设位姿，**改不动**；
  要自定义视角就 `add_camera` 新建一路。

## 两个坑（都踩过）

1. **mimic 只在第一帧算过一次就永久冻结**。原 `_apply_software_mimic` 的
   `elif self._mimic_targets is not None: return` 让函数从第二帧起直接返回，
   跟随关节停在"首帧实测角 × 倍率"≈0 → 主关节单独转动 = 连杆脱节、手指不动。
   修法：每帧都重算；取值优先 VR 全量数组 `_q_all` → **指令目标**
   `_physics_view.get_dof_position_targets()`（干净，不是带噪声的实测角）→ 实测位置。
2. **先写缓存再写关节**：写失败时缓存已更新，下一帧 `isclose` 直接跳过 = 永久放弃。
   修法：只在写成功后更新 `_mimic_targets`。

另外 `_apply_software_mimic()` 必须排在 `apply_pending()` **之后**（`step()` 里），
否则跟随关节比主动关节晚一帧。

## 主动关节是 **两个**：`ctag_R2` 和 `ctag_L2`（2026-10-12 用户纠正）

之前说「只有 `ctag_R2` 一个主动关节」是错的。运动学树：

```
ctag_base_link
├─ ctag_R2 ── ctag_Right_in ── [Right_Pad]   右指根关节
└─ ctag_L2 ── ctag_Left_in  ── [Left_Pad]    左指根关节
   （R3/Right_Main/R1/Right_Slip/L3/Left_Main/L1/Left_Slip 是两指内部的连杆）
```

厂家 URDF 里**只有 R2 和 L2 写 `effort="2"`，其余 10 个全是 `effort="0"`** ——
这是"哪两个是真关节"的明确信号。

`ctag_L2` 带 `<mimic joint="ctag_R2" x1>`，所以在**支持 mimic 的工具**里只给 R2 就行；
但 PyBullet 完全不支持 mimic、Isaac 要开 `parse_mimic`。**在不支持 mimic 的地方
只驱动 `ctag_R2`，左指一动不动，看起来像夹爪坏了**（2026-10-12 实测：把
`_mimic_map` 置 None 后拉 R2 到 0，只有 R2 动，其余 11 个全停在原位）。

对策（发 URDF 给别人的时候尤其重要）：给一张 12 关节的显式取值表 ——
θ = 0(闭)~0.7(开)：
  · `R2` `L2` `Right_in` `R3` `Right_Main` `Right_Slip` `Left_in` `Left_Slip` = **+θ**
  · `L3` `Left_Main` = **−θ**
  · `R1` `L1` = **−0.15θ**
见 `assets/deliver/RM65-6FB-V_ctag2f120/scripts/ctag_gripper.py`。

## 夹爪闭合「穿模」= 驱动是 acceleration 型，刚度被惯量吃掉（2026-10-12）

**症状**：夹爪合拢时左右指连杆互相插进去（`ctag_Right_in_Link ↔ ctag_Left_Pad_Link`
29 个顶点、`ctag_Right_Pad_Link ↔ ctag_Left_in_Link` 15 个）。

**不是模型问题**：纯 FK 在厂家 URDF 上逐角扫过，`R2=0` 时只有两片指垫**刚好贴合**
（中心距 2.00mm = 2×垫厚），其余全干净。厂家 CAD 是「压紧」位，不是穿模。

**真因**：跟随关节**没到位**。命令 0，实测停在 +0.0755 rad —— 左右指的 `*_in`
连杆各偏 4.3°，于是插进对方。
再往下：导入器把驱动写成 **`drive:angular:physics:type = acceleration`**，
刚度按**惯量折算**。指节 `JointEquivalentInertia` 只有 2.1e-4 kg·m²，
k=100 折出来等效刚度才 **1.2 N·m/rad** —— 0.09 N·m 的重力负载就压出 4.3° 偏差。
臂的关节用同一个 k 没事，是因为它惯量大 800 倍（0.16）。

**修法**：`*.gains.json` 里夹爪 12 个关节 **stiffness 100 → 5000、damping 5 → 50**。
实测残差 0.0755 → **0.0008 rad**，速度 0 不振铃，FK 复检无穿插。
（残差 < 0.003 rad 就不会穿插，这是扫出来的阈值。）
单独加载官方 RM65+CT 时关节叫 `ctag_*`，`_tune_gripper_drives` 的
`"gripper" in n` 过滤条件要一起放宽，否则力矩上限提不上去。

## 指令接口（仿真/真机同刻度）

- `POST /execute {"action":"set_gripper_position","parameters":{"side":"both","position":1000}}`
  —— **睿尔曼语义** position 1~1000（1000=全开），与真机 `{ns}/set_gripper_position_cmd` 同参。
- `get_gripper_position` 回读（角度/比例/rm 单位）。
- VR 扳机：`grip_open × (1 − trigger)`（geniesim 语义：松=全开，扣=闭），
  遥测在 `vr_teleop_status` 的 `grip_cmd`（每侧 0~1），`input_peak.trigger_max` 看客户端有没有送触发值。

## 顺带修掉的"身体塌陷"

`send_joint_command` 现在支持 `joint_names` 部分下发（其余关节沿用**当前指令目标**）。
整向量下发时，调用方（如 rm_driver 桥）拿 `/joint_states` 反馈当输入 = 把"当前下垂姿态"
重新写成目标，全身被重力一帧帧压塌，并且和 VR 遥操作抢控制权。部分下发后实测 30 s
lift 目标/位置**一模一样**（−0.03851），塌陷消失。

相关 [[realman-pico-teleop]]、[[joint-sign-measurement]]、[[gripper-swap-task]]。