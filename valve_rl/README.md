# valve_rl — 阀门拧动 RL 任务（**归档，未跑通**）

Isaac Lab (28.0.0) + Isaac Sim 6.2 上的 manager-based 任务，用 RouteManager 那个人形
上装拧阀门。**代码可用、任务没成功**，作为参考实现归档。

> 结论性的踩坑记录在 [`../memory/isaaclab-valve-rl-findings.md`](../memory/isaaclab-valve-rl-findings.md)
> ——**先看那份**，很多坑不用重踩。

## 状态

| 项 | 结果 |
|---|---|
| 环境能建、能 step | ✅ 动作 (N,7)、观测 (N,109)、奖励有限 |
| 机器人能当 articulation 加载 | ✅ 50 DOF |
| PPO 训练 | ⚠️ 两版都停在奖励平台期（11.05 / 12.61） |
| 拧动阀门 | ❌ 策略只会悬停在阀门上方 0.22 m，转角恒为 0 |

阀门拧动改走 IK 脚本轨迹：`../scripts/valve_trajectory.py`。

## 依赖的资产（不在本仓库内）

两个 USD 需要自行准备，默认从下面的位置读，可用环境变量覆盖：

| 环境变量 | 默认值 | 内容 |
|---|---|---|
| `VALVE_ASSETS_ROOT` | `/home/jy/datasets/vla_data/L2_仿真运行` | `机器人/humanoid_upper_body_v3_ctag2f120_rm65/`、`场景/操作台_piper_table_v2/asserts/valve/valve_1/` |
| `VALVE_GAINS_ROOT` | `/home/jy/datasets/vla_data/L1_制作资产/载体设备/humanbody_urdf` | `humanoid_upper_body_v3/urdf/*.gains.json` |

这三个东西（机器人 USD、阀门 USD、gains.json）来自 `/home/jy/datasets/vla_data`
这套私有资产，仓库里没有。**注意 actuator 增益必须用它，别自己写**——
原因见 findings 第 1 条。

## 跑法

```bash
pip install -e . --no-deps          # 装 isaaclab.tasks entry point
./run.sh smoke.py                   # 冒烟：建环境 + 随机动作
./run.sh train_local.py --num_envs 32 --iterations 800    # 无头训练
RM_KEEP_DISPLAY=1 DISPLAY=:1 ./run.sh train_local.py --gui --num_envs 8   # 可视化
```

`run.sh` 会 unset DISPLAY、摘掉 `/opt/ros/*` 的 PYTHONPATH/LD_LIBRARY_PATH（否则
py3.10 的 ROS 包会污染这个 py3.12 环境），并通过 `micromamba run` 启动。

> 不走 Isaac Lab 自带的 `train.py`：它在这套环境里会撞 USD 库版本冲突
> （`libusd_usdPhysics.so: cannot open`）。`train_local.py` 是照抄其后端准备顺序
> 手写的等价入口。

## 诊断脚本

| 脚本 | 用途 |
|---|---|
| `probe_robot.py` | 机器人能否作为 articulation 加载、打印 50 个关节名 |
| `probe_drift.py` | **零动作漂移**——检查 actuator 增益是否写对（健康应 <0.02 rad） |
| `probe_reach.py` | 末端到阀门距离、臂展占比 |
| `probe_pose.py` | 随机采样 + 正运动学搜「朝阀门的前伸姿态」 |
| `probe_valve.py` | 手轮关节是否真的能转 |
| `eval_stages.py` | 按五阶段评估 checkpoint 实际做到哪一步 |
