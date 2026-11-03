# 把这个人形搬进 Isaac Lab 做阀门任务：四条硬结论（2026-10-28）

代码归档在仓库 `valve_rl/`（Isaac Lab manager-based 任务）。**任务本身没成功**，
但过程里挖出的四条结论是通用的，换任何任务都要用。

## 结论 0：RL 这条路对「采数据」不划算

五阶段（张开→从上接近→下降→抓握→拧动→松开→收回）用 PPO 从零学，两版都停在
奖励平台期（11.05 / 12.61）不再上涨，策略只会**悬停在阀门上方 0.22 m**，
`eval_stages.py` 实测阀门转角恒为 0。

原因：分阶段信用分配 + 接触丰富 + 稀疏成功，32 环境 × 800 轮解决不了。
**采数据应该用 IK 脚本轨迹**（`scripts/valve_trajectory.py`），轨迹形状本来就是
已知的，不需要学。RL 留给「以后要做闭环策略 / sim2real 泛化」。

## 结论 1：actuator 增益必须读项目自己的 gains.json

这个 humanoid USD 是厘米级换算过的，**标定值比直觉大 2~4 个数量级**：

| 关节 | 项目标定 stiffness |
|---|---|
| `lift_joint` | **182356** |
| `torso_pitch_joint` | 6021 |
| 手臂各关节 | ~1139–1309 |

一开始按「只有右臂要被驱动」把其余关节设成 `stiffness=0`，结果零动作时
`lift_joint` 漂 -0.141 rad、左臂垂下来 —— GUI 里看就是「一直升降、手臂莫名其妙地动」。
改成 45/1000 仍然不够。正确做法是读
`.../humanoid_upper_body_v3/urdf/*_isaac.gains.json` 按 (stiffness, damping) 分组，
覆盖全部 50 个 DOF（RouteManager 早已把这些值烘进 USD 的 drive，写错值等于把它们盖掉）。

**自检手法**：给零动作跑 120 步，看各关节漂移。健康的话应 <0.02 rad
（`valve_rl/probe_drift.py`）。

## 结论 2：夹爪 12 关节必须按 mimic 链联动

主动关节只有 `{side}_gripper_right_2_joint`，其余 11 个靠 URDF mimic 跟随：

```
right_2 / _in / _3 / _main / _slip / left_2 / left_in / left_slip  → × 1
right_1 / left_1                                                  → × -0.15
left_3 / left_main                                                → × -1
```

Isaac Lab 里用 `BinaryJointPositionActionCfg` 把这 12 个关节一起写开合值。
只驱动 `right_2` 的话四连杆不会合拢（RouteManager 仿真侧靠 `_apply_software_mimic`
补，Isaac Lab 没有这套）。

## 结论 3：Isaac Lab 训练的两个必踩坑

1. **必须调 `handle_deprecated_rsl_rl_cfg(agent_cfg, installed_version)`**
   （`isaaclab_rl/rsl_rl/utils.py`）。漏了会报
   `MLPModel.__init__() got an unexpected keyword argument 'stochastic'`
   —— 已弃用参数没被迁移成 `distribution_cfg`。
   这个报错**会被块缓冲的 stdout 埋掉**，看起来像「静默卡死」；调试时必须加
   `PYTHONUNBUFFERED=1`。
2. **所有 isaaclab / omni / pxr 导入必须在 `launch_simulation` 之后**。
   否则 USD 插件在启动阶段报一堆
   `undefined symbol: pxrInternal_v0_25_11__pxrReserved__`。

## 附带：Isaac Lab 的 train.py CLI 在这个环境里用不了

`isaacsim_physx.KitLauncher` 那条启动路径下 import pxr 会撞 USD 库版本冲突
（`libusd_usdPhysics.so: cannot open`）。同样的 cfg 走自己写的
`launch_simulation` 完全正常 —— 所以 `train_local.py` 是照抄 CLI 后端的准备顺序
（`entrypoints/backends/train_rsl_rl.py:129-160`）手写的入口。

## 另一个坑：PhysX 雅可比不是 TCP 的

Isaac Lab 的 `DifferentialIKController` 吃 `root_physx_view.get_jacobians()`，
而**这个雅可比是关于连杆质心的、不是 TCP 的**（`arm_kin.py:3-5` 有记录）。
直接拿它做位置控制，末端会**越走越远**（实测距离单调从 0.558 涨到 1.204）。
做 IK 应该用仓库自带的 `arm_kin.py`（numpy FK/雅可比，交叉验证到 0.05mm）
或 pink/placo。

相关 [[isaac62-migration]]、[[ros2-inprocess-contract]]。
