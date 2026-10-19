# 颈部/腕部相机转台：烘出来的阻尼几乎为 0（2026-10-14）

用户："机身还是会轻微晃动"。

## 量出来的结论

先排除法量了一遍（**静止整定后所有 link 世界位置摆幅 0.000mm** —— 静止时
机器人完全不动，所以晃一定发生在运动过程中）：

| 部位 | 抓取全程摆幅 |
|---|---|
| 底盘 base | 位置 ≤0.6mm、roll/pitch ≤0.17° |
| 躯干 torso_pitch | 0.10° |
| 手臂各关节 | 位置摆幅 ≤0.3° |
| **head_pitch** | **极差 61.7 mrad = 3.54°** |

抖动强度（逐帧二阶差分中位数）排第一的也是 `head_pitch_joint`（3.35 mrad，
峰值 30 mrad），第二是 `{left,right}_camera_rojoint`（1.0~1.4 mrad）。

## 根因

URDF 里 `head_{pitch,yaw}_joint` 写的是 `damping=0.5`，但烘进 `*.gains.json`
时变成了 **0.0215 / 0.0435**（×0.043）。`_fix_gripper_joint_gains` 本来就在
修"阻尼过小的关节"，但它只挑**名字含 `gripper`** 的，颈部被漏掉。

## 修法

新增 `robot_manager._tune_neck_drives(zeta=1.5)`，在 `_build_mimic_map()`
末尾和 `_tune_wheel_drives()` 一起调用。对
`head_pitch_joint` / `head_yaw_joint` / `*_camera_rojoint` 按
`d = ζ·2·√k` 补阻尼（acceleration 型的 ζ = d/(2√k)，跟着各自刚度走，
不用手填绝对值）。

实测：抓取全程 head_pitch 极差 **61.7 → 32.2 mrad（3.54° → 1.85°）**，
单次最好到 22.9 mrad (1.31°)。静止时本来就完全不晃。

> 注意：试过把颈部刚度也往上推，结果**非单调** —— k×3 时 0.84°、
> k×10 时反而 7.2°、k×51 时 0.07°、k×1528 直接发散。别乱加刚度，加阻尼。

相关 [[rm65-grasp-failure-root-cause]]。
