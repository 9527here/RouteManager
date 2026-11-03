> 2026-11-02：本文的203/0.08修复仅用于旧skid-steer位置积分控制。当前四轮八驱是原生速度驱动，轮kp=0正确，不要改回位置弹簧；见[四轮八驱新结论](mobile-asset-approach-swerve.md)。

# 底盘推不动：轮关节被烘成纯阻尼驱动（2026-10-13 修）

用户跑完整任务序列时底盘**一步不动** —— `set_base_velocity` 一路返回成功，
90 秒只挪 15mm。

## 根因

`_update_base_velocity` 走的是**虚拟速度模式**：把 (vx, wz) 折算成四轮角速度，
再逐帧把**位置目标**前移 `ω*dt`（钳在实测位置 ±0.6 rad 内），靠位置驱动推。

但人形资产烘出来的 `*.gains.json` 里四个轮关节是：

```
front_left_wheel_joint   {'stiffness': 0.0, 'damping': 99.72583}
```

**stiffness = 0 的驱动会完全忽略位置目标**，只按 `targetVelocity`（默认 0）
当刹车用。所以位置目标写了也白写，轮子只在原地抖（实测速度 ±0.3 rad/s 的噪声）。

## 修法

`robot_manager._tune_wheel_drives()`，在 `_build_mimic_map()` 末尾（两条加载
路径都会走到）调用，与 `_tune_gripper_drives()` 同一套思路：
把四个轮关节的 stiffness/damping 统一写成
`config.json` → `base_teleop.wheel_stiffness / wheel_damping`（默认 **203 / 0.08**，
USD 级数值，写进物理视图要 ×57.29578）。

实测效果：同样 `vx=0.3` 指令，5 秒从 **1.1mm → 963mm**（≈0.16 m/s），
轮速到 1.5~2.2 rad/s（指令 1.88）。

日志会打：`底盘轮驱动刚度 0.0 → 11631.0 (USD 203.0/0.08, 4 个轮)`。

## 遗留

- 直行会**轻微右偏**（1m 约偏 0.10m）。`left_sign/right_sign` 在
  `config.json` 的 `base_teleop` 里，要调就调这两个。
- 转向比直行慢得多（skid-steer 原地转要克服地面摩擦），
  `task_sequence.py` 的 `goto()` 用死区 + 离散两拍，仍可能超时。
- **底盘没开到位会连带抓取失败**：`_grasp_random_place` 的落点是
  "机器人底座 + 前方 0.44~0.58m"，底盘停在离桌 0.4m 处时落点会被
  `clip` 到桌面边缘，变成臂展够不着 → `auto_pick` 报 "规划失败: None"。
  排查抓取失败时先确认底盘在不在 `_TABLE_POSE`。

相关 [[rm65-grasp-failure-root-cause]]、[[scene-table-grasp-setup]]。
