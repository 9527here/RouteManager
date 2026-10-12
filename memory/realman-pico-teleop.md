# 睿尔曼 RM65 × Pico 4 Ultra 遥操作（官方调研 + 复刻位置）

2026-09-28 调研结论（用户真机臂 = 睿尔曼 RM65）：

## 官方现状
- **睿尔曼官方无 Pico/VR 遥操作**。官方遥操套件 = 主从同构臂（GLN-RX75/RX71/Aloha，
  拖动零力跟随，RJ45）。官方 ROS2 包 `RealManRobot/ros2_rm_robot`(humble) 无 teleop 包，
  但提供全部透传原语（下）。
- **Pico 官方 (XR-Robotics) XRoboToolkit** 是 Pico 侧标准栈：PC-Service-Pybind（本机
  `/home/jy/XRoboToolkit-PC-Service-Pybind`）+ Teleop-Sample-Python（硬件样例只有
  UR5e/ARX-R5/Galaxea-R1）+ Teleop-ROS（picoxr 节点发 xr_msgs）。
- "官方合作" = XRoboToolkit(输入+IK) ↔ rm_driver CANFD 透传(输出)，中间桥使用方自写。

## rm_driver 关键点（真机执行层）
- 关节透传 `/rm_driver/movej_canfd_custom_cmd`(Jointposcustom): joint[6] **弧度**、
  follow=高跟随、trajectory_mode 0透传/1曲线拟合/2滤波 + radio 平滑系数；须连续发点
  （moveit2 即用此模式）。位姿透传 `movep_canfd_cmd`(Cartepos, 臂端IK, 2ms周期) —
  人形(臂装在动躯干上)不适用, 用 PC 端 IK + 关节透传。
- 夹爪 `set_gripper_position_cmd`(Gripperset): position 1~1000 ↔ 开口 0~70mm；
  `set_gripper_pick_cmd` 力感自动夹取(抓取打标可用)。急停 `move_stop_cmd`(Empty)。
- 反馈 /joint_states 或 config 开 UDP 主动上报(udp_cycle 5ms→200Hz)。
- 双臂 = 两个 driver 实例: 独立 namespace + arm_joints 改名(left_joint1..)，否则
  /joint_states 撞名。升降柱原生支持(set_lift_height_cmd)。

## 官方 Pico 控制律（Teleop-Sample-Python, 实测代码）
grip>0.9 离合; 接合锚 EE+手柄, 松开把任务目标同步回当前EE(防跳变);
Δee = R_headset_world·Δp_ctrl×0.8 世界系左乘; **R_HEADSET_TO_WORLD =
[[0,0,-1],[-1,0,0],[0,1,0]] 与我们仿真端独立推导的 P 矩阵逐元素一致**(强验证);
PlaCo QP IK: frame/position task 1.0 + manipulability 5e-2 + 动能正则 1e-6;
control_mode "position"(仅位置)=我们仿真验收过的语义; trigger 线性夹爪;
B 键=数据录制开关, right_axis_click=丢弃(DataLogger 内置, VLA 采集现成)。

## 复刻产物（已写, 未上真机验证）
`/home/jy/XRoboToolkit-Teleop-Sample-Python/`（官方仓库持久克隆 + 新增）:
- `xrobotoolkit_teleop/hardware/interface/realman_rm65.py` — rm_driver ROS2 桥
- `xrobotoolkit_teleop/hardware/dual_arm_rm65_controller.py` — 按官方 UR 控制器 1:1 结构,
  关节按名字映射 placo idx_q(人形URDF), 默认 position 模式, 左摇杆按下=双臂急停
- `scripts/hardware/teleop_dual_rm65_hardware.py` — 入口(tyro CLI)
- `README-REALMAN.md` — 全链路/部署/标定/安全/与仿真端对应表
另有 `/home/jy/XRoboToolkit-Teleop-ROS/`（Pico→ROS2 话题桥, 备选输入路径）。
人形 URDF 臂关节名: left/right_arm_1..6_joint; EE link left/right_gripper_base_link。

相关 [[geniesim-official-config]]、isaac-camera-aim-math(全局)。
