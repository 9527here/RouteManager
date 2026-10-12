# Memory Index

- [Gripper swap task](gripper-swap-task.md) — 把 changingtek 夹爪换进人形上装 URDF；ctag2f120 已合并+修复转腕关节，cthm3f460 版未做
- [Ask user for data, don't guess](ask-user-for-data-dont-guess.md) — 需要规格/实测数据必须列清单向用户要，推定值必须标注，不自己猜
- [RouteManager project](routemanager-project.md) — Isaac 仿真 HTTP 服务器；env=isaacsim5.1；2026-09-23 七项修复（ROS2 闭环/加载等待/相机挂载等），实机链路未测
- [Isaac Teleop docs](isaac-teleop-docs.md) — XR 遥操作框架文档；1.6 起包名 isaaccapture；VLA 数据采集前端备选
- [Realman×Pico 遥操作](realman-pico-teleop.md) — rm_driver CANFD 透传要点、官方 XRoboToolkit 控制律（P 矩阵与仿真一致）、真机复刻代码位置
- [关节正方向实测法](joint-sign-measurement.md) — head_yaw 正值=右转（URDF 推反了）；驱动关节+抓图测质心位移的通用判定法；/python 不回显 result 要用写文件
- [场景摩擦≈0](scene-friction-zero.md) — 抓不住/机器人打滑的根因；static_scene.usda 加材质+地面绑材质；过盈可降到 0.5mm
- [夹爪 mimic 链与指令接口](gripper-mimic-chain.md) — 主动关节力矩被跟随关节反锁=闭不到底；leader↔开口实测表；TCP 随开口漂移 11mm；抓取配方（过盈 3mm）；set_gripper_position 睿尔曼语义；部分下发治身体塌陷
- [自动抓取（VR 菜单键）](auto-pick-menu-key.md) — 左手菜单键→抓最近物体；非阻塞状态机；grasp_planner 共用；遥测字段
- [仿真臂 vs 官方 RM65](arm-model-vs-official-rm65.md) — 已重建：官方6轴(891mm与官方逐位相同)；腕壳旧夹爪安装板已切(cut_wrist_shell.py，相机在壳顶不受影响)；rm65变体+build脚本+5.x load_urdf修复(两步流程/defaultPrim/热reload失效)；config robot.name 已指向 rm65
