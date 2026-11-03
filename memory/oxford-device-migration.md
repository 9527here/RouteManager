# Oxford 机构与 Schneider CAD 迁移（2026-10-30）

## 现象
线上有 ROS1/Gazebo 机构，但项目需要 Isaac Sim 6.2、人形接触前的独立验证。

## 迁移与修复
- assets/oxford_devices：Oxford 固定提交 3a5b26c785709ffb88048c917d6de84d234bce72，BSD3，22 源文件 SHA256。
- 三类为 e_stop_button（2 活动关节）、isolator_switch（1）、safety_switch（1），各 3 组碰撞。
- Xacro 系统模块在 /opt/ros/humble/local/lib/python3.10/dist-packages；只给展开进程设置，不污染 Isaac。
- 6.2 官方导入器转换、选择 physx、合并固定零位 dummy link；isolator 握柄 visual/collision 原点差 6 mm 已对齐。
- USD angular Drive 增益为每度，源 Gazebo/运行时为每弧度；直接复制放大 57.2958 倍，写 USD 前乘 pi/180。
- root_joint.body0 指向资产根 Xform，局部零锚点已跟随摆放位姿；不要再写世界位姿，避免双重偏移。
- runtime 按关节名称映射，不依赖源 Gazebo joint_states 顺序；只在主线程跑。

## 验证数据和边界
- 独立 Isaac 6.2/PhysX：两个开关用临时位置驱动各 3 轮，约 ±89.994°，撤去测试驱动后恢复源 normal 增益并保持/回位。
- 急停用关节加载 3 轮，约 5 mm 按下锁定、旋至约 -84.99° 解锁、回位；9/9 基础机构循环通过。
- 两开关直接力矩加载未通过（isolator 约 0.0032 rad，safety 约 -0.415 rad）；去位置驱动后能继续转动，原因未证实。不能称外力操作/人形接触通过。
- 新增完整人形 VLA 验收动作 0；三类只是机构/驱动/状态机迁移。尺寸/反力未按实物标定，急停不满足先转独立外圈再按中心。
- Schneider GV6P320H 官方 STEP 1,329,459B 已入 assets/manufacturer_cad/schneider_gv6p320h；米制 usd/asset.usda 19 Mesh，独立 MANETTE，包络 .255×.179600×.139715m；仅几何，未造关节或标定力矩，厂家再分发授权未确认。
- 当前 18765 人形场景保留，50 DOF 读回有限；没加铭牌、没注册机器人相机。

脚本：scripts/*oxford*.py、scripts/convert_operation_step.py。报告：docs/线上操作资产迁移验收_2026-10-30.md。
失败轮次与最终通过数据：outputs/oxford_migration/；包内 validation_summary.json 便于外移时查看边界。

项目 HTTP 接入实测：独立 18766 空白服务 setup/status 均成功，3 类反馈有限、无状态机错误；测试服务已停止，原 18765 人形会话保留。
