# 仿真臂模型 vs 官方 RM65-6FB-V 对比（2026-10-10）

官方 URDF：`RealmanRobot/rm_models` → `RM65/urdf/RM65-6FB-V`（sparse clone 拉的，
2026-10-10 起在仓库内 `assets/RM65-6FB-V/`，含 meshes/config/launch，
`package://RM65-6FB-V/meshes/...` 引用自洽）。
本地模型：`humanoid_upper_body_v3_ctag2f120.urdf` 的左右臂。

## 核心结论：仿真臂不是 RM65 的运动学构型

零位相邻关节轴夹角序列（旋转不变，最硬指标，脚本 `/tmp/compare_arm_kin.py`）：

| | 轴夹角序列 |
|---|---|
| 真机 RM65（6轴） | 90°, **0°**, 90°, 90°, 90° — j2∥j3（肩肘两平行轴，相距256mm） |
| 仿真模型（7轴） | 90°×6，无平行轴 — 全正交 R-P-R-P-R-P-R |

整体旋转无法对齐（平行对消不掉）。总臂展接近（仿真沿-y 874mm，真机竖直 891mm），
但轴构型/分段长度全不同 = IK、奇异位、工作空间、笛卡尔轨迹都不可直接迁移。

## 差异清单

1. **自由度 7 vs 6**：仿真多 `{left,right}_wrist_joint`（原人形模型 ZJ/YJ 转腕轴，
   ±175°），RM65 无此关节；末端法兰真机由 J6 直驱。
2. **arm_6 限位 ±120°，真机 J6 ±360°**（±6.28）。
3. **effort**：仿真 70/60/40/30/20/10/10；官方 60/60/30/10/10/10。
4. 官方 URDF 无 dynamics damping/friction；仿真有（1.4→0.5 梯度）——仿真保留可接受。
5. **左右臂疑似不对称（待确认 CAD）**：arm_1 y 偏置 左136.5/右146.5（差10mm）；
   arm_2 y 偏置 左55.9/右38.1（差17.8mm）。镜像应相等，疑似旧抄错残留。
6. **相机**：官方在 link_6 → camera_rojoint（手动 0~60.2°，effort 0）→ camera_link
   fixed，(0.029,-0.100,-0.0005) rpy=(0,π/2,0)，朝法兰前下方；本机定制相机嵌在
   wrist 腕壳内、朝上——安装位/朝向完全不同，手眼关系必须按真机重新标定。
   官方配套相机 = 奥比中光 Gemini 335Lg（白皮书）。
7. 官方 CSV（RM65-6FB-V.csv）包名/数据是旧 RM65-B（j6 origin -0.144、axis -z），
   与 URDF（-0.18448、+z）矛盾，以 URDF 为准。

## 重建已完成（2026-10-10，用户拍板"除相机按官方参数"）

生成器：人形包 `scripts/build_rm65_arm.py`（idempotent，可重跑）。产物：
- `urdf/humanoid_upper_body_v3_ctag2f120_rm65.urdf` + `_isaac.urdf`（绝对路径版）
- `meshes/visual/rm65_{base_link,link_1..6}.STL`（官方网格拷入，visual+collision 同文件）
- `meshes/visual/{ZJ,YJ}_wrist_rm65.STL` —— **腕壳切除旧夹爪安装板**（切面 y=±15mm；
  壳体主体 ≤12mm、板 ≥19mm、相机筒无伤）；碰撞凸包 6 段重切 `_hull{0..5}`
- 官方 6 轴：origin/axis/limit/effort/inertia/mesh 全官方；关节名沿用 arm_1..6；
  **删除 7th 轴 wrist_joint**（真机 RM65 无此自由度）
- 安装：`lift_to_*_arm_joint` rpy=(0,∓90°,±90°)（xyz 肩座位不变）——官方零位"臂向上"
  映射为水平外伸=旧零位；正方向逐关节与旧臂同号（j1 外滚、j2 正=下摆）
- 腕壳固定于法兰（`*_wrist_mount_joint`，rpy 使零位腕系∥世界，相机朝上保持），
  CT 夹爪经 wrist_to_gripper 直装切面（0.0155m，0.5mm 间隙），mimic 链原样
- 腕壳惯性按保留三角形比例 ×0.942（推定值）

零位验证：腕 y=∓986mm（官方 891 臂长）、指垫 ∓1258.7（旧 ∓1276.4）、腕系旋转偏离
单位阵 2.4e-11、check_urdf 过、Isaac 实测 dofs=48（12 臂+24 爪+12 底盘）、
渲染确认站立+T-pose+夹爪直连无多余段。

## Isaac 5.x load_urdf 修复（robot_manager，通用）

- **5.x 两步流程**：`parse_urdf(目录, 文件名, cfg)`（第二个参数是文件名不是 dest！传错
  会去解析 .usd 报 Failed to parse）→ `import_robot(dir, file, robot, cfg, dest, True)`
- **产物无 defaultPrim**：add_reference 会 Unresolved reference（空壳 prim，GetAllChildren
  为空、后续 Articulation 报 backend None 的迷惑错误）——转换后必须
  `Usd.Stage.Open(dest)` + `SetDefaultPrim(顶层)` + Save
- **干净场景直接 load_urdf 需先 world.reset()**（tensor 后端 backend=None 断言炸）
- **热 reload(robot_manager) 在该服务不生效**（类换新方法体仍旧代码，co_names 可判）——
  只能重启服务
- 枚举改名：`UrdfJointTargetType.JOINT_DRIVE_POSITION`（旧 UrdfJointTarget.DRIVE_ANGLE）
- URDF/引用含中文路径 OK（Isaac 解析正常）

## 腕壳相机筒朝向（2026-10-10，最终版）

**腕壳上带格栅槽、朝外凸出的那块 = 相机筒**（不是废料，我一开始误判成"旧夹爪
安装板"按 z 平面切掉了——用户指出"这个是对的，但是相机要朝上"，已恢复并翻正）。

- rm65 重建版继承的**左右壳朝向本来就不一致**（资产 bug）：
  `ZJ(左)` 凸出在局部 z<0，左腕零位腕系∥世界 ⇒ 凸出朝【下】；
  `YJ(右)` 凸出在局部 z>0 ⇒ 凸出朝【上】。所以仿真里左低右高。
- 修法：**只把左壳 z 镜像**（z→−z + 三角绕向 v0↔v1），右壳不动 ⇒ 两边都朝上。
  脚本 `humanoid_upper_body_v3/scripts/flip_wrist_camera.py`（幂等，输入取
  `meshes/visual_backup_wristcut/` 的未切版）。
- `cut_wrist_shell.py` 已**作废**——它按 z=∓32mm 切掉的就是相机筒。别再跑。
- 碰撞凸包按原配方（y 向 6 等分段）随壳重切。
- rm65 壳 vs v3 壳的差别：rm65 版多了 y>15mm 的切割（那才是真的旧夹爪安装板
  残留，与相机无关），81847 vs 86884 三角。
- 生效：改 URDF 直接引用的 STL ⇒ 重跑 bringup。`config.json` 的 `robot.name`
  已改 `v3_ctag2f120_rm65`。

## 官方夹爪 URDF 已经在模型里（逐位核对）

用户问"为什么不能直接用官方的夹爪 URDF 替换"——答案：**已经在里面了**。
把 `crt_ctag2f120_isaac.urdf` 的 14 个关节与人形 URDF 的 `left_gripper_*` 逐位比：

- origin xyz / rpy / axis / type / mimic 倍率 **全部一致**；
- 唯二差别：① 人形把官方零散关节名（R2/Right_in/…）加了 `left_gripper_` 前缀、
  mimic 的引用名同步改；② **官方的 limit 全是 (0,0)/(−1,1) 一次性导出废值**，
  这正是"人形原版夹爪不能抓"的原因，人形里填了真实值 [0, 0.7] / [−0.105, 0]。
- 所以"直接替换"没有意义——换了也是同一套数；官方 URDF 是独立夹爪（根
  base_link），要装上臂必须重新挂到腕 link 上（=当年合并做的事）。

## 遗留待办

- **抖动未解决（用户喊停）**：已做 robot_manager.load_urdf 增益烘焙机制
  （<stem>.gains.json，值=USD 级；arm_1 USD 1.524≈旧温和值 1.5✓）+ _fix_gripper_joint_gains
  进 load_urdf + defaultPrim 必须顶层 Xform（指向 base_link 的嵌套 defaultPrim 会
  segfault！）+ staged_path 指向挂载后 articulation root（/humanoid/base_link，
  prim 树多一层 base_link）。加载成功但实测仍抖（max Δ0.16rad/帧、臂关节为主）——
  下次从下述方向查：bake 值是否真写进关节 prim（"无 drive 跳过 2"是哪两个）、
  位置增益 vs URDF effort 的关系、或者直接走离线 finalize_usd 流程。
- 官方配置文档已整理：assets/RM65-6FB-V/OFFICIAL_SPECS.md
- config preset `v3_ctag2f120_rm65` 已加（urdf_path 走 load_urdf）
- isaac_teleop arm_joints 需从 7 改 6（wrist 没了）；ros2_relay/rm_driver_bridge 映射核对
- 7 相机变体（cameras.urdf）未在新臂上重建（add_cameras.py 基于旧臂）
- 肩座位左右差 2mm（原 CAD 就有）保留

相关 [[gripper-swap-task]]、[[realman-pico-teleop]]、[[joint-sign-measurement]]。
