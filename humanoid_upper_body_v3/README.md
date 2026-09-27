# humanoid_upper_body_v3

轮式底盘 + 升降 + 头部云台 + 双臂（29 link / 28 joint）的 Isaac Sim 就绪 URDF/USD 包。

源 CAD：`../urdf/人形上装部分9.9.urdf`（**只读，永不修改**）。
所有 link/joint 命名遵循 `/home/jy/文档/urdf命名规则`（v1.0），映射表唯一权威在
`scripts/naming_rules.py`。

## 流水线

```
../urdf/人形上装部分9.9.urdf          collision_shapes.json   drive_gains.json  joint_dynamics.json
        │                             (源名键)                (源名键, 带连字符)
        ▼                                   ▼                        ▼
scripts/build_isaac_urdf.py ──► urdf/humanoid_upper_body_v3.urdf    │  (末段按规则重命名)
        │                                                           │
        ▼                                                           │
scripts/validate.py  (0 FAIL 门禁)                                  │
        ▼                                                           │
scripts/gen_joint_config.py ──► config/joint_names.yaml ◄───────────┘
        ▼
scripts/import_to_isaac.py ──► usd/humanoid_upper_body_v3.usda
        ▼
scripts/finalize_usd.py ──► usd/humanoid_upper_body_v3_flat.usd  (含自检)
        ▼
物理回归: /tmp/smoke_v3.py + /tmp/roll_v3.py
```

注意：`drive_gains.json` / `joint_dynamics.json` / `collision_shapes.json` 的键是
**源 CAD 关节/连杆名**（如 `SX-JOINT`、`SX_Link`），有意不改 —— 消费端通过
`naming_rules.py` 翻译（`joint_source_key()` / `REVERSE_JOINTS`）。

## 命名映射表（源名 → 规则名）

### 连杆 (29)

| 源名 | 规则名 | 说明 |
|---|---|---|
| `base_link` | `base_link` | 根，本身合规 |
| `SX_Link` | `lift_top_link` | 升降活动级（prismatic 滑块）|
| `XQ` | `torso_pitch_link` | 躯干俯仰级 |
| `TZ` | `head_yaw_link` | 头偏航 |
| `TF` | `head_pitch_link` | 头俯仰 |
| `Z1` | `left_arm_base_link` | 左臂座（Z=左）|
| `Z2`..`Z7` | `left_arm_1_link` .. `left_arm_6_link` | 左臂按基座→末端编号 |
| `ZJ` | `left_wrist_link` | 左腕回转（CAD 无夹爪，故不用 gripper 名）|
| `Y1` | `right_arm_base_link` | 右臂座（Y=右）|
| `Y2`..`Y7` | `right_arm_1_link` .. `right_arm_6_link` | 右臂，同左臂 |
| `YJ` | `right_wrist_link` | 右腕回转 |
| `ZQY` | `caster_front_left_link` | 万向轮支架（Q=前 H=后）|
| `ZQL` | `front_left_wheel_link` | 从动轮 |
| `ZHY` | `caster_rear_left_link` | 万向轮支架 |
| `ZHL` | `rear_left_wheel_link` | 从动轮 |
| `YQY` | `caster_front_right_link` | 万向轮支架 |
| `YQL` | `front_right_wheel_link` | 从动轮 |
| `YHY` | `caster_rear_right_link` | 万向轮支架 |
| `YHL` | `rear_right_wheel_link` | 从动轮 |

### 关节 (28)

| 源名 | 规则名 | 类型 |
|---|---|---|
| `SX-JOINT` | `lift_joint` | prismatic z |
| `XQ-JOINT` | `torso_pitch_joint` | revolute +Y |
| `TZ-JOINT` | `head_yaw_joint` | revolute |
| `TF-JOINT` | `head_pitch_joint` | revolute |
| `Z1-JOINT` | `lift_to_left_arm_joint` | fixed（规则 3.3.4 明文；物理父级是躯干俯仰级）|
| `Z2-JOINT`..`Z7-JOINT` | `left_arm_1_joint` .. `left_arm_6_joint` | revolute |
| `ZJ-JOINT` | `left_wrist_joint` | revolute |
| `Y1-JOINT` | `lift_to_right_arm_joint` | fixed |
| `Y2-JOINT`..`Y7-JOINT` | `right_arm_1_joint` .. `right_arm_6_joint` | revolute |
| `YJ-JOINT` | `right_wrist_joint` | revolute |
| `ZQY-JOINT` / `ZHY-JOINT` / `YQY-JOINT` / `YHY-JOINT` | `caster_<front\|rear>_<left\|right>_joint` | revolute（±5° 锁定，见 gen_drive_gains.py）|
| `ZQL-JOINT` / `ZHL-JOINT` / `YQL-JOINT` / `YHL-JOINT` | `<front\|rear>_<left\|right>_wheel_joint` | continuous |

### 有意不重命名的

- **网格文件名**（`TF_0.STL`、`ZQL_0.STL`…）：保持源 CAD 命名便于溯源；规则只约束模型名。
- **源键 JSON**（增益/动力学/碰撞形状）：见上。
- **包名 / robot 名** `humanoid_upper_body_v3`：不在规则约束范围内。
- **USD 内部引用的视觉节点名**（`Geometry/lift_top_link/visuals/SX_Link/…` 的末段）：
  来自网格/源节点名，与 link 命名规范无关。

## 关键设计（改动与 v2 的差异见 build_isaac_urdf.py 文档串）

- 12 mm 左右不对称**保留**（用户决定）；只修 z 镜像（Y 臂 origin z 与 Z 臂对齐）。
- 万向轮锁定建模（±5°）：底盘甲板侵入 0.21 m 旋转扫掠，真实结构转不动。
- 轮子 = **自由滚设计（2026-09-14 用户拍板 B）**：jointFriction 1 N·m、速度驱动
  maxForce 钳到 2 N·m（低于地面摩擦力矩 ~28 N·m = μ0.5 × 轮载 293 N × r 0.19 m）。
  推/拖底盘时四个 castor 自由滚动不打滑；代价是静止时不能靠轮子自驱（2 < 28，
  物理上两者互斥），且坡道上可能缓滑。从动轮本来就不需要自驱。
- USD 结构与 v2 已知好基线对齐：240 Hz、solver 32/4、0 实例、80 个
  guide-purpose convexHull 碰撞、材质在 `/humanoid_upper_body_v3/Materials`。
- 网格在 finalize 6c 段做顶点焊接（1e-9 m 容差）+ 删 authored 法线：5.1 API
  导入器原样保留 STL 面片汤（每面 3 独立点 + per-point 法线，84 B/三角形），
  焊接后 1330 万点 → 211 万点（与 v2 的焊接结果一致，无损），flat 从 320 MB
  降到 42.7 MB；物理回归数字与焊接前逐位相同。

## 夹爪变体（2026-09-15）

`urdf/humanoid_upper_body_v3_ctag2f120.urdf`（59 link / 58 joint）＝ 基座 v3 + 双 ctag2f120
两指夹爪。源 `~/下载/人形上装部分9.9/urdf/人形上装部分9.9_ctag2f120.urdf`（只读）。

```
scripts/build_isaac_urdf.py <源路径>      # 带 ctag2f120 自动进夹爪模式
scripts/validate.py urdf/humanoid_upper_body_v3_ctag2f120.urdf   # 0 FAIL 门禁
```

- **robot/包名不变**：仍为 `humanoid_upper_body_v3`，`usd/ros2_graph.usd` 零改动迁移；
  夹爪 24 个活动关节进 articulation 后 /joint_states 自动多带（发布端未设名单）。
- **命名**：数字式扩展（规则文档 3.3.3 精神）——`left_gripper_right_2_joint`、
  `left_gripper_right_main_link`、挂载 `left_wrist_to_gripper_joint`、基座
  `left_gripper_base_link`；指内 right/left 是夹爪自身两指，与整臂左右无关。
  映射在 `naming_rules.py` 夹爪段（唯一权威）。
- **每手 15 关节**：挂载 fixed + **真驱动只有 R2 一个**（`*_gripper_right_2_joint`；
  L2 也是跟随——源 URDF 里 `ZJ_L2 mimics ZJ_R2 ×1`，左指整链挂在右指后面）+
  10 个 mimic 跟随 + 2 pad fixed。`<mimic>` **保留**（ROS2/RViz 正确）。
  行程与乘数自洽（如 right_1 [-0.105,0] = -0.15 × right_2 [0,0.7]）。
  **行程语义：0 位 = 闭合（pad 间距 ~2 mm），0.7 rad = 全开（~75 mm）。**
- **mimic 在 Isaac 5.1 的实况（2026-09-15 无头实测）**：导入器默认把 `<mimic>`
  转成 PhysX 原生 `PhysxMimicJointAPI`（带 referenceJoint/gearing/offset），同指
  ×1 跟随链工作正常；但**跨指链（L2→R2、L3→R3、Left_Main→Right_Main、L1→R1）
  导入行为错误**：gearing 与 URDF 乘数反号、左手跟随关节限位丢成 ±inf、试关时
  关节跑到 1.6~2.2 rad（设计行程 ±0.7）。**故导入时勾选 Ignore Mimic**：跟随
  关节成为普通有限位关节（实测限位与 URDF 一致），闭合在指令侧做——每手下发
  12 个关节 = 驱动 θ + 跟随 ×乘数（ROS2 bridge 本就全关节下发）。
- **动力学**：CAD 给驱动 effort=2 / velocity=1000 rad/s（导出垃圾值）→ 统一钳为
  e=2 / v=4 / damping 0.1 / friction 0.05（`build_isaac_urdf.py` 的 `GRIPPER_DYN`），
  上机后按实测收。
- **碰撞**：基座 29 link 复用 v3 spec（80 形状）；30 个夹爪 link 保留源 STL 网格
  碰撞（小件，抽稀不划算），17 个 ctag2f120_*.STL 拷入 `meshes/visual/`（碰撞与
  视觉同文件）。
- validate.py 对照源随 argv 切到 ctag2f120 导出；第 5 节质量比对经
  RENAME_LINKS 归一（修掉了原先规则名 vs 源名空转比对）。基线 v3 回归不变
  （0 FAIL，12 mm y 不对称 WARN 为用户拍板保留项）。
- **finalize（导入后处理）**：`scripts/finalize_usd_ctag.py [导入的.usda]`
  （v3 基线 `finalize_usd.py` 不动，夹爪差异全在独立脚本）。除 v3 全套
  （recenter / 材质拯救 / 扁平化 / 碰撞规整 / 240 Hz / solver 32/4 / 网格焊接 /
  自检门禁，产物 `<stem>_flat.usd`）外，夹爪特有：
  - 24 个夹爪活动关节全部上位置驱动（k=30 N·m/rad、c=0.2、maxForce=2 N·m，
    **初值上机按实测收**），target 0 = 闭合保持；
  - 限位从 URDF 重写（防导入没勾 Ignore Mimic 时 5.1 把左手跟随限位丢成
    ±inf）；跟随关节限位 = 乘数 × 主关节行程；
  - 剥除任何 `PhysxMimicJointAPI` 残留（跨指链导入行为错误，见上）——
    无论导入时是否勾了 Ignore Mimic 都强制干净。
  - 两种导入器布局都兼容：API 导入把 body 嵌在 payload wrapper 里
    （joints 在 `{ROOT}/base_link/joints`）；**GUI 导入是扁平布局**
    （body 与 joints 都直接在 `{ROOT}` 下）。2026-09-15 用户 GUI 导入
    （`urdf/humanoid_upper_body_v3_ctag2f120/`，保存的
    `usd/humanoid_upper_body_v3_ctag2f120.usda` 只是引用它的 /World 壳）
    finalize 通过：59 体 / 50 关节 / 限位 24 重写 / 焊接 1290 万点→204 万 /
    40.4 MB / 质量 120.939 kg / 0 FAIL / 无 mimic 残留（证明导入勾了
    Ignore Mimic）。
  - 已修两个实测 bug：①负乘数跟随关节（`*_1`×−0.15、`left_3`/`left_main`×−1）
    限位 = 母关节行程 × 乘数后**上下限要翻转**，否则 lower>upper，PhysX 把
    一切负向指令钳回 0（关节不动）；②指令侧闭合的跟随目标 = **链式乘数**
    （如 `left_1`→`right_1`→`right_2` 两个乘数连乘 = −0.15）。
- **ROS2 图层（夹爪）**：`scripts/build_ros2_graph.py --ctag` 产出
  `usd/ros2_graph_ctag2f120.usd`（引用指向夹爪 flat，图名/锚点零改动，
  /joint_states 自动 50 关节）。验证仍须 `ROS_DOMAIN_ID=42` 隔离。
- **闭合指令（指令侧 mimic）**：每手 12 关节 = 主驱动
  `*_gripper_right_2_joint` 发 θ，其余 11 个发 乘数×θ（乘数表 =
  URDF `<mimic>` 标签，22 条；主要：×1 直跟、`*_1` = −0.15×θ、
  `left_3`/`left_main` = −1×对侧、`left_1` = 右手 `*_1` 同值）。
  **0 = 闭合（pad 间距 ~2 mm），0.7 rad ≈ 40° = 全开（~75 mm）。**

### 手臂静止下垂与 GUI 再保存警告（2026-09-16 实测）

- **静止下垂是力矩驱动的固有行为，不是 bug**：flat 用 force 驱动（真电机模型，
  无重力补偿），稳态下垂角 = 重力矩/刚度。实测（固定基座、零指令、5 s）：
  240 Hz/32-4 下 arm_2 1.5°、arm_4 1.75°、arm_6 −1.9°（与 τ/k 理论值一致），
  60 Hz/8-1 下约 ×2（3.4°/4.8°）。用户的原始导入是 acceleration 驱动（等效
  刚度与质量无关），所以"纹丝不动"——那才是非物理的。指令跟踪不受影响
  （12/12 零误差回归）。想减小下垂：GUI 里按比例调大臂关节
  `drive:angular:physics:stiffness`（arm_2 现值 19.88 deg 单位 ≈ 1139 N·m/rad）。
  **2026-09-16 下午实测补充**：两份 flat（v3 与 ctag）的臂驱动增益**逐位一致**
  （arm_2 均 19.881/1.3027），行为差异全部来自运行配置与载荷。×5 增益 +
  60 Hz/8-1 下垂 2.3°/3.7°（×1 时 3.4°/4.8°）——增益已非瓶颈，剩余 ~2° 是
  低迭代求解器柔性。**同 60 Hz 只把 solverPositionIterationCount 8→32、
  velocity 1→4：0.8°/1.1°**；再回 240 Hz/32-4：0.4°/0.5°。想要手臂更挺，
  调迭代数比继续加刚度有效。
  另：v3 flat 自带 /GroundPlane z=0.3295 是**正确**的（其轮底 0.3366，留 7 mm
  落差）；ctag 的 GUI 导入布局根比 v3 API 导入低 0.3295，故其地面 −0.002。
  两份文件各按自己几何配地面，勿互相套用数值。
- **⚠️ 不要在 GUI 里改参数后直接保存 flat**：一次 GUI 保存实测把
  `physics:gravityMagnitude` 写成 **−inf**（实测为惰性属性、非失稳元凶，但别依赖）、
  joints scope 重排进 `Physics/joints`、当前姿态烘焙进文件；此后**任何**求解配置
  （含交付的 240 Hz/32-4）都在 2 s 内数值失稳——手臂甩到 ±300°~1500°，限位完全
  压不住。从原始导入重新 finalize 即恢复（损坏文件保留为
  `*_flat_USEREDIT_BAK.usd`）。干净文件上 60/8-1、120/16-2、240/32-4 全部稳定，
  卡顿调参本身没问题；要调参数建议放在引用 flat 的 usda 层里做 override，
  或调完重新 finalize。
- **地面放置**：flat 里 finalize 已把几何上移 0.3295 m（轮底 = 机器人根原点
  z=0）。GUI 加 Ground Plane 时它会落在当前选中位置——实测一次地面被放到
  z=0.3295（机器人根高度），轮子"没入地下"33 cm。正确位置：地面 z = −0.002 m
  （留 2 mm 间隙防初始穿插）。地面与机器人本就是 stage 顶层兄弟节点
  （`/GroundPlane` 与 `/humanoid_upper_body_v3` 平级），无需再分家。
  2026-09-16 已在用户 13:54 保存的 flat 里修好并验证：不钉基座自由站地
  3 s，底盘稳（漂移 ~3 mm/3 s，万向轮自由滚设计使然），轮子与地面接触
  处有数毫米物理压陷属正常求解器接触变形。

## ROS2 图层（2026-09-15）

`usd/ros2_graph.usd`（4.5 KB 独立层，引用 flat USD，不改原文件）含 action graph
`/ROS2_Bridge_humanoid_v3`，纯代码由 `scripts/build_ros2_graph.py` 生成：

```
OnPlaybackTick ─┬─ ROS2PublishJointState   /joint_states  ← targetPrim = articulation 根
                ├─ ROS2PublishTransformTree /tf           ← targetPrims = articulation 根
                ├─ ROS2PublishClock        /clock
                └─ ROS2SubscribeJointState /joint_states ─┬─ execOut/positionCommand/jointNames
                                                          └→ IsaacArticulationController
                                                             (robotPath = articulation 根)
```

- 绑定锚点 = `/humanoid_upper_body_v3/Geometry/base_link`（articulation 根）。
  URDF 重导后 robot 名/结构不变 → 图零改动；夹爪关节进 articulation 后
  /joint_states 自动多节（PublishJointState 未设关芓名单 = 全关节）。
- 图只在 timeline play 时活动；ROS2 env 须在 Isaac 启动的父 shell source。
- 无头回归已过：`scripts/ros2_sub_check.py`（系统 python3.10）收到 26 关节
  （名单与 config/joint_names.yaml 一致）+ /tf + /clock。验证须隔离
  `ROS_DOMAIN_ID`（本机默认域上有另一台 48 关节带夹爪机器人同时在发）。
- 5.1 API 备忘（pip wheel 环境）：`omni.kit.app.get_app().get_extension_manager()`
  （snake_case）；`og.get_graph_registry/get_node_type_registry` 均不存在，
  节点类型存在性靠 `og.Controller.edit` 兜底；OG 节点 rel 属性未赋值时不在
  USD 物化，须 `CreateRelationship(attr).SetTargets([Sdf.Path(...)])`；
  `SimulationApp.close()` 硬退出进程，收尾写文件必须在 close 之前。
  节点属性以 wheel 内 `OgnROS2*Template.usda` 为准：PublishJointState 是
  `inputs:targetPrim`（单数）且无 frameId；Controller 绑定用 `inputs:robotPath`。
