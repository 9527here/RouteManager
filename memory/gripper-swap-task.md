---
name: gripper-swap-task
description: 把 changingtek 夹爪模型替换进人形上装 URDF 的任务（目标目录 /home/jy/下载/人形上装部分9.9/）
metadata: 
  node_type: memory
  type: project
  originSessionId: 097eee3c-01ca-40c0-8b90-c5932e31cc80
  modified: 2026-09-21T07:39:50.397Z
---

任务：把 changingtek 仓库里的夹爪模型替换进人形上装部分模型。

- 目标模型：`/home/jy/下载/人形上装部分9.9/`（urdf/人形上装部分9.9.urdf + meshes/，ROS 包名就是中文名"人形上装部分9.9"）
- 夹爪来源：`/home/jy/datasets/vla_data/数字资产库/changingtek/crt_ctag2f120_gripper_visualization/` 和 `crt_cthm3f460_gripper_visualization/`
- 结构对应：Z 系列链 = 左臂（Z1-Z7），Y 系列链 = 右臂（Y1-Y7）；ZQY/ZHY/YQY/YHY 等 8 个 link 挂在 base_link 下（x=±0.25, y=±0.2425, z=0.055），是底盘四角的结构件，与手无关，需保留

已完成：
1. ctag2f120 合并文件 `urdf/人形上装部分9.9_ctag2f120.urdf`（42 个 mesh 引用全部为 package://人形上装部分9.9，自洽）
2. **2026-09-15 修复转腕关节**：上个会话合并时把 ZJ-JOINT/YJ-JOINT（continuous，轴≈±y，origin xyz=(-0.0126, ±0.2198, ±0.0034)）误改成了 fixed 挂载，丢失腕部旋转自由度。修复 = 恢复原 continuous 关节、手掌 ZJ/YJ 退化为无视觉/惯量的纯框架 link、夹爪经 ZJ_gripper_mount（fixed，xyz=0，rpy=∓1.5708）挂在框架下。零位变换与修复前等价。关节在 URDF 中位于 Z7-JOINT/Y7-JOINT 之后，JSP GUI 滑块顺序与原模型一致。
3. RViz2 一键验证脚本（免 colcon，用户级 ament prefix trick）：`display_ctag2f120.sh` + `launch/display_ctag2f120.launch.py` + `config/人形上装部分9.9_ctag2f120.rviz`，prefix 在 `~/rxzj_prefix`
4. **2026-09-15 修复腕部相机缺失**：用户反馈"相机那一部分还是缺失了"。排查发现原手掌 mesh ZJ.STL/YJ.STL（201k 三角形，单一连通体）内嵌腕部相机筒（ZJ 坐标系 y≈0.01, z 0.02~0.043）+ 法兰 + 壳体 + 下挂板，合并时删手掌导致全部消失。修复 = 按质心 y 切割（ZJ 取 y<0.048，YJ 取 y>-0.048，即去掉原夹爪指/轭）→ `meshes/ZJ_wrist.STL`/`YJ_wrist.STL`（各 86884 三角形）；ZJ/YJ link 恢复 visual+collision(新STL)+原惯性；ZJ_gripper_mount origin 从 (0,0,0) 外移到 (0,±0.05,0)（原腕壳前端面=原夹爪安装位），CT 夹爪不再与腕壳干涉，同时消除黑环缝隙。原版对比验证法：`launch/display_original.launch.py` 可单独显示原始 URDF 截图对比。

关键坑：
- ament 的 Python API（get_package_share_directory）拒绝非 ASCII 包名 → launch 文件从 __file__ 推导路径；C++ 侧（rviz2 mesh 解析）不受影响
- 同机多会话易残留 robot_state_publisher/jsp 进程 → /robot_description 串台（旧 URDF 的 mesh 包名错误日志）。清理用 pkill 模式 `[r]viz2` 方括号技巧防自杀；两个 RViz 会话不能并存（transient local /robot_description 串台）。2026-09-17 根治：v3 的 display.launch.py 加 RegisterEventHandler(OnProcessExit(rviz)→Shutdown)，关 RViz 窗口即整组退出，不再留孤儿（注意：pkill 复合命令若自身命令行含匹配串会自杀，见 09-16 教训）
- meshes/ 共 213MB（base_link.STL 122MB），RViz 首次加载 ~3 分钟内高 CPU、"未响应"属正常
- 原始 URDF 是多行标签+CRLF，grep '<link name=' 匹配不到，要用 python xml 解析做结构 diff

2026-09-16 v3 分发包校验+修复：
- 用户拿到第三方重打包 `~/下载/humanoid_upper_body_v3_ctag2f120_min/humanoid_upper_body_v3/`（ASCII 包名 humanoid_upper_body_v3，英文关节名，effort/velocity 已填真实值，visual/collision 分离目录）
- 修复 1：**右臂 6 关节（right_arm_2..6/right_wrist）origin 的 z 分量被错抄成左臂 Z 链的值**（x,y 对、z 全错，右腕整体偏 9.27mm）→ 已按 9.9 原始导出恢复全精度值
- 修复 2：腕部 12 块 collision（ZJ_0..5/YJ_0..5）切自旧手掌，8 块含已删旧爪指（y 到 ±0.226 幻影碰撞）→ 从 ZJ_wrist/YJ_wrist 按 y 向 6 等分段凸包重切（scipy ConvexHull，每块 ~500-1000 三角）
- 备份：`urdf/*.urdf.bak_20260916` + `meshes/collision_backup_20260916/`
- 验证方法（可复用）：零位 FK 全 link 位姿对比 59/59 一致 + check_urdf + 142 mesh 引用解析 + mimic/惯量/visual origin 逐项对比。左臂/头/躯干/底盘 v3 原本就与 CAD 一致；惯量、visual 全对，只有上述两处错
- v3 关节名映射：ZJ→left_wrist_link、Z2..Z7→left_arm_1..6_link、Z1→left_arm_base_link、SX_Link→lift_top_link、XQ→torso_pitch_link、TZ→head_yaw_link、TF→head_pitch_link、ZQY→caster_front_left_link、ZQL→front_left_wheel_link（Y 侧同理）；夹爪 ZJ_R2→left_gripper_right_2_link 等
- vendor 底盘角关节本来就是 revolute/continuous（脚轮+轮），v3 只是加了 ±5° 限位

2026-09-16 v3 两个用户反馈问题修复：
- 修复 3：**左右腕相机不对称**（vendor 的右掌 YJ.STL 相对 ZJ.STL 是**绕 x 轴 180°**：(x,y,z)→(x,−y,−z)，KDTree 全顶点验证 0.00mm；此前记"绕 y 转 180°"有误。z 分量翻转 ⇒ vendor 原右掌相机朝上、左掌朝下，这就是原始不对称）→ 用 y 镜像从 ZJ_wrist.STL 重建 YJ_wrist.STL（先全顶点 y 取反、再换三角形绕向 v0↔v1、重算法线），v3 与 9.9 两个包都已替换，备份在 meshes/visual_backup_20260916/ 和 meshes/backup_20260916/；12 块腕部 collision 凸包同步重切。STL 写盘必须含 50 字节/三角形（48 数据+2 属性字节），且必须**重新从磁盘读回验证**。⚠ 此修复的"两侧都朝下"方向后被修复 6 推翻：用户确认正确姿态是零位相机**朝上**
- 修复 4：**torso_pitch_joint 后仰限位**（用户：低 lift 后仰撞支撑杆）。厂商原值 continuous 废数据，无参照。网格碰撞扫掠结论：立柱区细杆带（x≈0.03-0.09，z 0.15-1.0，含 y≈0.02 近中心杆）在低 lift 时嵌在头部行程内，lift=-0.6 时头部后上角距杆仅 10mm，后仰 >1.5° 即接触；前倾全程安全（+90° 仍 78mm）。URDF 静态限位无法表达 lift-后仰联动，按全 lift 最坏情况取 lower=-0.0175（-1°，留 2mm 裕量），upper 保持 +90°
- **扫掠方法论**：判据必须排除设计嵌套——(a) lift=0 时 XQ 躯干罩壳与 base_link 立柱顶盖有 828 个重合点（同一表面在两个 mesh 各导出一份，0mm 不是碰撞）；(b) lift=-0.4 头壳底边与立柱顶盖 0.24mm 设计间隙；(c) SX_Link 升降机构与躯干座 ~0-1mm 嵌套。排除法：转轴 12cm 球内不计 + 障碍只取 base_link 固定部分。细杆区 stride-4 顶点采样有毫米级伪影，要用全顶点+三角形质心
- 修复后已重启 RViz 会话展示（旧会话加载的是修复前 mesh，必须杀掉重开）

2026-09-16 修复 5：**arm_2/arm_4 限位穿模**（用户：厂家限位不可信，手臂从胸腔穿模；确认 wrist ±3.054 正确）：
- 厂家对称限位 arm_2 ±130° / arm_4 ±178° 范围内即已穿透（±165° 时前臂近万采样点嵌入选障）。可复检脚本：`scripts/check_joint_limits.py`（包内，纯 numpy/scipy）——对任意关节全范围扫掠，动侧 distal 全部 visual 网格采样、障碍侧其余 link 建 KDTree，判据 d<1.5mm 且 d<d0−1mm（d0=零位距离，自动排除铰孔设计嵌套），输出两侧首触角（二分到 0.01°）、障碍件名、接触点、推荐限位与曲线图
- 扫掠结论（v3 包，全 lift [−0.6,+0.05] 最坏 + 2° 裕量）：left_arm_2 [−121.1°,+78.0°]、right_arm_2 [−120.6°,+78.3°]、left_arm_4 [−135.0°,+108.2°]、right_arm_4 [−105.9°,+136.9°]。左右互为镜像但差 2-3°（CAD 左右件微不对称），按实测各自取值
- 关键规律：上摆碰的是贯穿立柱长杆/长侧面 ⇒ 接触角对 lift 不敏感（lift +0.05 与 0 完全同角，接触点沿面滑 0.05）；下摆在低 lift 扫到底盘/脚轮层（z≈0.03）⇒ 下摆限位由 lift=−0.6 决定。URDF 静态限位只能取最坏值，高 lift 时真实边界更宽（lift=0: arm_2 +114°、arm_4 +134°），控制器应按 lift 动态放宽
- 手臂整体装在 lift_top 上 ⇒ 臂-胸腔碰撞与 lift 无关；臂-立柱/底盘碰撞才有 lift 耦合。脚本支持 --lift/--torso-pitch 参数（躯干前倾对手臂限位的耦合未扫）

2026-09-16 修复 6：**腕部相机朝向翻转**（用户确认"相机确认，你做吧"；正确=零位朝上，等价于其验证过的转腕 ±3.054 状态）：
- 结构真相：相机模组嵌在腕壳内部穹顶下，镜头盘（51 三角，z≈+0.0327，法线 −z）透过腕底板大圆孔朝下看 ⇒ "朝地面"。局部翻转模组不可行（镜头翻上去后被壳体挡住不可见）；正确做法=整掌翻转
- 左掌 ZJ_wrist.STL = mirror_z(原版)（z 取反+换绕向 v0↔v1+重算法线）；右掌 YJ_wrist.STL = 恢复 vendor 原版（R180x(ZJ) 天然朝上）。两掌互为精确镜像（KDTree 0.0000mm），翻转后镜头开口在顶面、镜头盘 z≈−0.0327 法线 +z（nz≈0.97）
- 翻转后掌板移到轴承上方：干涉检查 vs 近端臂杆 dmin 0.30mm / vs 夹爪 0.91mm（均比原状态更远，无新穿透）。左腕惯量 ixz/iyz 变号（mirror_z 变换律），右腕惯量本就是 vendor 按 R180x 几何算的、恢复原版后自洽（v3+9.9 都已改）
- 12 块 collision 凸包按同配方重切（y 向 6 等分，436-928 三角/块）。备份：v3 meshes/visual_backup_20260916_2/ + collision_backup_20260916_2/，9.9 meshes/backup_20260916_2/。脚本：scripts/camera_flip_fix.py / camera_flip_verify.py / camera_flip_sync99.py（v3+9.9 都已应用，check_urdf 均过）
- **关键教训：二进制 STL 读取必须 reshape(n,4,3) 后 [:,1:,:] 跳过法线行**——漏掉会把单位法线当顶点（bbox 恰好 ±1.0000），曾当场写坏 ZJ_wrist.STL 一次，靠 bbox sanity 断言（|坐标|<0.5）发现并从备份恢复。读任何 STL 先看 bbox 再继续；凸包块数 sanity：正常 500-1000 三角/块

2026-09-17 7 路外置相机扩展（按 ~/文档/相机安装需求）：
- 变体文件 urdf/humanoid_upper_body_v3_ctag2f120_cameras.urdf 由 scripts/add_cameras.py 生成（基础 URDF 不动），生成器含视轴断言+贴面间隙断言。display_cameras.sh/launch/display_cameras.launch.py/config/*_cameras.rviz（TF 开启）。README_cameras.md 有清单
- CAM-01 ZED X 头顶前部支架8mm（前额竖装下俯10°时顶角扫进壳体 9.8mm 被断言拦下→弃用竖装）；CAM-02/03/04 One S 头左/右/后 yaw ±90/180 pitch −10；CAM-05 335L 胸前 XQ 面 x=0.124 z≈0.24 pitch −5；CAM-06/07 305 腕顶靠背侧抬高12mm 视轴沿夹爪指向（左+y_w/右−y_w）pitch −15
- 相机角度换算：需求文档 pitch 负=下俯，URDF rpy 绕+y 正角度=视轴向下 ⇒ urdf_pitch=−doc_pitch；yaw 同号。每 link 挂 *_optical_link rpy(−π/2,0,−π/2)
- 官方 mesh：zedx.stl/zedxones.stl（stereolabs/zed-ros2-description）、gemini335L.stl（orbbec/OrbbecSDK_ROS2 orbbec_description），已拷入 meshes/visual/camera/。ZED mesh 约定=镜头朝 +x、基线沿 y、底部 z=0 平面；实测 zedx 163.6×47.4×31.8mm、zedxones 52.3×25×25mm、gemini335L 27×124×29mm（官网 124×29×36 含接头）
- 待核实：ZED X/One S 质量（0.22/0.07 占位）。Gemini 305：用户提供官方 SW 导出 gemini_305.urdf.xacro（~/下载/），腕部两台已对齐官方 camera_link 约定=底面安装板上方 21mm、RGB 基线 18mm、外形占位盒 50×30×30（box_center=(0,0,−0.006)）；其分件 mesh（orbbec_description/meshes/gemini305/ 5 件）GitHub 各分支 404、官网当夜不通，待用户提供下载包。用户同时给的 gemini_335_L_336_L.urdf.xacro 与已实现的 ROS2 仓库约定一致（camera_link 挂 base_link.STL），无需改动
- 限位复检：check_joint_limits.py 关节名要带 _joint 后缀；后台扫 left/right_arm_2/4 对比基准（左2 [−121.1,+78.0] 右2 [−120.6,+78.3] 左4 [−135.0,+108.2] 右4 [−105.9,+136.9]）。2026-09-17 发现旧基准是修复6腕翻转**之前**跑的，为归因相机影响，补了当前基础URDF对照组（/tmp/v3fix/cams_base）+ 相机组（cams）同时扫 lift=-0.6
- **限位复检结论（2026-09-17 完成）**：对照组负侧首触与修复6前旧基准一致（腕翻转不影响 arm_2/4 首触）→ 归因干净。**相机版 arm_2 负侧穿透：左 −118.09°/右 −118.39°（裸腕 −123.09/−122.55），障碍=base_link 衣领 z≈0.845——是腕部305相机（抬高12mm）先触躯干，头部/胸部相机无影响**；arm_4 两组完全一致。已把收紧写进 add_cameras.py 的 LIMIT_TIGHTEN（左 lower→−2.0261、右→−2.0314，只收负侧留2°裕量，基础URDF不动）并重生成通过 check_urdf。**2026-09-20 用官方 305 整机 mesh 复测：首触角逐度一致，收紧值不变（ cams2 目录）**
- **Gemini 305 官方 mesh（2026-09-18 解决）**：orbbec/OrbbecSDK_ROS2 已发布 meshes/gemini305/（之前查时 404）——5 分件已收进 meshes/visual/camera/gemini305_parts/，scripts/merge_gemini305_mesh.py 按 gemini_305.urdf.xacro 位姿（visual rpy=(π,0,π/2)，camera_link 在 root 上方 (0.0081,0.009,0.021)，RGB 基线18mm）合并成 gemini305.stl（整机 23.3×42×42mm，camera_link 帧、安装板 z=−0.021）。add_cameras.py 腕部条目改 mesh+keep_frame=True（保留官方帧不归零，bbox/碰撞盒按真实包络）。7 台相机 mesh 全齐
- **Isaac Sim 脚本（2026-09-20）**：scripts/isaac_add_cameras.py——按 URDF 外参在 Isaac 场景建 7 个 USD Camera prim（Merge Fixed Joints 勾不勾都兼容：link prim 在就用/被合就在父link下按外参重建 Xform），camera 局部 rpy=(90,0,−90)（USD −Z 视向↔本体+x），frameId={名}_optical_link，FOV/分辨率/量程在 CAMERA_PARAMS（**hfov 待规格书核实**），ROS2 发布 /{名}/image_color(+depth)；--check 离线预览。**Script Editor 两个坑（已修）**：(1) sys.argv 是 Isaac 启动参数，argparse 必炸→手动解析；(2) 脚本被拷到 /tmp/carb.*/ 执行，__file__ 推算的 PKG 失真→候选路径回退（硬编码本机路径兜底）。用户环境 Isaac Sim 5.1（micromamba env isaacsim5.1）。配套 scripts/make_isaac_urdf.py 把 package:// 改绝对路径生成 *_isaac.urdf（Isaac Importer 不解析 package://）
- 教训：旋转盒 AABB 必须用 Σ|R_ij|·half_j 逐轴算，center±R@half 只是一个对角会低估（腕部 z 向差 4 倍）

2026-09-17 预留安装位对齐（用户指出"相机跟预留的位置有出入"）：
- 壳体网格边界环检测（scripts/find_mount_holes.py）：**头壳 TF.STL 前脸两个 28.8×20.1mm 圆角眼窗**，窗心距 120.0mm=ZED X 官方基线，窗后~33mm 有同基线 Ø6.8 孔×2（相机安装板）；胸壳 XQ 封闭无开孔；腕壳只有 M2 级螺孔
- XQ 沉台检测（scripts/find_pocket.py）：**胸前预留安装台**，端面中心 (0.123,−0.005,0.231)，法线 pitch −8.5° yaw 0（文档 CAM-05 可调区间 0~−30 内），台高 28.6mm≈335L 的 29mm。原 chest 安装面 x=0.124/z=0.24 是 bbox 极值处的局部凸点，已改贴台面（背面距台 4mm，rpy=(0,0.147580,0)）
- **CAM-01 ZED X 改装头壳内眼窗位**（scripts/fit_zedx_bay.py）：视轴=窗法线 (doc yaw −4.0° pitch +0.6°，与文档 0/−10 冲突——预留窗竖直，俯仰不可调，以预留位为准待用户确认)；前面板沉入窗面 3mm（镜筒Ø23>窗高20.1，沉入不碰窗缘，FOV 校验 ±78°>±60° 通过）。⚠ **ZED X 装入需壳体让位：顶部超局部壳顶 3.1mm、右侧超局部壳壁 3.5mm**（量产业打 2 个沉台，机械决策）。窗中 z=0.132+半高15.9=0.1479 与壳 bbox 顶精确吻合但那是远处极值，局部壳顶实为 0.1449
- 环视 One S×3、腕部 305×2 **无预留位**（头壳除眼窗外无中等开孔），维持支架安装
- 2026-09-17 傍晚用户截图"头顶相机不太对/三个白盒是安装位吗"→ VTK 离屏渲染（/tmp/v3fix/vtk_render.py，matplotlib 3D 无深度排序不可用）复核真相：**TF.STL 不是头盔壳，是自带"横梁式相机舱"的机械支架**——顶部横梁两端漏斗形镜头口（即那两个"眼窗"）中心距 120mm，ZED X 嵌在横梁内、镜头从口露出，看起来"坐"在头上是因为深色本体高出横梁顶 3.1mm/右壁 3.5mm（让位问题）+黑白对比强烈。**位置本身正确**。用户俯视图三个白色法兰盒=TF.STL 自带的**电机模组外形**（左/右/后各一，带输出法兰轴），不是相机安装座；头架正面中央还有 TF 自带圆柱镜头模组外形（Ø~50mm 黑镜盘，原生内置传感器预留，不在 7 路方案内）
- 网格教训2：**此包 STL 的 normal 行全是零（垃圾）**，按法线筛面必得空集，要用 cross(v1−v0,v2−v0) 现算；VTK 渲染 URDF 的完整管线在 /tmp/v3fix/vtk_render.py（FK 累乘+材质）
- 内外参答疑：URDF 只提供标称外参（*_link + *_optical_link），真机装配后手眼标定修正放 TF 发布节点；内参归驱动（ZED SDK / Orbbec 驱动自动发 camera_info），不进 URDF

待办/未完成：
- cthm3f460 版的人形上装合并文件尚未制作（同样注意保留其腕部相机部分）
- 切割位置 ZJ y<0.048 / 挂载 y=±0.05 是按壳体前端面定的，若用户希望保留原夹爪轭架或调整间隙可再调
- 相机版待办：Gemini 305 尺寸/质量核实后重跑 add_cameras.py；限位扫掠结果待确认无新干涉；ZED 驱动接入时 base_frame 用对应 *_link
