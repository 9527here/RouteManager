# Memory Index

- [平台定位与源码整理](repository-layout-20261103.md) — Real→Sim→Real主线、旧资产外置与恢复、tests集中；192项严格回归/公开克隆烟测，保留双远端历史与并行新文件。

- [Isaac Lab 阀门 RL 四条结论](isaaclab-valve-rl-findings.md) — **RL 对采数据不划算**（两版都卡在平台期，只会悬停 0.22m）；actuator 增益必须读 gains.json（lift 标定 182356，写零刚度整机塌陷）；夹爪 12 关节 mimic 链 ±1/-0.15；训练必调 handle_deprecated_rsl_rl_cfg 且导入必须在 launch_simulation 之后；**PhysX 雅可比是连杆质心的不是 TCP 的**，拿它做 IK 会越走越远

- [SimScene当前会话资产工具](simscene-attached-tools.md) — 34个MCP工具、资产搜索与静态层导入/移除/侵入拒绝通过；原PID/96形状/原生参数保持，不启动独立worker。

- [SimPeople附着单人巡检](simpeople-attached-patrol.md) — 原会话单人两点与原地阀门并行通过；完整Start事件与零时间范围坑、取消/卸载验收、重复动作选连续IK分支，冷启动仅语法检查。

- [阀门夹爪分段提速](valve-gripper-two-stage-speed.md) — 预张开实测宽+8mm，空段1000/近接触180、3mm锁存慢速；阀门3闭爪5.14→0.442sim秒/1.34wall秒，预收拢另2.79wall秒；三阀全流程通过、93项回归。

- [阀门2与阀门3操作](valve-variants-2-3.md) — 两者独立台架及GUI完整通过，实转91.60°/89.92°，底盘0；保留祖先缩放、扳杆局部夹持和原预算预紧、耦合轮轴扫掠，87项回归。

- [阀门动作提速](valve-operation-speed.md) — GUI90°完整流程220.9→16.83s（新含预检）；240Hz反馈+60Hz预检插值、静态元数据/精确相对几何缓存、Fabric、右臂速度前馈；默认1.2/转动2rad/s，原生cap3.14，底盘不动、保护保留。

- [阀门机械停稳与驻车验收](valve-grip-hold-and-parking.md) — GUI固定底盘真实89.861°、复测89.799°；双载+实际q/pad连续窗口修单SDK速度误拒，底盘全程0；未驻车拒绝、Tensor失效暂停止刷屏，53项回归。

- [阀门 IK 离线复核](valve-ik-offline.md) — SO(3)/雅可比/180°与dry-run误下发已修；历史原位不可达，摆位候选131点maxTCP0.455mm仅计算；旧valve_turn执行早拒绝、9/9回归；后续授权实测另见阀门夹持/抖动条目。
- [阀门真实夹持几何](valve-physical-grasp-geometry.md) — 2r不等于夹宽、源Hull不等于cooked面；11-03复测补齐逐步q/v/载荷，双载仍原生速度约−.28rad/s；一次Force增益参考也失败未采用，固定台架尚未转90°、未放宽门。

- [L3人形回位穿模](l3-return-self-collision.md) — 当前96碰撞体存在，但自碰撞读回0；回位仅关节插值、IK未做避碰；已定位，尚未修复，全身无碰撞验收缺失

- [L3 参考规格可操作机构](l3-operable-controls.md) — 4按钮/6旋钮/5压板全部在场景可见/可选，TGS240Hz独立75/75；现场15/15夹具往返通过；中文动作插件支持一键按压/换档/投退/停止，绿色按钮人形接触；小惯量旋钮需比例减力，真实力学标定与其余人形流程未完成

- [L3 前端资产可用性](l3-display-assets.md) — 23静态单网格GLB已试转预览；按钮/旋钮外观可用，object_14四连通优先改螺丝压板；8–12/17–18为铭牌；真实尺度与物理缺失

- [操作面板场景演示](operation-panel-demo.md) — 当前GUI已摆三类机构+静态断路器，39秒循环/28阶段通过；原人形50DOF正常；有开始/停止与近景UI，机构驱动不等于人形接触

- [Oxford 操作机构迁移](oxford-device-migration.md) — 三类 USD/状态机迁入，临时驱动往返与急停加载共 9/9；开关直接力矩未通过、人形接触未验；另有 Schneider 断路器分件 CAD 底模

- [线上操作资产候选](online-operation-asset-candidates.md) — Oxford manipulation_playground有急停双关节/隔离与安全开关(BSD3、Gazebo需迁移)；Schneider断路器STP、EMKA锁具CAD；压板未找到可信公开模型，PartNet仅非商业研究教育

- [USD001 新资产接入与动作覆盖](usd001-import-and-action-coverage.md) — 11包已接入 assets/USD001 并预览；全为合并 Mesh、无物理；010优先改造按钮/旋钮/门；用户要求仅按外观候选，尺寸/电气语义未确认

- [人形与电柜真实接触](cabinet-humanoid-contact.md) — Btn05 真实按压/回位两次通过；Knob01 自由转轴夹持正反转，反馈后 ±30° 状态误差 <1.4°；前移 12 cm、升降 -8 cm；真实档位阻力与电气逻辑待数据

- [动力柜资产操作验证](cabinet-asset-operations.md) — dongligui 11 关节中 10 个往返通过，锁柄偏差未解决；无额外铭牌版已配人形 RM65 整机(50 DOF)；中置柜/直流电源柜仅扫描、无关节；不把顶部三灯当按钮

- [Gripper swap task](gripper-swap-task.md) — 把 changingtek 夹爪换进人形上装 URDF；ctag2f120 已合并+修复转腕关节，cthm3f460 版未做；2026-10-13 双臂 RM65 直连版完成（install_standalone_arm.py 一键幂等）
- [Ask user for data, don't guess](ask-user-for-data-dont-guess.md) — 需要规格/实测数据必须列清单向用户要，推定值必须标注，不自己猜
- [RouteManager project](routemanager-project.md) — Isaac 仿真 HTTP 服务器；env=isaacsim5.1；2026-09-23 七项修复（ROS2 闭环/加载等待/相机挂载等），实机链路未测
- [Isaac Teleop docs](isaac-teleop-docs.md) — XR 遥操作框架文档；1.6 起包名 isaaccapture；VLA 数据采集前端备选
- [Realman×Pico 遥操作](realman-pico-teleop.md) — rm_driver CANFD 透传要点、官方 XRoboToolkit 控制律（P 矩阵与仿真一致）、真机复刻代码位置
- [关节正方向实测法](joint-sign-measurement.md) — head_yaw 正值=右转（URDF 推反了）；驱动关节+抓图测质心位移的通用判定法；/python 不回显 result 要用写文件
- [场景摩擦≈0](scene-friction-zero.md) — 抓不住/机器人打滑的根因；static_scene.usda 加材质+地面绑材质；过盈可降到 0.5mm
- [夹爪 mimic 链与指令接口](gripper-mimic-chain.md) — **主动关节是两个：ctag_R2(右指根)+ctag_L2(左指根)，只驱动 R2 在不支持 mimic 的工具里打不开**；主动关节力矩被跟随关节反锁=闭不到底；leader↔开口实测表；TCP 随开口漂移 11mm；抓取配方（过盈 3mm）；set_gripper_position 睿尔曼语义；部分下发治身体塌陷；闭合穿模=acceleration 型驱动刚度被小惯量吃掉，stiffness 要 5000
- [自动抓取（VR 菜单键）](auto-pick-menu-key.md) — 左手菜单键→抓最近物体；非阻塞状态机；grasp_planner 共用；遥测字段
- [仿真臂 vs 官方 RM65](arm-model-vs-official-rm65.md) — 已重建：官方6轴(891mm与官方逐位相同)；腕壳旧夹爪安装板已切(cut_wrist_shell.py，相机在壳顶不受影响)；rm65变体+build脚本+5.x load_urdf修复(两步流程/defaultPrim/热reload失效)；config robot.name 已指向 rm65
- [URDF 颜色不生效](urdf-materials.md) — 导入器把颜色做成 link 级 material_<hex>，但被更近的 mesh 级空壳 DefaultMaterial 盖住；去实例化 visuals 后摘掉它；必须从 GetPrototype 读才判得出来；官方 RM65/CT 的颜色表
- [GUI 里手动驱动关节](gui-joint-control.md) — Physics Inspector 的 Joint Drives Target Position 滑块；写 USD targetPosition 不传 PhysX；Script Editor 用 standalone_server.EXECUTOR；必须跑服务否则夹爪散架；相机与夹爪同在 link_6，转 joint_6 无效
- [GitHub/GitLab 推送链路](github-push-large-files.md) — ssh.github.com:443+SOCKS5；历史大文件 filter-repo 清洗；GH001；双远端双推(GitLab 子组 vla/RouteManager，main 受保护禁强推)；提交人已改 o_range
- [抓取下降段抖动](auto-pick-descent-jitter.md) — 根因是每个笛卡尔路点发一次 `send_joint_command` → smoothstep 两端速度为 0 = N 次启停；改成 Catmull-Rom + 单条 smoothstep 时间律每帧流送；另附"别用墙钟算速度"的采样假象教训
- [RM65 抓不住 vs 旧臂抓得住](rm65-grasp-vs-old-arm.md) — 失败在合爪接触几何：旧臂虎口偏 3.4° 咬棱、方块落进楔形锁死；RM65 虎口对正、指垫平贴零力接触。附完整排除清单 + 两个被证伪的假设 + 位姿矩阵约定坑
- [抓不住/"抓住但抓不起来"根因链](rm65-grasp-failure-root-cause.md) — **结案**：三个缺陷叠加（夹爪连杆碰撞被剥、下降段在手臂没到位时就开跑 16°、夹爪驱动等效刚度只有 14 N·m/rad）；修完 0/N → 8/10，抬升 188mm 不滑
- [底盘推不动](base-wheel-drive-stiffness.md) — 轮关节被烘成 `stiffness 0 / damping 99.7` 纯阻尼，位置目标被忽略；`_tune_wheel_drives` 补成 203/0.08，5 秒 1.1mm → 963mm
- [远物抓取：降机身 + 接近轴铅垂](grasp-verticality-and-body-lowering.md) — 规划选的 body_dz 以前被丢掉(真 bug)；修好后远物自动降 0.18m，接近轴偏离铅垂 0.3°
- [臂的增益不是官方值](arm-gains-vs-official.md) — 质量/惯量来自 URDF 是官方的；官方 URDF 没有 `<dynamics>`，阻尼是我们烘的；附在 Isaac 里核对驱动参数的三种方法
- [颈部/相机转台阻尼](neck-camera-damping.md) — 烘出来的 damping 只有 0.02(几乎自由关节)，手臂一动头就点头 3.5°；`_tune_neck_drives` 按 ζ=1.5 补
- [静止待机抖动](idle-jitter-wake-lock.md) — 根因是 `set_base_velocity` 一旦调用过就永久每帧写轮子目标、把整机锁在唤醒态；PhysX 睡着=完全静止、醒着=0.3~4mrad 宽带微振；修完静止抖动 0.979→0.000。附官方 Gripper Tuning 方法与被排除的做法
- [四轮八驱待机抖动复查](idle-jitter-swerve-20261102.md) — 本次没有持续写目标；唤醒高层setter污染全身速度目标、L3重置丢颈部阻尼已修。真实短移后4.73秒全身零摆幅；运行期补根API会破坏tensor视图，试验已回滚。
- [阀门验证前接触抖动与状态恢复](valve-runtime-contact-jitter.md) — 支腿接触/90度轮体装配问题须独立诊断；fresh受控退离被Foot拦截未下发，单独授权恢复运输态；新prim暂停tensor读回会被首物理步覆盖，须先实例化再验真实帧，不能以sleep零值报成功。
- [地面physics材质绑定覆盖](ground-physics-material-binding.md) — 6.2 GroundPlane.geom默认physics绑定实际.5/.5/.8覆盖项目普通1/1/0绑定；修正physics用途强绑定、保留所有碰撞；须ComputeBoundMaterial验证，不能据此声称所有抖动已解决。
- [人形 URDF 对外交付包](humanoid-urdf-delivery.md) — assets/deliver/ 的包结构约定；源用 rm65_isaac 那份；visual 精简/collision 原样；open3d 精简后必须先算面法线否则写盘静默失败
- [VR 遥操作接入 XRoboToolkit](vr-teleop-xrobotoolkit-bridge.md) — 进程外 UDP 通路（placo IK + xrobotoolkit_sdk），与进程内 CloudXR 那套互斥；必须 setsid 起服务（SIGHUP 会带走）；PC Service 只推一个客户端；errno=28 是 inotify 不是磁盘
- [ROS2 sim 契约](ros2-sim-contract.md) — `/<robot>/sim/...` 手臂+相机话题；**运行期停用任何 prim 会让 articulation 物理视图失效**；`body_names` 不是 `link_names`；控制话题不能自发布（自环占满队列）；中继要按占空比让出 HTTP 队列
- [ROS2 契约改进程内](ros2-inprocess-contract.md) — Isaac 6.2 自带 py312 rclpy（`isaacsim.ros2.core/humble/rclpy`，消息包也在那），**不需要 py3.10 中继**；sys.path 插路径前要先清 sys.modules 里的坏 rclpy；回调跑在主线程可直接写 articulation；实测 40Hz（旧中继 5~8Hz）；**`LD_LIBRARY_PATH` 必须让 ext 的 lib 排在 `/opt/ros/humble/lib` 前面，否则 `create_node()` SIGABRT**
- [Isaac GUI 中文字形](kit-ui-text-gone-cjk.md) — `font.customFontPath` 是**整体替换**不是补缺字，字体必须**同时含拉丁**且是 **TrueType 轮廓**；DroidSansFallback 无拉丁 → 全界面空白（零报错）；Noto CJK 是 CFF 用不了；可用方案 NotoSansSC 实例化 wght=500 + region 每行 192 字 + `overresolutionSize=66`
- [迁移到 Isaac Sim 6.2](isaac62-migration.md) — 5.1 被 10-20 的升级覆盖成 py312 torch prebundle 而报废；URDF 导入器两套 API；**mimic 必须剥 NewtonMimicAPI 否则夹爪把仿真带崩**；GroundPlane.set_world_pose 不生效；USD 限位存的是度
- [背景场景渲染开销](backdrop-scene-render-cost.md) — NuRec 扫描吃满 GPU：可见 15~17fps / 隐藏 42~45fps；分辨率/采样/反射/render_interval **全无效**；crop 运行时改不了（插件换成代理 prim）；唯一对策 `set_scene_visible`；加载后头一分钟只有半速
- [ROS humble 污染 py3.12 运行时](ros-env-python-pollution.md) — pink 扩展报 pinocchio 找不到子模块的真因；PYTHONPATH+LD_LIBRARY_PATH 两层，**/opt/ros/humble/lib 不能动**
- [STEP→USD HOOPS 导入验收](step-hoops-import.md) — headless 调 hoops_core 转 STEP；原型实例化导致 Traverse 数不到 Mesh；mm 双层 wrapper + 180° 翻转落地；pxr 先开 stage 会搞崩 World；Gf Matrix3d.SetRow 按列存坑；6 开口曲面=相机/电机外购件
- [L3中文插件与Isaac Lab批量采集](l3-isaaclab-batch.md) — 33任务、4环境66/66，RGB与合作停止；世界固定锚点与Lab3配置扫描坑，机构数据不是人形示教。
- [电柜实际名称、前置走位与四轮八驱](mobile-asset-approach-swerve.md) — 原生4转向+4滚轮、参考±90、到位停稳门控、90度分支与停车反扭矩、参考提速。
- [纯侧移与柜体净空纠正](pure-lateral-and-cabinet-clearance.md) — 禁止临时转车身代横移、停轮90对舵后斜坡、柜壳补碰撞/96覆盖、运输收臂与实时净空。
- [轮脚包络拦截与安全退出](wheel-foot-envelope-recovery.md) — 整轮高投影误挡3cm脚，当前无碰但回正真会碰；用户要求默认自动回退：停稳/真实几何复核/一次退出重试/停止不重启，手动退离明确未完成。
- [动作提速与面板对正](motion-speed-and-panel-facing.md) — 用户选择胸部垂直朝操作面；独立对正后固定朝向平移，首次机构误差0.78°通过；90°换轴10cm初轮19→13秒，近脚座仍慢。现场因阀门聊天独占暂停，显式锁定已验舵角的进一步优化仅离线，须配套恢复后验收。
