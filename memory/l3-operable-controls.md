# L3 参考规格可操作机构

2026-10-30，用户明确授权“先按参考规格做可操作版，参数注明待实测”。

## 现象与改造

L3原23 GLB均为归一化静态单网格，直接加刚体会整件运动。按几何/纹理拆成基座和活动件，构建 `assets/l3_operable/`：4按钮、6旋钮、5螺丝压板，35活动DOF/50刚体/49碰撞体。原GLB不动，副本和SHA随catalog保存；15件搬目录验证无缺失依赖。型号/真实尺寸和功能语义未知，全部 `calibrated_to_user_hardware=false`。

按钮22mm参考安装规格、4.3mm总行程、2.6mm触发、名义3.8N来自XB4BA31参考数据表；预载/阻尼/活动帽质量为假设。旋钮档位借用90°/±45°形式；阻力待标定。压板80mm/M6×1/松2圈/90°均为工程参考。14原4连通件，其余为近似重建，导电桥补建；等效螺旋耦合不是螺纹牙接触。

## 验证与坑

- TGS/240Hz独立75/75循环通过，外部广义力/扭矩测试，无机构关节传送。最大旋钮档位误差0.317°；压板锁止载荷下摆角<0.0058°，等效轴移误差<0.000171mm。`outputs/l3_operable/validation_tgs_final.json`。
- 常量外力检查：TGS名义3.8N只得到约2.237mm，速度读回与静止q不一致；PGS约2.600mm，但改现有全人形场景为PGS后底盘漂移。因此交付保留TGS，力学标定明确未通过，不能宣传3.8N整机按压力已校准。
- 60Hz小惯量螺母会跨锁紧终点；当前会话使用240Hz。保持 `config.json` 与原机器人参数不变。
- tensor相等上下限偶发放行，压板锁止改为[0,0.0001]rad窄正范围。运行时必须写tensor DOF limits，USD层限位不足；World.reset后清 `_limits_<name>`缓存再写。
- source热加载后旧的绑定callback仍执行旧代码：除了重赋 `__class__`，要remove/add physics callback。UI完成字段 `reached` 要显式bool，numpy.bool_不可JSON序列化。
- 使用World.reset时保存/恢复机器人q/根姿态/默认态和所有旧Oxford机构q，再 `_on_physics_ready(None)` 重建视图；不能重新load_robot。保存原场景根层+flatten+robot_before。
- 实际人形接触只验绿色object_0_variant，其他机构仍是夹具验证。移动底盘受接触反力漂移，重复按压需每次用最新base/真实指垫重规划；不能复用第一圈固定世界路点。最终TGS/240Hz三次均通过，触发行程3.304/3.041/2.969mm，每次有独立非零冲量。原三次证据已备份为 `live/robot_button_before_plugin.json`。

## 当前操作入口

GUI `/World/L3Station`：现已显示全部15件，左侧原绿色按钮/3旋钮/14压板，右侧12件按按钮/旋钮/压板分行。此前只摆三件示例，用户明确指出后补齐；“资产包15”与“场景可见15”必须分别核对。UI已替换为中文“电力柜动作控制”，具体用法见 `docs/电力柜中文动作插件.md`。按钮保持/释放/单次按压、旋钮指定档位、压板投出/投入直接施加力/扭矩；单列人形右手按钮执行绿色0(1)真实接触。

客户端 `python3 scripts/l3_station.py install|all|status|press|selector|plate_out|plate_in|stop|overview|closeup|robot_button`。物理runtime `l3_controls_runtime.py` 必须每步执行（弹簧、档位、灯、等效螺纹联锁）。直接开USD不等于完整可操作逻辑。机器人仍为全人形RM65双6轴/CT夹爪50DOF；未新增相机或铭牌。旧Oxford面板在当前会话沿Y错开−1.65m，避免大手柄进入L3区域；全量安装函数用带后缀的幂等transform，不改旧源资产。

说明 `docs/L3_电气机构物理改造与验收.md`，参数/来源 `assets/l3_operable/reference_profile.json` / `REFERENCES.md`。下一步实测参数与人形旋钮/压板接触闭环，暂不能认定1.6–1.8m高度/25mm净空/真实锁紧扭矩等实物验收通过。

全量场景组合 `scripts/l3_all_assets.py` / `assets/l3_operable/station_all.usda`，12新件与3旧件路径统一在Controls下。扩展时停World、切reference、恢复50DOF人形与Oxford/旧L3的q，并重新attach15件runtime和callback。不要只加入视觉引用却漏控制器。UI按源编号选择资产，CLI `--asset object_5` 可选动作对象。全量记录 `live/all_assets_install.json`、`all_15_live_checks.json`、`all_15_overview.png`、`scene_all_assets.usda`。

全量验收：15/15可见且物理视图有效、每种UI动作启用匹配，15/15逐件夹具流程通过（按钮回位、旋钮所有档位、压板投出/投入），人形50DOF/J有限。证据 `all_15_inventory.json` / `all_15_live_checks.json`。

## 中文动作插件（2026-10-30）

用户要求“按插件按钮就能执行”，并要求以后插件全部中文。`scripts/l3_action_plugin.py` 是 `L3Station` 的中文界面与编排 mixin；执行层继续用主线程物理回调，显示实测状态/到位判定/超时中止，互锁同一时刻一个动作。人形 UI 通过后台线程只提交 localhost HTTP，由 `/python` 在仿真主线程执行；线程不能碰 USD 或 articulation。停止人形时立刻将右臂目标设为实测关节位置，并在每个接触步检查取消标志。

发现原恒定正反0.1Nm施力使小惯量三档旋钮5从-45回0时出现约±0.316rad极限环，4s超时。改成 `clip(0.12*角度误差, ±0.1Nm)`，只调整夹具控制律，未改资产本体卡位/摩擦参数。旧数据 `live/selector_5_diagnostic.json` / `chinese_plugin_checks_before_selector_fix.json`。

中文渲染沿用当前Kit已经配置的全局字体（参见 `kit-ui-text-gone-cjk.md`），不要在插件frame上另指定DroidSansFallbackFull.ttf，否则本次UI文本全空白。Unicode减号字形缺失，角度写ASCII负号“-45°”。通过 Workspace WindowHandle.dock_in 将中文面板放在视口右侧，隐藏旧英文演示窗口；不新建相机。

最终中文UI验收：`live/chinese_plugin_checks.json` 59/59通过、覆盖15件；`plugin_robot_result.json` 从人形按钮入口1/1真实接触通过，行程3.147mm/冲量0.02057Ns；`plugin_cancel_result.json` 中途停止后右臂漂移0.000338rad。可重跑 `python3 scripts/validate_l3_action_plugin.py`。

长时间全场景运行后基座横向漂移约13cm、偏航约8°，固定正对姿态的法兰目标超出臂展（IK残差13–17mm）；不能再靠缩短35mm预接近距离解决。人形L3规划改为保持35mm间距、在0/±10/±20°方向中选可达完整路径，不移动机器人基座或工位。当前-10°规划通过，实际指垫触发通过；仍不可达时明确提示调整站位。`cabinet_contact.plan_button` 新增默认0的 `approach_yaw_deg`，原动力柜默认行为保持；未放宽1mm/1°IK阈值。底盘长期漂移是现有整机限制，未把一次成功宣传为任意站位稳定性通过。
