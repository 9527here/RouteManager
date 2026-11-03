# L3中文插件与Isaac Lab批量采集

2026-11-02用户明确暂缓旧裸机的自碰撞工作，等带外壳新URDF；先完成动作插件和Isaac Lab批采。插件中文、机构直接操作与人形真实接触必须区分。

## 现象与根因

1. 旧人形按压入口强制依赖未完成的collision_guard，暂缓后会把已验证动作锁死。改为executor._defer_self_collision=True时不依赖守卫，保留停止、有限值和跟踪误差检查；不声称做过避碰验证。
2. Lab3的launch_simulation扫描器对普通dict调用vars()后直接返回，不会扫描里面的CameraCfg。错误启动非渲染Kit导致SyntheticData的SDGPipeline Invalid graph。使用SimpleNamespace(sim=sim_cfg,scene=cfg)，相机才自动启用。额外update/更换stage不能解决传入dict这个根因。
3. 原L3的Joints/Mount body0指向非刚体/Asset，克隆后世界固定锚点未按环境偏移。仅看关节反馈会漏报：早期33回合有30个数值上达标，但根位置偏差2–4m，整批不可用。采集场景将每env Mount的body0清空，localPos0/localRot0写成对应Base的世界位姿，replicate_physics=False；不改源资产。必须在sim.reset后实测各固定根相对env_origins的位置，并强制<2mm。
4. 按下任务在撤力一步后才写终态，会快速回弹导致误判。按下终态保留按压力，释放另列；每个env结束当步额外保存终止帧，不能只等整个batch结束。

## 当前实现

- scripts/l3_task_catalog.py：33个独立任务，按钮8、旋钮15、压板10；有效不同初态、中文指令、源SHA256、单位、执行方式。
- scripts/collect_l3_isaaclab.sh/.py：Isaac Lab3 InteractiveScene向量化Articulation，TGS240Hz，保存30Hz稀疏子步转移及终止帧，可选每env 640×480 RGB与内外参。reset完整写根位姿/速度和q/qd，再forward；运行中非有限反馈立即停止。
- scripts/l3_action_plugin.py：L3、旧动力柜、Oxford中文控制；采集独立子进程，progress.json原子更新，STOP文件合作停止。后台不访问Isaac对象。
- scripts/validate_l3_dataset.py：独立读盘检查布局、schema、有限值、任务覆盖、初态、实际反馈、保持、RGB和时间对齐。

## 验证数据

- outputs/l3_dataset/all_actions_04/：4env×33任务×2重复=66/66成功，独立验收pass。最大固定根位置误差1.67e-7m。
- outputs/l3_dataset/rgb_buttons_07/：2env、8/8按钮任务，224张640×480RGB、均值201.6–202.3；内外参、时间戳验收pass，原图已目视确认15件资产可见。
- 插件实际开始/停止：outputs/l3_dataset/ui_collection_started.json指向采集目录。停止后14回合已保存：12完成、2中断，cancelled=true；--allow-failed-episodes验收数据完整性pass，行为总通过为false符合预期。
- L3中文按钮现场59/59通过，见outputs/l3_operable/live/chinese_plugin_checks.json。
- 动力柜/Oxford新增入口24项现场检查，22项首次通过，锁按钮浮点限位和急停末段回零等待修复后回归通过；汇总legacy_current_validation.json。
- 早期试验移入outputs/l3_dataset/failed_trials，不得用于训练。

## 边界

批采execution_mode=fixture_effort、robot_contact_demonstration=false，尚无人形批量接触执行器。人形桌面现阶段仅绿色按钮接触流程，不能把机构施力当成人形示教。参考参数未实测。原动作文件实际11类22项，不是标题旧写的21项。旧柜和Oxford已接UI，但本轮IsaacLab批采只包括15件L3。

详见docs/IsaacLab电力柜批量采集.md、docs/电力柜动作与采集范围.md。新URDF到后先匹配关节名/TCP和真实碰撞外壳，再接人形批采；不要将旧guard或旧50DOF轨迹直接套到新机器人。

## 人形入口复测与初始化

本次新启动只恢复关节、未恢复底盘位姿时，IK能求解但-20度接近仅擦到按钮边缘，未触发；正确返回未达标。恢复原验收底盘世界位姿(.1228,.1394,.3294m、yaw约-9.3度)后，-10度接近真实触发 3.068mm、0.01920Ns，释放/回撤通过。不要把纯IK可达当接触必然成功。assets/l3_operable/humanoid_demo_pose.json保存该旧整机演示初态；scripts/setup_cabinet_actions.py在空白服务初始化整套工位，在已有工位只重开UI。
