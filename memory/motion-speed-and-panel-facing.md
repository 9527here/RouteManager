# 动作慢与操作面对正（2026-11-02）

用户再次反馈动作缓慢，随后明确“胸部正面垂直朝向柜门／面板”。当前仍为50DOF RM65、四轮八驱、Isaac6.2，不重启服务或改自碰撞。

## 朝向根因与实现

asset_approach.target原有右臂站位横偏约0.539m、示教yaw -9.324°，而指向按钮中心bearing约-39.379°。不能用atan2按钮位置代替面法线，也不能随手取消横偏。旧_plan_transport又把目标yaw覆盖成起始yaw，at_target只比yaw_hold，导致偏斜也被判到位。

目标朝向改为操作面朝外X轴的反向。资产请求收臂→验证完整转舵及旋转弧线→独立aligning→实际停稳→以新朝向做X/Y平移→位置/朝向/停稳三项放行。manual仍保持起始朝向与wz=0。已在2°目标门限内不重复转身，纠偏开始后才用0.75°停转细门限。底盘旋转用SteeringPrealignment和0.35rad/s²参考角加速度，保留倾角1°、平移漂移2°、全身12mm与手臂50mm。

原地旋转必须按半径×yaw弧长采2mm轨迹；不能使用旧edge纯XY判据导致原地仅3样本。motion_edge复用同批位姿验证全身和手臂。对正当前点不安全时，先保轮向退出，再对正；自动回退仍限一次，用户停止取消pending，不自行恢复。

现场panel_facing_workflow.json通过：从约-37°对正至面板0°，首次机构动作时胸部误差约-0.782°、at_target与stationary均true；全部平移样本wz=0。首次对正加短移共36.98秒，包括必要纠偏和终点再校正，不能把它宣传成整个流程已很快。

## 慢点与等价优化

初轮manual右移10cm：19.136秒/8.021仿真秒，其中等90°对舵约9.54墙钟秒。同步参考rate原0.6rad/s，smoothstep90°固定3.927仿真秒；改参考1.2rad/s，仍受URDF当前3rad/s限制。两条曼哈顿路线原按恒等长度排序偏X先走，改比较平移时间+转舵时间。profile值在request同步给实际_base_cfg，不能只改JSON而运行仍读旧值。

ArmTransitSafety._nearest缓存不变pairs/角点、批量AABB和懒coal.Transform：离线9.20→0.563ms，90组含手臂/舵角扰动距离、shape与判定逐项相同；Nav nearest2.258→1.438ms。同帧手臂检查后复用q/poses，免第二次refresh，运行check含refresh由12.6→4.7ms。

L3/Oxford每物理步给18件机构写相同目标，240Hz共4320setter/s。改读实时物理目标、仅float32同shape逐项完全相同时跳过写入，保留每帧状态/联锁/螺旋耦合；外部覆盖或重建后重写，getter不支持时沿旧路径写入。369帧等价检查通过，恒定目标240帧setter4320→18。没有改摩擦/刚度/力矩或物理Hz。

已热更新后的90°换轴10cm初轮对照13.001秒/5.867仿真秒；轮向已对齐时7.787秒。后续靠近脚座时因精确扫掠重复，另一轮为15.703秒，不能声称每处固定13秒。最初before完整JSON被after写入误覆，before摘要来自当时已读工具输出；逐帧原始样本未恢复，见navigation_before_summary.json。

## 仿真控制暂停与未现场验收项

收到共享18765的“修复阀门手轮 IK”聊天转达的用户授权：阀门验证独占现场，暂停本聊天全部底盘/手臂/机构测试和热更新，不重启/重置场景，待用户明确恢复。之后只进行离线代码工作。

暂停后继续准备的优化：高层显式锁定已验证的steering_targets传到底盘，避免近零噪声换±90组；_verify_steering只扫实际锁定的一组，transit_route对完全相同起点/轴/起舵角复用证明。此项必须与新版robot_manager一并应用，当前运行中的GUI尚未接入，不能作为现场速度改善数据。仍需恢复后验证近脚座受阻/退出、纯横移、取消和朝向全过程。

配套离线契约：locked_steering26/26、asset_facing28/28、纯旋转20/20、旧唤醒11/11通过。hint改变会重新进入真实停轮门控，非法hint在修改当前任务前拒绝；每腿扫掠、未来边检查、底盘命令使用同一组。当前不再发现场请求，等待用户明确恢复。

报告：outputs/action_speed/、outputs/facing_alignment/；关键geometry_optimization.json、fixture_target_dedup.json、rotation_drive_contract.json、asset_facing_contract.json。人形接触速度仍为参考0.5rad/s、TCP0.02m/s，本轮未擅自提高或重验完整按压。
