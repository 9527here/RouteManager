# 轮脚粗包络误报与安全退出（2026-11-02）

用户一直看到‘当前身体/手臂包络已接近障碍’。实际手臂min.242m安全，base(.318,-.780),yaw-10.6°过近台脚。当前全body真实min19.34mm、FL/Foot23.47mm；粗盒/整高circle却重叠5.91～107.79mm。Cylinder用8角Cube且随spindle转会膨胀，低3cm脚不能套整轮高XY半径。

重要：不是删保护就行。当前四舵[+90,+90,-90,-90]安全，原地q0扫掠最深穿Foot8.96mm；Body +Y退出会撞Leg。当前组Body -Y退出25cm全路径精确通过；退出5cm仍碰、10cm不足12mm，15cm以上回正通过。

修：navigation_narrowphase全机器人-环境3D FK/coal，显式排除self与ground支持，非有限失败；粗包络仅筛候选。每腿按将用舵角检平移，另检全部转舵扫掠，12mm/手5cm不放宽。当前safe但不能转舵时保轮向.025m/s退20～25cm，再接原目标；无安全路径仍拒。收臂前park也必须扫掠，不能先原地q0撞脚。

UI：_step_approach仅pending时同步失败使manual保旧preparing；request同步失败又被调用方覆‘底盘先驶向’。改终态无论pending都同步且每次只处理一次；同步失败raise且不排机构动作。

验收：实际Body -Y19.57cm、X4.60mm，wz全0、无pose瞬移、真实间隙通过；原object4开关切45实测pass，at_target/stationary true。原始危险起点采用只读虚拟FK重放，自动恢复规划pass、实际机器人未动，不故意推回脚座复测。数据outputs/envelope_recovery/；详细docs/轮脚包络拦截与安全退出.md。

用户随后明确要求“下次遇到这种问题自动回退”。默认扩展到已收臂的运行中受阻：仅NavigationBlocked→recovering_stop→真实停稳后刷新几何→全身12mm和手臂50mm退出路径→资产原待办继续，单次预算1。手动横移退出后rolled_back结束，不绕X补偿；用户stop/cancel始终清待办且不得自动重启。NaN、姿态/高度、真实净空或准备收臂异常不自动移动。配置automatic_recovery_max_attempts=1。宽泛RuntimeError不能归类成受阻，规划层也只catch NavigationBlocked。新增automatic_recovery_contract.json与automatic_recovery_live.json验证。

当前退出距离按10/15/20/25cm逐级找最短可行路径。17/17离线契约通过；原危险位只读规划最短15cm，脚23.47mm、手228.24mm。现场注入受阻完成约8.84cm退出并开始重试，但重试累计2°偏航触发保护，端到端pass:false，不能声称原动作成功；20cm版本更早也因偏航停止，均保留报告。真实恢复中点击停止通过，未执行机构，刹停约5mm。最终停稳、全身净空53.56mm。未放宽姿态保护；偏航继续是当前裸机物理的局限。
