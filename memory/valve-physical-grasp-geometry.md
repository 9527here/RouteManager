# 阀门夹宽与真实cooked碰撞面（2026-11-03）

后续已修复机械停稳判据并在GUI固定底盘完成89.861°，复测89.799°；见
`valve-grip-hold-and-parking.md`。以下为过程中的真实失败与几何分析，不再代表最终未成功。

## 实际执行暴露的问题

两次独立固定底盘台架（同一50DOF/61刚体/96碰撞形状RM65人形）均按原运行器
走完张爪、接近、下降，随后被真实夹持反馈拒绝，未转轮。
固定底盘与轮底2mm参考吊装间隙仅是隔离台架边界，不是原自由底盘通过证明。

1. 初版用`2*radius=79.9637mm`当平行夹宽，实际方向源网格宽约77.173mm。
   最大0.5mm收紧后仍留2mm以上空气；这不是IK成功就等于接触成功。
2. 改源网格支撑投影并沿jaw对中后，source几何预测两垫穿入.301/.133mm，
   PhysX却只有左侧patch、正separation约13～41µm、impulse=0，右侧无报告。

## 真实碰撞几何核验

官方`request_convex_collision_representation`返回wheel34顶点、两pad各15顶点；
source数学凸包wheel462点。按官方mesh-local→body米制→native pose转换，6条原生
ray命中点到cooked表面最大残差32nm，证明是实际物理面差异，不是单位猜测。

同一姿态下：source夹宽77.182669mm，cooked75.471717mm；source两垫测距
−.300575/−.132929mm，cooked实际净空+.076215/+1.276419mm。
cooked支撑mid相对内面mid偏−.637449mm，source仅−.083679mm。
原生restOffset均0，wheel contactOffset .643886mm、pad .340625mm；
右侧1.276mm空气超过offset之和.984511mm，符合右侧无报告。
合法contact patch可带正间隙且零冲量，不能称作承载夹持。

## 已写的代码修复

- feedback保留源CollisionAPI mesh支持点，planner按目标URDF pad轴、runtime按实测jaw轴投影。
- 以真实inner-face中点对中，不把body原点TCP自动当物理内面中点；保持原0.5mm总收紧。
- 必要对中IK显式要求0.1mm数学精度，不能返回未达标fallback并继续。
- 接触分为patch与承载：只有两侧有限正法向冲量才建立夹持；合法空报告允许接近，
  不可读/非有限拒绝；非pad实际正载荷拒绝。dwell/失联0.15秒及其它数值门未放宽。
- CLI热更新先feedback再plan，兼顾`scripts.*`与脚本导入别名，不整类reload执行器。
  历史缺几何快照仅离线显式`--legacy-grasp-reference`，实际执行仍必须新采样。

## 不采用的几何尝试

255顶点上限连inactive完整预写再原子CopySpec均未改变原34点，不能归因于64上限。
默认convexDecomposition产生16件，虽出现双侧正载荷，却外扩且初始右垫穿入1.369mm、
backing净空.632mm，违反原1mm预算，不能采用。未扫其它数值，生产setup仍原Hull。

## 规划自由度的可行候选（未等同实测成功）

原碰撞模型不改，roll90/270时Source/Cooked在夹持轴的支持端点吻合。
仅按已有12方向选择抓取姿态，六方向通过整90°几何与IK校核；最优90/270：
0.5mm总预紧时双垫约−.250mm，backing最小1.75048mm，普通环境5mm组14.56309mm、
普通self8mm组9.42245mm。保持双边接触横向预算约.24957mm，不能用数mm到位误差冒称抓紧。
IK的机器精度计算值不是仿真实测精度，独立FK对native误差≤.923µm/.000018°。
应先张爪再完成接近与转腕规划，不能在旧接触点直接转腕。

第三轮`trial_roll270_loaded/`已实际执行并按原门控停止：全预检、开爪、预接近、下降通过，
`contact_close`超时；未进入contact_replan/turn/release/exit，stop_errors为空。
最终双pad正法向冲量0.001258426 / 0.016806842 N·s，leader原生速度−0.277566642rad/s
高于原0.03rad/s门，不能完成双侧正载荷且停稳0.15s驻留。最终正载荷只是一帧，
没有密采全程impulse，不反推持续双承载历史。被动轮全程最大+0.040625°、末+0.040484°，
并非90°转动。夹持超时按仿真时钟，整轮8053物理步，不是wall超时造成。

独立FK初/末误差0.000211 / 0.000166mm、姿态小于0.000028°；计划133点最大TCP残差
0.500643mm / 0.047593°，末实际TCP到夹持参考0.141065mm。Robot M/I/COM逐项0差。
8293个只读主线程post-physics样本含初始化，回调全部卸载，App/PID已关闭。
末2s pad中心距极差1.2µm、TCP各轴不足1µm与SDK leader v存在差异，但未保存逐步leader q，
不足以证明velocity假读数；不以末帧载荷或几何静止放宽门槛。

随后仅补私有逐物理步夹持诊断：raw leader q/v、native target、已发command、双侧
normal impulse、both/stopped/dwell及finally退出原因；不改任何控制门或物理参数。
该补充尚未重新实跑，不追补第三轮缺失的历史；原冻结源码与失败证据保持原样。

## 证据

`outputs/valve_fixed_base_bench_20261102/`：

- `trial/`：原半径夹宽错误的第一次实际失败。
- `trial_corrected_support/`：第二次实际失败，4914/4914主线程物理步密采。
- `diagnostic_cooked_20261103_native_rays/`：真实cooked/原生offset/material/ray/报告核验。
- `inactive_geometry_comparison_20261103.json`：255与默认分解未采用的证据。
- `offline_roll_grasp_math_20261103/`：纯数学/煤距离/90°扫掠候选，未宣称真实抓转。
- `trial_roll270_loaded/measured_result_summary.json`：第三轮固定底盘实际失败、独立FK/IK/原生轮轴密采结果。

原18765 GUI进程在最后22:53业务记录后退出，之后检查端口无监听；未查得明确退出原因，
没有自动重启。不能再写“现场仍暂停”冒充服务存在；源修复与数据均已保存。

## 11-03 上午：按用户优先验阀门的实际复测

`trial_20261103_step_feedback/`沿用原物理、roll270、原全部门控重新实跑，
补齐2026个夹持样本。最后1.996s双垫正载荷比例100%，leader q极差5.841µrad，
位置差分速度峰0.000772rad/s，但SDK原生速度始终−0.28177～−0.27454rad/s，
停稳比例0%。原8.35s左右夹持超时正确停止；未进入contact_replan/turn。
这次终于有逐步q/v/载荷证据，但不据此声称原生速度无效或绕过原速度门。

只在独立台架做一次Force驱动参考对照`trial_20261103_force_reference/`：
原生确认右夹爪12关节Force模式；cap仍各2Nm，K由cap/既有.02rad误差预算得到100Nm/rad，
D按2sqrt(锁定Mii*K)取0.0489～0.2562Nms/rad。不是厂家参数，不改源URDF/USD/生产默认。
其它关节增益、质量/惯量/COM和全部碰撞/收紧/停稳/驻留门保持原值。
该参考未改善：末2s原生速度−0.36927～−0.36324rad/s，双垫正载荷比例0%，未采用。
因此“仅把高位置增益降下来就能正常夹转”不受本次试验支持。

两轮均通过96形状预检和开爪/接近/下降，各8053操作物理步，均contact_close失败、
stop_errors为空、actual_angle_deg=null。手轮仅被动微动+0.04046° / +0.03920°。
两独立App均已退出，原空场景GUI服务保留；没有自由底盘整机拧阀门成功证明。
汇总：`outputs/valve_fixed_base_bench_20261102/comparison_20261103_step_feedback.json`。
