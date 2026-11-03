# 阀门机械停稳、驻车与GUI验收（2026-11-03）

## 现象与原因

原参数roll270固定底盘夹持最后2s持续双侧正载荷，leader角极差5.841µrad、
差分速度峰0.000772rad/s，但SDK速度持续−0.282～−0.275rad/s，旧单SDK速度门无法放行。
不是无接触。Force增益参考、PGS、TGS最小速度迭代8次均未通过旧门，未采用。
不能称SDK接口单位错误；它是当前原生DOF速度，与实际逐帧位移不一致。

## 修法

1. `valve_grip_stability.py`：0.15s连续双侧正法向冲量；逐物理步全部右夹爪q差分、
   两pad平移/旋转及窗口漂移共同证明机械停稳。仍用既有.03rad/s、2mm/1°预算；
   pad线速度预算由已有2mm/.2s派生。缺步、非有限、实际运动、载荷中断拒绝。
   原始SDK速度保留为诊断，不能改成0或强制sleep。
2. `valve_runtime`实时臂座笛卡尔IK闭环，保持.2rad/s命令速率并逐步全身扫掠；
   `_settle`在世界TCP未到位时重解，不再只等旧关节目标。自由底盘旧预接近曾因
   基座平移/转动而TCP偏18.29mm，关节却已到位；另一次移动10.06mm被保护停止。
3. 用户明确“拧阀门的时候当然不能动”。当前仿真验收选择World FixedJoint驻车台架，
   运行器要求真实启用固定链证明+native fixed_base，未驻车在启动前拒绝；
   活动期基座默认1mm/.1°。没有猜实际底盘机械抱闸或位置保持规格。

## 实际结果

- GUI `outputs/gui_valve_20261103/run_contact_hold_v3/`：89.861179°，夹持→转动→松爪→
  退出全通过，最大相对滑移.160165mm/.202254°，10649物理步，44.37sim秒。
  最终独立native body-frame角差89.861179°；无轮轴DriveAPI；全50 kp/kd/maxForce精确原值。
  最终FK对PhysX法兰.000216mm/.000028°，最终环境51.322mm。
- `outputs/valve_fixed_base_bench_20261102/trial_20261103_fixed_closedloop/`：加入全段实际
  臂座闭环后89.798794°，滑移.160812mm/.203718°，原质量/惯量/COM逐项一致，96形状。
  post-physics密采基座位移/转角全程0。冻结源比后续驻车门收紧早，输出仍写10mm/1°；
  原始运行记录不篡改，实际全程0满足后来1mm/.1°，新驻车门另有启动前拒绝回归。
- 当前GUI显示第一次完成后的台架姿态，物理暂停；不是已实现真实轮式底盘驻车接口。
  轮底2mm间隙+世界固定是明确台架边界，源URDF/USD、全部碰撞与手轮被动关节保留。

用户要求GUI再执行一次：`outputs/gui_valve_20261103/demo_20261103_110642/`，从上一轮
结束状态用roll0反向目标−90°，实测−88.819522°（原±3°验收内），夹持/松爪/退出通过，
5974物理步，最大滑移.225834mm/.209220°。底盘仍固定，完成后暂停，无重新加载场景。

用户反馈动作缓慢：该次trace中运行期24.8875sim秒用了220.8616wall秒（不含前置规划），
约慢8.87倍，5974步平均36.97ms，240Hz每步预算仅4.17ms。单turn段约11.8sim秒/
107.1wall秒。当前关节参考限速.2rad/s，S形时间律另含1.5系数；stream每物理步都做
全量capture、实时FK/IK、96形状新目标扫掠，guard另每.04sim秒做实态几何检查，外加
渲染/输出。慢有动作参数和执行循环两部分，优先分析重复反馈与几何计算；尚未做各函数
占比profile，不能把所有开销笼统归为渲染。详见demo目录performance_summary.json。

## GUI Tensor失效与修复

初次直接open_stage销毁旧World，继续使用旧World._scene会退出；加载流程已改为重建World。
10:13:09运行前set_simulation_dt改变渲染步长1/60→1/240时，日志出现“All physics
information was deleted…”；旧mimic每帧native读失败再回退读q，共163598条，最后10:15:22。
10:15显式重建backend、恢复本台架目标/增益后，后续GUI全过程通过，10:25再次读50q/
targets/links全有限。旧日志仍在，不是继续发生。不要把此已确认的新错误原因倒推成
昨晚22:53后原进程未知退出的原因。

`physics_view_guard.py`在控制入口先检查World._scene、SimulationView.is_valid和
ArticulationView.check；失效就暂停、清待执行运动、锁存并只报一次，渲染继续响应。
显式ack只解除锁存且仍暂停，不自动恢复旧动作。没有用getDofPositions反复探测坏后端。
只热替换当前RobotManager.step/_require_robot；移除新增guard后的字节码与旧方法完全匹配，
没有整类reload、部署其它未验底盘扩展。证据`outputs/gui_valve_20261103/guard_hotpatch.json`。

## 回归

40项运行时（含无驻车零下发、1.1mm漂移拦截、实时IK扫掠、原错误不被日志遮蔽），
6项机械窗口（实际运动但SDK0也拒绝、缺步/接触丢失重启），7项view生命周期均通过。
