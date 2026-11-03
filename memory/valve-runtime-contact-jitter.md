# 阀门验证前的接触抖动与运输状态恢复（2026-11-02）

## 现象与诊断边界

50DOF RM65 整机在原 L3 支腿旁持续抖动。没有高层目标写入、50维速度目标全0，
颈部阻尼和轮子 kp/kd 已是此前修正值，不能直接套用旧“速度目标污染”结论。
必须用真实 PhysX tensor 及逐物理步回调；USD 变换和睡眠后的零值不能证明问题修好。

支腿碰撞临时关/开对照后均精确恢复：关掉并唤醒后可稳定，恢复并唤醒后持续振荡。
真实射线核对了 Leg/新支架的物理边界，不是可视缩放误差。轮子 kd 的 .1/.01
单因素试验不能重复改善，全部 runtime/cached gains 已恢复；没有交付这些参数。
底盘90°舵角另有24个原生形状装配重叠，不可声称全局自身无碰撞。

## 退出失败与授权范围

用户允许≤0.015m/s、≤30cm沿当前轮向受控退离，保留全部碰撞/车身朝向。
fresh 预检失败，`controlled_retreat/result.json` 为 `motion_started:false`。
最新冻结几何下 +Y 靠柜；-Y 中段左前轮对原 L3 Foot 穿深最高4.219mm。
移远本轮新增工位也不能让该路径通过。于是暂停物理，未强行运动。

用户随后另行明确允许“恢复运输姿态并复测”。运输来源是同批
`outputs/valve_operation_20261102/current_inventory.json` 的实测root和50维已发指令目标，
不是把带物理偏差的全身反馈回灌。SHA256:
`3a3c37aeeb80ca1f01b922827b6f5551095b70853334c2fbdd053c71e05cc033`。
只将本轮新增 ValveStation 移到[-2,-2,.84]；原柜/原支腿/操作面板与碰撞保留。
这是直接仿真状态恢复，绝不能记为真实退离或阀门拧动成功。

## 6.2 的暂停写入陷阱

删除/新增工位后，仅initialize旧view并不能保证PhysX拓扑已提交。
暂停下tensor root/q/target立即读回可通过，但首个play物理步仍会从旧USD重建，覆盖回旧位置。
先完成新增工位物理实例化、重新获取有效view，再零scene-edit恢复机器人。
在play已恢复、尚未推进物理时用native tensor写root/q/v/targets，必须验收随后真实物理帧。
`scripts/restore_robot_transport.py` 按此分阶段处理，异常保持暂停。
不得重载机器人、reset全场景或补运行期PhysxArticulationAPI来试运气。

## 当前实测与证据

证据目录 `outputs/valve_operation_20261102/jitter/transport_reset/`。
`retry_after_fixture_bootstrap/` 首物理帧root位置误差23.71µm，fresh实际96形状环境
最小49.2889mm、双臂/手最小473.757mm；驱动kp/kd/maxforce保持原值。
`baseline/` 2402逐物理样本约10s：初始约3.8s振荡，此后全50维速度及link摆幅0。
`repeat_awake/` 原生sleep→wake后1920样本约8s：前约1.68s有瞬态，2s以后0；
base全段XYZ摆幅0.877/0.479/0.293mm，最高关节速度0.632rad/s(head_pitch)。
以上观测没有新增motor指令，仍不能替代小幅实际动作后的稳定验证。

首次头部小动作因非头关节速度门中止：`head_motion/` 在wake后.583s启动，.608s
head_pitch达到-.030191rad/s，超过.03参考门；只实际动了约17µrad，不能称完成往返。
原49位置目标、50速度目标、kp/kd/maxforce全部保留，原head目标恢复并暂停。
这说明.25s的稳定窗会选中1.68s唤醒瞬态内部的安静间隙；后续应加强为2s连续速度
停稳，动作速度阈值仍保持.03，不能为通过而放宽。后续稳定验证待现场结果补充。
阀门接触/转动尚未执行。

## 后续对照尚未证明修复

2s连续停稳与从原head目标连续起步后，`head_motion_continuous_target/` 仍在首个
动作帧被全50维非头速度.03门拦截。不得把这些失败改成通过。
不过该帧head_pitch SDK速度+.04917rad/s，而离散q导数为-.001366rad/s；末.3s
head_pitch角极差5.693µrad、base XYZ极差0.000570/0.000477/0.000060mm，head
位置极差0.001505/0.003040/0.047326mm。瞬时后求解器速度不能单独等同可见摆幅；
后续可单独验证实际位姿包络，同时保留原速度门失败诊断。

两项参考参数试验都未改善strict小动作，并已回滚：

- `jitter/solver_scene_trial/`：仅已有SceneAPI minVelocityIterationCount 0→4，
  一物理帧view/目标/增益保持，head仍失败，精确撤销Session属性意见至原0。
  没有native计数getter，不能声称热改已生效。源码`_ensure_physics_scene`也注明
  场景调参须在PhysX建场景前；后续移至独立最小场景验证，不再拿USD读回作证明。
- `jitter/neck_inertia_trial/`：真实56×56质量矩阵head_pitch锁定其余坐标惯量
  .00763097kg·m²，按2√(kI)仅将该关节kd从157.8179降至9.19083。仍失败，
  runtime与缓存全增益恢复原值。这只是耦合系统的参考近似，不能称根因已确认。

独立对照输入`jitter/isolated_solver/`含同一机器人/地面与真实摩擦材质，以及
native masses/inertias/coms/质量矩阵。不要漏材质而把地面摩擦差异当求解器收益。


## 独立最小场景已排除的两项参数方案

`jitter/isolated_solver/`有效试验保留96机器人+2地面形状，native masses/inertias/COM
与现场导出完全一致。最初遗漏def父节点导致地面未构建的两轮标为invalid，不能纳入结论。
原minVel0与构建前minVel4均6s未通过原strict wake门，未发任何头目标；root XYZ极差
分别2.381/.801/.533mm与3.435/2.407/.649mm。没有原柜/Leg/新增工位也能复现振荡，
所以Leg接触不是唯一因素。不得把USD热改未生效与此建前对照混淆。

只改lift阻尼的模型参考试验也不通过：native Force模式，Mii33.0334、K182356.4，
D90.88185→2sqrt(K*Mii)=4908.70898；lift跨度1.488→1.176mm，但root XYZ反而
3.906/2.544/.557mm、wheel峰1.494rad/s。没有据此修改live。

现场已另外修正地面physics用途绑定，详见`ground-physics-material-binding.md`；
上述独立基线使用原有错误材质，不能自动外推为修正后最终结果。

## 诊断度量须尊重控制自由度

独立pose诊断中，两腕部相机实际maxJointVelocity为0，仍能读到投影速度；保留原0和
原目标并单列raw cap超出，不能伪造正上限。受控head和其他有限正速度上限仍检查。
空载夹爪在0止挡处有约1µrad反馈越限。已有标定表令position2仍映射leader0，首个
非零刻度为8→leader.000593456rad；现场经全96预检后只给双爪该小幅指令，3s全61体
扰动<1mm/.1°、其他26目标/全部速度目标/增益保持。未使用任何q/v/root状态setter。

随后pose诊断的.10456°最大角来自rear_left_wheel_link绕轴滚动，root/其他身体并非
同角摆动。四轮由速度接口控制，不锁滚轮位置；应比较轴向swing、轮心和真实圆柱
包络，不能把旋转对称圆柱的虚构角点对应或自由滚角当成碰撞/机身朝向错误。
原strict verifier及每次失败证据仍保留，不得重新标成通过。

## 11-03 本轮收尾状态

地面physics绑定修正后最后一次`passive_settle_after_diagnostics/`真实推进7.996s、
零actuator目标写入，root最大57.50µm / 0.009501°；末1.996s全部50关节速度、
位置极差和base位置极差均0。原目标/速度目标/kp/kd/maxforce精确保留。
这是有明确范围的待机观测，不改变之前头部小动作诊断失败，也不证明运动期抖动已修复。

原GUI/18765进程后来退出，检查端口无监听，日志未给出原因；未自动重启。
退出前源修复、Session改动的证据和恢复/测量工具已保存，不能写成“现场仍暂停”。
独立固定底盘台架第三轮已实际到位并接触，但因夹持停稳门超时而未转90°，
详见`valve-physical-grasp-geometry.md`。没有在原自由底盘场景完成阀门操作。

## 11-03 用户反馈与异常退出复核

用户确认“整个机身，静止会有，动起来不确定”，不能将本轮现象归为仅颈部抖动。
上述末2s零摆幅只是有限窗口，尚未复现/解释用户看到的整机静止抖动。
用户也确认原GUI不是手动关闭。业务/Kit日志均停在22:53最后成功的只读任务，
没有正常关闭或退出信号记录；22:45至23:15系统kernel无记录、warning日志及用户日志
未提供对应OOM/崩溃证据，/var/crash与systemd coredump没有本次转储。
不能据“无日志”判定是SIGKILL、内存不足、GPU故障或与抖动同一根因。

新增`scripts/run_observed.py`：独立父进程保存子进程PID、退出码/终止信号、标准输出，
启用PYTHONFAULTHANDLER与无缓冲输出，不自动重启，不导入Isaac、不下发动作。
纯Python子进程0/7/SIGTERM与旧证据目录防覆盖检查通过。监控器和子进程各自独立会话，
避免启动终端关闭直接挂断。整台机器或监控器也被强杀时仍不能保证留下最终退出记录。
11-03仅恢复带观测的空场景GUI服务，health已IDLE；没有自动重建旧现场或加载机器人。
启动位置见`outputs/server_observed_latest.json`；空场景就绪不算整机稳定验收。
