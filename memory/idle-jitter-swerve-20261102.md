# 四轮八驱整机待机抖动（2026-11-02）

## 现象与现场测量

当前50DOF RM65整机在操作结束后仍抖。必须读PhysX tensor，不能用USD变换判断静止。约7.10秒实测：base XYZ极差4.55/8.21/0.56mm，head_pitch_link俯仰极差0.555°。四轮瞬时速度峰0.31～0.67rad/s。

同期包装人形set_joint_position_targets、set_joint_velocity_targets、set_joint_velocities、apply_action计数全部为0；base_cmd/motion/auto_pick/pending均空，UDP和VR均未运行。因此不能套用旧的“每帧写轮目标导致不睡眠”结论，也没有去改UDP或mimic。

## 查到并修复的代码问题

1. 底盘首次启动用高层set_joint_velocities(当前实测速度)唤醒。本机Isaac6.2 deprecated Articulation源码1028～1029同时写实际速度和驱动速度目标。于是全身微振被锁成目标：head_pitch约+1.6976rad/s、torso约+0.1549、lift约-0.0296。改为低层_physics_view.set_dof_velocities(同一实测速度, [0])，只唤醒而保留全部速度目标。当前待机的残留速度目标已一次性清零。
2. L3安装、切solver、扩展资产的World.reset会把颈部阻尼恢复为导入缓存初值，头俯仰/偏航runtime kd仅1.230/2.490。三处恢复关节状态后重新调用项目已有_tune_neck_drives，保持已做过对照的参考阻尼。头部摆幅试验由约0.555°降至0.175°。该公式原适用于acceleration驱动，当前资产是force型，因此这里只称仿真参考阻尼，不把参数zeta称作实物阻尼比；刚度、力矩上限、摩擦不改。

不要夸大单因素结论：仅清速度目标时底盘仍漂、头部仍抖；补颈部阻尼后也没有单独解决全部底盘漂移。最终还恢复了当前物理后端状态、清除了残留振荡。各阶段报告均保留，不能把最后零抖动全部归于其中某一项。

## 撤销的危险试验

运行中补PhysxArticulationAPI及速度迭代8使全局tensor视图失效。已移除该API/两项属性，未交付任何迭代或自碰撞改动。仅重建view无效，最后用最近有效tensor快照恢复同一USD场景、重新初始化物理后端与全部设备view，再恢复目标、增益与参考限位。没有重新导入机器人。不要在当前场景重复这一做法。

get_sleep_thresholds读出NaN来自缺少USD schema属性，不能据此说runtime阈值非法。旧Physx simulation接口get_attached_stage读0也不能单独证明6.2 core后端无效。恢复必须用实际动作与tensor反馈验收，不是仅看有限值/无报错。

## 验证

证据目录outputs/idle_jitter/：

- baseline.json/npz：最初抖动与0次目标写入。
- physics_attributes.json、velocity_target_correction.json：残留速度目标及清零；after_target_clear仍抖。
- neck_damping_correction.json、after_neck_damping.json：头部参考阻尼对照；底盘仍有漂移。
- wake_contract.json：真实set_base_velocity方法体11/11离线契约，首动一次state写、全部50维速度目标保持、连续指令和零速不重复唤醒。
- motion_response.json：头俯仰从约0.00376走到0.03376rad再返回；唤醒保留速度目标。证明没有靠冻结模拟制造零抖动。
- base_move_then_idle.json/npz：真实Body Y移动-45.08mm、X偏0.24mm，正常停轮后约4.73秒全部50关节摆幅/速度为0，base XYZ摆幅为0。退出路径3D净空最小约60.23mm，转舵扫掠通过。

本次验证覆盖一次头部往返与一次真实底盘短移后的待机，不代表长时间任意接触任务都已无抖动。保留真实物理、原自碰撞暂停策略、TGS与240Hz；没有每帧强制休眠或锁定机身。
