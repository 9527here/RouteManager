# L3 人形回位穿模：碰撞体存在但自碰撞关闭

2026-10-30 用户指出人形归位时机械臂与身体穿模。本次只诊断，未切换物理设置或移动手臂。

## 现象与实测

通过当前 Isaac Sim 6.2 RouteManager `/python` 主线程读取：

- 当前 `/humanoid` 为 50 DOF RM65 人形。
- 遍历实例代理后共 96 个 CollisionAPI 碰撞体；右臂1–6各有1个启用的碰撞体，torso_pitch_link有3个、lift_top_link有2个，均启用。
- `articulation.get_enabled_self_collisions()` 返回 `[0]`；`config.json` 中 `urdf.self_collision=false`。
- 机器人子树无显式 `physics:filteredPairs`，场景无 CollisionGroup。关闭关节树内部自碰撞足以解释为何碰撞体不能阻止手臂穿过自身部件。
- 证据：`outputs/l3_operable/live/collision_audit.json`。仅靠截图不能确定发生接触的具体两条连杆，本次没有重放穿模轨迹。

## 代码原因

`scripts/l3_robot_contact.py::_move_arm` 仅进行6关节smoothstep插值，通过 `send_joint_command` 写物理驱动目标，没有对中途姿态做自身/环境碰撞检查。

`button()` 最后 `_move_arm(saved_q)` 返回动作前姿态；并非全关节清零。`scripts/cabinet_contact.py::plan_button` 只检查IK位姿残差和关节限位，没有碰撞约束。端点可达不等于中间路径无碰撞。

## 修复边界与验收缺口

需审查启用自碰撞及相邻/装配连杆排除规则，检查初始姿态是否已重叠，并对接近和回位轨迹做全路径碰撞验证、分段绕行与运行中停止判据。不要在可能重叠的当前姿态直接全开自碰撞，也不要仅改config就认定当前已加载USD生效。

此前59项中文UI机构检查和真实指垫触发验证仍有效，但**没有覆盖人形全身无碰撞路径**；“按钮按压通过”不能表述成完整轨迹安全通过。本次未完成修复。
