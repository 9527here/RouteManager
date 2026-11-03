# 当前场景加入操作面板（2026-10-30）

用户要求“放到场景里面我看看操作”。已在当前 18765 GUI 的人形＋动力柜旁加
/World/OperationPanel，上排隔离开关和急停，下排安全开关和静态 Schneider 断路器 CAD。
无额外铭牌、无机器人相机。UI Operation panel demo 有 Start loop/Stop/Overview/Panel close-up/E-stop detail。

## 可重复方法
先保存 stage 引用层、扁平 usdc 和机器人实际 q/base。对现有机器人设置 reset 默认状态，
stop → 只添加面板引用（不删/停原 prim）→ world.reset → 原机器人 _on_physics_ready(None)
→ 恢复 q/base/velocity/targets，再建新机构 view。不重复 load_robot。
此法仅在当前 50 DOF RM65 整机验证；不是普遍允许运行期任意 prim 编辑。

## 验证
50 DOF/link/Jacobian 有限，camera_count=0；手臂最大变化 .00269rad、底座约 .090mm。
两轮各14阶段，28/28实测目标/状态通过。39秒墙钟循环，临时位置驱动仅驱动机构，
人形不参与接触，Stop 恢复源normal增益。急停仍是按下后旋转解锁，不是先转外圈后按。

## 热修注意
importlib.reload 曾从Kit旧loader缓存返回无record方法旧类；要用读文件exec(compile)
覆盖模块字典、更新实例__class__，回调需停止再重新注册。已修复。

脚本 scripts/operation_panel_demo.py、build_operation_panel.py；面板 assets/operation_panel.usda。
报告 docs/操作面板场景演示.md；备份/28阶段记录/截图/MP4在 outputs/operation_panel/。
当前启动循环，用户可UI Stop或 python3 scripts/operation_panel_demo.py stop。
