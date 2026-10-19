# 在 Isaac Sim GUI 里手动驱动关节

**2026-10-12 实测。**

## 三种方式

**A. Physics Inspector（最接近 RViz 的 joint_state_publisher 滑块）**
菜单 `Tools → Physics Inspector`（也挂在 `Window → Physics → Physics Inspector`，
由 `omni.physx.supportui.show_physics_inspector` 提供）。
在窗口的树里选中 articulation，然后工具栏：

- **Slider Control**：`Automatic` / `Joint States Position` /
  **`Joint Drives Target Position`** / `Joint Drives Target Velocities`
  —— 拖关节前先切到 **Joint Drives Target Position**（对应 DriveAPI），
  默认 Automatic 会按情况挑。
- **Limits / Gains**：`Show Joint Limits` / `Show Joint Stiffness / Damping`

**B. Property 面板**
Stage 里选 `<articulation>/joints/<关节名>` → Property → Drive → Angular →
Target Position。

**C. Script Editor**（最稳，已接好钩子）
```python
import standalone_server as ss
ss.EXECUTOR.set_gripper_position(side="right", position=500)
```
`main.py` 启动时把 RobotManager 挂在 `standalone_server.EXECUTOR` 上 —— Script
Editor 有自己的命名空间，看不到 `main()` 的局部变量，只能走模块属性。
⚠️ Script Editor 跑在仿真主线程，**不要**在里面发 HTTP 打自己的端口，会死锁。

## 坑：直接写 USD 属性**不会**传到 PhysX

```python
st.GetPrimAtPath(".../joints/ctag_R2").GetAttribute(
    "drive:angular:physics:targetPosition").Set(0.45)   # ❌ 关节纹丝不动
```
实测物理目标仍是原值、关节不动。Isaac 不会因为 USD 属性变了就把新的 drive
target 推给 PhysX。所以脚本路线要用 `EXECUTOR` / Articulation API，别写 USD。
（Property 面板走 `omni.kit.property.physx` 的专用通道，理论上会通知 PhysX，
但没在 GUI 里验证过。）

## 前提：必须跑 RouteManager

**不能**只用 Isaac Sim 单独打开转换后的 USD 就去拖关节 —— CT 夹爪的 11 个跟随
关节是靠 `step()` 里的软件 mimic 每帧写的，只动 `ctag_R2` 会让连杆脱节、夹爪散架
（就是用户之前遇到的"散架"）。带窗口起服务：

```bash
cd /home/jy/RouteManager
ROUTE_MANAGER_PORT=18765 ISAAC_PYTHON="micromamba run -n isaacsim5.1_teleop python" \
    DISPLAY=:1 ./scripts/run.sh          # 不要 HEADLESS=1
# 另一个终端（先停 relay，否则它高频轮询会抢单任务队列）
./scripts/start_ros_relay.sh stop
ROUTE_MANAGER_PORT=18765 ./scripts/load_scene.sh -n rm65_ctag2f120
```

## 腕上相机与夹爪都挂在 link_6 上

`camera_rojoint: link_6 → camera_rolink`、`link_6_to_ctag_joint: link_6 → ctag_base_link`。
所以**转 `joint_6` 是没用的** —— 相机和夹爪一起转，相对视角不变。
想让腕上相机看到两指，得转「夹爪相对 link_6」：
`make_rm65_ctag.py` 的 `MOUNT_RPY`（绕 Z 自转，法兰本来就是 90° 一档）。
2026-10-12 用户要求转 90°，已设为 `"0 0 1.5708"`。

相关 [[gripper-mimic-chain]]、[[urdf-materials]]。
