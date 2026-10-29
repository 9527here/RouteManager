# `/<robot>/sim/...` ROS2 契约（手臂 + 相机）

2026-10-22 实现。目标是**同一套话题既能驱仿真、又能接真机**。
完整话题表见 `docs/ROS2接口_iasscsim_robot.md`。

## 架构

- **手臂**：`scripts/ros2_sim_contract.py`（系统 python3.10 + rclpy）↔ HTTP
  `get_sim_state` / `move_j` / `send_joint_command` / `set_gripper_position`。
  Isaac 是 py3.12，进程内 import 不了 rclpy。
- **相机**：Isaac 内 OmniGraph 原生发布（`ros2_bridge/contract_graph.py`），
  图像不跨进程。`camera_info` 由 `ROS2CameraInfoHelper` 单独发 ——
  **`ROS2CameraHelper` 不会自动派生内参**，6.2 必须再挂一个节点。

## 五个踩过的坑

### 1. 必须启用 `isaacsim.ros2.nodes`

只开 `isaacsim.ros2.bridge` 时日志刷 `Creating unregistered node type
isaacsim.ros2.bridge.ROS2CameraHelper (version 4)`，节点建不出来。
已在 `sim_env.py._enable_plugins` 里补上。

### 2. 运行期停用任何 prim 都会让 articulation 报废

`isaacsim.core.prims.Articulation._on_prim_deletion` 在**任何** prim 删除/停用事件后
**无条件** `del self._physics_view`（基类 `XFormPrim._on_prim_deletion` 虽然按路径
过滤，但派生类不看返回值）。之后所有 `get_joint_positions()` 之类调用都抛
`'Articulation' object has no attribute '_physics_view'`。

实测：为「不加载相机」加的 `/World/cams SetActive(False)` 一执行，机器人立刻不可控。
**修法**：停用相机必须在 `load_scene` 阶段（机器人加载之前）做；
`_disable_cams_prim()` 里加了 `articulation is not None` 的保护。
救急：`articulation._on_physics_ready(None)` 能把视图补回来。

### 3. `link_names` 不存在，是 `body_names`

6.2 的 `Articulation` 暴露 `body_names`，顺序与 `get_link_transforms()` 的行一一对应。
写 `art.link_names` 会 `AttributeError`。

### 4. 中继不能往自己订阅的话题发布（自环）

最初把 `control/joint_states` 同时当输入订阅、又当"指令回显"发布 →
自己的回显被自己当成指令 → `send_joint_command` 无限循环，**单任务队列 100% 占用**，
外部 HTTP 全 409。删掉回显发布才解决。教训：控制话题只订阅不发布。

### 5. 中继会饿死外部 HTTP 调用

RouteManager 是单任务队列。`get_sim_state` 在带 NuRec 背景 + 4 路相机时**一次约 140ms**，
按固定 10Hz 周期轮询就是 100% 占用。改成**按占空比自适应**：
睡 `elapsed * (1/duty - 1)`，默认 `duty=0.4`。实测外部 HTTP 从 0/5 成功 → 8/10。

## 实测频率（本机，电力院 NuRec 场景）

| 场景 | 手臂反馈 | 相机 |
|---|---|---|
| 只有中继 | 5~8 Hz | — |
| 中继 + 4 路相机 | 4~5 Hz | 1~4 Hz |

主循环本身只有 ~15 FPS（NuRec 把 GPU 打满，见
[[backdrop-scene-render-cost]]）。要更高只能减相机 / 降分辨率 / 提渲染频率。

## 相机

- 4 路：`left_hand_camera`←`left_wrist_gemini_335lg`、`right_hand_camera`←`right_wrist_gemini_335lg`、
  `chest_camera`←`chest_gemini_335lg`、`head_camera`←`head_zed_x`
- 内参 `fx = fy = w * focalLength_mm / 20.955`，`cx=w/2, cy=h/2`
  （实测 `head_zed_x` 960×600 → fx 572.04；三路 Gemini 1280×800 → fx 640）
- 图像 QoS 是 **best-effort**；用 reliable 订阅收不到
- 建图前 `/World/cams` 若 inactive（`cameras.auto_register=false`）要先
  `SetActive(True)`（激活不是删除，安全）

## 旧话题

`/humanoid/joint_states`、`/humanoid/joint_command`、`/cmd_vel` 已废弃。
`start_ros_relay.sh` 变成转发到 `start_ros2_contract.sh` 的 shim，
`bringup.sh` 也改用新脚本，并把 `isaacsim5.1_teleop` 改成 `isaacsim6.0`。

相关 [[isaac62-migration]]、[[backdrop-scene-render-cost]]、[[vr-teleop-xrobotoolkit-bridge]]。