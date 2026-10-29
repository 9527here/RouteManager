# Isaac Sim 仿真机器人 ROS2 接口（手臂 + 相机）

机器人名 `iasscsim_robot`，role 固定 `sim`，所有话题挂在：

```
/iasscsim_robot/sim/...
```

同一套话题在真机上应由真机驱动发布/订阅相同名字与类型 —— 运行时不需要区分仿真与真机。

> 本轮只覆盖**手臂**与**相机**。底盘 `cmd_vel/odom`、升降 `lift`、躯干 `waist`、
> 头部 `head`、TF、`/clock` 见文末「后续」。

---

## 1. 手臂

`{side}` ∈ `left` / `right`。

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/iasscsim_robot/sim/{side}_arm/feedback/joint_states` | `sensor_msgs/JointState` | 出 | 6 个臂关节实际状态 |
| `/iasscsim_robot/sim/{side}_arm/control/joint_states` | `sensor_msgs/JointState` | 入 | 流式关节目标（latest-wins） |
| `/iasscsim_robot/sim/{side}_arm/feedback/gripper_joint_state` | `sensor_msgs/JointState` | 出 | 夹爪主动关节角度 |
| `/iasscsim_robot/sim/{side}_arm/feedback/gripper_status` | `std_msgs/Float32MultiArray` | 出 | 夹爪状态（4 个字段，见下） |
| `/iasscsim_robot/sim/{side}_arm/control/gripper` | `std_msgs/Float32` | 入 | 1–1000（1000 = 全开） |
| `/iasscsim_robot/sim/{side}_arm/control/move_j` | `trajectory_msgs/JointTrajectory` | 入 | 点到点关节目标 |
| `/iasscsim_robot/sim/{side}_arm/feedback/flange_pose` | `geometry_msgs/PoseStamped` | 出 | 法兰位姿 |
| `/iasscsim_robot/sim/{side}_arm/feedback/tcp_pose` | `geometry_msgs/PoseStamped` | 出 | TCP 位姿 |

### 1.1 `feedback/joint_states`

```yaml
header:
  stamp: {仿真时间}
  frame_id: ''
name:  [left_arm_1_joint … left_arm_6_joint]
position: [6 个弧度值]
velocity: [6 个 rad/s]
```

- `name` 固定为 `{side}_arm_1_joint` … `{side}_arm_6_joint`（与 profile 的 joint_names 一致）
- 位置/速度都是**弧度**
- 时间戳用**仿真时间**，不是墙钟

### 1.2 `control/joint_states`（流式）

```yaml
name:     [left_arm_1_joint … left_arm_6_joint]   # 可省略，省略时按默认顺序
position: [6 个弧度目标]
```

- **latest-wins**：只执行最新一条，积压消息丢弃
- 只驱动这一条臂；**躯干/升降/头不会被动到**
- 越限的值会被仿真侧 clamp

### 1.3 `control/move_j`（点到点）

```yaml
joint_names: [left_arm_1_joint … left_arm_6_joint]
points:
  - positions: [6 个弧度目标]
    time_from_start: {sec: 1, nanosec: 0}
```

- 只取 `points` 的**最后一点**（多点是预留）
- `time_from_start` 就是平滑插值时长；为 0 时用默认 1.0 s
- 与 `control/joint_states` 的区别：`move_j` 是带时长的离散动作，流式接口没有时间参数

### 1.4 夹爪

**反馈 `gripper_joint_state`**

```yaml
name:     [left_gripper_right_2_joint]
position: [主动关节角度，弧度，0 ~ 0.7]
```

**反馈 `gripper_status`** — `std_msgs/Float32MultiArray`，`data` 固定 4 个元素：

| 下标 | 含义 | 范围 |
|---|---|---|
| 0 | `position`（与 `control/gripper` 同一刻度） | 1 – 1000 |
| 1 | `opening_mm` 指垫开口 | 0 – 86.2 mm |
| 2 | `fraction` 开口比例 | 0 – 1 |
| 3 | `angle_rad` 主动关节角 | 0 – 0.7 rad |

**控制 `control/gripper`** — `std_msgs/Float32`，1–1000，`1000` = 全开、`1` = 全闭。

### 1.5 末端位姿

`flange_pose` / `tcp_pose`，`geometry_msgs/PoseStamped`：

- `header.frame_id` = **`base_link`**（不是世界系；`world.base_z` 之类的偏移不会带进来）
- `flange` = `{side}_gripper_base_link`（夹爪法兰）
- `tcp` = 左右指垫中点；**姿态沿用 flange**（TCP 不是 URDF 里的 link，没有独立姿态）
- 四元数顺序是 ROS 约定 `x, y, z, w`

---

## 2. 相机

| 角色 | 对应机体位置 | 分辨率 | fx = fy | cx, cy |
|---|---|---|---|---|
| `left_hand_camera` | 左腕 | 1280 × 800 | 640.0 | 640, 400 |
| `right_hand_camera` | 右腕 | 1280 × 800 | 640.0 | 640, 400 |
| `chest_camera` | 胸部 | 1280 × 800 | 640.0 | 640, 400 |
| `head_camera` | 头部 | 960 × 600 | 572.04 | 480, 300 |

每路两个话题：

| 话题 | 类型 |
|---|---|
| `/iasscsim_robot/sim/<role>/color/image_raw` | `sensor_msgs/Image` |
| `/iasscsim_robot/sim/<role>/color/camera_info` | `sensor_msgs/CameraInfo` |

- `encoding = rgb8`
- `header.frame_id = <role>_optical_link`（ROS 光学系：z 前、x 右、y 下）
- `camera_info`：`distortion_model = plumb_bob`、`d = [0,0,0,0,0]`、`K/P` 由相机内参换算
- 图像 QoS 是 **best-effort**（传感器数据标准做法）；用 reliable 订阅会收不到。
  `camera_info` 是 reliable。

内参换算（供真机侧核对）：

```
fx = fy = width * focalLength_mm / 20.955
cx = width / 2,  cy = height / 2
```

---

## 3. 怎么起来

前提：RouteManager 已 `load_scene` + `load_robot`。

### 3.1 进程内（推荐）

Isaac 6.2 自带 **py3.12 版 rclpy**（`isaacsim.ros2.core` 扩展），所以节点直接跑在
仿真进程里，不走 HTTP：

> ⚠️ **启动前提**：`LD_LIBRARY_PATH` 里 `isaacsim.ros2.core/<distro>/lib` 必须排在
> 系统 `/opt/ros/<distro>/lib` **前面**。两套 ROS C 库不是同一次构建，顺序反了
> `rclpy.create_node()` 会在声明 `use_sim_time` 时 SIGABRT
> （`rcl_interfaces__msg__parameter_event__convert_from_py:
> Assertion 'PyUnicode_IS_READY(name_attr)' failed`）。`scripts/run.sh` 已自动前置。

```bash
curl -s -X POST localhost:18765/execute -H 'Content-Type: application/json' \
  -d '{"action":"setup_ros2_contract","parameters":{"enabled":true,"rate_hz":50}}'

# 查状态 / 关闭
curl -s -X POST localhost:18765/execute -H 'Content-Type: application/json' \
  -d '{"action":"ros2_contract_status"}'
curl -s -X POST localhost:18765/execute -H 'Content-Type: application/json' \
  -d '{"action":"setup_ros2_contract","parameters":{"enabled":false}}'
```

- 反馈按 `rate_hz` 发（默认 50），控制回调**每帧都处理**
- 没有 HTTP 往返，不受 `409 Conflict` 影响
- 所有 ROS 回调跑在仿真主线程，与物理天然同步

### 3.2 独立中继（兜底）

Isaac 环境里 rclpy 不可用时才用：

```bash
./scripts/start_ros2_contract.sh                 # 默认 10Hz / 40% 占空比
./scripts/start_ros2_contract.sh stop | status
```

中继必须用**系统 python3.10**。`start_ros2_contract.sh` 已经处理，不要手工用
micromamba 的 python 跑。

> ⚠️ 两种实现话题完全一致，**二选一，别同时开**（会撞同名节点和重复话题）。

### 3.3 相机（两条路都不经过）

图像由 Isaac 内 OmniGraph 原生发布，和上面选哪条无关：

```bash
curl -s -X POST localhost:18765/execute -H 'Content-Type: application/json' \
  -d '{"action":"setup_ros2_cameras","parameters":{"enabled":true,"fps":15}}'
```

---

## 4. 已知限制

| 项 | 现状 |
|---|---|
| 状态频率 | 进程内实测 **~40 Hz**（`rate_hz=50`，受场景渲染帧率压制；电力院背景可见时约 40 FPS）。旧中继因每次 `get_sim_state` 要 ~140 ms HTTP 往返，只有 5~8 Hz |
| 相机频率 | 4 路相机各需一次 render pass，主循环 ~15 FPS 时实测 **1~4 Hz**。要更高就减相机数 / 降分辨率 / 提高渲染频率 |
| 手臂 ↔ 相机互相影响 | 相机开着时手臂反馈会从 8 Hz 掉到 4~5 Hz |
| HTTP 与 ROS2 争用 | 中继运行期间外部 `POST /execute` 仍可能偶发 409，客户端应重试 |
| 深度图 | 未实现（仿真侧没有深度读取接口） |
| `control/target_pose` | 未实现（需要 IK 封装）；点到点请用 `move_j` |

---

## 5. 后续（本轮未做）

- TF（`/tf`、`/tf_static`）与 `/clock`
- 底盘：`/iasscsim_robot/sim/base/control/cmd_vel`、`/iasscsim_robot/sim/base/odom`
- `lift` / `waist` / `head` 的 `feedback|control/joint_states`（中继其实已按同一张表
  发布，见 `config.json` 的 `ros2_contract.groups`）
- 深度图 `<role>/depth/image_raw`
- `control/target_pose`（需要阻尼最小二乘 IK）

---

## 6. 实现位置

| 内容 | 位置 |
|---|---|
| 契约配置（机器人名、关节组、相机映射） | `config.json` → `ros2_contract` |
| 进程内节点（py3.12 rclpy） | `ros2_bridge/contract_node.py` |
| 中继进程（py3.10 + rclpy） | `scripts/ros2_sim_contract.py` |
| 启动封装 | `scripts/start_ros2_contract.sh` |
| 相机 OmniGraph 构建 | `ros2_bridge/contract_graph.py` |
| 仿真侧 action | `robot_manager.py`：`get_contract_meta` / `get_sim_state` / `move_j` / `setup_ros2_cameras` / `get_camera_specs` |
| 快捷控制脚本 | `scripts/rosj.sh`（`move` / `home` / `grip` / `show` / `hz` / `name`） |
| 旧接口 | `scripts/ros2_relay.py`（已废弃，`start_ros_relay.sh` 转发到新脚本） |