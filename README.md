# RouteManager

基于 NVIDIA Isaac Sim 的人形机器人仿真管理服务器。通过 HTTP API 完成机器人仿真的全流程操作：URDF 导入与修改、相机传感器管理、ROS2 关节控制，开箱即用，无需编写 Isaac Sim 代码。

## 特性

- **URDF 一键导入**：URDF → USD 自动转换，注册为可控制的 Articulation；导入前可用纯 XML 工具修改关节限位、动力学参数
- **相机传感器**：在机器人任意位置挂载虚拟相机，支持设置分辨率、焦距、位姿，抓图返回 base64 JPEG
- **ROS2 闭环**：外部 ROS2 节点往 `/humanoid/joint_command` 发 `JointState` 即可驱动机器人，仿真的实际关节状态以固定频率发布到 `/humanoid/joint_states` —— 同一套控制代码可无缝切换仿真/真机。**走 `scripts/ros2_relay.py` 中继，不用进程内的 `setup_ros2_bridge`**（原因见「ROS2 闭环」一节）
- **底盘遥控**：四轮 skid-steer 速度控制（`set_base_velocity`），附带 WASD 键盘遥控脚本，失联自动刹停
- **VR 遥操作**：Pico 4 Ultra 通过 CloudXR 接入，控制器位姿驱动手臂关节、扳机控制夹爪，机器人相机画面回流到头显
- **HTTP 任务服务**：所有操作通过 `POST /execute` 调用，参数自动合并默认配置；另提供 Python 直通接口用于在线调试
- **有界面 / 无头双模式**：支持带 GUI 交互，也支持服务器上无头运行

## 环境要求

| 依赖 | 说明 |
|---|---|
| Isaac Sim | 需自带 Python 环境（内置 fastapi/uvicorn，缺失时 `pip install fastapi uvicorn`） |
| ROS2（可选） | 仅 ROS2 闭环需要，Humble 测试通过。**Humble 的 rclpy 只有 python3.10 版，isaacsim5.1 却是 python3.11 —— 必须走中继脚本，见「ROS2 闭环」** |
| opencv-python | 相机抓图编码用 |

## 完整启动流程

### 第 0 步：环境准备（只需一次）

```bash
# Isaac Sim Python 环境自检（缺 fastapi/uvicorn/opencv 就补装）
micromamba run -n isaacsim5.1 python -c "import isaacsim, fastapi, uvicorn, cv2" \
  || micromamba run -n isaacsim5.1 pip install fastapi uvicorn opencv-python

# 没有 isaacsim5.1 环境时创建（Isaac Sim 5.1 官方 pip 包，需 Python 3.11）
# micromamba create -n isaacsim5.1 python=3.11
# micromamba run -n isaacsim5.1 pip install "isaacsim[all,extscache]==5.1.*" fastapi uvicorn opencv-python

# ROS2（可选，仅闭环控制需要）
# 装在 /opt/ros/humble 时 run.sh 自动 source；装在别处请改 scripts/run.sh 里的 ROS_SETUP
# 注意：rclpy 只能给系统 python3.10 用，中继脚本走它，见「ROS2 闭环」
```

### 第 1 步：放置资产

把 URDF（连同 `meshes/` 目录）放入 `assets/`，服务按仓库根的相对路径解析（如 `assets/humanoid.urdf`）。

### 第 2 步：启动服务

```bash
git clone https://github.com/9527here/RouteManager.git
cd RouteManager

# 有界面启动（桌面环境）
./scripts/run.sh

# 无头模式（服务器 / CI）
HEADLESS=1 ./scripts/run.sh

# 其他 Isaac 环境（脚本默认找 micromamba 的 isaacsim5.1）
ISAAC_PYTHON="/path/to/envs/isaacsim6.0/bin/python" ./scripts/run.sh
```

脚本自动 source ROS2 环境、用 isaacsim5.1 的 Python 起服务。**首次启动加载 Isaac Sim 需 1~3 分钟属正常**；看到日志 `HTTP 服务启动: http://127.0.0.1:18765` 即就绪。

### 第 3 步：确认服务可用

```bash
curl http://127.0.0.1:18765/health
# {"status":"IDLE","task":null}   ← IDLE = 就绪
```

### 第 4 步：标准工作循环

```bash
# 1) 地面场景 → 2) 加载机器人（路径已内置 config.json，可不传参数）→ 3) 查关节
curl -X POST http://127.0.0.1:18765/execute -H 'Content-Type: application/json' \
  -d '{"action": "load_scene"}'
curl -X POST http://127.0.0.1:18765/execute -H 'Content-Type: application/json' \
  -d '{"action": "load_robot"}'
curl -X POST http://127.0.0.1:18765/execute -H 'Content-Type: application/json' \
  -d '{"action": "get_joint_names"}'

# 相机 / ROS2 闭环控制 / 抓图 —— 见下方「使用示例」
```

**或用一键脚本**（等价于上面三连，内置服务等待与结果摘要）：

```bash
./scripts/load_scene.sh                        # 地面 + config 默认机器人
./scripts/load_scene.sh -n v3_ctag2f120_cameras # 按预设名加载
./scripts/load_scene.sh -a /path/to/robot.urdf  # 直接指定 URDF/USD
```

> **当前默认机器人是 `v3_ctag2f120_cameras`**（带 7 路相机的 USD 资产包，约 291MB，首次挂载需等待加载收敛）。`robot.presets` 里另有 `humanoid_upper_body_v3_ctag2f120`（无相机版）。**换机器人改 config.json**：`robot.name` 切默认，`robot.presets` 加预设；或调用时传 `{"name": "..."}` / `{"usd_path": "..."}` 临时指定。
>
> 预设路径是**绝对路径、随部署机器不同**，换机器要改 `config.json` 的 `robot.presets`。

### 停止服务

前台 `Ctrl+C` 即优雅退出（自动销毁 ROS2 节点并关闭 SimulationApp）。异常退出后的残留进程：

```bash
pkill -f "[r]outeManager.*main.py"   # 方括号写法防止误杀 pkill 自身
```

### 常见问题

| 现象 | 处理 |
|---|---|
| `ModuleNotFoundError: isaacsim` | run.sh 没找到 Isaac Python，用 `ISAAC_PYTHON` 显式指定 |
| `setup_ros2_bridge` 报"未初始化" | **预期行为** —— python3.11/3.10 冲突，改用 `scripts/ros2_relay.py` |
| `ros2_relay.py` 报 rclpy 导入失败 | 中继要跑在系统 `python3`(3.10) 上，不是 isaacsim5.1 |
| 物理/画面卡成 10Hz | 默认已按 `render.interval=5` 降频渲染；需要更高画面用 `set_render_interval`，或启动前设 `ROUTE_MANAGER_RENDER_INTERVAL=1` |
| VR 画面仍低帧率 | VR 会按 `teleop.vr_frame_interval` 提交画面；默认 2 个物理步一帧，并把 `head_zed_x` 降到 960×600 以降低 GPU 压力 |
| 静态相机抓图/VR 黑屏 | `load_robot` 会自动注册 `/World/cams/*`；默认只初始化 `head_zed_x` 渲染管线，其余相机保留为待启用对象 |
| 头显画面左右转反 | 调 `config.json` 的 `teleop.head_yaw_sign`：`1` 表示与头显 yaw 同号，`-1` 表示取反；当前机器人实测为 `1` |
| 启动日志有 `[Error] [py stderr]` | Kit 转发 uvicorn 的 INFO 日志，**不是错误** |
| 启动日志有 `node_STL_BINARY_` 警告 | 渲染端良性噪音 |
| 抓图返回空/黑图 | 刚启动渲染未收敛，稍后重试；确认场景里有光源（`load_scene` 默认地平面+材质） |
| 启动卡在加载 | 大 URDF/mesh（数百 MB）首次转换需数分钟，看日志是否在持续输出 |

## API 一览

| 接口 | 说明 |
|---|---|
| `POST /execute` | 执行 action，参数自动与 `config.json` 中的默认值合并 |
| `POST /python` | 在仿真主线程执行任意 Python（调试用，可访问 `executor`） |
| `GET /health` | 服务 / 任务状态查询 |

### Action 列表

| Action | 说明 | 主要参数 |
|---|---|---|
| `load_scene` | 加载场景 USD / 默认地平面 | `usd_path`, `prim_path` |
| `load_robot` | 加载 USD 机器人资产并注册 Articulation（推荐；支持 config 预设库按名切换） | `name`, `usd_path`, `prim_path` |
| `load_urdf` | URDF→USD 导入并注册 Articulation | `urdf_path`, `prim_path`, `fix_base`, `merge_fixed_joints` |
| `get_joint_names` | 列出导入后的关节 | — |
| `set_joint_positions` | 直接设关节位置（物理仿真） | `positions`, `joint_names` |
| `get_joint_positions` | 读取关节位置 | — |
| `set_home_pose` | 设置基础姿态（默认写回 config.json 的 `home_pose`，重启仍生效） | `pose`（`{"关节名": rad}`）, `save` |
| `add_camera` | 创建相机传感器（`attach_to` 可挂到 link 上随动） | `prim_path`, `attach_to`, `name`, `position`, `rotation_euler`, `resolution`, `focal_length`, `intrinsics` |
| `get_camera_image` | 抓图（返回 base64 JPEG） | `prim_path` |
| `move_camera` | 移动相机（挂载相机按本地坐标） | `prim_path`, `position`, `rotation_euler` |
| `send_joint_command` | 下发关节目标到仿真（非阻塞） | `positions`, `duration`（>0 时平滑插值，主循环逐帧推进）, `joint_names`（**只下发这些关节**，其余沿用当前指令目标） |
| `set_gripper_position` | 夹爪指令，**睿尔曼 rm_driver 语义** | `side`（left/right/both）, `position`（1~1000，1000=全开）；与真机 `{ns}/set_gripper_position_cmd` 同参 |
| `get_gripper_position` | 夹爪开口回读（角度/比例/rm 单位） | `side` |
| `set_base_velocity` | 底盘速度指令（skid-steer 四轮） | `vx`（m/s）, `wz`（rad/s）；需 ≥10Hz 持续发送，超时自动刹停 |
| `get_joint_states` | 读取指令/反馈状态 | — |
| `start_vr_teleop` | 启动 Pico VR 遥操作 | `rate_hz`, `workspace_scale`, `max_step`, `deadzone`, `grip_open`（默认取 config.json `teleop`） |
| `stop_vr_teleop` | 停止 VR 遥操作 | — |
| `vr_teleop_status` | VR 遥操作状态（`running`/`frames`/`last_error`） | — |
| `setup_ros2_bridge` | 创建**进程内** ROS2 闭环。⚠️ isaacsim5.1 是 python3.11、Humble rclpy 是 3.10，**此接口在本项目环境不可用**，请改用 `scripts/ros2_relay.py` | `joint_names`, `command_topic`, `state_topic`, `publish_rate` |

## 使用示例

```bash
# 加载默认机器人（config.json 的 robot.name 决定加载谁，可不传 parameters）
curl -X POST http://127.0.0.1:18765/execute -H 'Content-Type: application/json' \
  -d '{"action": "load_robot"}'

# 按预设名切换（robot.presets 里配置过的机器人）
curl -X POST http://127.0.0.1:18765/execute -H 'Content-Type: application/json' \
  -d '{"action": "load_robot", "parameters": {"name": "humanoid_upper_body_v3_ctag2f120"}}'

# 或临时指定路径（不入预设库）
curl -X POST http://127.0.0.1:18765/execute -H 'Content-Type: application/json' \
  -d '{"action": "load_robot", "parameters": {"usd_path": "assets/robot.usd", "prim_path": "/humanoid"}}'

# 或从 URDF 导入（注意：package:// 形式的 mesh 引用导入器不解析，
# 建议优先用转换好的 USD 资产，见下方「URDF 导入前修改」）
curl -X POST http://127.0.0.1:18765/execute -H 'Content-Type: application/json' \
  -d '{"action": "load_urdf", "parameters": {"urdf_path": "assets/humanoid.urdf"}}'

# 在头部挂一个相机（世界坐标定位）
curl -X POST http://127.0.0.1:18765/execute -H 'Content-Type: application/json' \
  -d '{"action": "add_camera", "parameters": {"prim_path": "/humanoid/head_camera", "position": [0, 0, 1.6]}}'

# 或挂到 link 上随机器人运动（本地坐标定位；需 merge_fixed_joints=false）
curl -X POST http://127.0.0.1:18765/execute -H 'Content-Type: application/json' \
  -d '{"action": "add_camera", "parameters": {"prim_path": "/humanoid/head_link/head_camera", "attach_to": "/humanoid/head_link", "position": [0, 0, 0.1]}}'

# 抓图（返回 data.image_base64）
curl -X POST http://127.0.0.1:18765/execute -H 'Content-Type: application/json' \
  -d '{"action": "get_camera_image", "parameters": {"prim_path": "/humanoid/head_camera"}}'

# ROS2 闭环：另开一个终端跑中继（用系统 python3.10，不是 isaacsim5.1），
# 之后外部节点发 /humanoid/joint_command 即可驱动机器人，
# 实际关节状态以 50Hz 发布到 /humanoid/joint_states
python3 scripts/ros2_relay.py

# HTTP 直发关节目标（duration > 0 平滑插值，非阻塞）
curl -X POST http://127.0.0.1:18765/execute -H 'Content-Type: application/json' \
  -d '{"action": "send_joint_command", "parameters": {"positions": [0,0,0,0,0,0], "duration": 2.0}}'

# 读取指令/反馈
curl -X POST http://127.0.0.1:18765/execute -H 'Content-Type: application/json' \
  -d '{"action": "get_joint_states"}'

# 在线调试：直接在仿真主线程执行代码
curl -X POST http://127.0.0.1:18765/python -H 'Content-Type: application/json' \
  -d '{"code": "import omni.usd; print(omni.usd.get_context().get_stage())"}'
```

## ROS2 闭环

**为什么不用进程内的 `setup_ros2_bridge`**：isaacsim5.1 环境是 Python 3.11，
而 ROS Humble 的 rclpy C 扩展只编了 python3.10 版本，仿真进程里 `import rclpy`
直接失败（`No module named 'rclpy._rclpy_pybind11'`）。该 action 会走降级分支
报「未初始化」，不影响仿真其余功能。

**实际通路**是 `scripts/ros2_relay.py` —— 用**系统 python3.10** 跑 rclpy，
把 ROS2 话题与 HTTP 接口互转：

```
/humanoid/joint_command (JointState) → POST /execute send_joint_command
轮询 get_joint_states → /humanoid/joint_states (JointState)
```

```bash
python3 scripts/ros2_relay.py          # 默认 http://127.0.0.1:18765
# 环境变量：ROUTE_MANAGER_URL / RELAY_RATE（默认 50Hz）
```

对上层 ROS2 节点而言接口不变，仿真/真机仍可切换。附带的好处：
`send_joint_command` 走 HTTP 直连路径、不回发 command_topic，不会有回声。

## 底盘控制

四轮 skid-steer 由 `set_base_velocity` 速度控制**专管**；关节位置指令
（ROS/HTTP）里的轮子分量会被忽略，避免两路控制打架。

```bash
# 前进 0.2 m/s（wz=0）。需以 >=10Hz 持续发送，超时 0.5s 自动刹停
curl -X POST http://127.0.0.1:18765/execute -H 'Content-Type: application/json' \
  -d '{"action": "set_base_velocity", "parameters": {"vx": 0.2, "wz": 0.0}}'

# 终端键盘遥控（WASD，纯标准库无需装包；空格急停）
python3 scripts/wasd_teleop.py
python3 scripts/wasd_teleop.py --speed 0.3 --turn 1.0
```

轮径 / 轮距 / 限速 / 超时 / 左右轮符号在 `config.json` 的 `base_teleop`。
**实测若前进变后退或转向反向，改 `left_sign` / `right_sign`**（URDF 左右轮轴镜像差异）。

## VR 遥操作（Pico 4 Ultra）

控制器位姿 → 手臂关节，扳机 → 夹爪开合，机器人相机画面流到头显。

```bash
./scripts/teleop/setup_isaacteleop.sh     # 一次性：克隆环境装 isaaccapture
sudo ufw allow 47998/udp && sudo ufw allow 49100/tcp && sudo ufw allow 48322/tcp
ISAAC_PYTHON="micromamba run -n isaacsim5.1_teleop python" ./scripts/run.sh
# 加载机器人 → start_vr_teleop → Pico 浏览器打开客户端页，Server Address 填电脑 IP
```

`start_vr_teleop` 参数默认取 `config.json` 的 `teleop` 节（`workspace_scale` /
`max_step` / `deadzone` / `grip_open` / `rate_hz`），调用传参可覆盖。
手臂关节留空则按 `arm_*` 名称自动识别，识别不到就手工填 `teleop.arm_joints`。

完整步骤、Pico 端配置、故障排查见 **[scripts/teleop/README.md](scripts/teleop/README.md)**。
要点：头显里要有画面必须先给机器人加相机（`add_camera` 挂 `head_link`）——
CloudXR 流的是 XR 应用提交的图层，不提交就是黑屏。

## URDF 导入前修改

> **提示**：Isaac 的 URDF 导入器不解析 `package://` 形式的 mesh 引用（ROS 包路径）。
> 手头只有这种 URDF 时，要么把 mesh 路径批量改写为绝对路径再导入，要么直接用
> 转换好的 USD 资产走 `load_robot`（本项目默认机器人即为此格式）。

`urdf_tools/editor.py` 提供纯 XML 级别的 URDF 修改（不依赖 Isaac Sim，导入前使用）：

```python
from urdf_tools.editor import UrdfEditor

ed = UrdfEditor("assets/humanoid.urdf")
print(ed.list_joints())
ed.set_joint_limit("joint1", lower=-2.6, upper=2.6, effort=100)
ed.set_joint_dynamics("joint1", damping=0.5)
ed.save("assets/humanoid_modified.urdf")
```

## 项目结构

```
RouteManager/
├── main.py                  # 入口：SimEnv → RobotManager → IsaacTaskServer
├── standalone_server.py     # HTTP 任务服务器（双线程：HTTP 收请求、主线程跑仿真）
├── sim_env.py               # SimulationApp 启动 + ros2.bridge / DebugDraw / Omnigraph 插件启用
├── robot_manager.py         # 所有 HTTP action 的实现（执行器）
├── isaac_teleop.py          # Pico VR 遥操作会话管理（VRTeleopManager）
├── isaac_add_cameras.py     # 独立脚本：按 URDF 外参批量建 7 路相机（Script Editor 里跑，不走 HTTP）
├── urdf_tools/
│   └── editor.py            # URDF 纯 XML 修改工具（导入前：限位/动力学/origin）
├── ros2_bridge/
│   └── controller.py        # 进程内 ROS2 关节控制节点（⚠️ 本项目 python 版本冲突，见「ROS2 闭环」）
├── config.json              # action 默认参数表 + 机器人预设库 + home_pose/teleop/base_teleop
├── assets/                  # 存放 URDF / USD 资产
├── humanoid_upper_body_v3/  # 随机型 URDF 包（urdf + meshes）
├── memory/                  # 项目记忆：MEMORY.md 是索引，实测细节/踩坑记这里
└── scripts/
    ├── run.sh               # 启动脚本（自动 source ROS2 环境 + isaacsim5.1）
    ├── load_scene.sh        # 一键加载场景（地面 + 机器人，支持预设名/资产路径）
    ├── ros2_relay.py        # ROS2 ↔ HTTP 中继（系统 python3.10 跑，见「ROS2 闭环」）
    ├── wasd_teleop.py       # 终端 WASD 键盘遥控底盘
    └── teleop/              # VR 遥操作安装脚本 + 详细文档
```

## 注意事项

1. **初始化顺序**：`SimulationApp` 必须最先创建（`main.py` 已保证）；不要在 `robot_manager.py` 顶层 import isaacsim/omni/pxr。
2. **action 命名**：`RobotManager` 上的公开方法自动成为 HTTP action；`config.json` 的 key 为 action 去掉 `load_` 前缀。
3. **并发限制**：同一时刻只允许一个任务，忙时返回 409。
4. **Python 版本**：isaacsim5.1 是 **python3.11**，ROS Humble 的 rclpy 是 **python3.10** —— 两者不能互相 import。所以服务进程内跑不了 rclpy，ROS2 一律走中继脚本（见「ROS2 闭环」）。`run.sh` 会 source ROS2 环境，但对仿真进程无实际作用。
5. **Isaac 版本兼容**：`parse_urdf` 返回值因 Isaac 版本而异，`load_urdf` 已做兼容处理；如遇导入报错，检查 `isaacsim.asset.importer.urdf` 的版本签名。相机 `set_focal_length()` 收的是**米**（不是 mm），传错会差 1000 倍。
6. **渲染降频**：主循环默认每 5 个物理步渲染一次（`config.json` 的 `render.interval`），避免 7 路相机把仿真拖到 10Hz。`ROUTE_MANAGER_RENDER_INTERVAL` 可在启动时覆盖；运行中可用 `set_render_interval` 调整。显式调用 `executor.step(render=True)` 仍会强制渲染。
7. **相机抓图时机**：服务主循环自动渲染，正常情况下 `get_camera_image` 直接可用；刚启动或刚把 interval 调大时可能取到上一帧，稍后重试即可。
8. **ROS2 闭环**：`setup_ros2_bridge`（进程内）在本项目环境不可用；用 `scripts/ros2_relay.py`，外部节点发布到 `command_topic` 的 `JointState` 由仿真主线程施加到机器人，仿真以 `publish_rate`（默认 50Hz）把实际关节状态发到 `state_topic`。`send_joint_command` 走 HTTP 直连路径（不进 ROS hop），同样驱动仿真且不回发话题。
9. **挂载相机**：`add_camera` 的 `attach_to` 要求对应 link prim 存在 —— URDF 导入保持 `merge_fixed_joints: false`（config.json 默认值）。**多相机场景必须显式传 `name`**：默认名取 `prim_path` 末段，多个相机都叫 `camera` 会撞场景注册表。
10. **端口**：默认 **18765**，用 `ROUTE_MANAGER_PORT` 覆盖。启动前 `main.py` 会自动清理占用该端口的本项目残留实例（非本项目进程则报错退出）。
11. **底盘与关节指令互斥**：四轮由 `set_base_velocity` 速度控制专管，关节位置指令里的轮子分量被忽略；VR 遥操作和底盘控制可同时使用，手臂与底盘互不干扰。
12. **调试技巧**：改完 `robot_manager.py` 不想重启丢场景，可热替换：
    ```
    POST /python  {"code": "import importlib, robot_manager; importlib.reload(robot_manager); executor.__class__ = robot_manager.RobotManager"}
    ```
    `/python` 传多行代码容易被 JSON 转义搞炸，用 python urllib 驱动更稳。

## License

MIT
