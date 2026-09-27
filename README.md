# RouteManager

基于 NVIDIA Isaac Sim 的人形机器人仿真管理服务器。通过 HTTP API 完成机器人仿真的全流程操作：URDF 导入与修改、相机传感器管理、ROS2 关节控制，开箱即用，无需编写 Isaac Sim 代码。

## 特性

- **URDF 一键导入**：URDF → USD 自动转换，注册为可控制的 Articulation；导入前可用纯 XML 工具修改关节限位、动力学参数
- **相机传感器**：在机器人任意位置挂载虚拟相机，支持设置分辨率、焦距、位姿，抓图返回 base64 JPEG
- **ROS2 Bridge（闭环）**：外部 ROS2 节点往 `command_topic` 发 `JointState` 即可驱动机器人，仿真的实际关节状态以固定频率发布到 `state_topic` —— 同一套控制代码可无缝切换仿真/真机
- **HTTP 任务服务**：所有操作通过 `POST /execute` 调用，参数自动合并默认配置；另提供 Python 直通接口用于在线调试
- **有界面 / 无头双模式**：支持带 GUI 交互，也支持服务器上无头运行

## 环境要求

| 依赖 | 说明 |
|---|---|
| Isaac Sim | 需自带 Python 环境（内置 fastapi/uvicorn，缺失时 `pip install fastapi uvicorn`） |
| ROS2（可选） | 仅 ROS2 Bridge 需要，Humble 测试通过 |
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

# ROS2（可选，仅 ROS2 bridge 需要）
# 装在 /opt/ros/humble 时 run.sh 自动 source；装在别处请改 scripts/run.sh 里的 ROS_SETUP
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

> 默认机器人是 `humanoid_upper_body_v3_ctag2f120`（USD 资产包，约 291MB，首次挂载需等待加载收敛）。**换机器人改 config.json**：`robot.name` 切默认，`robot.presets` 加预设；或调用时传 `{"name": "..."}` / `{"usd_path": "..."}` 临时指定。

### 停止服务

前台 `Ctrl+C` 即优雅退出（自动销毁 ROS2 节点并关闭 SimulationApp）。异常退出后的残留进程：

```bash
pkill -f "[r]outeManager.*main.py"   # 方括号写法防止误杀 pkill 自身
```

### 常见问题

| 现象 | 处理 |
|---|---|
| `ModuleNotFoundError: isaacsim` | run.sh 没找到 Isaac Python，用 `ISAAC_PYTHON` 显式指定 |
| ROS2 相关 action 报"未初始化" | 启动前未 source ROS2，或 rclpy 缺失；仿真其余功能不受影响 |
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
| `add_camera` | 创建相机传感器（`attach_to` 可挂到 link 上随动） | `prim_path`, `attach_to`, `position`, `rotation_euler`, `resolution`, `focal_length` |
| `get_camera_image` | 抓图（返回 base64 JPEG） | `prim_path` |
| `move_camera` | 移动相机（挂载相机按本地坐标） | `prim_path`, `position`, `rotation_euler` |
| `setup_ros2_bridge` | 创建 ROS2 关节闭环（指令→仿真执行，状态→话题回发） | `joint_names`, `command_topic`, `state_topic`, `publish_rate` |
| `send_joint_command` | 下发关节目标到仿真（非阻塞） | `positions`, `duration`（>0 时平滑插值，主循环逐帧推进） |
| `get_joint_states` | 读取指令/反馈状态 | — |

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

# 建立 ROS2 bridge（闭环）：外部 ROS2 节点发 /humanoid/joint_command 即可驱动机器人，
# 实际关节状态以 publish_rate（默认 50Hz）发布到 /humanoid/joint_states
curl -X POST http://127.0.0.1:18765/execute -H 'Content-Type: application/json' \
  -d '{"action": "setup_ros2_bridge"}'

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
├── sim_env.py               # SimulationApp 启动 + ROS2 Bridge / DebugDraw 插件启用
├── robot_manager.py         # 所有 HTTP action 的实现
├── urdf_tools/
│   └── editor.py            # URDF 纯 XML 修改工具（导入前：限位/动力学/origin）
├── ros2_bridge/
│   └── controller.py        # 通用 ROS2 关节控制节点（指令订阅→仿真，状态发布）
├── config.json              # action 默认参数表
├── assets/                  # 存放 URDF / USD 资产
└── scripts/
    ├── run.sh               # 启动脚本（自动 source ROS2 环境 + isaacsim5.1）
    └── load_scene.sh        # 一键加载场景（地面 + 机器人，支持预设名/资产路径）
```

## 注意事项

1. **初始化顺序**：`SimulationApp` 必须最先创建（`main.py` 已保证）；不要在 `robot_manager.py` 顶层 import isaacsim/omni/pxr。
2. **action 命名**：`RobotManager` 上的公开方法自动成为 HTTP action；`config.json` 的 key 为 action 去掉 `load_` 前缀。
3. **并发限制**：同一时刻只允许一个任务，忙时返回 409。
4. **ROS2 环境**：需先 source ROS2 环境再启动；bridge 或插件初始化失败会自动降级，不影响仿真运行。
5. **Isaac 版本兼容**：`parse_urdf` 返回值因 Isaac 版本而异，`load_urdf` 已做兼容处理；如遇导入报错，检查 `isaacsim.asset.importer.urdf` 的版本签名。
6. **相机抓图时机**：服务主循环持续渲染，正常情况下 `get_camera_image` 直接可用；刚启动时立即抓图可能返回空图，稍后重试即可。
7. **ROS2 闭环**：`setup_ros2_bridge` 后，外部节点发布到 `command_topic` 的 `JointState` 由仿真主线程施加到机器人；仿真以 `publish_rate`（默认 50Hz）把实际关节状态发到 `state_topic`。`send_joint_command` 走 HTTP 直连路径（不进 ROS hop），同样驱动仿真。
8. **挂载相机**：`add_camera` 的 `attach_to` 要求对应 link prim 存在 —— URDF 导入保持 `merge_fixed_joints: false`（config.json 默认值）。

## License

MIT
