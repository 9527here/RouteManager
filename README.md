# RouteManager

人形机器人仿真管理服务器：URDF 导入与修改、相机传感器、ROS2 bridge。
HTTP 框架来自 [simManager](../VLA/simManager) 抽取的单文件模板 `standalone_server.py`。

## 目录结构

```
RouteManager/
├── main.py                  # 入口：SimEnv → RobotManager → IsaacTaskServer
├── standalone_server.py     # 单文件 HTTP 任务服务器（双线程：HTTP 收请求、主线程跑仿真）
├── sim_env.py               # SimulationApp 启动 + ROS2 Bridge / DebugDraw 插件启用
├── robot_manager.py         # 执行器：所有 HTTP action 的实现
├── urdf_tools/
│   └── editor.py            # URDF 纯 XML 修改工具（导入前用：限位/动力学/origin）
├── ros2_bridge/
│   └── controller.py        # 通用 ROS2 关节控制节点（发布指令/订阅反馈，独立线程 spin）
├── config.json              # action 默认参数表（前端只传需覆盖的参数）
├── assets/                  # 放你的 URDF / USD 资产
└── scripts/
    └── run.sh               # 启动脚本（自动 source ROS2）
```

## 启动

```bash
cd /home/jy/RouteManager
./scripts/run.sh               # 有界面
HEADLESS=1 ./scripts/run.sh    # 无头
```

启动后 HTTP 服务在 `http://127.0.0.1:8765`：
- `POST /execute` — 执行 action（自动合并 config.json 默认参数）
- `POST /python` — 在仿真主线程执行任意 Python（调试用，可访问 `executor`）
- `GET /health` — 服务/任务状态

## Action 一览

| Action | 说明 | 主要参数 |
|---|---|---|
| `load_scene` | 加载场景 USD / 默认地平面 | `usd_path`, `prim_path` |
| `load_urdf` | URDF→USD 导入并注册 Articulation | `urdf_path`, `prim_path`, `fix_base`, `merge_fixed_joints` |
| `get_joint_names` | 列出导入后的关节 | — |
| `set_joint_positions` | 直接设关节位置（物理仿真） | `positions`, `joint_names` |
| `get_joint_positions` | 读关节位置 | — |
| `add_camera` | 创建相机传感器 | `prim_path`, `position`, `rotation_euler`, `resolution`, `focal_length` |
| `get_camera_image` | 抓图（返回 base64 JPEG） | `prim_path` |
| `move_camera` | 移动相机 | `prim_path`, `position`, `rotation_euler` |
| `setup_ros2_bridge` | 创建 ROS2 关节控制节点 | `joint_names`, `command_topic`, `state_topic` |
| `send_joint_command` | 通过 ROS2 发关节指令 | `positions`, `duration`（>0 平滑插值） |
| `get_joint_states` | 读 ROS2 反馈 | — |

## 使用示例

```bash
# 导入人形 URDF
curl -X POST http://127.0.0.1:8765/execute -H 'Content-Type: application/json' \
  -d '{"action": "load_urdf", "parameters": {"urdf_path": "/home/jy/RouteManager/assets/humanoid.urdf"}}'

# 在头部挂一个相机
curl -X POST http://127.0.0.1:8765/execute -H 'Content-Type: application/json' \
  -d '{"action": "add_camera", "parameters": {"prim_path": "/humanoid/head_camera", "position": [0, 0, 1.6]}}'

# 抓图（返回 data.image_base64）
curl -X POST http://127.0.0.1:8765/execute -H 'Content-Type: application/json' \
  -d '{"action": "get_camera_image", "parameters": {"prim_path": "/humanoid/head_camera"}}'

# 建 ROS2 bridge 并发指令
curl -X POST http://127.0.0.1:8765/execute -H 'Content-Type: application/json' \
  -d '{"action": "setup_ros2_bridge"}'
curl -X POST http://127.0.0.1:8765/execute -H 'Content-Type: application/json' \
  -d '{"action": "send_joint_command", "parameters": {"positions": [0,0,0,0,0,0], "duration": 2.0}}'

# 在线调试：直接在主线程跑代码
curl -X POST http://127.0.0.1:8765/python -H 'Content-Type: application/json' \
  -d '{"code": "import omni.usd; print(omni.usd.get_context().get_stage())"}'
```

## URDF 修改流程

导入前用 `urdf_tools/editor.py`（纯 XML），导入后直接操作 USD：

```python
from urdf_tools.editor import UrdfEditor

ed = UrdfEditor("assets/humanoid.urdf")
print(ed.list_joints())
ed.set_joint_limit("joint1", lower=-2.6, upper=2.6, effort=100)
ed.set_joint_dynamics("joint1", damping=0.5)
ed.save("assets/humanoid_modified.urdf")
```

## 已知约定与坑

1. **初始化顺序**：SimulationApp 必须最先创建（main.py 已保证）；不要在
   robot_manager.py 顶层加 isaacsim/omni/pxr import。
2. **action 名 = 方法名**：`RobotManager` 上的公开方法自动成为 HTTP action；
   config.json 的 key 是 action 去掉 `load_` 前缀。
3. **并发**：同一时刻只允许一个任务，忙时返回 409。
4. **ROS2**：必须 source ROS2 环境再启动；bridge 初始化失败会降级（不阻塞仿真）。
5. **`parse_urdf` 的返回值因 Isaac 版本而异**：robot_manager.load_urdf 已做兼容
   （自动判断是否已挂到 stage）。如果导入报 API 变动，检查
   `isaacsim.asset.importer.urdf` 的版本签名。
6. **抓图前需要渲染收敛**：服务主循环持续 step(render=True)，正常情况
   get_camera_image 直接可用；刚开服立刻抓可能返回空，稍后重试即可。

## 依赖

- Isaac Sim（含 fastapi/uvicorn，或 pip install fastapi uvicorn）
- ROS2（可选，bridge 用；humble 测试通过）
- opencv-python（抓图编码用）
