# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## 项目是什么

RouteManager：基于 Isaac Sim 的人形机器人仿真 HTTP 管理服务器。外部只通过
`POST /execute {"action": ..., "parameters": {...}}` 驱动仿真——URDF/USD 导入、
相机挂载抓图、关节控制、ROS2 闭环、Pico VR 遥操作。目标是同一套控制代码能在仿真
与真机间无缝切换。

## 环境与命令

```bash
# 启动（有界面）
./scripts/run.sh
# 无头
HEADLESS=1 ./scripts/run.sh
# 用别的 Isaac 环境（默认自动探测 micromamba 的 isaacsim5.1）
ISAAC_PYTHON="/path/to/envs/isaacsim6.0/bin/python" ./scripts/run.sh

# 就绪判据：日志出现 "HTTP 服务启动: http://127.0.0.1:18765"
curl http://127.0.0.1:18765/health        # {"status":"IDLE"} = 就绪

# 一键加载（地面 + 机器人，内置等服务与关节摘要）
./scripts/load_scene.sh                             # config.json 的 robot.name
./scripts/load_scene.sh -n v3_ctag2f120_cameras     # 按预设名
./scripts/load_scene.sh -a /path/to/robot.urdf      # 按扩展名选 loader
./scripts/load_scene.sh --health

# 停止：前台 Ctrl+C 优雅退出；残留进程
pkill -f "[r]outeManager.*main.py"       # 方括号防 pkill 自杀

# 不起 Isaac，只测 HTTP 层（假执行器）
python standalone_server.py --demo

# 终端键盘控制底盘
python3 scripts/wasd_teleop.py
```

- **本项目环境是 micromamba 的 `isaacsim5.1`**。机器上另有 `isaacsim6.0`,不要混用(用户明确指定 5.1)。
- 端口 **18765**(`ROUTE_MANAGER_PORT` 可覆盖，`load_scene.sh` 同名变量生效)。
- ROS2 在 `/opt/ros/humble`；`run.sh` 找不到会打印警告并让 ROS2 功能降级，不影响仿真。
- 首次启动加载 Isaac 需 **1~3 分钟**，热缓存时约 8s，属正常。
- 没有测试框架。验证靠 HTTP 打接口 + `/health`。

## 架构

### 硬性约束：初始化顺序

`SimulationApp` 必须最先创建，之后才能 import 任何 `isaacsim`/`omni`/`pxr` 模块。

因此 **`standalone_server.py`、`robot_manager.py`、`ros2_bridge/` 顶层不 import 这些包**，
全部在函数内延迟导入。改动这些文件时必须保持这一约定，否则启动即崩。

```
main.py              cleanup_stale_server → SimEnv → RobotManager → IsaacTaskServer.start()
sim_env.py           SimulationApp 启动 + 启用插件(ros2.bridge / debug_draw / omnigraph)
robot_manager.py     执行器：所有 action 的实现
standalone_server.py HTTP 框架层(与 Isaac 无关，可整体拷到别的项目)
isaac_teleop.py      Pico VR 遥操作(VRTeleopManager，由 RobotManager 持有)
ros2_bridge/         ROS2 关节闭环节点
urdf_tools/editor.py 纯 XML 改 URDF(不依赖 Isaac，导入前用)
config.json          action 默认参数表 + 机器人预设库
```

### 双线程模型

- **HTTP 线程**(uvicorn，异步)：只做参数合并 + 提交任务，然后阻塞轮询结果。
- **仿真主线程**(`IsaacTaskServer._run_loop`)：轮询状态机，`PENDING` 则在本线程执行。
- 同一时刻只允许一个任务，忙时 HTTP 返回 **409**。

所有 action 都在仿真主线程执行，因此**可以放心操作 stage / 物理，无需加锁**。
反向约束：HTTP 线程里绝不能碰 Isaac 对象。

### action 分发靠反射

`RobotManager` 上的公开方法自动成为 HTTP action(`getattr` 分发)，无需注册。
两个配套约定：

1. `config.json` 的 key 是 action 名**去掉 `load_` 前缀**(见 `_merge_default_params`)，
   请求传入的参数优先于默认值。
2. 结果序列化：`@register_serializer("<action>")` 注册专用转换器(如
   `get_camera_image` → base64 JPEG)，未注册的走 `_to_jsonable` 递归转 numpy。

`main.py` 显式传 `loading_tasks={load_scene, load_urdf, load_robot, add_camera}` ——
这些 action 执行完会等 stage 加载收敛(`_wait_stage_loading`)再返回，让前端拿到就已经能用。

### 退出与残留清理

- `main.py` 在 `SimulationApp` **之前**用 `cleanup_stale_server` 清掉占端口的残留实例
  (本项目进程 SIGTERM→SIGKILL；别的程序占用则报错退出)。放在前面是为了避免白等 1~3 分钟加载。
- `IsaacTaskServer` 必须**在 SimulationApp 之后**注册 SIGINT/SIGTERM/SIGHUP 处理
  (覆盖 Kit 自带的)，第一次优雅关闭，第二次 `os._exit`。

### ROS2 闭环

```
外部节点 publish → command_topic →[rclpy spin 线程]→ 只暂存"最新指令"
                                                     ↓
RobotManager.step() (仿真主线程) 取用 → 施加到 articulation
                                     └→ publish → state_topic →[订阅回读作 feedback]
```

**回调线程只更新缓冲，物理操作全在主线程 `step()` 里做**——这是线程安全的唯一原因。

`send_joint_command`(HTTP 直连)走非阻塞插值状态机，`_advance_motion` 逐帧推进，
且**不回发 command_topic**(避免回声)。

### 底盘控制

`_BASE_WHEEL_JOINTS`(四轮 skid-steer)由 `set_base_velocity` 速度控制专管；
关节位置指令(ROS/HTTP)里的轮子分量会被**忽略**，避免两路控制打架。
超过 `base_teleop.timeout`(默认 0.5s)收不到速度指令自动刹停。

## 踩坑清单

这些是实测撞过的，改代码前先看：

**Isaac Sim 5.1 API**(`robot_manager.py` 已适配，别改回去)
- `isaacsim.core.prims.Articulation` 第一参改名为 `prim_paths_expr` → 用**位置传参**兼容新旧。
- 相机 `set_focal_length(米)` / `set_clipping_range(near, far)` 是 setter；构造不再收这两个参数。
  **把 mm 数喂给米制 setter 会差 1000 倍**(110° 变 0.16°)。
- `set_local_pose` 参数名是 `translation`(不是 position)，且多了 `camera_axes`
  (`"usd"` = 原始 USD 约定 [0,0,0] 朝 -Z，本项目用它)。
- view API 关节位置形状 `(M, K)`，单机器人 M=1，读写都要 reshape。
- 查 API 签名不用起 Kit：直接 grep `site-packages/isaacsim/exts/isaacsim.*.*/impl/*.py`。

**pxr / USD**
- `UsdPrim` **没有 `GetDescendants()`**，遍历后代用 `GetAllChildren()`。
- USD 禁止直接编辑 instanceable 原型，要写 over(见 `_hide_mount_visuals`)。

**资产与相机**
- Isaac 的 URDF 导入器**不解析 `package://` 形式的 mesh 引用**。手头只有这种 URDF 时，
  要么批量改写成绝对路径，要么直接用转换好的 USD 走 `load_robot`。
- `add_camera` 的 `name` 默认取 `prim_path` 末段 → 多个相机都叫 `camera` 会撞场景注册表，
  多相机场景**显式传 `name`**。
- `attach_to` 要求 link prim 存在，所以 URDF 导入要保持 `merge_fixed_joints: false`。
- 刚启动立即抓图可能返回空/黑图(渲染未收敛)，稍后重试。

**不丢场景的热修复**
```
POST /python  {"code": "import importlib, robot_manager; importlib.reload(robot_manager);
                       executor.__class__ = robot_manager.RobotManager"}
```
`/python` 端点多行代码经 JSON 传容易炸转义，用 python urllib 驱动脚本更稳。

**日志噪音**(不是错误)
- `[Error] [py stderr]` 前缀的 uvicorn INFO 行 —— Kit 转发所致。
- `node_STL_BINARY_` 警告 —— 渲染端良性。

## 项目自带记忆目录 memory/

`memory/` 是本仓库自带的记忆**不进 `.Codex/`**，`memory/MEMORY.md` 是索引。
里面记了实测细节：机器人预设路径、7 路相机挂载配方(attach_to / rotation_euler /
内参)、Isaac 5.1 API 兼容教训、VR 遥操作链路。

**开工前先读 `memory/MEMORY.md`**；有新发现/修复就更新对应文件并在索引里加一行。
注意区分：这里的 `memory/` 在仓库内跟着 git 走；`~/.Codex/projects/-home-jy-RouteManager/memory/`
是 Codex 的全局 auto-memory，在用户目录里。