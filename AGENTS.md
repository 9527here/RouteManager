# AGENTS.md

> 给 AI agent（Claude Code / Codex / Cursor / Copilot …）看的项目说明。
> 人类上手流程在 [README.md](README.md)；本文件只写**agent 必须知道、README 里没有**的东西：
> 不可违反的约束、改了怎么验、踩过的坑、以及知识存在哪。

## 先读这三样（顺序别换）

1. [README.md](README.md) —— 这是什么、怎么装、有哪些 action。
2. **本文件** —— 动手前必须遵守的约束和踩坑清单。
3. [memory/MEMORY.md](memory/MEMORY.md) —— 项目自带记忆的索引。里面是**实测结论**
   （抓取为什么失败、相机焦距单位、底盘为什么推不动…），每条都指向一个 md 文件。
   **动手前先扫一遍索引**，很可能你要查的坑已经有人填过了。

## 这是什么

基于 Isaac Sim 的人形机器人仿真服务器。外部只通过 HTTP 驱动：

```
POST /execute  {"action": "<动作名>", "parameters": {...}}
```

覆盖 URDF/USD 导入、相机挂载抓图、关节控制、ROS2 闭环、底盘行走、自动抓取、
Pico VR 遥操作。**设计目标：同一套控制代码在仿真与真机之间无缝切换**——所以
很多接口刻意对齐了真机（例如夹爪用睿尔曼 `position 1~1000` 语义）。改动接口时
不要破坏这个对齐。

另有三个辅助端点：`GET /health`（就绪判据）、`POST /python`（在仿真主线程执行任意
代码，热调试用）、`POST /execute`。

## 硬性约束（违反 = 启动即崩 / 静默错乱）

### 1. 初始化顺序：`SimulationApp` 必须最先

必须先创建 `SimulationApp`，之后才能 import 任何 `isaacsim` / `omni` / `pxr` 模块。

因此 **`standalone_server.py`、`robot_manager.py`、`ros2_bridge/` 的顶层不 import 这些包**，
一律在函数内延迟导入。往这几个文件加功能时，import 必须写在函数体内。
（`arm_kin.py` / `grasp_planner.py` / `gripper_model.py` 是纯 numpy 的，随便 import。）

### 2. 双线程：物理只在仿真主线程碰

- **HTTP 线程**（uvicorn）：只做参数合并 → 提交任务 → 阻塞轮询结果。
- **仿真主线程**（`IsaacTaskServer._run_loop`）：状态机轮询，`PENDING` 就在本线程执行。

推论 A：所有 action 都在仿真主线程跑，**操作 stage / 物理不需要加锁**。
推论 B：**HTTP 线程里绝不能碰 Isaac 对象**。
推论 C：同一时刻只允许一个任务，忙时返回 **409**——外部脚本要容忍 409 并重试。

### 3. action 靠反射分发，不用注册

`RobotManager` 上的**公开方法自动成为 HTTP action**（`getattr` 分发）。两个配套约定：

- `config.json` 的 key 是 action 名**去掉 `load_` 前缀**（见 `_merge_default_params`）；
  请求里传的参数优先于默认值。
- 返回值序列化：`@register_serializer("<action>")` 注册专用转换器
  （如 `get_camera_image` → base64 JPEG）；没注册的走 `_to_jsonable` 递归转 numpy。

注意：**加一个 `def foo(self, ...)` 就等于对外开了一个 HTTP 接口**。不想暴露就加下划线。

`main.py` 里显式列了 `loading_tasks={load_scene, load_urdf, load_robot, add_camera}`：
这些 action 执行完会等 stage 加载收敛（`_wait_stage_loading`）再返回，让前端拿到就能用。
新增同类"加载型" action 记得加进去。

### 4. 底盘轮子归 `set_base_velocity` 专管

`_BASE_WHEEL_JOINTS`（四轮 skid-steer）只由速度控制驱动。**关节位置指令（ROS/HTTP）
里的轮子分量会被忽略**，避免两路控制打架。超过 `base_teleop.timeout`（默认 0.5s）
收不到速度指令自动刹停。

### 5. Python 版本是分裂的

isaacsim5.1 是 **python3.11**，ROS Humble 的 `rclpy` 是 **python3.10**，两者不能互相 import。
所以**仿真进程内跑不了 rclpy**，ROS2 一律走中继脚本 `scripts/ros2_relay.py`（用系统
python3.10 跑）。`run.sh` 会 source ROS2 环境，但对仿真进程无实际作用，别被误导。

## 环境与命令

```bash
./scripts/run.sh                    # 有界面
HEADLESS=1 ./scripts/run.sh         # 无头
ISAAC_PYTHON="/path/to/envs/x/bin/python" ./scripts/run.sh   # 换 Isaac 环境

curl http://127.0.0.1:18765/health  # {"status":"IDLE"} = 就绪

./scripts/load_scene.sh                          # 加载 config.json 的 robot.name
./scripts/load_scene.sh -n <预设名>               # 按预设
./scripts/load_scene.sh -a /path/to/robot.urdf   # 按扩展名自动选 loader
./scripts/bringup.sh                             # 一键：服务+场景+抬臂+桌子+7相机+VR

pkill -f "[r]outeManager.*main.py"  # 方括号防止 pkill 杀掉自己
```

- 环境是 micromamba 的 **`isaacsim5.1`**（机器上另有 `isaacsim6.0`，**不要混用**）。
- 端口 **18765**，`ROUTE_MANAGER_PORT` 可覆盖。
- **首次启动加载 Isaac 要 1~3 分钟**，热缓存约 8s。别以为卡死了就去杀进程。
- 改完代码验证不需要重启：`POST /python` 热替换 `RobotManager`（见踩坑清单）。

## 改了代码怎么验（没有测试框架）

项目**没有测试框架，没有 CI**。验证靠实际打接口，标准动作：

```bash
curl http://127.0.0.1:18765/health                       # 1. 服务活着
curl -s -X POST http://127.0.0.1:18765/execute \
  -H 'Content-Type: application/json' \
  -d '{"action":"get_joint_names"}'                      # 2. 场景/机器人已加载
curl -s -X POST http://127.0.0.1:18765/execute \
  -H 'Content-Type: application/json' \
  -d '{"action":"get_joint_positions"}'                  # 3. 关节可读
```

- 只测 HTTP 层、不起 Isaac：`python standalone_server.py --demo`（假执行器）。
- 抓取/走位这类行为验证：`python3 scripts/task_sequence.py`（走位+抓取全流程）、
  `python3 scripts/pick_object.py --object cube_a --side right --dry-run`（只规划不动）。
- 断言行为正确时**看实测数值，别看日志有没有报错**——Isaac 大量错误只打日志不抛异常。

## 代码地图

```
main.py                 入口：cleanup_stale_server → SimEnv → RobotManager → IsaacTaskServer
sim_env.py              SimulationApp 启动 + 插件（ros2.bridge / debug_draw / omnigraph）
robot_manager.py        ★ 执行器：所有 action 的实现（14 万字节，最大的文件）
standalone_server.py    HTTP 框架层（与 Isaac 无关，可整体拷到别的项目）
isaac_teleop.py         Pico VR 遥操作（VRTeleopManager，由 RobotManager 持有）
ros2_bridge/            ROS2 关节闭环节点（本项目环境不可用，见约束 5）
urdf_tools/editor.py    纯 XML 改 URDF（不依赖 Isaac，导入前用）
config.json             action 默认参数表 + 机器人预设库 + home_pose/teleop/base_teleop
memory/                 项目记忆（见上）

── 纯 numpy，可离线单测 ──
arm_kin.py              FK + 数值雅可比（不依赖 Isaac，见下）
grasp_planner.py        抓取规划（被 pick_object.py 和自动抓取状态机共用）
gripper_model.py        夹爪关节角 ↔ 开口的实测标定表

── scripts/ ──
run.sh / load_scene.sh / bringup.sh    启动与一键搭场景
pick_object.py          离线抓取（HTTP 驱动，不依赖 VR）
task_sequence.py        完整演示：走位 → 开回桌前 → 抓取
wasd_teleop.py          终端 WASD 遥控底盘
ros2_relay.py           ROS2 ↔ HTTP 中继（系统 python3.10 跑）
rm_driver_bridge.py     真机 rm_driver CANFD 桥
scene/                  场景构建/烘焙/相机注册等一次性脚本
```

**关于 `arm_kin.py`**：故意**不用** Isaac 的实时雅可比。PhysX 给的是绕**连杆质心**
的雅可比，而我们要控的是 TCP，两者差一个 `ω×(p_tcp − p_com)` 项（实测线速度行差
~5cm×ω）。FK 数值微分对任意参考点都精确。也别退回预计算的雅可比表——表只在 home
附近采样，远处位形线性外推完全失效（这是"一按离合机械臂就甩"的根因）。

**关于 TCP**：抓取规划必须用**闭合态**的 TCP。两指垫中点随开口沿接近轴漂移 ~11mm，
用张开态规划会让物体落在指垫外侧。调用方负责在目标闭合角下量出 `tcp_local`。

## 踩坑清单

都是实测撞过的，改代码前先看。

### 刚 clone 下来必须先改的东西

**`config.json` 和几个脚本里写死了 `/home/jy/...` 的绝对路径**，在别的机器上必然找不到：

- `config.json` → `robot.presets[*].usd_path` / `urdf_path`（4 处）
- `arm_kin.py` → `URDF` 默认值（可用 `arm_kin.set_urdf(path)` 覆盖；`load_urdf` 会自动调用）
- `scripts/pick_object.py` → `URDF` 默认值（有 `--urdf` 参数）
- `scripts/scene/*.py`、`scripts/build_deliver_humanoid.py`、`assets/RM65-6FB-V/make_*.py`

另外 **`.gitignore` 排除了 URDF 导入产物和超大网格**（`humanoid_upper_body_v3/usd/`、
`*/urdf/*/configuration/`、`meshes/visual/base_link.STL`）。clone 后这些不存在，
需要按 README 的 URDF 导入流程在本地重新生成。

### Isaac Sim 5.1 API（`robot_manager.py` 已适配，别改回去）

- `isaacsim.core.prims.Articulation` 第一参改名为 `prim_paths_expr` → 用**位置传参**兼容新旧。
- 相机 `set_focal_length(米)` / `set_clipping_range(near, far)` 是 setter，构造不再收这两个参数。
  **把 mm 数喂给米制 setter 会差 1000 倍**（110° 变 0.16°）。
- `set_local_pose` 参数名是 `translation`（不是 position），且多了 `camera_axes`
  （`"usd"` = 原始 USD 约定 [0,0,0] 朝 -Z，本项目用它）。
- view API 关节位置形状 `(M, K)`，单机器人 M=1，读写都要 reshape。
- **查 API 签名不用起 Kit**：直接 grep `site-packages/isaacsim/exts/isaacsim.*.*/impl/*.py`。

### pxr / USD

- `UsdPrim` **没有 `GetDescendants()`**，遍历后代用 `GetAllChildren()`。
- USD 禁止直接编辑 instanceable 原型，要写 over（见 `_hide_mount_visuals`）。

### 资产与相机

- Isaac 的 URDF 导入器**不解析 `package://` 形式的 mesh 引用**。手头只有这种 URDF 时，
  要么批量改写成绝对路径，要么直接用转换好的 USD 走 `load_robot`。
- `add_camera` 的 `name` 默认取 `prim_path` 末段 → 多个相机都叫 `camera` 会撞场景注册表，
  多相机场景**显式传 `name`**。
- `attach_to` 要求 link prim 存在，所以 URDF 导入要保持 `merge_fixed_joints: false`。
- **加相机 prim 会让物理视图永久失效** → `bringup.sh` 里"抬臂"必须排在加相机之前。
- 刚启动立即抓图可能返回空/黑图（渲染未收敛），稍后重试。

### 热修复（不丢场景）

```
POST /python  {"code": "import importlib, robot_manager; importlib.reload(robot_manager);
                       executor.__class__ = robot_manager.RobotManager"}
```

`/python` 传多行代码容易被 JSON 转义搞炸，**用 python urllib 写驱动脚本更稳**。

### 日志噪音（不是错误，别去"修"）

- `[Error] [py stderr]` 前缀的 uvicorn INFO 行 —— Kit 转发所致。
- `node_STL_BINARY_` 警告 —— 渲染端良性。

## memory/ 的用法（本项目的重要习惯）

`memory/` 是**仓库自带**的记忆目录，跟着 git 走（区别于 agent 工具自己的
`~/.claude/...` auto-memory）。`memory/MEMORY.md` 是索引，一行一条。

- **开工前先读 `memory/MEMORY.md`**。
- **有新发现 / 修好一个 bug，就更新对应文件并在索引里加一行。** 这是这个项目
  能积累的原因——踩过的坑不重复踩。
- 一条记忆 = 一个文件，写清**现象 → 根因 → 修法 → 验证数据**（现有文件都是这个结构）。

## 工作约定

- **不要猜规格/实测数据**。需要参数、标定值、真机规格时，列清单向用户要；
  万不得已用推定值必须显式标注。参见 `memory/ask-user-for-data-dont-guess.md`。
- **改动前先确认改的是哪个臂/哪套 URDF**。项目里有 7 轴旧臂、官方 RM65 六轴重建版、
  以及单独验证用的 `rm65_ctag2f120`，运动学和关节名都不一样（`*_wrist_joint` 有无）。
  `config.json` 的 `home_pose` 只对 7 轴旧臂标定过，六轴版会被整侧跳过。
- **写代码风格跟着周围走**：注释用中文，讲清"为什么"而不是"做了什么"（现有代码里
  大量注释记录的是实测数据与被否决的方案，别当成冗余删掉）。
- 改动物理/驱动参数（刚度、阻尼、摩擦）前，先读 memory 里对应那篇——好几个值
  是反复试错才定下来的，随手改回去会重现"推不动 / 抓不住 / 静止抖动"。
