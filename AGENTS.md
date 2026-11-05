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

基于Isaac Sim 6.2的一站式工厂仿真配置、任务验证与数据采集平台，配合自研机器人、
多方资源和Isaac Lab并行环境，服务Real→Sim→Real。HTTP执行器是交互运行核心：

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

### 4. 底盘8个关节归 `set_base_velocity` 专管

当前整机是四轮八驱：4个 `caster_*_joint` 用位置目标转向，4个 `*_wheel_joint` 用速度目标滚动，统一由 `set_base_velocity(vx, wz, vy=0)` 驱动。**整机位置指令（ROS/HTTP/UDP）
里的这8个底盘分量会被忽略**，避免两路控制打架。超过 `base_teleop.timeout`（默认 0.5s）
收不到速度指令自动刹停。
用户于2026-11-02授权仿真参考舵角±90°，待实测；只做SessionLayer及runtime限位覆盖，不写回源URDF。原资产为±5°。几何/轴向从实际URDF读，不套旧skid-steer轮距。

**纯侧移不得用转车身代替**（2026-11-02用户明确纠正）：先真实停轮、四轮对齐90°滚动平面、稳定后从0斜坡起步，全程wz=0、锁定起始车身朝向；不再用±55°临时yaw。走位前验证并收纳双臂，主柜壳须有环境碰撞/可视包络。忽略可见性不能漏掉隐藏Cube/Cylinder碰撞体，当前应覆盖96形状；运行时必须读实际手部位姿与环境净空，不只平移冻结初态。

**资产操作须正对操作面**（2026-11-02后续明确选择）：胸部正面垂直朝柜门／面板，不能把右臂横向站位偏置当成身体看向控件中心。资产请求先在安全点独立对正，之后才锁定新朝向沿X/Y平移；手动左右横移仍全程wz=0。到位必须同时检查目标面朝向、位置、停稳，不能仅比起始yaw。原地旋转的轮向准备和全身/手臂弧线扫掠必须验证，不能直接大角打舵或只查三个角度。

**轮脚粗包络不能直接判碰**：轮胎圆柱的XY盒/整高扫掠圆会误挡3cm台脚。近障碍必须用当前全身真实3D形状窄相复核，转舵过程另作扫掠；当前姿态安全不等于能转到0。已验证可保当前轮向低速退出后再转舵，不得一律永久拒绝、忽略脚座或关闭保护。见memory/wheel-foot-envelope-recovery.md。

**受阻默认自动回退**（用户明确要求）：已收臂走位中仅转舵/通道受阻可先停稳，再验证沿当前轮向退出，最多一次，资产待办保留至到位执行；手动横移退离后须报未完成。用户停止后不得自行恢复。真实净空不足、姿态/反馈异常不得归类成可恢复受阻；禁止宽泛catch RuntimeError后自动移动。

**唤醒不能污染速度目标**：6.2高层`set_joint_velocities`同时写实际速度与驱动速度目标，不能把全身实测速度传进去当无副作用唤醒。底盘首动只用低层`set_dof_velocities`回写同一实际速度，目标须逐项保持。L3重置物理后恢复颈部参考阻尼；运行中补根PhysxArticulationAPI会使tensor后端失效，别用它临时试迭代。见memory/idle-jitter-swerve-20261102.md。

### 5. ROS2：使用与Isaac匹配的Python绑定

Isaac Sim 6.2是Python 3.12，系统ROS Humble的Python 3.10绑定不能直接混入。
**6.2自带兼容的rclpy和消息包**：优先使用`setup_ros2_contract`及
`ros2_bridge/contract_node.py`，回调由仿真主线程spin处理。
`run.sh`会前置Isaac自带ROS2库；系统同名库排前面可能导致create_node时SIGABRT。
详见[memory/ros2-inprocess-contract.md](memory/ros2-inprocess-contract.md)。

`scripts/ros2_sim_contract.py`是Isaac内绑定不可用时的独立py3.10中继兜底，
不能与进程内契约同时开。相机图像由Isaac内OmniGraph原生发布。

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

- **默认环境是 micromamba 的 `isaacsim6.0`** —— 目录名有误导性，这个 env 已于
  2026-10-20 就地升级到 **Isaac Sim 6.2.0.0**（python3.12）。`run.sh` 现在默认走它，
  `ISAAC_PYTHON` 可覆盖。
- `isaacsim5.1` **已不可用**：那次升级把它的 Kit 扩展缓存
  （`omni.isaac.ml_archive` 的 `pip_prebundle/torch`）覆盖成了 **py312** 的 C 扩展，
  而 5.1 是 py311 → `import torch` 失败 → `isaacsim.core.*` 全部加载不了，服务
  12 秒即退。要用 5.1 得先把那份 prebundle 换回 py311 版本。
- 端口 **18765**，`ROUTE_MANAGER_PORT` 可覆盖。
- **首次启动加载 Isaac 要 1~3 分钟**，热缓存约 8s。别以为卡死了就去杀进程。
- 改完代码验证不需要重启：`POST /python` 热替换 `RobotManager`（见踩坑清单）。

## 改了代码怎么验（没有测试框架）

项目有`tests/test_*.py`离线unittest回归，尚无CI。离线回归不代替Kit行为验收；现场验证的标准动作：

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
robot_manager.py        ★ 执行器：所有 action 的实现
standalone_server.py    HTTP 框架层（与 Isaac 无关，可整体拷到别的项目）
isaac_teleop.py         Pico VR 遥操作（VRTeleopManager，由 RobotManager 持有）
ros2_bridge/            6.2进程内ROS2契约与相机图；外部中继作兜底
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
ros2_sim_contract.py    ROS2 契约中继（系统 python3.10 跑）
ros2_bridge/contract_graph.py  相机 OmniGraph 构建（顶层不 import omni）
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

### Isaac Sim 版本差异（5.1 ↔ 6.2，`robot_manager.py` 两者都适配了）

**URDF 导入器是两套完全不同的 API**（`_import_urdf_to_usd` 按版本自动分派）：

| | 5.1 | 6.2 |
|---|---|---|
| 入口 | `from ... import _urdf` | `from ... import URDFImporter, URDFImporterConfig` |
| 流程 | `parse_urdf` + `import_robot` 两步 | `URDFImporter().import_urdf(cfg)` 一步 |
| 产物 | 单个 `.usd` 文件 | **分层目录**：`<dir>/<stem>/<stem>.usda` + `payloads/` |
| defaultPrim | 不带，要自己补写 | 带 |
| 变体 | 无 | 顶层带 `Physics` 变体集（none/physics/physx/mujoco），**必须选 `physx`** |
| mimic | `parse_mimic=False` 关掉 | **没有这个开关**，导入后剥 `NewtonMimicAPI` |
| `dest_path` | 文件路径 | **目录**；且不会覆盖，每次建 `<stem>_N`（每个 53 MB） |

- **mimic 必须关/剥**：本项目夹爪靠 `_apply_software_mimic` 每帧给跟随关节写位置目标。
  原生约束同时生效会互相顶 —— 6.2 上实测夹爪跑到 63 rad、PhysX 刷 Invalid transform、
  整个仿真发散成 NaN。5.1 用 `parse_mimic=False`，6.2 事后 `RemoveAPI("NewtonMimicAPI")`。
- **关节限位在 USD 里存的是「度」**（`physics:lowerLimit = 40.107` 对应 0.7 rad），
  这是 UsdPhysics 的约定；Isaac 建 articulation 时会转成弧度，运行时读到的仍是弧度。
  别拿 USD 层的数当弧度用。
- `GroundPlane.set_world_pose()` **在 6.2 上不生效**（设 6.222 读回 0）→ 直接写
  `xformOp:translate`（见 `_set_prim_translate_z`）。
- 同一会话**不能二次加载机器人**：6.2 抛 `scene ... name is not unique`，5.1 直接
  abort 把进程带死。`_reject_if_robot_loaded` 提前拦成一句人话。

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
- **静态相机默认不注册**（`config.json` → `cameras.auto_register: false`，2026-10-21
  用户要求"以后都不要加载相机"）。每路相机都要建 render product，`initialize()` 那几路
  实打实吃 GPU，而背景 NuRec 场景本来就把 GPU 打满。要出图时打开这个开关，或用
  `add_camera` 临时加一路。别在调试脚本里随手加探针相机。
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

- **插件面向用户的界面统一中文**：窗口名、按钮、状态反馈和使用说明均用中文；已有代码接口名保留兼容。

- **不要猜规格/实测数据**。需要参数、标定值、真机规格时，列清单向用户要；
  万不得已用推定值必须显式标注。参见 `memory/ask-user-for-data-dont-guess.md`。
- **改动前先确认改的是哪个臂/哪套 URDF**。项目里有 7 轴旧臂、官方 RM65 六轴重建版、
  以及单独验证用的 `rm65_ctag2f120`，运动学和关节名都不一样（`*_wrist_joint` 有无）。
  `config.json` 的 `home_pose` 只对 7 轴旧臂标定过，六轴版会被整侧跳过。
- **写代码风格跟着周围走**：注释用中文，讲清"为什么"而不是"做了什么"（现有代码里
  大量注释记录的是实测数据与被否决的方案，别当成冗余删掉）。
- 改动物理/驱动参数（刚度、阻尼、摩擦）前，先读 memory 里对应那篇——好几个值
  是反复试错才定下来的，随手改回去会重现"推不动 / 抓不住 / 静止抖动"。

## 源码与本机产物（2026-11-03整理）

- 目录与资产管理见`docs/项目目录与文件管理.md`、`assets/README.md`。
- `outputs/`保持本机原位且不上传；不要清理活动USD引用和失败证据。
- 旧`humanoid_upper_body_v3`完整包外置，原路径为本机软链接；不要把目标目录重新加进Git。
- 17个离线回归模块集中到`tests/`；`scripts/validate_*.py`仍可能是原生验收/数据入口，先读源码再运行。
- 代码、参数/来源清单与文档可提交；原CAD、GLB、转换缓存、纹理、录像与数据集按各自来源留在外部。

## 现状与文档维护

- `TODO.md`统一记录项目现状、待办ID、验收条件和证据；功能/缺口变化时同步更新，区分已发布与本机未提交研发。
- 常用操作集中到`docs/README.md`使用手册；专题保留详细契约和历史日期，不在多处复制路线图。
- 用户资产库通过`third_party/vla_assets`本机忽略链接访问，接入脚本不自动改配置或源文件；库登记见`third_party/libraries.json`。
