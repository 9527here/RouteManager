# RouteManager使用手册

[项目主页](../README.md) · [项目现状与TODO](../TODO.md)

本页按“接资产→起服务→验任务→采数据”组织，常用步骤直接在这里阅读。专题页保留详细接口、参数和实测证据，集中在页末查阅。

## 本页目录

1. [环境与资产](#环境与资产)
2. [启动和检查](#启动和检查)
3. [任务与机器人验证](#任务与机器人验证)
4. [数据与批量环境](#数据与批量环境)
5. [资源与外部集成](#资源与外部集成)
6. [排错与开发验证](#排错与开发验证)
7. [专题参考](#专题参考)

## 环境与资产

运行底座统一为Isaac Sim **6.2 / Python 3.12**，批量环境使用配套Isaac Lab **3.0**。本机解释器目录虽叫`isaacsim6.0`，安装内容已升级为6.2。

HTTP服务依赖FastAPI/uvicorn，离线运动学用NumPy/SciPy；相机、ROS2、VR和外部工具按实际任务配置。不要用通用`pip install -U`覆盖Isaac配套依赖。

### 连接资产库

```bash
# 本机已使用这个资产库；新克隆需要执行一次
python3 scripts/link_asset_library.py link --root /home/jy/datasets/vla_data
python3 scripts/link_asset_library.py status

# 其它机器可用自己的目录，也可通过环境变量指定
ROUTE_MANAGER_ASSET_ROOT=/data/vla_data python3 scripts/link_asset_library.py link
```

`third_party/vla_assets`是本机目录链接，Git只保存接入脚本与库登记。链接不会修改库内文件，也不会替换当前`config.json`或活动USD引用。该目录目前没有独立Git仓库，尚未使用submodule。

资产库沿用L0原始档案、L1制作资产、L2仿真运行、L3前端展示分层。**L3展示网格不自动具备关节、碰撞或可操作语义。** 新接机构先核对单位、坐标、尺寸、部件、转轴、行程、抓取区域及物理参数来源，再做任务验收。

新部署参照`config.example.json`准备`config.json`，替换`robot.presets`中的URDF/USD路径。已有部署保留自己的配置。当前六轴RM65整机、旧七轴模型和单臂模型的关节名、运动学、姿态表不能混用；先确认实际加载对象。

外部库的目录、版本和接入约定集中在[外部库说明](../third_party/README.md)。源码内的小型参数与来源文件继续放`assets/`，场景输出和数据放`outputs/`。

## 启动和检查

### 启动一次服务

```bash
./scripts/run.sh                         # GUI
HEADLESS=1 ./scripts/run.sh              # RouteManager无头模式
ISAAC_PYTHON=/path/to/isaac/python ./scripts/run.sh
```

三个命令是可选启动方式，不要依次启动三次。`run.sh`隔离ROS Humble污染并前置Isaac自带ROS2库；首次加载可能需1～3分钟。

RouteManager的`HEADLESS=1`与Isaac Lab不同：Lab 3.0清除`DISPLAY`，省略可视化参数启动，不使用已删除的`--headless`参数。

### 空会话加载模型

```bash
curl -s http://127.0.0.1:18765/health

# IDLE表示服务空闲；已有机器人则跳过加载，直接读关节
./scripts/load_scene.sh
# 或按自己的预设加载：./scripts/load_scene.sh -n <预设名>
# 或给出文件：./scripts/load_scene.sh -a /path/to/robot.urdf

curl -s -X POST http://127.0.0.1:18765/execute \
  -H 'Content-Type: application/json' -d '{"action":"get_joint_names"}'
curl -s -X POST http://127.0.0.1:18765/execute \
  -H 'Content-Type: application/json' -d '{"action":"get_joint_positions"}'
```

服务只允许一个任务，忙时返回409，客户端需等待重试。`IDLE`只证明服务空闲，机器人是否已加载要读实际关节反馈。同一会话不能二次加载机器人。

静态相机默认不自动注册，需要图像时按任务增加。`bringup.sh`是旧VR演示流程，会重启服务和加载相机；一般任务使用上述入口。

## 任务与机器人验证

### 教程一：从现场反馈做可达性规划

前提：已有匹配的RM65/CTAG2F120模型和阀门工位。下面先读取当前反馈，再离线规划，不下发运动。

```bash
python3 scripts/valve_operation.py --help
python3 scripts/valve_operation.py capture --output outputs/valve/new_capture
python3 scripts/valve_operation.py plan \
  --snapshot outputs/valve/new_capture/snapshot.json \
  --output outputs/valve/new_plan
```

每次使用新输出目录。检查计划的模型、真实轴、夹爪TCP、路径和碰撞报告；几何可达只是一项条件。实际执行还需要当前全身几何、臂关节跟踪、夹持载荷、底盘驻车和退出门控。

固定台架是当前阀门接触验证条件。离线假设底盘平移/升降不表示机器人已经移动到位；自由底盘“走到阀门→驻车→转动”的连续流程仍需单独验收。

### 教程二：已有电柜场景打开动作面板

```bash
python3 scripts/setup_cabinet_actions.py --help
python3 scripts/setup_cabinet_actions.py
```

已有工位时重开面板；空服务按本机资产搭建前先检查路径。选择具体对象，再使用与该对象匹配的动作。区分机构直接施力/施矩和机械臂真实接触，只有实际反馈达到行程或转角、接触和退出要求才算完成。

按钮、旋钮、压板的型号、档位含义、按压力/扭矩与实物标定仍有缺口；工程参考必须保留标签。不要把外观相似认定为相同电气功能。

### 走位与停止

四轮八驱底盘由`set_base_velocity(vx, wz, vy)`独立管理。全身位置指令忽略这8个底盘分量；速度指令超时会自动停轮。

工位操作要同时检查位置、胸部朝向和实际停稳。纯横移保持`wz=0`，先停轮对舵再起步；走位前收臂并验证全身/手部净空。用户停止后不自动恢复。

台架、可移动场景、直接状态恢复有不同验证含义。停止后先查反馈及任务记录，不能删除STOP文件把失败动作当作正常续跑。

## 数据与批量环境

### 教程三：采集一个小批次并独立验收

前提：本机L3机构USD、参考参数和Isaac Lab已准备。采集独立于桌面机器人，先用2～4个环境检查资源和输出，再增加规模。

```bash
./scripts/collect_l3_isaaclab.sh --num-envs 4 --episodes 2 \
  --tasks all --output outputs/l3_dataset/new_batch

# 需要图像时单独创建一批
./scripts/collect_l3_isaaclab.sh --num-envs 2 --episodes 1 \
  --tasks button --rgb --output outputs/l3_dataset/new_rgb_batch

/path/to/isaac/python scripts/validate_l3_dataset.py \
  outputs/l3_dataset/new_batch --require-all-tasks
```

当前15件L3机构对应33个任务配置，数据标记`execution_mode=fixture_effort`、`robot_contact_demonstration=false`。这批是机构广义力控制数据，尚未包含人形并行接触执行器。桌面新增接触动作不改变旧批次的执行方式。

每批至少检查：

| 输出 | 验收内容 |
|---|---|
| `manifest.json` | 任务、模型/资产hash、单位、初态/目标、种子与执行模式 |
| `clone_validation.json` | 机构固定根相对各环境原点的位置一致性 |
| `episodes.h5` | 实际状态、控制、时间、阶段、终态及成功/失败标记 |
| `summary.json`、`validation.json` | 回合统计、独立行为和完整性验收 |
| RGB及相机参数 | 图像实际有效、相机约定与状态时间戳一致 |

控制240Hz、保存30Hz及额外终止帧。每条`q/qd/effort → next_q/next_qd`对应同一个240Hz子步；不能把相邻30Hz保存记录误当作连续控制子步。`STOP`支持合作停止，已完成和中断回合分别保存。

需要字段定义和历史验收数据时查页末的数据契约。发布前按执行模式区分机构数据、人形示教和真机数据，不能仅凭画面中有人形就改变标签。

## 资源与外部集成

| 资源 | 接入方式与使用范围 |
|---|---|
| 本机资产库 | `third_party/vla_assets`链接L0～L3及数据归档；当前预设不自动改写 |
| ROS2 | 优先用Isaac 6.2自带py312 rclpy的`setup_ros2_contract`；独立py3.10中继为兜底，两者互斥 |
| Pico / XRoboToolkit | 先确认真实SDK位姿、时间戳和输入变化，再检查映射、IK目标与实际末端 |
| SimPeople | 原生人物附着既有场景，现有单人两点巡检；不另起仿真 |
| SimScene | 发现/缓存/测量资产、保存方案、附着静态视觉层；不启独立worker |

已有SimPeople附着会话可只读检查：

```bash
python3 scripts/simpeople_patrol.py status
```

原生步行动画的时间与Kit更新有关；人物巡检通过不等于人体物理接触、机器人避让或传感器识别已通过。

SimScene方案先测量和预览，再在暂停且无任务时按场景身份/版本附着。当前只允许静态视觉资产，不带物理API、相机和活动灯；侵入作业区会拒绝。视觉导入不能替代可操作机构迁移。

## 排错与开发验证

| 现象 | 先查什么 |
|---|---|
| 启动慢或服务不可达 | 首次加载窗口、18765监听与运行日志；避免反复启动 |
| Python/ROS导入失败 | 是否使用6.2解释器，ROS路径是否污染py312运行时 |
| 机器人/相机不存在 | 实际模型身份、prim路径和关节反馈；不依赖旧截图 |
| 点击后无动作 | 任务忙、预检/停稳失败、STOP或当前对象不支持；读最终结果 |
| 物理view失效 | 暂停并检查Tensor生命周期；不继续读旧view或随意reset |
| 资产不显示/缺贴图 | 根文件与相对依赖、单位、轴向、材质和实际可见性 |

```bash
# 离线回归：不启动Kit；缺少未分发模型输入时明确skip
env -u PYTHONPATH -u LD_LIBRARY_PATH /path/to/isaac/python \
  -m unittest discover -s tests -p 'test_*.py'

# 模型和现场输入齐备时禁止跳过
env -u PYTHONPATH -u LD_LIBRARY_PATH ROUTE_MANAGER_REQUIRE_MODEL_TESTS=1 \
  /path/to/isaac/python -m unittest discover -s tests -p 'test_*.py'

# 仅验证HTTP框架的独立假执行器，避免与正式18765服务同时运行
python3 standalone_server.py --demo
```

通过离线回归后，运动/接触/UI变更仍要在真实Kit里验实际数值和取消路径；数据变更独立读回文件验收。完整真机验证需要对应实物参数和硬件现场。

## 专题参考

以下按需展开。旧记录中的“当前场景”“尚未完成”属于其记录日期；项目统一现状以TODO为准。

<details>
<summary>接口、资产与维护</summary>

- [HTTP接口](HTTP接口.md)、[ROS2话题与控制契约](ROS2接口_iasscsim_robot.md)
- [外部库接入](../third_party/README.md)、[源码资产与旧包恢复](../assets/README.md)
- [目录与文档维护约定](项目目录与文件管理.md)、[离线测试输入与严格模式](../tests/README.md)
- [历史模型相机清单](相机清单_humanoid_upper_body_v3.md)

</details>

<details>
<summary>任务操作、物理边界与数据契约</summary>

- [任务需求](电力柜任务需求.md)、[动作与采集范围](电力柜动作与采集范围.md)
- [电柜动作面板](电力柜中文动作插件.md)、[机构与人形场景](电柜资产清单与人形场景.md)
- [阀门执行器使用与门控](阀门手轮操作.md)、[三类阀门验证](其他阀门操作验证.md)
- [四轮八驱](四轮八驱底盘控制.md)、[工位接近](电力柜资产前方走位.md)
- [Isaac Lab数据契约](IsaacLab电力柜批量采集.md)、[L3参考机构参数与验收](L3_电气机构物理改造与验收.md)
- [VR输入与遥操作](../scripts/teleop/README.md)、[SimPeople巡检](SimPeople_场景附着与单人巡检.md)、[SimScene静态附着](SimScene_资产工具与当前场景接入.md)

</details>

<details>
<summary>历史实测、资产候选与研究归档</summary>

- [动力柜操作记录](动力柜资产操作验证.md)、[早期人形按钮/旋钮接触](人形操作电柜_接触验证.md)、[机构面板演示](操作面板场景演示.md)
- [纯侧移净空修正](纯侧移与柜体净空修正.md)、[轮脚包络与退出](轮脚包络拦截与安全退出.md)
- [L3外观候选](L3_前端展示资产可用性.md)、[USD001动作候选](USD001_新资产与电力柜动作对应.md)、[名称与功能待确认项](资产实际名称与功能待确认.md)
- [线上来源候选](线上操作资产候选_2026-10-30.md)、[Oxford/CAD迁移记录](线上操作资产迁移验收_2026-10-30.md)
- [早期阀门IK交接](交接_阀门IK_2026-10-29.md)、[未成功策略的阀门RL参考](../valve_rl/README.md)
- [项目工程记忆索引](../memory/MEMORY.md)

</details>
