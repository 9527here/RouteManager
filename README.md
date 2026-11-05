# RouteManager

**面向自研机器人的一站式工厂仿真配置、可达性验证与数据采集平台。**

以 **Isaac Sim 6.2 + Isaac Lab 3.0** 为底座，把真实机器人、工厂资产、任务验证、交互示教和批量采集接到一起，逐步实现 **Real → Sim → Real**。

```text
真实机器人与工厂任务 → 资产与参数配置 → 场景/控制接入 → 可达性与接触验证
        ↑                                             ↓
     真机反馈 ← 策略评估与部署 ← 训练 ← 示教与Isaac Lab批量采集
```

## 项目现状

当前重点是RM65六轴双臂轮式人形、CTAG2F120夹爪，以及电柜按钮、旋钮、压板和阀门。以下是已有记录的验收范围；本次文档整理未重新运行机器人。

| 方向 | 已有能力 | 仍需完成 |
|---|---|---|
| 场景与控制 | URDF/USD导入、预设、HTTP/ROS2、关节/夹爪/四轮八驱控制 | 统一机器路径与资产版本 |
| 可达性与操作 | FK/连续IK、全身几何与接触门控；固定台架三阀真实转动 | 移动到位、驻车和接触操作的连续验收 |
| 并行采集 | L3机构33任务，4环境×2回合66/66；另有RGB和合作停止验证 | 人形接触执行器与整机并行采集 |
| 多方资源 | Pico/XRoboToolkit、SimPeople单人巡检、SimScene静态方案接入 | 人物避让/物理交互和更多任务覆盖 |
| Real → Sim → Real | 已有控制语义和真机桥接基础 | 实物标定、策略训练/评估、真机回验闭环 |

**机构直接施力数据、人形真实接触数据、几何可达和真机验收分别标记。** 现有Lab数据为`fixture_effort`，不是人形示教；固定台架结果也不代表自由底盘操作已通过。

统一工作站、下把手旋钮/压板接触及新增采集相关代码仍在本机未提交研发区，需独立验收后发布。已发布运行能力沿用11-03基线`38a780b`，文档/资产接入在11-05更新；旧阀门RL参考尚未学会拧阀门。

## 接入资产并启动

环境统一为 **Isaac Sim 6.2 / Python 3.12、Isaac Lab 3.0**。本机解释器是`/home/jy/micromamba/envs/isaacsim6.0/bin/python`，目录沿用旧名，实际已升级为6.2。

### 1. 连接外部资产库

从项目根执行：

```bash
python3 scripts/link_asset_library.py link --root /home/jy/datasets/vla_data
python3 scripts/link_asset_library.py status
```

建立的`third_party/vla_assets`链接由Git忽略，模型保留原位。其它机器把`--root`换成自己的资产目录，或设置`ROUTE_MANAGER_ASSET_ROOT`。脚本不复制模型、不覆盖已有不同入口，也不改当前配置和USD引用；这不是Git submodule。

| 资产层级 | 用途 |
|---|---|
| L0原始档案 | CAD、扫描、厂商原件，保留源数据 |
| L1制作资产 | URDF/网格、载体设备和操作资产，核对依赖、单位及参数 |
| L2仿真运行 | USD场景/机器人，检查相对引用、材质和物理 |
| L3前端展示 | GLB外观，需另做关节、碰撞与操作语义适配 |

源码`assets/`保存小型参数、来源清单和构建说明；完整模型在外部库，采集与验收产物在本机`outputs/`，均不混入源码提交。外部库及工具版本登记在`third_party/libraries.json`，不是自动安装器。

### 2. 准备配置

新部署参照`config.example.json`准备`config.json`，填写`robot.presets`里的URDF/USD路径，核对地面高度、夹爪和关节命名；已有部署保留自己的配置。当前六轴RM65、旧七轴和单臂模型的姿态表不能混用。路径存在不等于该资产已完成物理适配或实物标定。

### 3. 启动服务并确认模型

```bash
./scripts/run.sh                    # GUI；先在这个终端保持运行

# 另一个终端：检查服务
curl -s http://127.0.0.1:18765/health

# 仅空会话执行一次；已有机器人则跳过
./scripts/load_scene.sh

# 读取实际关节，确认模型已加载
curl -s -X POST http://127.0.0.1:18765/execute \
  -H 'Content-Type: application/json' -d '{"action":"get_joint_names"}'
curl -s -X POST http://127.0.0.1:18765/execute \
  -H 'Content-Type: application/json' -d '{"action":"get_joint_positions"}'
```

`IDLE`表示服务空闲，是否已有机器人以关节反馈为准。同一会话只能加载一次机器人；任务忙时HTTP返回409，客户端等待重试。首次启动可能需1～3分钟。相机默认按需启用。

RouteManager无头启动用`HEADLESS=1 ./scripts/run.sh`；换环境用`ISAAC_PYTHON=/path/to/isaac/python ./scripts/run.sh`。脚本会隔离ROS Humble路径污染。**Lab 3.0**另需清除`DISPLAY`、省略可视化参数，不使用`--headless`。旧`bringup.sh`会重启并加载相机，一般任务使用上述入口。

## 常用任务

### 电柜交互

已有电柜/L3工位时重开面板：

```bash
python3 scripts/setup_cabinet_actions.py
```

空服务会按本机资产搭建，先核对路径；已有其它机器人场景时不能二次加载。选定对象后执行对应动作，区分“机构直接操作”和“人形接触”，以实际行程、转角、接触与退出结果判断完成。机构型号、档位语义、阻力等工程参考值仍待实测。

### 阀门可达性与接触验证

前提是已有匹配的RM65/CTAG2F120与固定台架阀门。先采现场反馈，再离线规划，每次使用新的输出目录：

```bash
python3 scripts/valve_operation.py capture --output outputs/valve/new_capture
python3 scripts/valve_operation.py plan \
  --snapshot outputs/valve/new_capture/snapshot.json \
  --output outputs/valve/new_plan
```

检查模型身份、真实轴、TCP与完整路径。下面预检/执行只用于已驻车的匹配台架；`--audit-static-assembly`审计既有不变装配重叠，保留碰撞检查，不代表全局无碰撞。

```bash
python3 scripts/valve_operation.py preflight --audit-static-assembly
python3 scripts/valve_operation.py run --audit-static-assembly --angle 90 \
  --output outputs/valve/new_run

# 另一个终端查看或合作停止同一运行
python3 scripts/valve_operation.py status --output outputs/valve/new_run
python3 scripts/valve_operation.py stop --output outputs/valve/new_run
```

运行器会重新采样并预检，确认稳定、双侧承载、实际转动、滑移和退出后才判成功。离线假设底盘平移/升降不会让机器人自动到位。走位前收臂并查全身净空，操作时核对位置、胸部朝向与停稳；纯横移保持`wz=0`。用户停止后不自动恢复，不能删除STOP文件强续失败动作。

## Isaac Lab批量采集

准备好L3机构USD、参考参数和Lab环境后，先用2～4环境验证，再扩大规模。批采使用独立进程，不接管桌面机器人：

```bash
./scripts/collect_l3_isaaclab.sh --num-envs 4 --episodes 2 \
  --tasks all --output outputs/l3_dataset/new_batch

# 需要图像时另建一批
./scripts/collect_l3_isaaclab.sh --num-envs 2 --episodes 1 \
  --tasks button --rgb --output outputs/l3_dataset/new_rgb_batch

/path/to/isaac/python scripts/validate_l3_dataset.py \
  outputs/l3_dataset/new_batch --require-all-tasks
```

15件机构对应33个任务，标记`execution_mode=fixture_effort`、`robot_contact_demonstration=false`。每批新建目录，保留成功、失败和中断；`STOP`支持合作停止并保存。

| 产物 | 检查内容 |
|---|---|
| `manifest.json` | 任务/模型身份、资产hash、单位、种子、初态/目标与执行模式 |
| `clone_validation.json` | 各环境固定根的位置一致性 |
| `episodes.h5` | 状态、控制、时间、阶段及终态 |
| `summary.json` / `validation.json` | 回合结果与独立行为/完整性验收 |
| RGB与相机参数 | 实际有效图像、内外参和状态时间对齐 |

控制240Hz、保存30Hz及额外终止帧；一条`q/qd/effort → next_q/next_qd`对应同一个240Hz子步，不是相邻30Hz保存记录。桌面新增接触动作不会改变旧数据的执行模式，画面中出现人形也不能重标为人形示教。


## 外部资源

| 资源 | 在本项目中的作用 |
|---|---|
| ROS2 | 优先用Isaac 6.2自带py312绑定的`setup_ros2_contract`；独立py3.10中继仅作兜底，两者互斥 |
| Pico / XRoboToolkit | 交互遥操作；先验证真实SDK输入，再验证映射、IK目标和实际末端 |
| SimPeople | 附着既有会话的原生单人巡检；已附着会话可用`python3 scripts/simpeople_patrol.py status`检查 |
| SimScene | 资产发现、缓存、测量和静态视觉层附着，禁用独立仿真worker；不替代可操作机构迁移 |
| Oxford机构 | 固定版本来源与迁移参数已登记；保持原始来源和许可 |

人物巡检不等于人体接触/机器人避让已通过。SimScene在暂停、无任务且场景身份/版本匹配时导入静态视觉方案，物理API、相机和活动灯会拒绝；当前姿态的布局检查不代表未来轨迹安全。

## 开发与排错

```text
核心Python模块 / ros2_bridge/  服务、控制与运动学
scripts/ / tests/             任务、转换、采集与离线回归
assets/ / third_party/        参数、来源与外部资产入口
docs/ / memory/               详细契约与有日期的工程记录
outputs/                      本机数据和验收证据，Git忽略
```

离线回归不启动Kit，缺少未分发模型输入时会明确跳过；输入齐备时可增加`ROUTE_MANAGER_REQUIRE_MODEL_TESTS=1`禁止跳过：

```bash
env -u PYTHONPATH -u LD_LIBRARY_PATH /path/to/isaac/python \
  -m unittest discover -s tests -p 'test_*.py'
```

启动/导入失败先查解释器和ROS路径；无动作先查409、STOP及预检结果；资产异常先查相对依赖、单位和材质。物理view失效时暂停检查，不继续读旧view或随意reset。离线回归通过后，动作/UI仍须在真实Kit中验反馈与取消路径，数据另做独立读回。

后续工作保留在[TODO](TODO.md)，按稳定ID记录验收条件。**项目现状和常用教程只维护本页。**

<details>
<summary>需要查接口、完整数据契约或历史证据时展开</summary>

- [专题参考与历史索引](docs/README.md)：接口、任务、集成及旧试验原文；普通使用无需先读。
- [源码资产与旧包恢复](assets/README.md)：旧资源包hash与恢复命令。
- [目录与文档维护约定](docs/项目目录与文件管理.md)：源码/产物边界与归档规则。
- [Agent工程约束](AGENTS.md)：初始化、主线程、控制与物理边界。

</details>

代码采用[MIT许可证](LICENSE)，第三方工具和资产遵循各自来源与许可。
