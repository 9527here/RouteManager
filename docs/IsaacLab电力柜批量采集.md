# Isaac Lab 电力柜机构批量采集

中文窗口“电力柜动作控制”的下半部分提供任务、每动作回合数、并行环境数、输出父目录、彩色图像开关，以及“开始批量采集”“停止采集并保存”。窗口内容可向下滚动。每次启动创建新目录，不覆盖已有数据。采集使用独立 Isaac Lab 进程，重置采集环境不会重置桌面机器人。

## 已实现的范围

当前批量数据为 **15 件 L3 机构的广义力控制数据**：4 个按钮各按下/释放，3 个两档旋钮各两个目标档，3 个三档旋钮各三个目标档，5 个压板各投出/投入，共 **33 个任务配置**。“每动作回合数=10”会生成330回合，由所选并行环境分批执行。

按钮按压力、旋钮扭矩、压板旋松/拨动都由物理求解得到实际运动；运行期间不直接写关节位置。reset时设置任务初态：释放从按下开始、旋钮从其他档开始、投入从已投出的压板开始。压板沿用等效螺旋副与解锁联锁。参数为参考规格，待实测。

数据标记 `execution_mode=fixture_effort`、`robot_contact_demonstration=false`。**批量环境尚未包含人形手臂接触执行器，不能直接当作人形关节示教数据。** 人形真实接触仍是桌面插件内的绿色按钮流程，待新机器人URDF后再接同一任务契约。旧动力柜和Oxford动作已接中文面板，本轮批采清单只包含L3，范围见[动作与采集范围](电力柜动作与采集范围.md)。

## 命令行

项目根目录执行，默认使用 `/home/jy/micromamba/envs/isaacsim6.0/bin/python`（实际Isaac Sim 6.2、Isaac Lab 3.0）：

```bash
# 4个环境；每个任务2回合，共66回合
./scripts/collect_l3_isaaclab.sh --num-envs 4 --episodes 2 \
  --tasks all --output outputs/l3_dataset/新批次

# 按钮任务，保存每环境640×480同步彩色图像
./scripts/collect_l3_isaaclab.sh --num-envs 2 --episodes 1 \
  --tasks button --rgb --output outputs/l3_dataset/按钮图像新批次

# 独立验收（不启动Kit、不移动桌面机器人）
/home/jy/micromamba/envs/isaacsim6.0/bin/python scripts/validate_l3_dataset.py \
  outputs/l3_dataset/新批次 --require-all-tasks
```

`--tasks` 为 `all/button/selector/plate`；`--seed` 默认42。按压力/扭矩有0.9–1.1倍回合随机化并记录实际系数；不是实物参数随机标定。不要复用已有HDF5的输出目录。先从2–4环境验证本机资源，再增加并行数。`--rgb`会启用独立采集进程的相机；不会额外注册桌面机器人相机。

启动脚本隔离 `DISPLAY/PYTHONPATH/LD_LIBRARY_PATH/VIRTUAL_ENV`；不使用已删除的 `--headless` 参数。

## 文件与数据契约

| 文件 | 内容 |
|---|---|
| `manifest.json` | 任务中文指令、初态/目标、资产源SHA256、参考参数、单位、种子、频率和执行方式 |
| `clone_validation.json` | 每资产固定根相对环境原点的位置误差，必须通过才能采集 |
| `episodes.h5` | `/episodes/000000` 起的逐回合轨迹；属性 `metadata_json` 含任务、关节顺序、初态反馈、成功、超时、随机力系数 |
| `progress.json` | 原子更新的中文进度，供插件读取 |
| `summary.json` | 回合结果汇总，含失败回合；不把未达标回合伪装成功 |
| `validation.json` | 独立读取HDF5后生成的数据完整性与行为判据验收 |
| `launcher.log` | 从插件启动时的独立进程日志 |

每条记录有 `time/q/qd/effort/spring_target/next_time/next_q/next_qd/feedback/phase`。关节位移单位为米或弧度；`effort`对应牛顿或牛顿米；`spring_target`是弹簧、卡位和等效螺旋副目标，不是人形动作。

物理与控制为240Hz，保存30Hz抽样和额外终止帧。每条 `q, qd, effort → next_q, next_qd` 对应**同一个240Hz物理子步**，`next_time-time=1/240`；两条保存记录之间通常隔8子步，因此 `next_q[t]` 不保证等于下一条记录的 `q[t+1]`。此契约可用于机构动力学建模；训练需要连续30Hz动作序列时，应另行聚合控制子步。

开启图像后，每条包含 `rgb`（uint8）和 `rgb_time=next_time`。渲染时不推进物理；`/camera`保存各环境相机内参、世界位置和xyzw四元数，约定世界相机朝+X、上方+Z。图像是整块15件资产面板视图。

`feedback`各列定义：

| 类型 | 列顺序 |
|---|---|
| 按钮 | 行程m、已触发、指示灯亮 |
| 旋钮 | 角度rad、最近档索引、已到位 |
| 压板 | 上旋松圈数、下旋松圈数、摆角rad、已解锁、已投出、已投入锁紧 |

成功判据由实际反馈决定：按钮越过2.6mm触发点或释放到0.2mm内，旋钮进入目标档3°范围并保持，压板完成解锁与90°投出或拨回旋紧。独立验收另查初态确有区别、末段保持、有限状态和可选图像有效性。仿真指示不等于实物电气通断。

## 停止与异常

点击“停止采集并保存”写入该批目录的 `STOP` 文件，采集循环逐物理步检查；保留已采集回合，当前未完成回合标为 `interrupted=true, success=false`，关闭文件后退出。启动阶段的停止会在环境初始化完成后生效。停止采集不等于“停止当前动作”，两者控制不同进程。

正常完成/主动停止写 `episodes.h5`。异常退出保留 `episodes.partial.h5` 供排查，插件显示日志位置；未通过验收的数据不要混入正式样本。程序遇非有限物理反馈立即停止。未达标回合保留并标失败，完整批次有失败时退出码2，程序错误退出码1。

## 实现要点

使用Isaac Lab `InteractiveScene`批量Articulation与显式reset/step采集，不是强化学习训练任务。每环境独立碰撞分组；L3原资产固定关节的body0指向非刚体Xform，在克隆后会形成错误世界锚点，因此采集场景逐环境覆盖固定锚点并关闭物理schema复制，资产源文件不改。

`launch_simulation`接收 `SimpleNamespace(sim=..., scene=...)`。当前Lab3的配置扫描器只递归对象属性，普通dict会跳过相机，导致启动错误的非渲染Kit配置并引发SyntheticData图错误。图像输出需检查实帧，不以“无异常”代替验收。

自碰撞工作按用户要求暂缓；本批机构数据不声称通过人形避碰验收。

## 2026-11-02 验收

- `outputs/l3_dataset/all_actions_04/validation.json`：4环境、33任务各2回合，共66/66通过；独立检查实际终态与末段保持，固定根最大位置误差约0.00017mm。
- `outputs/l3_dataset/rgb_buttons_07/validation.json`：2环境、按钮8/8通过，224帧640×480彩色图像，逐帧RGB均值201.6–202.3，内外参完整，目视确认面板15件模型正常。
- 从插件真实启动采集后点击停止，`outputs/l3_dataset/ui_collection_started.json` 记录子进程与目录；停止后保存14回合，其中12完成、2标为中断，`progress.json`为cancelled。
- 早期固定锚点/相机启动排查产生的试采不合格，收在`outputs/l3_dataset/failed_trials/`，不能用于训练。
