# SimPeople：现有场景中的单人巡检

本机2026-11-03已实测，仍用原RouteManager/Isaac Sim 6.2进程（PID 377760），没有启动独立仿真。
机器人是RM65六轴整机配CTAG2F120，世界固定底盘；人物由官方`rosclaw.simpeople` Kit扩展和原生Behavior控制。

## 当前怎么用

Isaac窗口有“人物巡检与阀门演示”中文面板。按钮支持开始/停止巡检、机器人原地拧阀门、总览、暂停场景。
阀门按钮使用工位2，成功后下次交替转向；失败只停住，不自动重试。
“暂停场景”会先写合作停止文件和取消人物任务，再暂停物理；“停止巡检”只影响人物。

等价命令（仓库根执行）：

```bash
python3 scripts/simpeople_patrol.py status
python3 scripts/simpeople_patrol.py start       # 默认100次往返，提交后控制端可退出
python3 scripts/simpeople_patrol.py stop
python3 scripts/simpeople_patrol.py demo
python3 scripts/simpeople_patrol.py pause
python3 scripts/simpeople_patrol.py view
python3 scripts/simpeople_patrol.py attach      # 重新显示面板、附着已测量覆盖层
```

`--url`为RouteManager地址，`--endpoint`为本机SimPeople桥地址，`--output`可改证据目录。
人物桥默认`127.0.0.1:8768`，RouteManager仍为`18765`。代码都通过主线程操作USD/物理。

## 安装与场景

- 外部源码：`/home/jy/simpeople`，固定本轮checkout提交；未把模型依赖装进Isaac Python。
- 控制环境：`/home/jy/simpeople/.controller-venv`（Python 3.10），安装`.[mcp,assets]`。
- 原生扩展110.4.0；缓存官方Default Human与HumanMotionLibrary，共199个文件，约470MB。
- 当前是官方默认灰色人形；未下载工服人物，不能把它称为已经有真实工服外观。
- 原运行现场只有阀门台架。旁侧加入已有电柜的静态引用，移除该副本的独立关节测试配置，保留完整保守柜体碰撞包络；源USD未写回。
- 人物只在电柜外侧A`[2.6,-2.1,0]`、B`[2.6,0.5,0]`之间行走，原生NavMesh已验证连通。尺寸、布局为本次场景实测/配置，不是硬件规格。
- 覆盖层、语义点、缓存清单、原始失败、录像、实测报告在`outputs/simpeople_patrol_20261103/`。

## 热加载初始化的实测坑

原台架文件`startTimeCode=-1,endTimeCode=0,timeCodesPerSecond=240`。
时间线会回绕；不能拿人物accepted或机器人World时间继续增长当动画正常。
在已经播放后暂停的场景才加载动画核心，会漏掉完整启动事件：人物对象、寻路、动画编译均可报告正常，实际仍T姿态、无有效位移。
本轮在原进程内作受控Stop/Play和状态恢复后，原生PLAYINFO开始输出Idle/Walk，随后实际走通；未重载机器人或另开仿真。
恢复明确属于仿真状态恢复，不计为人物行走或阀门运动。早期试验记录全部保留。

未来启动时可先预载原生运行时，避免在暂停现场才首次启用：

```bash
ROUTE_MANAGER_SIMPEOPLE_ROOT=/home/jy/simpeople ./scripts/run.sh
```

这仍是RouteManager原服务。`sim_env.py`在World首次Play之前预载动画/导航，在新场景OPENED时将零长度台架时间范围扩展到会话层；不改帧率或驱动参数。
随后加载已有验证场景，再`attach/start`。此冷启动预载分支仅完成语法检查，本轮没有为验它重启当前会话；不能称已验证冷启动重建。
本次`scene_overlay.usda`只含电柜和NavMesh，不包含原有阀门工位/机器人原生状态的完整重建，不能单独当成整场景启动文件。

运行期不要反复改TCPS/defaultPrim来试动画，会使物理后端失效。删除人物/卸载扩展也会触发项目旧Articulation删view问题。
本轮验证了只重绑定view后q、目标、速度目标、kp/kd/maxForce精确保留；不能因此对任意拓扑修改自动reset。

Kit fast importer还会让`importlib.reload`继续用旧模块。本入口对自己的无状态模块直接编译磁盘源码；不热替换整个RobotManager。

## 验收与限制

`acceptance.json`保留原生任务与轨迹：A/B到位误差分别4.55/8.24mm；移动中取消后速度0.02136m/s。
并行阀门成功三次，实转−92.071°、−89.318°、+90.863°，底盘平移均0；后两次实测底盘转角也为0。
最终96个机器人碰撞形状、有限反馈、有效view、全身速度目标0，已松爪退出，场景暂停。

重复阀门的失败包含一次IK拒绝（未运动）与两次接近段跟踪保护停止。入口现比较两个相差180°的对称夹持朝向，优先最接近当前实测关节姿态的解，避免选到翻肘/大幅绕腕分支；仍走原完整避碰与接触预检。
修后初段最大关节变化由5.977降至0.992rad；参考路径速度0.8rad/s，最终最大跟踪误差0.001010rad。原误差、接触、载荷和碰撞门限未放宽。

录像`patrol_valve_excerpt.mp4`是现有1280×720视口45秒连续片段，按实际墙钟时间戳编码，无新增相机传感器；片段不代替完整动作终态报告，也不证明实时运行容量。
人物动画时钟随Kit渲染更新，物理可有无渲染子步；未认证两者逐物理步同步、人体接触力、机器人避人、深度/LiDAR识别或训练标注。
