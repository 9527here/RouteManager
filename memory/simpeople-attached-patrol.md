# SimPeople附着巡检（2026-11-03）

现象：固定底盘阀门现场热加载Behavior后，预检、agent与路径都正常，实际T姿态不走。
根因：原台架时间范围-1~0帧会回绕；扩展在既有暂停会话中加载只得到Resume，没有完整动画Start。
实测修法：原进程内受控Stop/Play并恢复原生机器人/阀门状态后才开始输出PLAYINFO Idle/Walk。
不要改帧率/defaultPrim试运气，可能销毁Tensor后端；不重载机器人，不用根平移/骨骼写入伪造行走。

现场PID377760、Isaac6.2 rc14/Behavior110.4；官方Kit附着扩展8768，RouteManager18765。
源码`/home/jy/simpeople`、控制venv`.controller-venv`；官方199文件缓存约470MB，Isaac env未装usd-core/MCP。
默认灰色人形，非工服资产。原台架旁引用已有电柜静态副本；巡检A[2.6,-2.1,0]/B[2.6,.5,0]。
原生任务2331个范围内样本，B/A实际到位误差8.24/4.55mm；取消停稳0.02136m/s。
三次并行拧阀门成功−92.071/−89.318/+90.863度，底盘平移0；最终96形状/view有效/有限反馈/速度目标0，已松爪退出并暂停。

重复演示曾因首个IK可行分支大幅翻肘/绕腕，在接近阶段触发0.12rad跟踪门；放慢仍不能替代选解。
入口现枚举两个180度对称夹持朝向，选初段max关节变化最小者，再做原完整避碰/接触预检。
5.977→0.992rad，参考路径0.8rad/s，最终最大跟踪0.001010rad；门限/驱动增益未放宽，失败目录保留。
Kit fast importer可令importlib.reload仍读旧代码；对本轮无状态入口直接compile磁盘源码可靠，不部署整个RobotManager。

人物despawn/扩展unload后会删旧art view，只重新initialize绑定，实测q/目标/速度目标/kp/kd/maxforce保持。
入口`scripts/simpeople_patrol.py`，中文面板由`simpeople_scene.py`创建；pause先合作STOP与取消人物，随后暂停。
未来可用ROUTE_MANAGER_SIMPEOPLE_ROOT启动前预载，cold路径仅语法检查，未重启验收；本次overlay不是整场景重建文件。
轨迹/失败/45秒视口片段/最终验收见`outputs/simpeople_patrol_20261103/acceptance.json`，详见`docs/SimPeople_场景附着与单人巡检.md`。
未认证人体接触、避人、传感器识别、逐物理步时钟同步或实时性能。
