# ROS humble 污染 Isaac 的 py3.12 运行时（2026-10-26）

启动时 `isaacsim.robot_motion.pink` 扩展必然失败：

```
Failed to import python module isaacsim.robot_motion.pink
ModuleNotFoundError: No module named 'pinocchio.pinocchio_pywrap_default'
  File "/opt/ros/humble/lib/python3.10/site-packages/pinocchio/__init__.py", line 19
```

**不是缺 pinocchio**——环境里本来就有 py312 的 pinocchio 3.9.0 + eigenpy 3.12.0
（`site-packages/cmeel.prefix/lib/python3.12/site-packages/`）。根因是 `~/.bashrc`
里 `source /opt/ros/humble/setup.bash` 带进来的两份 py3.10 产物：

1. **PYTHONPATH** 前置 `/opt/ros/humble/lib/python3.10/site-packages`。py3.12 的
   Isaac 先命中那里的 pinocchio 包，而包里只有 `pinocchio_pywrap_default.cpython-310-*.so`，
   于是 `__init__.py` 能进、`from .pinocchio_pywrap_default import *` 失败。
   `isaacsim.ros2.core` 还会按 `AMENT_PREFIX_PATH ∩ OLD_PYTHONPATH` 把这类路径
   重新 append 回 `sys.path`（`ros2_common.py`），所以光清 `sys.path` 不够。
2. **LD_LIBRARY_PATH** 里的 `/opt/ros/humble/lib/x86_64-linux-gnu`。该目录下的
   `libeigenpy.so` / `libcoal.so` 是 py3.10+numpy1 的，会顶掉 cmeel 那份：
   `undefined symbol: EIGENPY_ARRAY_APIPyArray_RUNTIME_VERSION`。
   这一层只有在 sys.path 清干净后才会暴露——先修第一层看到的就是它。

## 修法

`scripts/run.sh`（shell 层）+ `main.py`（进程内兜底，覆盖不走 run.sh 的
`python main.py`）：

- PYTHONPATH：摘掉所有 `/opt/ros/*`
- LD_LIBRARY_PATH：**只摘** `/opt/ros/*/lib/*-linux-gnu`
- `LD_LIBRARY_PATH` 只在进程启动时被 ld.so 读取，进程内改 `os.environ` 无效，
  所以 main.py 检测到污染后清掉该变量再 `os.execve` 重启解释器
  （`ROUTE_MANAGER_ENV_CLEANED=1` 防循环；必须放在 `main()` 里，import 时不触发）

**绝不能把 `/opt/ros/humble/lib` 一起摘**：环境里有 `ROS_DISTRO` 时
`isaacsim.ros2.core` 走系统 ROS 库（`/exts/isaacsim.ros2.bridge/internal_lib_fallback`
不打开），整库隔离会 `ROS2 Bridge startup failed`，相机/关节话题全废（2026-10-26
实测复现）。真正的冲突只有 `x86_64-linux-gnu` 那个 arch 目录。

中继进程（`scripts/start_ros2_contract.sh`、py3.10 + 系统 rclpy）自己
`source /opt/ros/humble/setup.bash`，不受影响。

## 验证

污染环境下 headless 直接 `python main.py`，日志应有：

```
[WARNING] route_manager.main: LD_LIBRARY_PATH 里有 /opt/ros 的 arch 库目录 ... 重启解释器
[INFO] route_manager.sim_env: SimulationApp 初始化完成
```

且**没有** `robot_motion.pink` 报错、**没有** `ROS2 Bridge startup failed`。
剩下的 `[Error]` 只有 inotify 上限和 uvicorn 把 INFO 写 stderr（Kit 一律标 Error），属正常噪声。