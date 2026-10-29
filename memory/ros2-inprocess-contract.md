# 把 ROS2 契约从中继改成**进程内**（2026-10-26）

## 起因

`scripts/ros2_sim_contract.py`（py3.10 中继 + HTTP）的工作前提是「Isaac 是
py3.12、Humble 的 rclpy 是 py3.10，进程内不能混」。**这个前提对 Isaac 6.2 是错的**。

`/home/jy/VLA/simManager/core/robot_controller.py` 里直接在 Isaac 进程内
`rclpy.create_node()` —— 因为 Isaac 6.2 的 `isaacsim.ros2.core` 扩展打包了
**py3.12 版**的 rclpy 和全套消息包：

```
isaacsim/exts/isaacsim.ros2.core/humble/rclpy/{rclpy,sensor_msgs,std_msgs,...}
isaacsim/exts/isaacsim.ros2.core/humble/lib/librcl_action.so
```

## 怎么用起来（实测）

```python
sys.path.insert(0, ".../isaacsim.ros2.core/humble/rclpy")   # rclpy + 消息包都在这
import rclpy                                                # OK
```

- **`LD_LIBRARY_PATH` 必须把 `isaacsim.ros2.core/<distro>/lib` 放在前面**（2026-10-27
  实测，之前这条写反了）。系统 humble 的 `librcl*/librosidl*/lib*__rosidl_typesupport*`
  与扩展自带的那套**不是同一次构建**，谁在前用谁。顺序反了 `import rclpy` 还能过，
  但 `create_node()` 在建节点那一刻 SIGABRT：

  ```
  Node.__init__ → TimeSource.attach_node → declare_parameter('use_sim_time')
    → _set_parameters_atomically → 发 parameter_events
      r_cl_interfaces__msg__parameter_event__convert_from_py:
      Assertion `PyUnicode_IS_READY(name_attr)' failed.        (exit 134)
  ```

  放太后面（在 `/opt/ros/humble/lib` 之后）更早死：`librcl_logging_spdlog.so:
  undefined symbol ...fmt::v8...`。`RMW_IMPLEMENTATION` 设不设都无关。
  `run.sh` 已自动前置；`ext/humble/lib` 里 375 个库全是 ROS 专用，无
  libstdc++/libpython，前置不影响 Isaac 本身。

  排查手法：崩溃报告器会写一份 py-spy 栈到
  `isaacsim/kit/data/Kit/Isaac-Sim Python/6.2/<dumpid>.py.txt`，直接看真实 Python 栈比读日志快。
  另外 `setup_ros2_environment()` 在 Linux 上是**空函数**，所以能用
  `env ROS_DISTRO=humble LD_LIBRARY_PATH=... python -I probe.py` 脱离 Kit 秒级复现，
  不必每次重启 Isaac 两分钟。
- **必须清 `sys.modules` 里的 rclpy**：第一次 `import rclpy` 会命中
  `/opt/ros/humble/lib/python3.10/site-packages/rclpy`，报
  `No module named 'rclpy._rclpy_pybind11'`，但坏模块会留在缓存里 —— 不删掉
  再插路径重新 import 的话，第二次还是命中坏的那个。

## 实现

新文件 `ros2_bridge/contract_node.py`：

- `ensure_ros_python_path()` —— 幂等地把 Isaac 自带 rclpy 路径塞进 sys.path
  （先试直接 import；失败则清缓存 + 插路径 + 重试）
- `ContractNode(meta, handlers)` —— 建节点、发布者、订阅者。
  入参是 **`RobotManager.get_contract_meta()` 的返回值**，不是 `config.json`
  的 `ros2_contract` 段 —— 因为 `sides`（夹爪关节名/flange/tcp link）是运行时
  从 articulation 推导的，配置里没有。
- `spin_once()` / `publish(state)` 都由 `RobotManager.step()` 在主线程调用。

`robot_manager.py` 新增：

| 方法 | 作用 |
|---|---|
| `setup_ros2_contract(enabled, rate_hz, ref_link)` | 建/删节点（HTTP action） |
| `ros2_contract_status()` | 查状态 |
| `_service_ros2_contract()` | `step()` 里 spin 一次，处理 control/* 回调 |
| `_publish_ros2_contract()` | `step()` 里按 rate 发 feedback/* |

## 为什么这么放

**所有 ROS 回调都跑在仿真主线程上**：`spin_once()` 在 `step()` 里同步执行，
回调里直接调 `send_joint_command` / `move_j` / `set_gripper_position`，
不需要缓冲、不需要锁，也不存在"回调线程碰物理"的隐患。
这和旧中继的线程模型不同（那边 spin 在独立线程，回调只能写缓存）。

## 两种实现的关系

| | 进程内 `contract_node.py` | 中继 `ros2_sim_contract.py` |
|---|---|---|
| 位置 | Isaac 进程（py3.12） | 独立进程（py3.10） |
| 数据通路 | 直接写 articulation | HTTP `POST /execute` |
| 反馈频率 | 物理帧率（~50Hz，可配） | 5~8 Hz（受 HTTP 往返 140ms 限制） |
| 409 / 占空比问题 | 无 | 需要按占空比让路 |
| 话题集合 | **完全相同** | 相同 |

二选一，**别同时开**（同名节点 + 重复话题）。中继留给「Isaac 环境里 rclpy
不可用」的兜底场景。

相机话题两条路都不经过 —— 图像由 Isaac 内 OmniGraph 原生发布
（`setup_ros2_cameras` → `ROS2CameraHelper` + `ROS2CameraInfoHelper`）。

## 实测（2026-10-27，电力院背景 + 人形上装）

| 项 | 结果 |
|---|---|
| 节点 | `/iasscsim_robot_sim_contract`，9 订阅 + 14 发布，话题总数 **22** |
| 反馈频率 | **~40 Hz**（`rate_hz=50`，被场景渲染帧率压住；min 19 ms / max 30 ms） |
| `control/move_j` | 目标 `[0.3,-0.5,0.4,0,0,0]` → 实测 `[0.300,-0.499,0.402,0,0,0]` |
| `control/joint_states` | 目标 `[0,0,0,0.2,0.2,0]` → 实测 `[0.000,0.001,0.002,0.201,0.200,0.000]` |
| `control/gripper` | `1000` → 开口 86.16 mm；`1` → 0.53 mm |
| 旧中继 | 5~8 Hz，未启用 |

相关 [[ros2-sim-contract]]、[[isaac62-migration]]、[[ros-env-python-pollution]]。
