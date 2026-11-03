# HTTP接口

所有action在仿真主线程执行。默认本机18765；忙时409，客户端须重试。

| 端点 | 用途 |
|---|---|
| `GET /health` | IDLE为就绪 |
| `POST /execute` | 反射调用RobotManager公开action，parameters覆盖配置默认值 |
| `POST /python` | 主线程调试入口，可访问executor；仅在可信本机使用 |

### Action 列表

| Action | 说明 | 主要参数 |
|---|---|---|
| `load_scene` | 加载场景 USD / 默认地平面 | `usd_path`, `prim_path` |
| `load_robot` | 加载 USD 机器人资产并注册 Articulation（推荐；支持 config 预设库按名切换） | `name`, `usd_path`, `prim_path` |
| `load_urdf` | URDF→USD 导入并注册 Articulation | `urdf_path`, `prim_path`, `fix_base`, `merge_fixed_joints` |
| `get_joint_names` | 列出导入后的关节 | — |
| `set_joint_positions` | 直接设关节位置（物理仿真） | `positions`, `joint_names` |
| `get_joint_positions` | 读取关节位置 | — |
| `set_home_pose` | 设置基础姿态（默认写回 config.json 的 `home_pose`，重启仍生效） | `pose`（`{"关节名": rad}`）, `save` |
| `add_camera` | 创建相机传感器（`attach_to` 可挂到 link 上随动） | `prim_path`, `attach_to`, `name`, `position`, `rotation_euler`, `resolution`, `focal_length`, `intrinsics` |
| `get_camera_image` | 抓图（返回 base64 JPEG） | `prim_path` |
| `move_camera` | 移动相机（挂载相机按本地坐标） | `prim_path`, `position`, `rotation_euler` |
| `send_joint_command` | 下发关节目标到仿真（非阻塞） | `positions`, `duration`（>0 时平滑插值，主循环逐帧推进）, `joint_names`（**只下发这些关节**，其余沿用当前指令目标） |
| `set_gripper_position` | 夹爪指令，**睿尔曼 rm_driver 语义** | `side`（left/right/both）, `position`（1~1000，1000=全开）；与真机 `{ns}/set_gripper_position_cmd` 同参 |
| `get_gripper_position` | 夹爪开口回读（角度/比例/rm 单位） | `side` |
| `set_base_velocity` | 底盘速度指令（四轮独立转向/驱动；兼容旧差速） | `vx`、`vy`（m/s）, `wz`（rad/s）；需 ≥10Hz 持续发送，超时自动刹停 |
| `get_joint_states` | 读取指令/反馈状态 | — |
| `start_vr_teleop` | 启动 Pico VR 遥操作 | `rate_hz`, `workspace_scale`, `max_step`, `deadzone`, `grip_open`（默认取 config.json `teleop`） |
| `stop_vr_teleop` | 停止 VR 遥操作 | — |
| `vr_teleop_status` | VR 遥操作状态（`running`/`frames`/`last_error`） | — |
| `setup_ros2_contract` | Isaac 6.2自带rclpy的进程内ROS2契约；失败时可选独立中继 | `enabled`, `rate_hz`, `ref_link` |


表中列常用入口，完整签名以`robot_manager.py`为准。机器人/夹爪/相机须按已加载模型配置；旧七轴home_pose不能套到RM65六轴。

```bash
curl -s http://127.0.0.1:18765/health
curl -s -X POST http://127.0.0.1:18765/execute -H "Content-Type: application/json" \
  -d '{"action":"get_joint_names"}'
```

ROS2话题及语义见[接口契约](ROS2接口_iasscsim_robot.md)。
