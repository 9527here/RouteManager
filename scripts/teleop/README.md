# Pico 4 Ultra VR 遥操作（Isaac Teleop × RouteManager）

VR 头显（Pico 4 Ultra，Pico 4 同流程）→ 本仿真（humanoid_upper_body_v3），无真机。

```
┌──────────────┐  WebXR/CloudXR   ┌─────────────────────────────────────┐
│   Pico 4     │ ◄─────────────► │  RouteManager (main.py, 进程内)      │
│  浏览器客户端 │   控制器位姿/扳机  │  isaac_teleop.VRTeleopManager        │
└──────────────┘   流视频(机器人视角) │   TeleopSession(isaaccapture 1.6)    │
                                    │   → 关节目标 → articulation (50Hz)   │
                                    └─────────────────────────────────────┘
```

## 1. 安装（一次性）

```bash
cd ~/RouteManager
./scripts/teleop/setup_isaacteleop.sh
```

脚本会克隆 micromamba 环境 `isaacsim5.1 → isaacsim5.1_teleop`（不动原环境），
再在其中安装 `isaaccapture[cloudxr,retargeters]~=1.6.0`。

放行 CloudXR 防火墙端口：

```bash
sudo ufw allow 47998/udp
sudo ufw allow 49100/tcp
sudo ufw allow 48322/tcp
```

## 2. 启动服务（用 teleop 环境）

```bash
# scripts/run.sh 自动找 micromamba isaacsim5.1；指定克隆环境：
ISAAC_PYTHON="micromamba run -n isaacsim5.1_teleop python" ./scripts/run.sh
```

加载机器人（config.json 已指向 v3_ctag2f120_cameras）：

```bash
curl -s -X POST http://127.0.0.1:18765/execute \
  -H 'Content-Type: application/json' \
  -d '{"action": "load_robot", "parameters": {"name": "v3_ctag2f120_cameras"}}'
```

## 3. 开启遥操作

```bash
curl -s -X POST http://127.0.0.1:18765/execute \
  -H 'Content-Type: application/json' \
  -d '{"action": "start_vr_teleop", "parameters": {}}'
```

可选参数（默认值在 config.json `robot.teleop`）：
`workspace_scale`（VR 位移→机器人位移缩放）、`max_step`（每帧关节限幅）、
`grip_open`（扳机全开角度）、`rate_hz`。

## 4. Pico 4 Ultra 连接

1. Pico 4 Ultra 打开浏览器，访问 <https://nvidia.github.io/IsaacTeleop/client>
2. Server Address 填运行 RouteManager 的电脑 IP（同一局域网；
   查 IP：服务器上 `hostname -I`，头显与服务器建议连同一个
   Wi-Fi 6/7 路由的 5GHz/6GHz 频段，延迟越低越好）
3. 设备档案：NVIDIA 对 PICO 系列使用 Quest3 档案（`NV_DEVICE_PROFILE=Quest3`，
   服务端已默认处理；若客户端页面有 Profile 选项也选 Quest3）
4. 进入后手持手柄即被识别；扳机 = 夹爪开合，手柄移动 = 手臂跟随

> Pico 4 Ultra 支持手柄与手部追踪，Wi-Fi 7 对 CloudXR 串流延迟更友好；
> 全身动捕（身体重定向）需要 PICO 企业版系统支持。

## 5. 状态与停止

```bash
curl -s -X POST http://127.0.0.1:18765/execute \
  -H 'Content-Type: application/json' -d '{"action": "vr_teleop_status"}'

curl -s -X POST http://127.0.0.1:18765/execute \
  -H 'Content-Type: application/json' -d '{"action": "stop_vr_teleop"}'
```

## 6. 故障排查

| 现象 | 处理 |
| --- | --- |
| `未安装 isaaccapture` | 运行 setup_isaacteleop.sh；确认用 isaacsim5.1_teleop 环境启动 |
| `无法从 TeleopSession 读取控制器状态` | isaaccapture 版本 API 变化——错误信息里带了可用成员列表，据此适配 isaac_teleop.py 的 `_discover_reader` |
| Pico 连不上 / 黑屏 | 检查防火墙三个端口；确认 Pico 与电脑同网段；`vr_teleop_status` 看 frames 是否增长 |
| 头显里画面是空的 | CloudXR 流的是机器人视角；需要先给机器人加相机（`add_camera`，挂 head_link） |
| 手臂不动 | `config.json robot.teleop.arm_joints` 手工指定左/右臂关节名列表 |
| 遥操作时关节抖动 | 减小 `workspace_scale`、增大 `deadzone` |

## 7. 与真机的关系

链路设计为 VR→仿真先行；真机阶段同一 TeleopSession 的输出可改为发布
`/humanoid/joint_command`（ROS2），RouteManager 的 ROS2 bridge 原样复用。

## 8. 已知限制（v1）

- 手臂控制为"控制器位移→关节速率"映射（3 平移轴 → 每臂前 3 个关节），
  不是绝对位姿 IK；Se3AbsRetargeter + Pinocchio IK 留作 v2
- 夹爪联动用**软件 mimic**（robot_manager 每帧 follower 目标 = URDF 倍率
  ±1/−0.15 × 主动关节角度）；PhysX 的 mimic 约束在该资产上不生效，
  已从 USD 剥离。扳机模拟开合
- isaaccapture 与 isaacsim 版本强耦合，升级 5.x 需重新克隆环境
