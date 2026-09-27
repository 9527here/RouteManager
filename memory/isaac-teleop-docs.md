---
name: isaac-teleop-docs
description: NVIDIA Isaac Teleop（XR 遥操作→机器人）文档入口与快速开始要点
metadata:
  type: reference
---

Isaac Teleop：XR 头显（CloudXR）遥操作框架，手柄/手势 → retargeting → 机器人命令，支持 Isaac Lab 仿真与 ROS 2 真机。

- 快速开始：https://nvidia.github.io/IsaacTeleop/main/getting_started/quick_start.html （main=1.6.x；旧版用 release/1.0.x~1.5.x 切换）
- ⚠️ 1.6 起包改名 `isaacteleop` → `isaaccapture`，但 1.6 无法公开安装：公共 PyPI 无
  isaaccapture（404），pypi.nvidia.com 的桶也是空的（NoSuchKey，文档预发布通道失效）。
  **实际可装**：公共 PyPI 旧名 `isaacteleop==1.5.144rc1`（有 cp310/311/312/313
  manylinux_2_35 wheel；isaacsim5.1 用 cp311 正好匹配）。v1 关节速率映射只需
  `isaacteleop[cloudxr]`；retargeters 额外依赖 torch2.7+pinocchio ~2GB，v2 再装。
  GitHub 直连需代理（本机 127.0.0.1:7897）；源码构建是 CMake+clang-format-14 重构建，没必要。
- Pico 4 Ultra 官方原生支持，默认 90 FPS / 100 Mbps（Quest 仅 72/25）；防火墙
  47998/udp + 49100,48322/tcp（web 客户端 dev 模式另加 8080,8443/tcp）
- 没头显可测：浏览器开 nvidia.github.io/IsaacTeleop/client（IWER 模拟 Quest3）
- 示例：examples/teleop/python/ 下 gripper_retargeting_example_simple.py（夹爪）、se3_retargeting_example.py（SE3 末端）、dex_bimanual_example.py（双灵巧手）
- 防火墙（Quest/PICO）：47998/udp、49100+48322/tcp
- 与本项目的关联：VLA 数据采集前端 → [[routemanager-project]] 的仿真环境 / rosbag→LeRobot→GR00T 流程；官方有 Teleop+GR00T+Unitree G1 端到端管线
- WebFetch 对 nvidia.github.io 域名校验被挡，用 curl 抓取即可
