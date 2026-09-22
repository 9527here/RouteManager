#!/usr/bin/env bash
# ============================================================
# RouteManager 启动脚本
# 用法:
#   ./scripts/run.sh              # 有界面模式
#   HEADLESS=1 ./scripts/run.sh   # 无头模式
# 注意：ROS2 bridge 需要 rclpy —— 若脚本找不到 ROS2 环境，
#       修改下面的 ROS_SETUP 指向你机器上的 setup.bash
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SCRIPT_DIR"

ROS_SETUP="/opt/ros/humble/setup.bash"
if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
    echo "[run.sh] 已加载 ROS2 环境: $ROS_SETUP"
else
    echo "[run.sh] 警告: 未找到 $ROS_SETUP，ROS2 bridge 将降级运行"
fi

# 如果 Isaac Sim 有自己的 python 环境，把下面的 python3 换成它
exec python3 main.py
