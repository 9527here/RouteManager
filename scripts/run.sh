#!/usr/bin/env bash
# ============================================================
# RouteManager 启动脚本
# 用法:
#   ./scripts/run.sh              # 有界面模式
#   HEADLESS=1 ./scripts/run.sh   # 无头模式
# 注意：
#   1. ROS2 bridge 需要 rclpy —— 若脚本找不到 ROS2 环境，
#      修改下面的 ROS_SETUP 指向你机器上的 setup.bash
#   2. Isaac Sim 必须用它自带的 Python（系统 python3 没有 isaacsim 包）。
#      默认用 micromamba 环境 isaacsim5.1，可用 ISAAC_PYTHON 覆盖，
#      如: ISAAC_PYTHON="/path/to/envs/isaacsim6.0/bin/python" ./scripts/run.sh
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

# Isaac Sim 的 Python：优先 ISAAC_PYTHON 环境变量，其次 micromamba isaacsim5.1
if [ -n "$ISAAC_PYTHON" ]; then
    echo "[run.sh] 使用 ISAAC_PYTHON: $ISAAC_PYTHON"
elif command -v micromamba >/dev/null 2>&1 && micromamba env list 2>/dev/null | grep -q "isaacsim5.1"; then
    ISAAC_PYTHON="micromamba run -n isaacsim5.1 python"
    echo "[run.sh] 使用 micromamba 环境: isaacsim5.1"
else
    ISAAC_PYTHON="python3"
    echo "[run.sh] 警告: 未找到 Isaac Sim Python 环境（isaacsim5.1），回退 python3 —— 可能缺少 isaacsim 包"
fi

exec $ISAAC_PYTHON main.py
