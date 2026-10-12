#!/usr/bin/env bash
# start_ros_relay.sh — 后台启动 ros2_relay.py 的封装脚本。
#
# !! 必须在 load_robot 之后启动 !!
# ros2_relay.py 启动时会立即调用 RouteManager HTTP 接口拉取关节名，
# 若场景尚未 load_robot，会拿到空列表或直接报错。
#
# 用法：
#   scripts/start_ros_relay.sh            # 默认启动（含底盘 /cmd_vel）
#   scripts/start_ros_relay.sh --no-cmd-vel   # 不带底盘
#   scripts/start_ros_relay.sh stop       # 停止已在跑的 relay
#   scripts/start_ros_relay.sh status     # 查看运行状态
#
# 日志输出：/tmp/ros_relay.log
# PID 文件：/tmp/ros_relay.pid

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RELAY_SCRIPT="${SCRIPT_DIR}/ros2_relay.py"
PID_FILE="/tmp/ros_relay.pid"
LOG_FILE="/tmp/ros_relay.log"

start_relay() {
    if [ -f "$PID_FILE" ]; then
        local old_pid
        old_pid=$(cat "$PID_FILE")
        if kill -0 "$old_pid" 2>/dev/null; then
            echo "[start_ros_relay] 已在运行 (PID=$old_pid)，如需重启请先 stop"
            return 0
        else
            echo "[start_ros_relay] 清理失效的 PID 文件 ($old_pid)"
            rm -f "$PID_FILE"
        fi
    fi

    echo "[start_ros_relay] 启动 $RELAY_SCRIPT $*"
    echo "[start_ros_relay] 日志: $LOG_FILE"
    # 必须用系统 python3(3.10): ROS Humble 的 rclpy C 扩展只有 3.10 版本,
    # micromamba 环境的 python3 是 3.11, import rclpy 会报 ModuleNotFoundError。
    # source ROS 环境确保 rclpy 路径和 LD_LIBRARY_PATH 正确。
    nohup bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null; exec /usr/bin/python3 "$0" "$@"' \
        "$RELAY_SCRIPT" "$@" >> "$LOG_FILE" 2>&1 &
    local pid=$!
    echo "$pid" > "$PID_FILE"
    echo "[start_ros_relay] 已后台启动 (PID=$pid)"
}

stop_relay() {
    if [ ! -f "$PID_FILE" ]; then
        echo "[start_ros_relay] 未找到 PID 文件，relay 未在运行"
        return 0
    fi
    local pid
    pid=$(cat "$PID_FILE")
    if kill -0 "$pid" 2>/dev/null; then
        echo "[start_ros_relay] 停止 relay (PID=$pid)"
        kill "$pid"
        # 等待最多 5s
        for _ in $(seq 1 10); do
            if ! kill -0 "$pid" 2>/dev/null; then break; fi
            sleep 0.5
        done
        if kill -0 "$pid" 2>/dev/null; then
            echo "[start_ros_relay] 强杀"
            kill -9 "$pid"
        fi
    else
        echo "[start_ros_relay] 进程 $pid 已不存在"
    fi
    rm -f "$PID_FILE"
}

status_relay() {
    if [ ! -f "$PID_FILE" ]; then
        echo "[start_ros_relay] 未运行（无 PID 文件）"
        return 0
    fi
    local pid
    pid=$(cat "$PID_FILE")
    if kill -0 "$pid" 2>/dev/null; then
        echo "[start_ros_relay] 运行中 (PID=$pid)"
    else
        echo "[start_ros_relay] 进程已退出（PID=$pid 失效）"
    fi
}

case "${1:-}" in
    stop)    stop_relay ;;
    status)  status_relay ;;
    *)       start_relay "$@" ;;
esac
