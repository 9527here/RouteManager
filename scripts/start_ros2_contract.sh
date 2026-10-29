#!/usr/bin/env bash
# start_ros2_contract.sh — 后台启动 ros2_sim_contract.py（`/<robot>/sim/...` 契约）。
#
# !! 必须在 load_scene + load_robot 之后启动 !!
# 它启动时会调 get_contract_meta 拉关节名/相机规格；机器人没加载会一直重试
# 直到 --meta-timeout。
#
# 用法：
#   scripts/start_ros2_contract.sh                # 默认 20Hz
#   scripts/start_ros2_contract.sh --state-hz 30
#   scripts/start_ros2_contract.sh --camera-info  # 兜底：本进程也发 CameraInfo
#   scripts/start_ros2_contract.sh stop
#   scripts/start_ros2_contract.sh status
#
# 日志：/tmp/ros2_sim_contract.log
# PID ：/tmp/ros2_sim_contract.pid

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTRACT_SCRIPT="${SCRIPT_DIR}/ros2_sim_contract.py"
PID_FILE="/tmp/ros2_sim_contract.pid"
LOG_FILE="/tmp/ros2_sim_contract.log"

start_contract() {
    if [ -f "$PID_FILE" ]; then
        local old_pid
        old_pid=$(cat "$PID_FILE")
        if kill -0 "$old_pid" 2>/dev/null; then
            echo "[ros2_contract] 已在运行 (PID=$old_pid)，如需重启请先 stop"
            return 0
        fi
        echo "[ros2_contract] 清理失效的 PID 文件 ($old_pid)"
        rm -f "$PID_FILE"
    fi

    # 旧 relay 不能和本进程同时跑：两者都高频轮询 HTTP，会互相 409
    if [ -f /tmp/ros_relay.pid ] && kill -0 "$(cat /tmp/ros_relay.pid)" 2>/dev/null; then
        echo "[ros2_contract] 检测到旧的 ros2_relay 还在跑，先停掉它"
        "${SCRIPT_DIR}/start_ros_relay.sh" stop || true
    fi

    echo "[ros2_contract] 启动 $CONTRACT_SCRIPT $*"
    echo "[ros2_contract] 日志: $LOG_FILE"
    # 必须用系统 /usr/bin/python3（3.10）：ROS Humble 的 rclpy C 扩展只有 3.10 版，
    # micromamba 环境的 python 是 3.12（Isaac），import rclpy 会 ModuleNotFoundError。
    nohup bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null; exec /usr/bin/python3 "$0" "$@"' \
        "$CONTRACT_SCRIPT" "$@" >> "$LOG_FILE" 2>&1 &
    local pid=$!
    echo "$pid" > "$PID_FILE"
    echo "[ros2_contract] 已后台启动 (PID=$pid)"
}

stop_contract() {
    if [ ! -f "$PID_FILE" ]; then
        echo "[ros2_contract] 未找到 PID 文件，未在运行"
        return 0
    fi
    local pid
    pid=$(cat "$PID_FILE")
    if kill -0 "$pid" 2>/dev/null; then
        echo "[ros2_contract] 停止 (PID=$pid)"
        kill "$pid"
        for _ in $(seq 1 10); do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.5
        done
        kill -0 "$pid" 2>/dev/null && { echo "[ros2_contract] 强杀"; kill -9 "$pid"; }
    else
        echo "[ros2_contract] 进程 $pid 已不存在"
    fi
    rm -f "$PID_FILE"
}

status_contract() {
    if [ ! -f "$PID_FILE" ]; then
        echo "[ros2_contract] 未运行（无 PID 文件）"
        return 0
    fi
    local pid
    pid=$(cat "$PID_FILE")
    if kill -0 "$pid" 2>/dev/null; then
        echo "[ros2_contract] 运行中 (PID=$pid)"
    else
        echo "[ros2_contract] 进程已退出（PID=$pid 失效）"
    fi
}

case "${1:-}" in
    stop)    stop_contract ;;
    status)  status_contract ;;
    *)       start_contract "$@" ;;
esac