#!/usr/bin/env bash
# start_ros_relay.sh — **已废弃**，转发到 start_ros2_contract.sh。
#
# 旧的 ros2_relay.py 发的是 `/humanoid/joint_states`、`/humanoid/joint_command`、
# `/cmd_vel` 这套私有命名；现在统一成 `/<robot>/sim/<instance>/...` 契约
# （见 docs/ROS2接口_iasscsim_robot.md）。
#
# 保留这个文件名只是为了让仍在调用它的脚本（bringup.sh、用户自己的流程）
# 不用改就能继续工作。
#
#   scripts/start_ros_relay.sh              -> 启动契约中继
#   scripts/start_ros_relay.sh stop         -> 停止
#   scripts/start_ros_relay.sh status       -> 状态

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[start_ros_relay] 已废弃：转发到 scripts/start_ros2_contract.sh" >&2
exec "${SCRIPT_DIR}/start_ros2_contract.sh" "$@"