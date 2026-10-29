#!/usr/bin/env bash
# rosj.sh —— 用 ROS2 话题控制 / 查看仿真机器人手臂的快捷命令
#
# 把一长串 `ros2 topic pub ... trajectory_msgs/JointTrajectory "{...}"` 收成一行。
#
# 用法:
#   ./scripts/rosj.sh move left 0.3 -0.5 0.4 0 0 0        # 6 个弧度, 默认 2 秒到位
#   ./scripts/rosj.sh move right 0 0 0 0 0 0 -t 5         # 指定插值时长
#   ./scripts/rosj.sh home left                           # 回零 (= move 全 0)
#   ./scripts/rosj.sh grip left 1000                      # 夹爪 1~1000, 1000=全开
#   ./scripts/rosj.sh show left                           # 读一次当前关节
#   ./scripts/rosj.sh hz left                             # 看反馈频率
#   ./scripts/rosj.sh name left                           # 打印 6 个关节名
#
# 前提: 仿真已 load_scene + load_robot + setup_ros2_contract(见 docs/ROS2接口_iasscsim_robot.md)
# 需要终端已 source /opt/ros/humble/setup.bash (本脚本也会自动尝试 source)。

set -euo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [ -f "$ROS_SETUP" ] && [ -z "${ROS_DISTRO:-}" ]; then
    # shellcheck disable=SC1090
    source "$ROS_SETUP" >/dev/null 2>&1 || true
fi

BASE="${ROSJ_NS:-/iasscsim_robot/sim}"
JOINTS=()
for i in 1 2 3 4 5 6; do JOINTS+=("${i}_joint"); done

usage() { sed -n '2,18p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

cmd="${1:-}"
[ -z "$cmd" ] && { usage; exit 1; }
shift || true

case "$cmd" in
    move|home)
        side="${1:-left}"; [ "$cmd" = move ] && shift || true
        dur="2.0"
        vals=()
        if [ "$cmd" = home ]; then
            vals=(0 0 0 0 0 0)
        else
            while [ $# -gt 0 ]; do
                case "$1" in
                    -t|--time) dur="$2"; shift 2 ;;
                    *) vals+=("$1"); shift ;;
                esac
            done
        fi
        if [ "${#vals[@]}" -ne 6 ]; then
            echo "需要 6 个关节角(弧度), 收到 ${#vals[@]} 个" >&2; exit 1
        fi
        names=""; pos=""
        for i in 0 1 2 3 4 5; do
            names="${names:+$names, }${side}_arm_$((i+1))_joint"
            pos="${pos:+$pos, }${vals[$i]}"
        done
        sec="${dur%%.*}"; nsec=0
        case "$dur" in
            *.*) frac="${dur#*.}"; frac="${frac}000000000"; nsec="${frac:0:9}" ;;
        esac
        echo "→ ${side} arm: [$pos]  时长 ${dur}s"
        ros2 topic pub --once "$BASE/${side}_arm/control/move_j" \
            trajectory_msgs/JointTrajectory \
            "{joint_names: [$names], points: [{positions: [$pos], time_from_start: {sec: $sec, nanosec: $nsec}}]}" \
            | tail -1
        ;;

    grip)
        side="${1:-left}"; pos="${2:?用法: rosj.sh grip <left|right|both> <1-1000>}"
        if [ "$side" = both ]; then
            echo "夹爪指令只能单侧, 收到 both" >&2; exit 1
        fi
        echo "→ ${side} gripper = $pos"
        ros2 topic pub --once "$BASE/${side}_arm/control/gripper" \
            std_msgs/Float32 "{data: $pos}" | tail -1
        ;;

    show)
        side="${1:-left}"
        ros2 topic echo --once "$BASE/${side}_arm/feedback/joint_states" \
            | sed -n '/^name:/,/^velocity:/p'
        ;;

    hz)
        side="${1:-left}"
        echo "Ctrl-C 结束"
        ros2 topic hz "$BASE/${side}_arm/feedback/joint_states"
        ;;

    name)
        side="${1:-left}"
        for i in 1 2 3 4 5 6; do echo "${side}_arm_${i}_joint"; done
        ;;

    -h|--help|help)
        usage
        ;;

    *)
        echo "未知命令: $cmd" >&2; usage; exit 1
        ;;
esac
