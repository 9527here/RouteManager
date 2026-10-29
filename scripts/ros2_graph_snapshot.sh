#!/usr/bin/env bash
# ros2_graph_snapshot.sh — 把当前 ROS2 图导出成可 diff 的快照。
#
# 用途：跟另一台机器 / 另一个人对接口时，各自跑一次，然后 diff 两份文件。
# 只导出**稳定的结构信息**（节点、话题+类型+收发数+QoS、服务、参数），
# 不含时间戳/实时数值，所以两次快照可以直接比。
#
#   ./scripts/ros2_graph_snapshot.sh                 # 全部，打印到 stdout
#   ./scripts/ros2_graph_snapshot.sh > /tmp/a.txt    # 存文件
#   diff /tmp/a.txt /tmp/b.txt                       # 两边对
#
#   ./scripts/ros2_graph_snapshot.sh --grep iasscsim  # 只看某个命名空间
#
# 必须在 source /opt/ros/humble/setup.bash 之后跑（脚本里已 source）。

# 不要用 `set -u`：ROS 的 setup.bash 会引用未定义变量，
# 在 `set -u` 下 source 会直接退出，脚本一行都不输出。
set -o pipefail

FILTER=""
if [ "${1:-}" = "--grep" ] && [ -n "${2:-}" ]; then
    FILTER="$2"
fi

source /opt/ros/humble/setup.bash 2>/dev/null || true

pick() {  # 读 stdin，按 FILTER 过滤
    if [ -n "$FILTER" ]; then grep -E "$FILTER"; else cat; fi
}

echo "# ROS2 graph snapshot"
echo "# host: $(hostname)   ros: ${ROS_DISTRO:-unknown}"
echo "# 注意：只含结构信息（不含实时数值），可直接 diff"
echo

echo "=== NODES ==="
ros2 node list 2>/dev/null | sort | pick
echo

echo "=== TOPICS ==="
echo "# 格式: <topic>  <type>  pub=<发布端点数>  sub=<订阅端点数>  qos=<reliability>"
for t in $(ros2 topic list 2>/dev/null | sort | pick); do
    info=$(ros2 topic info -v "$t" 2>/dev/null)
    typ=$(printf '%s\n' "$info" | sed -n 's/^Type: //p' | head -1)
    pub=$(printf '%s\n' "$info" | sed -n 's/^Publisher count: //p' | head -1)
    sub=$(printf '%s\n' "$info" | sed -n 's/^Subscription count: //p' | head -1)
    rel=$(printf '%s\n' "$info" | grep -m1 'Reliability:' | awk '{print $2}')
    printf '%-58s %-38s pub=%s sub=%s qos=%s\n' \
        "$t" "${typ:-?}" "${pub:-?}" "${sub:-?}" "${rel:-?}"
done
echo

echo "=== SERVICES ==="
ros2 service list 2>/dev/null | sort | pick
echo

echo "=== NODE INTERFACES ==="
for n in $(ros2 node list 2>/dev/null | sort | pick); do
    echo "--- $n ---"
    ros2 node info "$n" 2>/dev/null \
        | sed 's/[[:space:]]*$//' \
        | grep -vE '^\s*$'
    echo
done

echo "=== PARAMETERS ==="
for n in $(ros2 node list 2>/dev/null | sort | pick); do
    out=$(ros2 param dump "$n" 2>/dev/null)
    [ -n "$out" ] && { echo "--- $n ---"; printf '%s\n' "$out"; }
done