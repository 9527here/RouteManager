#!/usr/bin/env bash
# 跑 valve_rl 下的脚本：隔离 ROS 污染 + 无头运行
# 用法: ./run.sh probe_robot.py [脚本参数...]
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

strip_path_var() {
    local var="$1" pat="$2" out="" p
    local -a parts=()
    if [ -n "${!var:-}" ]; then IFS=':' read -r -a parts <<< "${!var}"; fi
    for p in "${parts[@]}"; do
        [ -z "$p" ] && continue
        case "$p" in $pat) continue ;; esac
        out="${out:+$out:}$p"
    done
    export "$var=$out"
}
# RM_KEEP_DISPLAY=1 保留 DISPLAY（想看画面时用）；默认无头
if [ -z "${RM_KEEP_DISPLAY:-}" ]; then
    unset DISPLAY
fi
strip_path_var PYTHONPATH '/opt/ros/*'
strip_path_var LD_LIBRARY_PATH '/opt/ros/*'

export PYTHONUNBUFFERED=1
export PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}"
# 必须走 micromamba run：直接调 python 二进制不会铺开环境自身的库路径，
# Isaac 的 libusd_*.so 会找不到。
exec micromamba run -n isaacsim6.0 python "$@"
