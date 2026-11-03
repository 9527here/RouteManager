#!/usr/bin/env bash
# 采集单独启动 Isaac Lab，不接管桌面的 RouteManager 场景。
set -euo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
isaac_python="${ISAAC_PYTHON:-/home/jy/micromamba/envs/isaacsim6.0/bin/python}"
unset DISPLAY PYTHONPATH LD_LIBRARY_PATH VIRTUAL_ENV
export PYTHONUNBUFFERED=1
cd "$project_dir"
exec "$isaac_python" scripts/collect_l3_isaaclab.py "$@"
