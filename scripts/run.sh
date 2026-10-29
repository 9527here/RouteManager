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

# Isaac Sim 的 Python：优先 ISAAC_PYTHON 环境变量，其次 micromamba 的 6.2 环境
#
# 默认 **isaacsim6.0** —— 目录名有误导性，这个 env 已于 2026-10-20 就地升级到
# Isaac Sim **6.2.0.0**（Python 3.12），现在才是主力环境。
# isaacsim5.1 仅作兼容回退：2026-10-20 那次升级把 5.1 的 Kit 扩展缓存
# （omni.isaac.ml_archive 的 pip_prebundle/torch）覆盖成了 py312 的 C 扩展，
# 5.1 是 py311，`import torch` 直接失败 ⇒ isaacsim.core.* 全部加载不了。
# 要用 5.1 得先把那份 prebundle 换回 py311 版本。
if [ -n "$ISAAC_PYTHON" ]; then
    echo "[run.sh] 使用 ISAAC_PYTHON: $ISAAC_PYTHON"
elif command -v micromamba >/dev/null 2>&1 && micromamba env list 2>/dev/null | grep -q "isaacsim6.0"; then
    ISAAC_PYTHON="micromamba run -n isaacsim6.0 python"
    echo "[run.sh] 使用 micromamba 环境: isaacsim6.0 (Isaac Sim 6.2, py3.12)"
elif command -v micromamba >/dev/null 2>&1 && micromamba env list 2>/dev/null | grep -q "isaacsim5.1"; then
    ISAAC_PYTHON="micromamba run -n isaacsim5.1 python"
    echo "[run.sh] 警告: 回退到 isaacsim5.1 —— 该环境的 torch prebundle 已被"
    echo "          2026-10-20 的 6.2 升级覆盖成 py312，大概率起不来"
else
    ISAAC_PYTHON="python3"
    echo "[run.sh] 警告: 未找到 Isaac Sim Python 环境，回退 python3 —— 可能缺少 isaacsim 包"
fi

# ---- 隔离 ROS humble 对 Isaac(py3.12) 的两层污染 ----
# ~/.bashrc 会 source /opt/ros/humble/setup.bash，带进两份 py3.10 的东西：
#   1) PYTHONPATH 里 .../python3.10/site-packages —— py3.12 的 Isaac import
#      pinocchio 会先命中那儿只有 cp310 扩展的包：
#        ModuleNotFoundError: No module named 'pinocchio.pinocchio_pywrap_default'
#   2) LD_LIBRARY_PATH 里 /opt/ros/humble/lib/x86_64-linux-gnu —— 里面的
#      libeigenpy.so / libcoal.so 是 py3.10+numpy1 的，会顶掉环境 cmeel 那份：
#        undefined symbol: EIGENPY_ARRAY_APIPyArray_RUNTIME_VERSION
# 两者都会让 isaacsim.robot_motion.pink 加载失败。环境里本来就有 py312 的
# pinocchio 3.9 + eigenpy 3.12（cmeel.prefix 下），摘掉这两处即可。
# **只摘这两个目录**：/opt/ros/humble/lib 要留着 —— 环境里有 ROS_DISTRO 时
# isaacsim.ros2.core 走系统 ROS 库，抽掉会 "ROS2 Bridge startup failed"
# （2026-10-26 实测）。中继进程在 start_ros2_contract.sh 里自己 source，不受影响。
strip_path_var() {  # 用法: strip_path_var <变量名> <case 模式(命中则丢弃)>
    local var="$1" pat="$2" out="" p
    local -a parts=()
    if [ -n "${!var:-}" ]; then
        IFS=':' read -r -a parts <<< "${!var}"
    fi
    for p in "${parts[@]}"; do
        [ -z "$p" ] && continue
        # shellcheck disable=SC2254  # $pat 要当通配符用
        case "$p" in $pat) continue ;; esac
        out="${out:+$out:}$p"
    done
    export "$var=$out"
}
strip_path_var PYTHONPATH '/opt/ros/*'
strip_path_var LD_LIBRARY_PATH '/opt/ros/*/lib/*-linux-gnu'

# ---- 让 Isaac 自带的 ROS2 C 库排到系统 /opt/ros/humble/lib 前面 ----
# 系统 humble 的 librcl*/librosidl*/lib*__rosidl_typesupport* 与 isaacsim.ros2.core
# 自带的那套**不是同一次构建**。LD_LIBRARY_PATH 里系统那份在前时，进程内 rclpy
# 会加载到不配套的库，`rclpy.create_node()` 在建节点那一步直接 SIGABRT：
#
#   Node.__init__ → TimeSource.attach_node → declare_parameter('use_sim_time')
#     → _set_parameters_atomically → 发 parameter_events
#       rcl_interfaces__msg__parameter_event__convert_from_py:
#       Assertion `PyUnicode_IS_READY(name_attr)' failed.
#
# 2026-10-27 实测：只调换这一处顺序即通（RMW_IMPLEMENTATION 无关；放后面则
# import 阶段就报 librcl_logging_spdlog.so undefined symbol）。
# 目录里只有 ROS 专用库（无 libstdc++/libpython），前置不影响 Isaac 本身。
# RM_SKIP_ROS2_LIB_FIX=1 可跳过这一步（排查用：用来区分某个问题是不是这层
# 库顺序引起的）。注意跳过之后 setup_ros2_contract 会 SIGABRT。
if [ -z "${RM_SKIP_ROS2_LIB_FIX:-}" ]; then
    ROS2_CORE_LIB="$($ISAAC_PYTHON -c 'import os, sysconfig
p = os.path.join(sysconfig.get_paths()["purelib"], "isaacsim", "exts",
                 "isaacsim.ros2.core", os.environ.get("ROS_DISTRO", "humble"), "lib")
print(p if os.path.isdir(p) else "")' 2>/dev/null || true)"
    if [ -n "$ROS2_CORE_LIB" ]; then
        export LD_LIBRARY_PATH="$ROS2_CORE_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
        echo "[run.sh] ROS2 自带库优先: $ROS2_CORE_LIB"
    else
        echo "[run.sh] 警告: 未找到 isaacsim.ros2.core 自带 ROS2 库，进程内 rclpy 可能 SIGABRT"
    fi
else
    echo "[run.sh] RM_SKIP_ROS2_LIB_FIX=1 —— 跳过程序内 ROS2 库前置"
fi

exec $ISAAC_PYTHON main.py
