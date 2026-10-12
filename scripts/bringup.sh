#!/usr/bin/env bash
# ============================================================
# RouteManager 一键启动(VR 遥操作完整场景)
#   服务(teleop 环境) → 加载场景 → 抬臂 → 桌子+物体 → 7 相机 → VR 会话
#
# 用法:  ./scripts/bringup.sh            # 完整流程
#        ./scripts/bringup.sh --no-vr    # 不起 VR(只搭场景)
# 说明:  · 抬臂必须在加相机之前(相机 prim 会让物理视图永久失效)
#        · 每次重启场景都会重建,相机/桌子不复用
# ============================================================
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
PORT=18765
START_VR=1
[ "${1:-}" = "--no-vr" ] && START_VR=0

health() { curl -s -m 2 "http://127.0.0.1:$PORT/health" >/dev/null 2>&1; }
post_py() {  # $1 = 服务器端执行的 .py 文件; $2 = 标签
    python3 -c "import json; print(json.dumps({'code': open('$1').read()}))" \
        > /tmp/_bringup_payload.json
    local r
    r=$(curl -s -m 180 -X POST "http://127.0.0.1:$PORT/python" \
        -H "Content-Type: application/json" -d @/tmp/_bringup_payload.json)
    echo "  <- $2: $(echo "$r" | head -c 120)"
    sleep "${3:-8}"
}

# ---- 0. 活动 X display(用错的 display 会让 Isaac 无窗口运行并 segfault) ----
if [ -n "${DISPLAY_OVERRIDE:-}" ]; then
    DISP="$DISPLAY_OVERRIDE"
else
    DISP=":$(ls /tmp/.X11-unix/ 2>/dev/null | head -1 | sed 's/^X//')"
    [ "$DISP" = ":" ] && { echo "找不到活动 X display"; exit 1; }
fi
echo "▶ 使用 display: $DISP"

# ---- 1. 重启服务(teleop 环境,isaacteleop 只装在这里) ----
# 先停旧 relay: 它会在服务重启瞬间恢复高频轮询, 与 bringup 自己的请求
# 抢单任务队列, 导致 load_scene/load_robot 失败(2026-09-28 实测)。
[ -f scripts/start_ros_relay.sh ] && ./scripts/start_ros_relay.sh stop >/dev/null 2>&1
pkill -f "[m]ain.py" 2>/dev/null || true
sleep 3
if [ "${HEADLESS:-0}" = "1" ]; then
    # headless 模式: 不开窗口, 不占 DISPLAY(用户要用机器跑别的时用)
    echo "▶ HEADLESS 模式"
    HEADLESS=1 ISAAC_PYTHON="micromamba run -n isaacsim5.1_teleop python" \
        nohup ./scripts/run.sh > /tmp/rm_server.log 2>&1 &
else
    ISAAC_PYTHON="micromamba run -n isaacsim5.1_teleop python" DISPLAY="$DISP" \
        nohup ./scripts/run.sh > /tmp/rm_server.log 2>&1 &
fi
for i in $(seq 1 50); do health && break; sleep 2; done
health || { echo "服务未起来,见 /tmp/rm_server.log"; exit 1; }
echo "▶ 服务已就绪"

# ---- 2. 加载场景 ----
./scripts/load_scene.sh 2>&1 | grep -E "机器人已加载|关节数" || true
sleep 5

# ---- 3. 抬臂(必须在相机之前) ----
post_py scripts/scene/raise_arms.py "抬臂" 6

# (相机挂载连杆的 mesh/碰撞隐藏已离线烘焙进机器人 USD,
#  见 scripts/scene/bake_hide_visuals 流程; 运行时改会触发物理后端失效)

# ---- 3.5 手臂 Jacobian 标定(必须在加相机前: 物理视图届时还健康) ----
if [ -f scripts/scene/build_jac_table.py ]; then
    post_py scripts/scene/build_jac_table.py "Jacobian 标定" 5
fi

# ---- 4. 相机对象注册(静态场景里已有 prim; 同时验证物理视图健康) ----
if [ -f scripts/scene/register_cameras.py ]; then
    post_py scripts/scene/register_cameras.py "相机注册" 8
    python3 -c "
import json
try:
    d = json.load(open('/tmp/register_cams.json'))
    print('   视图健康:', d.get('jacobian'), '| DOF 读:', d.get('dof_read'))
except Exception:
    pass
"
fi

# ---- 6. VR 遥操作 ----
if [ "$START_VR" = "1" ]; then
    echo "▶ 启动 VR 遥操作..."
    curl -s -X POST "http://127.0.0.1:$PORT/execute" \
        -H "Content-Type: application/json" \
        -d '{"action": "start_vr_teleop", "parameters": {}}' | head -c 300
    echo
    sleep 3
    curl -s -X POST "http://127.0.0.1:$PORT/execute" \
        -H "Content-Type: application/json" -d '{"action": "vr_teleop_status"}'
    echo
fi

# ---- 7. ROS 中继(默认启动; START_ROS=0 可跳过) ----
if [ -f scripts/start_ros_relay.sh ] && [ "${START_ROS:-1}" = "1" ]; then
    ./scripts/start_ros_relay.sh
fi

echo "════════ 启动完成 ════════"
echo "Pico 浏览器连 https://nvidia.github.io/IsaacTeleop/client → 服务器 IP"
echo "手柄: 移动=手臂 / 扳机=夹爪 / 左摇杆=底盘 / 左手X=自动抓取 / 右手A=复位"