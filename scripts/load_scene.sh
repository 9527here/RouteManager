#!/usr/bin/env bash
# ============================================================
# RouteManager 一键加载场景脚本
# ============================================================
# 用法:
#   ./scripts/load_scene.sh                       # 地面 + config 默认机器人
#   ./scripts/load_scene.sh -n <预设名>            # 地面 + 按预设名加载
#   ./scripts/load_scene.sh -a <URDF/USD 路径>     # 地面 + 指定资产（按扩展名选 loader）
#   ./scripts/load_scene.sh -a <路径> -p /mybot    # 指定挂载路径
#   ./scripts/load_scene.sh --no-ground -a <路径>  # 不建地面
#   ./scripts/load_scene.sh --health               # 只看服务状态
#
# 环境变量: ROUTE_MANAGER_HOST / ROUTE_MANAGER_PORT（默认 127.0.0.1:18765）
# ============================================================
set -e

HOST="${ROUTE_MANAGER_HOST:-127.0.0.1}"
PORT="${ROUTE_MANAGER_PORT:-18765}"
BASE="http://$HOST:$PORT"

GROUND=1
NAME=""
ASSET=""
PRIM="/humanoid"

while [[ $# -gt 0 ]]; do
    case "$1" in
        -n|--name)     NAME="$2"; shift 2;;
        -a|--asset)    ASSET="$2"; shift 2;;
        -p|--prim)     PRIM="$2"; shift 2;;
        --no-ground)   GROUND=0; shift;;
        --health)      curl -s "$BASE/health" | python3 -m json.tool; exit 0;;
        -h|--help)     grep '^# ' "$0" | sed 's/^# \?//'; exit 0;;
        *) echo "未知参数: $1（-h 看用法）"; exit 1;;
    esac
done

# ---- 工具函数 ----

post() {  # post <json>：POST /execute，非 200 打印错误并退出
    local resp code body
    resp=$(curl -s -w '\n%{http_code}' -X POST "$BASE/execute" \
           -H 'Content-Type: application/json' -d "$1")
    code="$(echo "$resp" | tail -1)"
    body="$(echo "$resp" | sed '$d')"
    if [[ "$code" != "200" ]]; then
        echo "❌ 请求失败 (HTTP $code):" >&2
        echo "$body" | python3 -m json.tool 2>/dev/null || echo "$body" >&2
        exit 1
    fi
    echo "$body"
}

wait_server() {  # 最多等 120s（Isaac 启动慢属正常）
    echo -n "等待服务 $BASE ..."
    for _ in $(seq 1 60); do
        if curl -sf "$BASE/health" >/dev/null 2>&1; then
            echo " 就绪"; return 0
        fi
        echo -n "."
        sleep 2
    done
    echo
    echo "❌ 服务 120s 内未就绪，确认 run.sh 已启动: $BASE/health" >&2
    exit 1
}

# ---- 主流程 ----

wait_server
echo "▶ 服务状态: $(curl -s "$BASE/health")"

if [[ "$GROUND" == "1" ]]; then
    echo "▶ 创建地面场景..."
    post '{"action": "load_scene"}' >/dev/null
    echo "  地面就绪"
fi

# 组装加载请求
if [[ -n "$ASSET" ]]; then
    case "$ASSET" in
        *.urdf)
            REQ="$(python3 -c 'import json,sys; print(json.dumps({"action":"load_urdf","parameters":{"urdf_path":sys.argv[1],"prim_path":sys.argv[2]}}))' "$ASSET" "$PRIM")"
            KIND="URDF（首次会自动转换 USD，142 mesh 量级需数分钟）"
            ;;
        *.usd|*.usda|*.usdc)
            REQ="$(python3 -c 'import json,sys; print(json.dumps({"action":"load_robot","parameters":{"usd_path":sys.argv[1],"prim_path":sys.argv[2]}}))' "$ASSET" "$PRIM")"
            KIND="USD"
            ;;
        *) echo "❌ 无法识别的资产类型: $ASSET（支持 .urdf/.usd/.usda/.usdc）" >&2; exit 1;;
    esac
elif [[ -n "$NAME" ]]; then
    REQ="{\"action\": \"load_robot\", \"parameters\": {\"name\": \"$NAME\"}}"
    KIND="预设 '$NAME'"
else
    REQ='{"action": "load_robot"}'
    KIND="config 默认机器人"
fi

echo "▶ 加载机器人: $KIND（大资产需 1~3 分钟，请耐心等待）..."
RESP="$(post "$REQ")"

# 摘要
echo "$RESP" | python3 -c '
import json, sys
d = json.load(sys.stdin)["data"]
joints = d.get("joint_names", [])
prim = d.get("prim_path")
top10 = ", ".join(joints[:10])
print("✅ 机器人已加载")
print(f"   prim_path : {prim}")
print(f"   关节数    : {len(joints)}")
if joints:
    print(f"   前 10 关节: {top10}")
'
echo
echo "后续操作示例:"
echo "  curl -X POST $BASE/execute -H 'Content-Type: application/json' -d '{\"action\": \"get_joint_positions\"}'"
echo "  curl -X POST $BASE/execute -H 'Content-Type: application/json' -d '{\"action\": \"setup_ros2_bridge\"}'"
