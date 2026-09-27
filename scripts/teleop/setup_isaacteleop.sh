#!/usr/bin/env bash
# ===== Isaac Teleop 1.6 安装脚本（Pico 4 VR 遥操作）=====
# 依据 NVIDIA Isaac Teleop 快速开始：
#   https://nvidia.github.io/IsaacTeleop/main/getting_started/quick_start.html
#
# 策略：克隆 isaacsim5.1 环境后在其中安装 isaaccapture，
#       不污染正在使用的 isaacsim5.1 环境（RouteManager 默认仍用它）。
#       装完后 RouteManager 用克隆环境启动即可开启 VR 遥操作。
set -euo pipefail

SRC_ENV="${SRC_ENV:-isaacsim5.1}"
DST_ENV="${DST_ENV:-isaacsim5.1_teleop}"

echo "==> 克隆 micromamba 环境 $SRC_ENV -> $DST_ENV（已存在则跳过）"
if micromamba env list | awk '{print $1}' | grep -qx "$DST_ENV"; then
    echo "    环境已存在，跳过克隆"
else
    micromamba create -n "$DST_ENV" --clone "$SRC_ENV" -y
fi

echo "==> 在 $DST_ENV 中安装 isaacteleop（VR 遥操作框架）"
# 注：1.6 起包改名 isaaccapture，但公共 PyPI 无此包、NVIDIA 预发布源
# (pypi.nvidia.com) 的桶是空的；公共 PyPI 上实际可装的是旧名 isaacteleop
# 1.5.144rc1（有 cp311 manylinux wheel，匹配 isaacsim5.1 的 Python 3.11）。
# retargeters 额外依赖（torch2.7/pinocchio 等 ~2GB）v1 关节速率映射用不到，
# 需要时再补：pip install 'isaacteleop[retargeters]==1.5.144rc1'
micromamba run -n "$DST_ENV" pip install 'isaacteleop[cloudxr]==1.5.144rc1'

echo "==> 验证（宽松：只提示不中断）"
micromamba run -n "$DST_ENV" python - <<'EOF'
for name in ("isaacteleop", "isaaccapture"):
    try:
        mod = __import__(name)
        print(f"{name} 导入 OK，版本:", getattr(mod, "__version__", "?"))
        print("顶层成员:", [m for m in dir(mod) if not m.startswith("_")][:15])
    except Exception as e:
        print(f"提示: {name} 导入失败:", e)
EOF

cat <<'TIP'

安装完成。防火墙需放行（CloudXR）：
  sudo ufw allow 47998/udp && sudo ufw allow 49100/tcp && sudo ufw allow 48322/tcp

之后步骤见 scripts/teleop/README.md：
  1. 用克隆环境启动 RouteManager（先停掉旧服务）：
     ISAAC_PYTHON="micromamba run -n isaacsim5.1_teleop python" ./scripts/run.sh
  2. 加载机器人后 POST /execute 调用 start_vr_teleop 开启遥操作
  3. Pico 4 浏览器打开 https://nvidia.github.io/IsaacTeleop/client
TIP
