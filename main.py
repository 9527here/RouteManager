"""
main.py — RouteManager 入口
===========================

启动顺序（Isaac 硬性约束）：
  1. SimulationApp（SimEnv 内部完成）
  2. 之后才能 import RobotManager（内部 isaacsim 模块都是函数内延迟导入）
  3. IsaacTaskServer(executor).start()

退出清理：
  - 退出信号（Ctrl+C / 关终端 / kill）由 IsaacTaskServer 接管，优雅关闭
  - 启动前若发现上次残留实例占用端口，自动清理后再起服务
"""

import logging
import os
import re
import subprocess
import sys
import time

import standalone_server
from standalone_server import IsaacTaskServer
from sim_env import SimEnv

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("route_manager.main")

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))


def _pids_listening_on(port):
    """返回监听指定端口的进程 pid 列表（解析 ss -tlnp，本用户进程可见）。"""
    try:
        out = subprocess.run(
            ["ss", "-tlnp", f"sport = :{port}"],
            capture_output=True, text=True, timeout=5,
        ).stdout
    except Exception as e:
        logger.warning(f"查询端口占用失败（{e}），跳过残留实例清理")
        return []
    return sorted({int(m.group(1)) for m in re.finditer(r"pid=(\d+)", out)})


def _process_info(pid):
    """读取进程的 (cmdline, cwd)，进程已消失时返回 ('', '')。"""
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            cmdline = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
        cwd = os.path.realpath(f"/proc/{pid}/cwd")
        return cmdline, cwd
    except OSError:
        return "", ""


def cleanup_stale_server(port, timeout=8.0):
    """清理占用端口的残留实例：本项目进程 SIGTERM 优雅关闭（超时 SIGKILL），其他程序则报错退出。"""
    pids = [p for p in _pids_listening_on(port) if p != os.getpid()]
    if not pids:
        return

    stale, foreign = [], []
    for pid in pids:
        cmdline, cwd = _process_info(pid)
        is_stale = cwd == PROJECT_DIR or "main.py" in cmdline
        (stale if is_stale else foreign).append((pid, cmdline))

    for pid, cmdline in stale:
        logger.warning(f"发现残留实例 pid={pid} ({cmdline})，正在清理...")
        try:
            os.kill(pid, 15)  # SIGTERM，让它走优雅关闭路径
            deadline = time.time() + timeout
            while time.time() < deadline and os.path.exists(f"/proc/{pid}"):
                time.sleep(0.2)
            if os.path.exists(f"/proc/{pid}"):
                logger.warning(f"pid={pid} 未响应 SIGTERM，强制结束")
                os.kill(pid, 9)
            logger.info(f"残留实例 pid={pid} 已清理")
        except ProcessLookupError:
            pass  # 等待期间自己退出了

    if foreign:
        detail = "; ".join(f"pid={p} ({c})" for p, c in foreign)
        logger.error(
            f"端口 {port} 被其他程序占用: {detail}，"
            f"可用 ROUTE_MANAGER_PORT=<端口> 换端口，或手动处理后重启")
        sys.exit(1)

    time.sleep(0.5)  # 等内核回收端口


def main():
    headless = os.environ.get("HEADLESS", "0") == "1"
    port = int(os.environ.get("ROUTE_MANAGER_PORT", "18765"))

    # 0. 清理上次未正常退出的残留实例（必须在 SimulationApp 之前，避免白等 1~3 分钟加载）
    cleanup_stale_server(port)

    logger.info(f"正在初始化 Isaac Sim (headless={headless})...")

    # 1. 最先启动 SimulationApp + 插件（ROS2 Bridge 等）
    sim_env = SimEnv(headless=headless)

    # 2. SimulationApp 之后才能构造执行器（内部会创建 World）
    from robot_manager import RobotManager

    executor = RobotManager(simulation_app=sim_env.simulation_app, headless=headless)

    # 3. 可选：从 config.json 加载 action 默认参数
    config_path = os.path.join(os.path.dirname(__file__), "config.json")
    if os.path.exists(config_path):
        standalone_server.load_config_file(config_path)

    # 4. 接上 HTTP 框架并阻塞运行（默认 127.0.0.1:18765，可用 ROUTE_MANAGER_PORT 改）
    # 资产加载类 action 完成后会等 stage 加载收敛再返回
    server = IsaacTaskServer(
        executor, port=port,
        loading_tasks={"load_scene", "load_urdf", "load_robot", "add_camera"})
    logger.info(f"HTTP 服务启动: http://127.0.0.1:{port} (POST /execute, /python; GET /health)")
    server.start()


if __name__ == "__main__":
    main()
