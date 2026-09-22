"""
main.py — RouteManager 入口
===========================

启动顺序（Isaac 硬性约束）：
  1. SimulationApp（SimEnv 内部完成）
  2. 之后才能 import RobotManager（内部 isaacsim 模块都是函数内延迟导入）
  3. IsaacTaskServer(executor).start()
"""

import logging
import os

import standalone_server
from standalone_server import IsaacTaskServer
from sim_env import SimEnv

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("route_manager.main")


def main():
    headless = os.environ.get("HEADLESS", "0") == "1"
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

    # 4. 接上 HTTP 框架并阻塞运行（127.0.0.1:8765）
    server = IsaacTaskServer(executor)
    logger.info("HTTP 服务启动: http://127.0.0.1:8765 (POST /execute, /python; GET /health)")
    server.start()


if __name__ == "__main__":
    main()
