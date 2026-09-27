"""
sim_env.py — SimulationApp 启动与插件管理
=========================================

Isaac 硬性约束：SimulationApp 必须最先创建，之后才能 import 任何
isaacsim / omni / pxr 模块。ROS2 Bridge 等插件的启用也必须在
SimulationApp 之后、其他模块之前完成。
"""

import logging

logger = logging.getLogger("route_manager.sim_env")


class SimEnv:
    """封装 SimulationApp 的启动与必要插件的启用。"""

    def __init__(self, headless=False):
        logger.info(f"正在启动 SimulationApp，headless={headless}")
        from isaacsim import SimulationApp

        self.simulation_app = SimulationApp({"headless": headless})
        self._enable_plugins()
        logger.info("SimulationApp 初始化完成")

    def _enable_plugins(self):
        import omni.kit.app

        ext_manager = omni.kit.app.get_app().get_extension_manager()

        # ROS2 Bridge：关节指令/状态话题的收发都靠它。
        # headless / 无 ROS 环境下插件可能加载失败 —— 降级运行（ROS2 相关
        # action 会报"未初始化"），不让整个服务起不来
        logger.info("正在加载 ROS2 Bridge 插件...")
        try:
            ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
        except Exception as e:
            logger.warning(f"ROS2 Bridge 插件加载失败（ROS2 功能降级，不影响仿真）: {e}")

        # Debug Draw：调试可视化（骨架、路径、bbox）
        try:
            logger.info("正在加载 Debug Draw 插件...")
            ext_manager.set_extension_enabled_immediate("isaacsim.util.debug_draw", True)
        except Exception as e:
            logger.debug(f"Debug Draw 插件加载跳过: {e}")

        # Omnigraph：ROS2 Camera Helper 等 graph 节点的底层
        for ext in ["omni.graph", "omni.graph.nodes", "omni.graph.action", "omni.graph.ui"]:
            try:
                ext_manager.set_extension_enabled_immediate(ext, True)
                logger.debug(f"Omnigraph 插件已加载: {ext}")
            except Exception:
                logger.debug(f"Omnigraph 插件加载跳过: {ext}")

    def get_app(self):
        return self.simulation_app

    def close(self):
        logger.info("正在关闭 SimulationApp...")
        self.simulation_app.close()
        logger.info("SimulationApp 已关闭")
