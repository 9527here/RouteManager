"""
sim_env.py — SimulationApp 启动与插件管理
=========================================

Isaac 硬性约束：SimulationApp 必须最先创建，之后才能 import 任何
isaacsim / omni / pxr 模块。ROS2 Bridge 等插件的启用也必须在
SimulationApp 之后、其他模块之前完成。
"""

import logging
import os

logger = logging.getLogger("route_manager.sim_env")


class SimEnv:
    """封装 SimulationApp 的启动与必要插件的启用。"""

    def __init__(self, headless=False):
        logger.info(f"正在启动 SimulationApp，headless={headless}")
        from isaacsim import SimulationApp

        config = {"headless": headless}
        # RM_EXTRA_ARGS：给 Kit 追加 `--/setting=value` 形式的临时参数，排查用。
        #   RM_EXTRA_ARGS="--/renderer/multiGpu/enabled=False" ./scripts/run.sh
        extra = os.environ.get("RM_EXTRA_ARGS", "").split()
        if extra:
            config["extra_args"] = extra
            logger.info(f"追加 Kit 参数: {extra}")

        self.simulation_app = SimulationApp(config)
        self._enable_plugins()
        logger.info("SimulationApp 初始化完成")

    def _enable_plugins(self):
        import omni.kit.app

        ext_manager = omni.kit.app.get_app().get_extension_manager()

        # 人物运行时须在World第一次Play前预载，热加载到暂停现场会漏掉动画启动事件。
        # 显式开关保持普通仿真启动开销不变；场景与人物仍由attach入口附着。
        people_root = os.environ.get("ROUTE_MANAGER_SIMPEOPLE_ROOT")
        if people_root:
            from pathlib import Path
            extensions = Path(people_root).expanduser().resolve() / "extensions"
            if not (extensions / "rosclaw.simpeople/config/extension.toml").is_file():
                raise RuntimeError(f"SimPeople扩展目录不存在: {extensions}")
            ext_manager.add_path(str(extensions))
            for extension in ("omni.anim.behavior.core", "omni.anim.navigation.core",
                              "omni.anim.graph.core", "omni.anim.skelJoint"):
                if not ext_manager.set_extension_enabled_immediate(extension, True):
                    raise RuntimeError(f"人物原生运行时预载失败: {extension}")
            import omni.usd
            self._people_stage_subscription = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
                self._prepare_people_timeline, name="人物动画场景时间范围")
            logger.info("已预载SimPeople原生运行时，等待附着现有场景")

        # ROS2 Bridge：关节指令/状态话题的收发都靠它。
        # headless / 无 ROS 环境下插件可能加载失败 —— 降级运行（ROS2 相关
        # action 会报"未初始化"），不让整个服务起不来
        logger.info("正在加载 ROS2 Bridge 插件...")
        try:
            ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
        except Exception as e:
            logger.warning(f"ROS2 Bridge 插件加载失败（ROS2 功能降级，不影响仿真）: {e}")

        # ROS2 节点定义（ROS2CameraHelper / ROS2CameraInfoHelper 等）。
        # **6.2 必须单独启用这个扩展**：只开 isaacsim.ros2.bridge 的话，日志里会
        # 出现 "Creating unregistered node type isaacsim.ros2.bridge.ROS2CameraHelper
        # (version 4)"，节点类型注册不上，相机 OmniGraph 建不出来。
        try:
            ext_manager.set_extension_enabled_immediate("isaacsim.ros2.nodes", True)
            logger.info("已加载 ROS2 节点扩展（相机/时钟 OmniGraph 节点）")
        except Exception as e:
            logger.warning(f"ROS2 节点扩展加载失败（相机话题不可用）: {e}")

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

    def _prepare_people_timeline(self, event):
        """在新场景建物理之前处理零长度台架时间范围，避免运行中改帧率/重建后端。"""
        import omni.usd
        import omni.timeline
        from pxr import Usd
        if event.type != int(omni.usd.StageEventType.OPENED):
            return
        stage = omni.usd.get_context().get_stage()
        if stage is not None and stage.GetEndTimeCode() <= 0:
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                omni.timeline.get_timeline_interface().set_end_time(100000.)
            logger.info("已扩展本会话台架时间范围，源USD未写回")

    def get_app(self):
        return self.simulation_app

    def close(self):
        logger.info("正在关闭 SimulationApp...")
        self.simulation_app.close()
        logger.info("SimulationApp 已关闭")
