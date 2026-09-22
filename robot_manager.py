"""
robot_manager.py — 执行器（executor）
====================================

所有 Isaac 操作都在这里实现。每个公开方法就是一个 HTTP action
（standalone_server 靠反射分发），所有方法都在仿真主线程被调用，
可放心操作 stage / 物理。

模块顶层不 import 任何 isaacsim/omni/pxr —— 保证可以先于
SimulationApp 被加载（Isaac 硬性约束，见 standalone_server.py docstring）。
"""

import logging
import os

import numpy as np

from ros2_bridge.controller import Ros2JointController

logger = logging.getLogger("route_manager.robot")

# 图像结果转 base64（standalone_server 的序列化注册表）
from standalone_server import register_serializer


@register_serializer("get_camera_image")
def _serialize_camera_image(rgb):
    import base64
    import cv2

    _, buffer = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    return {
        "image_base64": base64.b64encode(buffer).decode("utf-8"),
        "width": int(rgb.shape[1]),
        "height": int(rgb.shape[0]),
    }


class RobotManager:
    """执行器：URDF 导入、相机、ROS2 bridge 的动作实现。"""

    # standalone_server 约定接口：simulation_app（None = 未启动）
    simulation_app = None

    def __init__(self, simulation_app, headless=False):
        from isaacsim.core.api.world import World

        self.simulation_app = simulation_app
        self.world = World(stage_units_in_meters=1.0)
        self.world.reset()

        self.articulation = None      # URDF 导入后的机器人
        self.robot_prim_path = None
        self._cameras = {}            # prim_path -> Camera sensor
        self._ros_controller = None

        logger.info("RobotManager 初始化完成（World 已创建并 reset）")

    # ==================== 生命周期（standalone_server 约定接口） ====================

    def start(self):
        self.world.reset()

    def step(self, render=True):
        self.world.step(render=render)

    def close(self):
        if self._ros_controller is not None:
            self._ros_controller.shutdown()
        self.world.stop()
        self.simulation_app.close()

    # ==================== 场景 ====================

    def load_scene(self, usd_path: str = None, prim_path: str = "/World"):
        """加载场景 USD；不传 usd_path 时创建默认地平面。"""
        from isaacsim.core.utils.stage import add_reference_to_stage

        if usd_path:
            if not os.path.exists(usd_path):
                raise FileNotFoundError(f"Scene USD not found: {usd_path}")
            add_reference_to_stage(usd_path, prim_path)
            logger.info(f"场景已加载: {usd_path} -> {prim_path}")
        else:
            self._create_ground_plane()
            logger.info("已创建默认地平面")

    def _create_ground_plane(self):
        """创建无限大地平面（带碰撞），适合人形/机械臂调试。"""
        from pxr import UsdGeom, UsdPhysics
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        UsdGeom.Xform.Define(stage, "/World/ground_plane")
        plane = UsdGeom.Plane.Define(stage, "/World/ground_plane/col")
        UsdPhysics.CollisionAPI.Apply(plane.prim)

    # ==================== URDF ====================

    def load_urdf(self, urdf_path: str, prim_path: str = "/humanoid",
                  dest_path: str = None, fix_base: bool = True,
                  merge_fixed_joints: bool = False, self_collision: bool = False):
        """
        导入 URDF 为 USD 并加载到场景，注册为 Articulation。

        :param urdf_path: URDF 文件路径
        :param prim_path: 场景挂载路径
        :param dest_path: 生成的 USD 保存路径（默认与 URDF 同目录同名 .usd）
        :param fix_base: 固定基座（人形通常 False；固定臂/台装设备 True）
        :param merge_fixed_joints: 合并固定关节（调试时建议 False，保留完整结构）
        :param self_collision: 自碰撞
        :return: {"usd_path": ..., "prim_path": ..., "joint_names": [...]}
        """
        from isaacsim.asset.importer.urdf import _urdf
        from isaacsim.core.prims import Articulation
        from isaacsim.core.utils.stage import add_reference_to_stage

        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF not found: {urdf_path}")

        if dest_path is None:
            dest_path = os.path.splitext(urdf_path)[0] + ".usd"

        urdf_interface = _urdf.acquire_urdf_interface()
        import_config = _urdf.ImportConfig()
        import_config.fix_base = fix_base
        import_config.merge_fixed_joints = merge_fixed_joints
        import_config.self_collision = self_collision
        import_config.import_inertia_tensor = True
        # 位置驱动（DRIVE_ANGLE）：可以直接 set_joint_positions 控制关节
        import_config.default_drive_type = _urdf.UrdfJointTarget.DRIVE_ANGLE

        result = urdf_interface.parse_urdf(urdf_path, dest_path, import_config)
        logger.info(f"URDF 已转换为 USD: {dest_path}")

        # 不同 Isaac 版本的 parse_urdf 可能已自动挂到 stage（result.prim_path），
        # 也可能只落盘 —— 统一确保以用户指定的 prim_path 存在于场景中
        from pxr import Usd
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        staged_path = getattr(result, "prim_path", None)
        if not (staged_path and stage.GetPrimAtPath(staged_path) and
                stage.GetPrimAtPath(staged_path).IsValid()):
            add_reference_to_stage(dest_path, prim_path)
            staged_path = prim_path

        self.robot_prim_path = staged_path

        # 包装为 Articulation 以便控制关节
        self.articulation = Articulation(prim_path=staged_path, name="humanoid")
        self.world.scene.add(self.articulation)
        if self.world.is_playing():
            self.articulation.initialize()

        joint_names = list(self.articulation.dof_names) if self.articulation else []
        logger.info(f"机器人已加载: {staged_path}，关节数: {len(joint_names)}")
        return {"usd_path": dest_path, "prim_path": staged_path, "joint_names": joint_names}

    def get_joint_names(self):
        """列出当前机器人的关节名与 DOF 索引。"""
        self._require_robot()
        return {
            "prim_path": self.robot_prim_path,
            "dof_names": list(self.articulation.dof_names),
            "num_dofs": int(self.articulation.num_dof),
        }

    def set_joint_positions(self, positions: list, joint_names: list = None):
        """直接设置关节目标位置（需要 physics playing）。"""
        self._require_robot()
        self.articulation.set_joint_positions(
            positions, joint_indices=None if joint_names is None
            else [self.articulation.get_dof_index(n) for n in joint_names])
        return {"status": "ok"}

    def get_joint_positions(self):
        self._require_robot()
        return {"positions": [float(v) for v in self.articulation.get_joint_positions()]}

    # ==================== 相机 ====================

    def add_camera(self, prim_path: str, name: str = None, position: list = None,
                   rotation_euler: list = None, resolution: list = None,
                   focal_length: float = 24.0, clipping_range: list = None):
        """
        在场景中创建一个相机传感器。

        :param prim_path: 相机 prim 路径，如 "/humanoid/head_camera"
        :param rotation_euler: 欧拉角 [roll, pitch, yaw]（度），None=朝 -Z 默认朝向
        :param resolution: [宽, 高]，默认 [1280, 720]
        """
        from isaacsim.sensors.camera import Camera
        from isaacsim.core.utils.rotations import euler_angles_to_quat
        from pxr import UsdGeom
        import omni.usd

        resolution = resolution or [1280, 720]
        clipping_range = clipping_range or [0.05, 100.0]
        name = name or prim_path.split("/")[-1]

        stage = omni.usd.get_context().get_stage()
        UsdGeom.Xform.Define(stage, prim_path)
        UsdGeom.Camera.Define(stage, prim_path)

        cam = Camera(
            prim_path=prim_path,
            name=name,
            resolution=tuple(resolution),
            focal_length=focal_length,
            clipping_range=tuple(clipping_range),
        )
        self.world.scene.add(cam)
        if self.world.is_playing():
            cam.initialize()

        if position is not None:
            kwargs = {"position": np.array(position, dtype=np.float32)}
            if rotation_euler is not None:
                kwargs["orientation"] = euler_angles_to_quat(
                    np.array(rotation_euler, dtype=np.float32), degrees=True)
            cam.set_world_pose(**kwargs)

        self._cameras[prim_path] = cam
        logger.info(f"相机已创建: {prim_path} (分辨率 {resolution}，焦距 {focal_length}mm)")
        return {"prim_path": prim_path, "resolution": resolution}

    def get_camera_image(self, prim_path: str):
        """抓取相机 RGB 图（返回 base64 JPEG，见文件顶部的序列化器）。"""
        cam = self._cameras.get(prim_path)
        if cam is None:
            raise ValueError(f"相机 '{prim_path}' 不存在，请先 add_camera。"
                             f"已有: {list(self._cameras)}")
        if not self.world.is_playing():
            self.world.reset()
        # 主循环持续 step(render=True)，annotator 缓冲已有数据
        rgb = cam.get_rgba()
        if rgb is None or rgb.size == 0:
            raise RuntimeError("get_rgba 返回空，渲染尚未收敛，请稍后重试")
        return rgb

    def move_camera(self, prim_path: str, position: list, rotation_euler: list = None):
        """移动相机。rotation_euler: [roll, pitch, yaw]（度）。"""
        from isaacsim.core.utils.rotations import euler_angles_to_quat

        cam = self._cameras.get(prim_path)
        if cam is None:
            raise ValueError(f"相机 '{prim_path}' 不存在")
        kwargs = {"position": np.array(position, dtype=np.float32)}
        if rotation_euler is not None:
            kwargs["orientation"] = euler_angles_to_quat(
                np.array(rotation_euler, dtype=np.float32), degrees=True)
        cam.set_world_pose(**kwargs)
        return {"prim_path": prim_path}

    # ==================== ROS2 Bridge ====================

    def setup_ros2_bridge(self, joint_names: list = None,
                          command_topic: str = "/humanoid/joint_command",
                          state_topic: str = "/humanoid/joint_states",
                          node_name: str = "route_manager_robot",
                          joint_limits: dict = None):
        """
        创建 ROS2 关节控制节点（发布指令 / 订阅反馈）。
        需要 source ROS2 环境后启动 server；失败会降级不阻塞仿真。

        :param joint_names: 默认取 URDF 导入后的全部 DOF 名
        :param joint_limits: {joint_name: {"lower": l, "upper": u}}
        """
        if joint_names is None:
            self._require_robot()
            joint_names = list(self.articulation.dof_names)

        self._ros_controller = Ros2JointController(
            joint_names=joint_names,
            command_topic=command_topic,
            state_topic=state_topic,
            node_name=node_name,
            joint_limits=joint_limits,
        )
        return {
            "initialized": self._ros_controller.is_initialized(),
            "joint_names": joint_names,
            "command_topic": command_topic,
            "state_topic": state_topic,
        }

    def send_joint_command(self, positions: list, duration: float = 0.0):
        """通过 ROS2 发送关节指令。duration>0 时平滑插值（阻塞）。"""
        if self._ros_controller is None:
            raise RuntimeError("ROS2 bridge 未创建，请先调用 setup_ros2_bridge")
        self._ros_controller.send_joint_command(positions, duration=duration)
        return {"sent": True}

    def get_joint_states(self):
        """获取 ROS2 反馈的关节位置（与仿真 set_joint_positions 相互独立）。"""
        if self._ros_controller is None:
            raise RuntimeError("ROS2 bridge 未创建，请先调用 setup_ros2_bridge")
        return {
            "cmd": self._ros_controller.get_cmd_positions(),
            "feedback": self._ros_controller.get_feedback_positions(),
            "feedback_received": self._ros_controller.is_feedback_received(),
        }

    # ==================== 内部 ====================

    def _require_robot(self):
        if self.articulation is None:
            raise RuntimeError("机器人未加载，请先调用 load_urdf")
