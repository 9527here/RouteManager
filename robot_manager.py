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
import time

import numpy as np

import arm_kin
import grasp_planner
import gripper_model
from ros2_bridge.controller import Ros2JointController, lerp, smoothstep

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


# 底盘驱动轮（skid-steer 四轮差速）。这些关节由 set_base_velocity 速度控制
# 专管；关节位置指令（ROS/HTTP）里的轮子分量会被忽略，避免两路控制打架。
_BASE_WHEEL_JOINTS = ("front_left_wheel_joint", "front_right_wheel_joint",
                      "rear_left_wheel_joint", "rear_right_wheel_joint")

_STATIC_CAMERAS = (
    ("head_zed_x", 960, 600),
    ("head_surround_left", 1920, 1200),
    ("head_surround_right", 1920, 1200),
    ("head_surround_rear", 1920, 1200),
    ("chest_gemini_335lg", 1280, 800),
    ("left_wrist_gemini_335lg", 1280, 800),
    ("right_wrist_gemini_335lg", 1280, 800),
    ("tv_cam", 1280, 800),
)
#: 需要 camera.initialize()(建 render product)的相机 —— 只有 initialize 过的
#: 才能 get_camera_image 出图。腕部相机是抓取/操作调试最关键的一路(头部相机
#: 近处会被自己的胸部挡住), tv_cam 是第三人称全局视角。
_STATIC_RENDER_CAMERAS = {"head_zed_x", "left_wrist_gemini_335lg", "tv_cam"}


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
        self._attached_cameras = {}   # prim_path -> attach_to（按本地姿态移动）
        self._ros_controller = None
        self._vr_teleop = None
        self._config = self._load_config()
        render_cfg = self._config.get("render") or {}
        raw_render_interval = os.environ.get(
            "ROUTE_MANAGER_RENDER_INTERVAL",
            render_cfg.get("interval", 5),
        )
        self._render_interval = max(1, int(raw_render_interval))
        self._render_counter = 0
        self._mimic_map = None       # 软件 mimic 表 {leader_idx: [(follower_idx, gearing)]}
        self._mimic_targets = None   # {leader_idx: 最近一次指令目标}
        # 夹爪全开时的主动关节角(rad)，与 URDF 主动关节上限 0.7 一致；
        # start_vr_teleop 会用 config.json teleop.grip_open 覆盖，保证
        # VR 扳机 / set_gripper_position / 真机 rm 指令三者同一刻度。
        self._vr_grip_open = float(
            (self._config.get("teleop") or {}).get("grip_open", 0.7))
        # 夹爪行程(mm)：position 1000 ↔ 该值。仿真 86.2（实测满行程），
        # 真机 rm 夹爪 70 —— 见 gripper_model.STROKE_MM 的说明。
        _stroke = (self._config.get("gripper") or {}).get("stroke_mm")
        if _stroke:
            gripper_model.STROKE_MM = float(_stroke)
        self._auto_pick = None         # 自动抓取状态机（VR 菜单键触发）
        self._last_auto_pick = None    # 上一次自动抓取的结果摘要
        self._base_cmd = None          # 底盘速度指令 {"vx","wz","t"}（set_base_velocity）
        self._wheel_pos_target = None  # 驱动轮积分位置目标（虚拟速度模式）
        self._base_cfg = self._config.get("base_teleop") or {}

        # ROS2 状态发布限频 + send_joint_command 的平滑插值状态机
        self._state_publish_period = 0.02   # 50Hz，setup_ros2_bridge 可改
        self._last_state_publish = 0.0
        self._motion = None           # {"start","target","t0","duration"}

        logger.info("RobotManager 初始化完成（World 已创建并 reset）")

    # ==================== 生命周期（standalone_server 约定接口） ====================

    def start(self):
        self.world.reset()

    def step(self, render=None):
        """主循环一帧：ROS2 指令 → 插值推进 → 软件 mimic → 底盘速度 → VR 目标 → 物理步进。

        render=None 表示主循环自动降频渲染（默认每 5 个物理步渲染一次）；
       显式传 True/False 时按调用方要求执行，供调试脚本和离线渲染使用。
        """
        self._process_ros_commands()
        self._advance_motion()
        self._update_base_velocity()
        if self._vr_teleop is not None:
            # VR 左手 **X 键** → 启动自动抓取（一次性，边沿触发）
            try:
                if self._vr_teleop.consume_menu_request():
                    res = self.auto_pick(side="left")
                    logger.info(f"X 键触发自动抓取: {res}")
            except Exception as e:
                logger.warning(f"X 键处理异常: {e!r}")
            self._vr_teleop.apply_pending()   # VR 手柄目标（主线程施加）
        self._auto_pick_step()
        # mimic 放在 VR 之后: 用**当帧**的主动关节指令算跟随关节, 零滞后。
        # (放在 VR 之前会晚一帧, 夹爪高速开合时能看到跟随指头慢半拍)
        self._apply_software_mimic()
        should_render = True if render is None else bool(render)
        if render is None and self._render_interval > 1:
            self._render_counter += 1
            if self._render_counter < self._render_interval:
                should_render = False
            else:
                self._render_counter = 0
        self.world.step(render=should_render)
        self._publish_ros_state()

    def set_render_interval(self, interval: int):
        """运行时调整自动渲染频率；1 表示每个物理步都渲染。"""
        self._render_interval = max(1, int(interval))
        self._render_counter = 0
        return {"interval": self._render_interval}

    def get_render_settings(self):
        return {
            "interval": self._render_interval,
            "mode": "auto",
            "env_override": "ROUTE_MANAGER_RENDER_INTERVAL",
        }

    def _register_static_cameras(self):
        """把静态场景里已有的 Camera prim 包装成可读图传感器。"""
        import omni.usd
        from isaacsim.sensors.camera import Camera

        stage = omni.usd.get_context().get_stage()
        registered = []
        for name, width, height in _STATIC_CAMERAS:
            path = f"/World/cams/{name}"
            if path in self._cameras or not stage.GetPrimAtPath(path).IsValid():
                continue
            camera = Camera(
                prim_path=path,
                name=f"static_{name}",
                resolution=(width, height),
            )
            if name in _STATIC_RENDER_CAMERAS:
                camera.initialize()
            self._cameras[path] = camera
            registered.append(path)
        if registered:
            logger.info(f"静态相机已注册: {registered}")

    def _apply_static_camera_offsets(self):
        """按已标定偏移把静态相机放到对应机器人连杆上。"""
        import json
        import omni.usd
        from pxr import Gf

        offset_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "scripts", "scene", "cam_follow_offsets.json",
        )
        if not os.path.exists(offset_path) or self.articulation is None:
            return
        try:
            with open(offset_path, encoding="utf-8") as file:
                offsets = json.load(file)
            stage = omni.usd.get_context().get_stage()
            link_transforms = np.asarray(
                self.articulation._physics_view.get_link_transforms()
            ).reshape(-1, 7)
            for name, config in offsets.items():
                path = f"/World/cams/{name}"
                prim = stage.GetPrimAtPath(path)
                if not prim.IsValid() or "parent" not in config or "off" not in config:
                    continue
                link_index = int(self.articulation.get_link_index(config["parent"]))
                if link_index < 0 or link_index >= link_transforms.shape[0]:
                    continue
                position, quaternion = np.split(link_transforms[link_index], [3])
                quaternion = quaternion / max(float(np.linalg.norm(quaternion)), 1e-9)
                w, x, y, z = quaternion
                rotation = np.array([
                    [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),
                     2 * (x * z + w * y)],
                    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z),
                     2 * (y * z - w * x)],
                    [2 * (x * z - w * y), 2 * (y * z + w * x),
                     1 - 2 * (x * x + y * y)],
                ])
                link_pose = np.eye(4)
                link_pose[:3, :3] = rotation
                link_pose[:3, 3] = position
                camera_pose = link_pose @ np.asarray(config["off"], dtype=float)
                camera_quaternion = self._matrix_to_quaternion(camera_pose[:3, :3])
                prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(
                    *map(float, camera_pose[:3, 3])))
                prim.GetAttribute("xformOp:orient").Set(Gf.Quatd(
                    float(camera_quaternion[0]),
                    Gf.Vec3d(*map(float, camera_quaternion[1:])),
                ))
        except Exception as e:
            logger.warning(f"静态相机偏移应用失败: {e}")

    @staticmethod
    def _matrix_to_quaternion(rotation):
        trace = float(np.trace(rotation))
        if trace > 0.0:
            scale = np.sqrt(trace + 1.0) * 2.0
            return np.array([
                0.25 * scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ])
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = np.sqrt(1.0 + rotation[0, 0]
                            - rotation[1, 1] - rotation[2, 2]) * 2.0
            return np.array([
                (rotation[2, 1] - rotation[1, 2]) / scale,
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
            ])
        if index == 1:
            scale = np.sqrt(1.0 + rotation[1, 1]
                            - rotation[0, 0] - rotation[2, 2]) * 2.0
            return np.array([
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
            ])
        scale = np.sqrt(1.0 + rotation[2, 2]
                        - rotation[0, 0] - rotation[1, 1]) * 2.0
        return np.array([
            (rotation[1, 0] - rotation[0, 1]) / scale,
            (rotation[0, 2] + rotation[2, 0]) / scale,
            (rotation[1, 2] + rotation[2, 1]) / scale,
            0.25 * scale,
        ])

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
            self._load_static_scene()
            logger.info("已创建默认地平面 + 静态场景")

    def _load_static_scene(self):
        """加载烘焙的静态场景(相机/桌子/抓取物体/参考球)。

        必须在机器人之前加载: 机器人加载后任何 prim 编辑(加相机/桌子)都会让
        物理后端失效, 之后 set_joint_position_targets / get_jacobians / 底盘
        读位置全部报 "Failed to ... from backend"。文件由
        scripts/scene/gen_static_usda.py 生成(值取自一次摆放好的场景)。
        """
        import omni.usd
        from pxr import Sdf

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "scripts", "scene", "static_scene.usda")
        if not os.path.exists(path):
            logger.warning(f"静态场景文件不存在, 跳过: {path}")
            return
        try:
            stage = omni.usd.get_context().get_stage()
            world = stage.GetPrimAtPath("/World")
            world.GetReferences().AddReference(path, Sdf.Path("/World"))
            logger.info(f"静态场景已加载: {path}")
        except Exception as e:
            logger.warning(f"静态场景加载失败(不影响机器人): {e}")

    def _create_ground_plane(self):
        """创建地平面（带视觉网格与碰撞）。幂等：已存在则跳过（重复 load_scene 不再报错）。"""
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath("/World/ground_plane").IsValid():
            logger.info("地平面已存在，跳过创建")
            return
        try:
            from isaacsim.core.api.objects import GroundPlane

            plane = GroundPlane(prim_path="/World/ground_plane", size=20.0,
                                color=np.array([0.5, 0.5, 0.5]))
            self.world.scene.add(plane)
            # 地面高度保持默认 z=0: 扫描场景改为整体下移使其地面贴合 z=0
            # (2026-09-29: 抬地面会让机器人跟着升高、桌子留原地 → 不对齐)
        except Exception as e:
            # 兜底：仅碰撞平面（无视觉），保证任何 Isaac 版本都可用
            logger.warning(f"GroundPlane 创建失败，退化为纯碰撞平面: {e}")
            from pxr import UsdGeom, UsdPhysics

            UsdGeom.Xform.Define(stage, "/World/ground_plane")
            plane = UsdGeom.Plane.Define(stage, "/World/ground_plane/col")
            UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
        logger.info("已创建默认地平面")

    #: 场景摩擦系数。**不设的话 PhysX 默认值在本场景实测 ≈0.05**（几乎无摩擦）：
    #: 夹爪压住物体也夹不住（一抬就滑脱）、机器人轮子在地面打滑（实测底座
    #: 被机械臂反作用力推走 0.2m，目标跑出手臂可达范围）。2026-10-09 实测。
    _FRICTION = 1.0

    def _ensure_friction_material(self):
        """建一个高摩擦 PhysicsMaterial 并绑到地面（幂等）。

        场景物体(桌子/方块/圆柱)的绑定在 `scripts/scene/static_scene.usda` 里；
        这里只补运行时创建的地面。夹爪指垫在机器人资产里，PhysX 按两侧材质
        取平均，地面/物体有 1.0 就够。
        """
        import omni.usd
        from pxr import Usd, UsdShade, UsdPhysics

        stage = omni.usd.get_context().get_stage()
        mpath = "/World/PhysicsMaterials/friction_mat"
        mat = UsdShade.Material.Define(stage, mpath)
        mapi = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        mapi.CreateStaticFrictionAttr(self._FRICTION)
        mapi.CreateDynamicFrictionAttr(self._FRICTION)
        mapi.CreateRestitutionAttr(0.0)
        bound = []
        for path in ("/World/ground_plane", "/World/ground_plane/geom",
                     "/World/ground_plane/collisionPlane",
                     "/World/ground_plane/col"):
            p = stage.GetPrimAtPath(path)
            if p.IsValid():
                UsdShade.MaterialBindingAPI.Apply(p).Bind(mat)
                bound.append(path)
        logger.info(f"摩擦材质 {self._FRICTION} 已绑定: {bound}")

    def _ensure_default_environment(self):
        """确保场景有灯光和地面（幂等）。只加载机器人时 GUI 会因无光源全黑，
        机器人（自由底盘）也会坠出视野——加载机器人前自动补齐。"""
        import omni.usd
        from pxr import UsdLux

        stage = omni.usd.get_context().get_stage()
        has_light = any("Light" in (p.GetTypeName() or "")
                        for p in iter(stage.Traverse()))
        if not has_light:
            dome = UsdLux.DomeLight.Define(stage, "/World/default_dome_light")
            dome.CreateIntensityAttr(1000.0)
            sun = UsdLux.DistantLight.Define(stage, "/World/default_sun")
            sun.CreateIntensityAttr(500.0)
            logger.info("已创建默认灯光（DomeLight + DistantLight）")
        self._create_ground_plane()
        try:
            self._ensure_friction_material()
        except Exception as e:
            logger.warning(f"摩擦材质创建失败: {e!r}")

    # ==================== 软件 mimic ====================

    def _build_mimic_map(self):
        """构建软件 mimic 表：夹爪跟随关节 = gearing × 主动关节角度。

        URDF mimic 链扁平化后的倍率规则（与 /humanoid 关节命名对应）：
          *left_main_joint / *left_3_joint → -1.0
          *_1_joint                        → -0.15
          其余                              → +1.0
        PhysX 的 PhysxMimicJointAPI 在该资产上不生效（已从资产剥离），
        由 step() 每帧软件强制执行，确定性有保障。
        """
        names = list(self.articulation.dof_names)
        mapping = {}
        for side in ("left", "right"):
            leader = f"{side}_gripper_right_2_joint"
            if leader not in names:
                continue
            entries = []
            for n in names:
                if not n.startswith(f"{side}_gripper") or n == leader:
                    continue
                if n.endswith(("left_main_joint", "left_3_joint")):
                    g = -1.0
                elif n.endswith("_1_joint"):
                    g = -0.15
                else:
                    g = 1.0
                entries.append((names.index(n), g))
            mapping[names.index(leader)] = entries
        self._mimic_map = mapping or None
        self._mimic_targets = None
        if self._mimic_map:
            n_followers = sum(len(v) for v in mapping.values())
            logger.info(f"软件 mimic 已启用: {len(mapping)} 个主动关节, "
                f"{n_followers} 个跟随关节")
            self._tune_gripper_drives()

    #: 夹爪关节驱动上限力矩(N·m)。资产里**主动关节只有 20、跟随关节 30**，
    #: 主动关节反而更弱 → 被 12 个跟随关节按各自目标"反锁"，实测夹爪只能闭到
    #: leader≈0.35(开口≈41mm)就卡死，60mm 的物体根本夹不到。统一提到 60 后
    #: 空载往返扫描完全重合、可闭到底(leader≈0.003，开口≈0.7mm)。
    _GRIPPER_MAX_FORCE = 60.0

    def _tune_gripper_drives(self):
        """统一夹爪关节驱动力矩上限。

        必须在物理视图健康时调用(load_scene/load_robot 之后即可)。
        """
        try:
            names = list(self.articulation.dof_names)
            idx = [i for i, n in enumerate(names) if "gripper" in n]
            if not idx:
                return
            view = self.articulation._physics_view
            mxf = np.asarray(view.get_dof_max_forces()).reshape(-1).copy()
            before = sorted({round(float(mxf[i]), 1) for i in idx})
            mxf[idx] = self._GRIPPER_MAX_FORCE
            view.set_dof_max_forces(mxf.reshape(1, -1), [0])
            logger.info(f"夹爪驱动上限力矩 → {self._GRIPPER_MAX_FORCE} N·m "
                        f"({len(idx)} 个关节; 之前 {before})")
        except Exception as e:
            logger.warning(f"夹爪驱动调参失败(夹爪可能闭不到底): {e!r}")

    def _apply_software_mimic(self, command_positions=None):
        """按主动关节的指令目标更新跟随关节目标。

        取值的优先级：VR 全量目标数组 → **指令目标**(get_dof_position_targets)
        → 实测位置。用"指令目标"而不是"实测角度"是关键：实测角带物理噪声，
        回灌成目标会被位置驱动放大成高频抽动；而指令是我们的写出去的值，干净。

        每帧都要重算：曾经的实现用 `_mimic_targets is not None` 提前 return，
        结果跟随关节只在第一帧算过一次就永久冻结在初值(≈0)，主关节单独转动
        → 连杆脱节、手指完全不动(2026-10-08 实测)。写回仍然只在变化时发生。
        """
        if not self._mimic_map:
            return
        names = list(self.articulation.dof_names)

        if command_positions is None:
            vr_q = getattr(self._vr_teleop, "_q_all", None) \
                if self._vr_teleop is not None else None
            if vr_q is not None and len(vr_q) == len(names):
                command_positions = vr_q
            else:
                command_positions = None
                try:
                    tgt = np.asarray(
                        self.articulation._physics_view
                        .get_dof_position_targets()).reshape(-1)
                    if tgt.size == len(names) and np.isfinite(tgt).all():
                        command_positions = tgt
                except Exception:
                    command_positions = None
                if command_positions is None:
                    try:
                        command_positions = np.asarray(
                            self.articulation.get_joint_positions()).reshape(-1)
                    except Exception:
                        return

        vals = np.asarray(command_positions, dtype=np.float64).reshape(-1)
        if vals.size != len(names) or np.isnan(vals).any():
            return

        if self._mimic_targets is None:
            self._mimic_targets = {}

        for leader_idx, entries in self._mimic_map.items():
            leader_target = float(vals[leader_idx])
            previous = self._mimic_targets.get(leader_idx)
            if previous is not None and np.isclose(
                    previous, leader_target, rtol=0.0, atol=1e-6):
                continue

            idxs = [i for i, _ in entries]
            raw = np.asarray([leader_target * g for _, g in entries],
                             dtype=np.float32)
            try:
                self.articulation.set_joint_position_targets(
                    raw.reshape(1, -1), joint_indices=idxs)
            except Exception:
                # 缓存**只在写成功后**更新: 失败(如物理视图暂时不可用)时
                # 下一帧还会重试；曾经先写缓存再写关节, 一次失败就永久放弃。
                continue
            self._mimic_targets[leader_idx] = leader_target
            # 同步到 VR 全量写入数组，防止 apply_pending 的 full-write
            # 用 home_q 初值覆盖 follower 目标（这是夹爪只动一根指头的根因）
            if (self._vr_teleop is not None
                    and self._vr_teleop._q_all is not None):
                self._vr_teleop._q_all[leader_idx] = np.float32(leader_target)
                self._vr_teleop._q_all[idxs] = raw

    # ==================== URDF ====================

    def load_urdf(self, urdf_path: str, prim_path: str = "/humanoid",
                  dest_path: str = None, fix_base: bool = False,
                  merge_fixed_joints: bool = False, self_collision: bool = False):
        """
        导入 URDF 为 USD 并加载到场景，注册为 Articulation。

        :param urdf_path: URDF 文件路径
        :param prim_path: 场景挂载路径
        :param dest_path: 生成的 USD 保存路径（默认与 URDF 同目录同名 .usd）
        :param fix_base: 固定基座（人形通常 False；固定臂/台装设备 True）
        :param merge_fixed_joints: 合并固定关节（调试时建议 False，保留完整结构；
            add_camera 的 attach_to 也需要 link prim 存在）
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
        # 位置驱动：可以直接 set_joint_positions 控制关节
        # (Isaac 5.1: 枚举 UrdfJointTargetType.JOINT_DRIVE_POSITION;
        #  旧版是 UrdfJointTarget.DRIVE_ANGLE)
        try:
            import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        except AttributeError:
            import_config.default_drive_type = _urdf.UrdfJointTarget.DRIVE_ANGLE

        # Isaac 5.x 两步流程: parse_urdf(目录, 文件名, cfg) 只内存解析;
        # import_robot 落盘 USD 并返回 articulation root
        # (4.x 的 parse_urdf(urdf_path, dest_path, cfg) 会直接落盘并挂 stage)
        urdf_dir, urdf_file = os.path.split(os.path.abspath(urdf_path))
        urdf_robot = urdf_interface.parse_urdf(urdf_dir, urdf_file, import_config)
        root_in_file = urdf_interface.import_robot(urdf_dir, urdf_file, urdf_robot,
                                                   import_config, dest_path, True)
        # 5.x 产物不带 defaultPrim —— 不补写的话挂载引用会 Unresolved reference。
        # defaultPrim 必须取顶层 Xform（lt10 实测）：指向 articulation root
        # （base_link）的嵌套 defaultPrim 会让 PhysX 展开异常甚至 segfault。
        # 代价是挂载后多一层目录：<prim_path>/base_link/<link>，
        # articulation root 实际路径 = prim_path + basename(root_in_file)。
        from pxr import Usd as _Usd
        prod = _Usd.Stage.Open(dest_path)
        if prod is not None and not prod.GetRootLayer().HasDefaultPrim():
            tops = prod.GetPseudoRoot().GetAllChildren()
            if tops:
                prod.SetDefaultPrim(tops[0])
                prod.GetRootLayer().Save()
                logger.info(f"已补写 USD defaultPrim: {tops[0].GetPath()}")
        result = None
        logger.info(f"URDF 已转换为 USD: {dest_path}")
        # FK/IK 的链来源必须跟着实际加载的模型走（臂可能是 6 轴 RM65 重建版，
        # 而 arm_kin 默认指向 7 轴旧 URDF）。不同步会让抓取规划形状对不上、
        # VR IK 被关节顺序安全闸停用。见 arm_kin.set_urdf 的注释。
        try:
            arm_kin.set_urdf(urdf_path)
            logger.info(f"FK 链来源已切换: {urdf_path}")
        except Exception as e:
            logger.warning(f"切换 FK 链来源失败(保持原样): {e}")

        # 转换后增益烘焙: URDF 无 stiffness 概念、官方臂无 dynamics, 直转产物
        # damping≈0 会纯弹簧振铃。URDF 同目录 <stem>.gains.json 存在时按关节名
        # 烘 stiffness/damping 进 USD (值=USD 级, 见 scene-friction/bake_damping
        # 记忆: 物理 damping ≈ USD 值 × 57.3, 生成侧已折算)
        gains_path = os.path.splitext(urdf_path)[0] + ".gains.json"
        if os.path.exists(gains_path):
            self._bake_joint_gains(dest_path, gains_path)

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
            # defaultPrim=顶层包装 Xform 时, articulation root 在其 base_link 子层
            if root_in_file:
                cand = f"{prim_path}/{os.path.basename(root_in_file.rstrip('/'))}"
                if stage.GetPrimAtPath(cand).IsValid():
                    staged_path = cand

        self.robot_prim_path = staged_path

        # 物理必须先运行: 干净场景直接 load_urdf 时, tensor 后端未启动会让
        # Articulation 构造里的 _physics_view 断言炸 backend=None
        if not self.world.is_playing():
            self.world.reset()
        # 夹爪 mimic 链增益修正(与 _load_robot_usd 对齐, 需在 Articulation 创建前)
        self._fix_gripper_joint_gains(staged_path)
        # 包装为 Articulation 以便控制关节
        # 位置传参：兼容 4.x(prim_path) / 5.x(prim_paths_expr) 两种构造签名
        self.articulation = Articulation(staged_path, name="humanoid")
        self.world.scene.add(self.articulation)
        # 再次 reset 让新 articulation 进入 tensor 视图 —— 之后 dof_names /
        # set_joint_positions / ROS2 指令闭环都立即可用
        self.world.reset()
        self.articulation.initialize()
        self._ensure_default_environment()
        self._build_mimic_map()
        self._apply_home_pose()
        self._register_static_cameras()
        self._apply_static_camera_offsets()
        self._motion = None   # 换机器人后旧插值作废
        self._auto_pick = None  # 换机器人后旧的自动抓取状态也作废

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
        """直接设置关节位置（瞬置语义；平滑控制用 send_joint_command）。"""
        self._require_robot()
        indices = (None if joint_names is None
                   else [self.articulation.get_dof_index(n) for n in joint_names])
        # 5.x view API 要求 (M, K)，单机器人 M=1
        pos = np.asarray(positions, dtype=np.float32).reshape(1, -1)
        self.articulation.set_joint_positions(pos, joint_indices=indices)
        return {"status": "ok"}

    def get_joint_positions(self):
        self._require_robot()
        return {"positions": self._joint_positions_flat()}

    # ==================== 夹爪（睿尔曼 rm_driver 语义） ====================

    #: 主动关节(URDF 里唯一的非 mimic 夹爪关节), 其余 11 个由软件 mimic 跟随
    _GRIPPER_LEADER = {"left": "left_gripper_right_2_joint",
                       "right": "right_gripper_right_2_joint"}

    def set_gripper_position(self, side: str = "both", position: float = 1000,
                             fraction: float = None):
        """夹爪指令，**睿尔曼语义**：position 1~1000 ↔ 开口 0~70mm(1000=全开)。

        与真机 rm_driver 的 `{ns}/set_gripper_position_cmd` 同参同语义，
        所以同一套上层代码（VR 遥操作/抓取脚本）在仿真与真机之间不用改数。
        关节侧只写 **主动关节**，其余 11 个跟随关节由 `_apply_software_mimic`
        按 URDF mimic 链（±1 / -0.15）同一帧补齐——这样连杆不会脱节。

        :param side: left / right / both
        :param position: rm 单位 1~1000（1000=全开）
        :param fraction: 直接给 0~1 **开口**比例（优先于 position，VR 内部用）。
            注意是开口比例、不是主动关节角比例 —— 两者经标定表换算，中间段能差 30%。
        """
        self._require_robot()
        if fraction is None:
            frac = (float(position) - 1.0) / 999.0
        else:
            frac = float(fraction)
        frac = min(max(frac, 0.0), 1.0)

        # 比例 → 主动关节角：走实测标定表，不按线性缩放。开口与关节角是
        # 四连杆的非线性关系，线性近似在中间段能差 5mm 以上——抓取时足以
        # 让指垫压根碰不到物体。
        target = gripper_model.leader_for_fraction(frac)
        names = list(self.articulation.dof_names)
        sides = ("left", "right") if side == "both" else (side,)
        done = {}
        for s in sides:
            leader = self._GRIPPER_LEADER.get(s)
            if not leader or leader not in names:
                continue
            self.articulation.set_joint_position_targets(
                np.asarray([[target]], dtype=np.float32),
                joint_indices=[names.index(leader)])
            done[s] = round(target, 5)
        # 立刻补跟随关节(不等下一帧 step), 保证外部单次调用也是一个刚体
        self._mimic_targets = None
        self._apply_software_mimic()
        return {"status": "ok", "side": side, "fraction": round(frac, 4),
                "position": int(round(1 + frac * 999)), "leader_target": done}

    def get_gripper_position(self, side: str = "right"):
        """回读夹爪开口：主动关节角 → rm 单位(1~1000) 与比例。"""
        self._require_robot()
        names = list(self.articulation.dof_names)
        leader = self._GRIPPER_LEADER.get(side)
        if not leader or leader not in names:
            raise ValueError(f"未找到夹爪主动关节: {leader}")
        pos = self._joint_positions_flat()
        ang = float(pos[names.index(leader)])
        frac = gripper_model.fraction_for_leader(ang)
        return {"side": side, "angle_rad": round(ang, 5),
                "opening_mm": round(gripper_model.opening_mm_for_leader(ang), 2),
                "fraction": round(frac, 4),
                "position": int(round(1 + frac * 999))}

    # ==================== 自动抓取（VR 左手菜单键触发） ====================

    #: /World 下可抓物体的 prim 名前缀
    _GRASP_OBJ_PREFIXES = ("cube_", "cyl_", "obj_", "box_")

    #: 自动抓取默认抓的物体（黄色方块）。用户指定"就一直只随机黄色物体位置"，
    #: 所以不再随机挑物体，只随机它的落点；其余物体一律挪到桌面远端停放。
    _GRASP_DEFAULT_OBJECT = "cube_c"

    def _grasp_link_T(self, link):
        """连杆世界位姿，列向量约定 4x4（USD 是行向量约定，转置）。"""
        import omni.usd
        from pxr import Usd, UsdGeom

        st = omni.usd.get_context().get_stage()
        p = st.GetPrimAtPath(f"{self.robot_prim_path}/{link}")
        m = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        return np.array([[float(m[i][j]) for j in range(4)]
                         for i in range(4)]).T

    def _grasp_probe(self, side):
        """抓取规划要的实测：基座/末端/指垫位姿 + 手臂关节角与**仿真实测限位**。"""
        names = list(self.articulation.dof_names)
        q = np.asarray(self.articulation.get_joint_positions()).reshape(-1)
        lim = np.asarray(self.articulation.get_dof_limits()).reshape(-1, 2)
        arm = [f"{side}_arm_{i}_joint" for i in range(1, 7)]
        if f"{side}_wrist_joint" in names:
            arm.append(f"{side}_wrist_joint")
        idx = [names.index(n) for n in arm]
        return {
            "T_ab": self._grasp_link_T(f"{side}_arm_base_link"),
            "T_ee": self._grasp_link_T(f"{side}_gripper_base_link"),
            "pad_r": self._grasp_link_T(f"{side}_gripper_right_pad_link")[:3, 3],
            "pad_l": self._grasp_link_T(f"{side}_gripper_left_pad_link")[:3, 3],
            "arm_names": arm,
            "arm_q": q[idx],
            "arm_limits": lim[idx],
        }

    def _grasp_find_object(self, tcp, max_dist=0.45,
                           min_w=0.005, max_w=0.075):
        """离 tcp 最近、且尺寸在夹爪行程内的可抓物体（/World 下的刚体）。"""
        import omni.usd
        from pxr import Usd, UsdGeom, UsdPhysics

        st = omni.usd.get_context().get_stage()
        root = st.GetPrimAtPath("/World")
        if not root.IsValid():
            return None
        bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        best = None
        for p in root.GetChildren():
            if not p.GetName().startswith(self._GRASP_OBJ_PREFIXES):
                continue
            if not p.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            r = bb.ComputeWorldBound(p).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            lo, hi = np.array(r.GetMin()), np.array(r.GetMax())
            size = self._object_true_size(p)
            if size is None:
                size = hi - lo
            w = float(min(size[0], size[1]))
            if not (min_w <= w <= max_w):
                continue
            c = 0.5 * (lo + hi)
            d = float(np.linalg.norm(c - tcp))
            if d > max_dist:
                continue
            if best is None or d < best["dist"]:
                best = {"name": p.GetName(), "center": c, "size": size,
                        "dist": d}
        return best

    def _object_true_size(self, prim):
        """物体的**真实**尺寸(米)，不是 AABB。

        AABB 会被朝向撑大（70mm 的方块转 5° 量出来 75.6mm），拿它当夹持宽度
        会算出过大的开口 → 指垫碰不到物体。Cube/Cylinder 直接从 size/scale 算。
        """
        import omni.usd
        from pxr import UsdGeom

        try:
            sc = np.array([1.0, 1.0, 1.0])
            xf = UsdGeom.Xformable(prim)
            for op in xf.GetOrderedXformOps():
                if op.GetOpName() == "xformOp:scale":
                    sc = np.abs(np.array(op.Get(), dtype=float))
                    break
            tn = str(prim.GetTypeName())
            if tn == "Cube":
                a = prim.GetAttribute("size")
                s = float(a.Get()) if a and a.IsValid() else 1.0
                return np.array([s, s, s]) * sc
            if tn == "Cylinder":
                r = prim.GetAttribute("radius")
                h = prim.GetAttribute("height")
                rv = float(r.Get()) if r and r.IsValid() else 1.0
                hv = float(h.Get()) if h and h.IsValid() else 1.0
                return np.array([2 * rv, 2 * rv, hv]) * sc
        except Exception:
            pass
        return None

    #: 机器人底盘在桌面正前方的标准位姿（场景默认位）。X 键抓取第一步会送回来。
    _TABLE_POSE = ((0.0, 0.0, 0.3295), (1.0, 0.0, 0.0, 0.0))

    def _reset_robot_to_table(self):
        """把机器人底盘送回桌面正前方的标准位姿，手臂回 home。

        用户要求：按 X 抓取前先"复位到桌前"，否则用 VR 把机器人开走之后
        随机摆放的可达范围就对不上了。
        """
        pos, quat = self._TABLE_POSE
        try:
            self.articulation.set_world_poses(
                positions=np.asarray([pos], dtype=np.float64),
                orientations=np.asarray([quat], dtype=np.float64))
            logger.info(f"自动抓取: 机器人复位到桌前 {pos}")
        except Exception as e:
            logger.warning(f"底盘复位失败(继续): {e!r}")
        try:
            self._motion = None
            names = list(self.articulation.dof_names)
            q = self._current_targets()
            for s in ("left", "right"):
                for n in [f"{s}_arm_{i}_joint" for i in range(1, 7)] + \
                        [f"{s}_wrist_joint"]:
                    if n in names:
                        q[names.index(n)] = 0.0
            self.articulation.set_joint_position_targets(
                q.reshape(1, -1).astype(np.float32))
        except Exception as e:
            logger.warning(f"手臂回 home 失败(继续): {e!r}")

    def _restore_body_home(self):
        """把机身升降/俯仰恢复到 home 值（随机可达范围是按 home 位姿标定的）。"""
        try:
            jt = getattr(self._vr_teleop, "_jac_table", None) \
                if self._vr_teleop is not None else None
            hq = np.asarray(jt.get("home_q", []), float) if jt else None
            names = list(self.articulation.dof_names)
            vals, jnames = [], []
            for n in ("lift_joint", "torso_pitch_joint"):
                if n not in names:
                    continue
                jnames.append(n)
                vals.append(float(hq[names.index(n)])
                            if hq is not None and hq.size == len(names) else 0.0)
            if jnames:
                self.send_joint_command(vals, duration=1.5, joint_names=jnames)
                logger.info(f"自动抓取: 机身复位 {dict(zip(jnames, vals))}")
        except Exception as e:
            logger.warning(f"机身复位失败: {e!r}")

    def _grasp_object_by_name(self, name, max_dist=None):
        """按名字取一个可抓物体（AABB 中心 + **真实**尺寸）。"""
        import omni.usd
        from pxr import Usd, UsdGeom

        st = omni.usd.get_context().get_stage()
        p = st.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            return None
        bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        r = bb.ComputeWorldBound(p).ComputeAlignedRange()
        if r.IsEmpty():
            return None
        lo, hi = np.array(r.GetMin()), np.array(r.GetMax())
        size = self._object_true_size(p)
        if size is None:
            size = hi - lo
        return {"name": name, "center": 0.5 * (lo + hi), "size": size,
                "dist": 0.0}

    def _grasp_random_place(self, side, obj_name=None, margin=0.05,
                            forward=(0.44, 0.58), lateral=(0.20, 0.38)):
        """随机挑一个可抓物体，随机放到**手臂前方桌面上**，返回落点。

        可达范围从**手臂基座**推：实测好抓点 ≈ 基座 + (0.54, 0.00)，
        所以取 前方 0.42~0.62m、左右 ±0.13m 的矩形，再夹到桌面 AABB 内。
        物体落点 z = 桌面高 + 半个物体高（贴桌面）。
        """
        import omni.usd
        from pxr import Usd, UsdGeom
        from isaacsim.core.prims import RigidPrim

        st = omni.usd.get_context().get_stage()
        bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

        # 候选物体（优先方块）
        cands = []
        root = st.GetPrimAtPath("/World")
        for p in root.GetChildren():
            if p.GetName().startswith(self._GRASP_OBJ_PREFIXES):
                # 尺寸必须用**真实**尺寸: 物体被夹住倾斜时 AABB 会胀到
                # 84mm(60mm 的方块转 45°), 用 AABB 筛会把在场的物体全过滤掉
                # → 报"场景里没有可抓物体"。
                size = self._object_true_size(p)
                if size is None:
                    r = bb.ComputeWorldBound(p).ComputeAlignedRange()
                    if r.IsEmpty():
                        continue
                    size = np.array(r.GetMax()) - np.array(r.GetMin())
                if 0.005 <= float(min(size[0], size[1])) <= 0.075:
                    cands.append(p.GetName())
        if not cands:
            raise RuntimeError("场景里没有可抓物体")
        default = self._GRASP_DEFAULT_OBJECT
        if obj_name and obj_name in cands:
            name = obj_name
        elif default and default in cands:
            name = default
        else:                       # 兜底：第一个方块
            cubes = [c for c in cands if c.startswith("cube")]
            name = (cubes or cands)[0]

        # 桌面范围（放物体用）
        tp = st.GetPrimAtPath("/World/vla_table")
        tr = bb.ComputeWorldBound(tp).ComputeAlignedRange()
        tlo, thi = np.array(tr.GetMin()), np.array(tr.GetMax())
        top_z = float(thi[2])

        # 可达范围用**机器人底座**系来定（不是手臂基座：后者随机身升降/俯仰
        # 整体漂移，实测同样 z 下 x 能从 −0.04 变到 +0.19，按它算会把物体放到
        # 够不着的地方）。底座系里实测好抓点 ≈ (0.50, 0.18)，取矩形
        # 前 0.44~0.58、侧 0.05~0.31（左臂在 +y 一侧）。
        Tb = self._grasp_link_T("base_link")
        # 其它物体的当前位置（**不动它们**，只用来避开：落点别贴着它们）
        others_xy = []
        for on in cands:
            if on == name:
                continue
            r2 = bb.ComputeWorldBound(
                st.GetPrimAtPath(f"/World/{on}")).ComputeAlignedRange()
            if not r2.IsEmpty():
                c2 = 0.5 * (np.array(r2.GetMin()) + np.array(r2.GetMax()))
                others_xy.append(c2[:2])
        # 侧向偏**左手边**(+y): 左臂在 +y 一侧, 放这边姿态更松、更好够
        x = y = None
        for _ in range(12):
            fwd = np.random.uniform(*forward)
            lat = np.random.uniform(*lateral)
            pw = Tb[:3, :3] @ np.array([fwd, lat, 0.0]) + Tb[:3, 3]
            cx = float(np.clip(pw[0], tlo[0] + margin, thi[0] - margin))
            cy = float(np.clip(pw[1], tlo[1] + margin, thi[1] - margin))
            if all(np.linalg.norm(np.array([cx, cy]) - o) > 0.18
                   for o in others_xy):
                x, y = cx, cy
                break
        if x is None:               # 实在抽不到就退而求其次
            x, y = cx, cy
        bx, by = float(Tb[0, 3]), float(Tb[1, 3])

        # 物体自身**真实**尺寸 → 贴桌面 + 后面算夹持宽度
        op = st.GetPrimAtPath(f"/World/{name}")
        size = self._object_true_size(op)
        if size is None:
            orr = bb.ComputeWorldBound(op).ComputeAlignedRange()
            size = np.array(orr.GetMax()) - np.array(orr.GetMin())
        z = top_z + float(size[2]) * 0.5 + 0.002
        # 随机朝向: 更像真随机摆放。抓取时虎口会对准这个 yaw 的面
        yaw = float(np.random.uniform(-np.pi / 2, np.pi / 2))
        q = np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])

        rp = RigidPrim(f"/World/{name}")
        rp.initialize()
        rp.set_world_poses(positions=np.asarray([[x, y, z]], dtype=np.float64),
                           orientations=np.asarray([q], dtype=np.float64))
        rp.set_linear_velocities(np.zeros((1, 3)))
        rp.set_angular_velocities(np.zeros((1, 3)))
        return {"name": name, "pos": np.array([x, y, z]), "yaw": yaw,
                "size": size, "table_top": top_z,
                "base": np.array([bx, by, float(Tb[2, 3])])}

    def auto_pick(self, side: str = "left", object_name: str = None,
                  approach: float = 0.12, lift: float = 0.20,
                  squeeze: float = 0.5, max_dist: float = 0.45,
                  randomize: bool = True, reset_robot: bool = True):
        """启动**自动抓取**（非阻塞）。由 step() 逐帧推进。

        流程：**随机放物体** → 张开 → 取物体中心 → 合到目标开口量出闭合态
        TCP → 规划三段轨迹(含机身高度搜索) → 升降机身 → 预备 → 直线下压 →
        闭合 → 直线上抬。

        :param randomize: True=每次先随机挑一个方块放到手臂前方可达处再抓
            （菜单键默认）；False=抓夹爪附近最近的物体。

        必须非阻塞：从 HTTP 协程里同步跑会等主线程的 step()，而死锁。
        """
        self._require_robot()
        if self._auto_pick is not None:
            return {"status": "busy", "phase": self._auto_pick["phase"]}
        self._auto_pick = {
            "side": side, "phase": "reset",
            "object_name": object_name, "randomize": bool(randomize),
            "approach": approach, "lift": lift, "squeeze": squeeze,
            "max_dist": max_dist, "t": time.time(), "note": "", "error": None,
            "object": None, "qs": None, "arm_names": None, "sent": False,
            "spawn": None, "reset_robot": bool(reset_robot),
        }
        self._set_vr_suspended(True)
        logger.info(f"自动抓取启动 (side={side}, randomize={randomize})")
        return {"status": "started", "side": side, "randomize": bool(randomize)}

    def auto_pick_status(self):
        ap = self._auto_pick
        if ap is None:
            return {"running": False, "last": self._last_auto_pick}
        return {"running": True, "phase": ap["phase"],
                "object": (ap["object"] or {}).get("name"),
                "error": ap.get("error"), "note": ap.get("note"),
                "obj_w_mm": ap.get("obj_w_mm"),
                "target_sep_mm": ap.get("sep_mm"),
                "close_sep_mm": ap.get("close_sep_mm"),
                "close_leader": ap.get("close_leader")}

    def abort_auto_pick(self):
        if self._auto_pick is None:
            return {"status": "idle"}
        self._auto_pick_finish("用户中止")
        return {"status": "aborted"}

    def _auto_pick_finish(self, note: str = ""):
        ap = self._auto_pick
        self._auto_pick = None
        self._set_vr_suspended(False)
        logger.info(f"自动抓取结束: {note or '完成'}")
        if ap is not None:
            self._last_auto_pick = {
                "note": note, "error": ap.get("error"),
                "object": (ap.get("object") or {}).get("name"),
                "obj_w_mm": ap.get("obj_w_mm"),
                "target_sep_mm": ap.get("sep_mm"),
                "close_sep_mm": ap.get("close_sep_mm")}

    def _set_vr_suspended(self, flag: bool):
        """自动抓取期间挂起 VR 手臂/夹爪写入，避免两边抢控制权。"""
        if self._vr_teleop is not None:
            try:
                self._vr_teleop.set_suspended(flag)
            except Exception:
                pass

    def _auto_pick_check_lifted(self, ap):
        """抬完后物体是否真的跟着起来了。None = 无法判定(没记起始高度)。"""
        obj = ap.get("object") or {}
        z0 = ap.get("obj_z0")
        if z0 is None:
            return None
        try:
            o2 = self._grasp_object_by_name(obj.get("name"))
            if o2 is None:
                return None
            return bool(float(o2["center"][2]) - z0 > 0.05)
        except Exception:
            return None

    def _auto_pick_step(self):
        ap = self._auto_pick
        if ap is None:
            return
        try:
            self._auto_pick_advance(ap)
        except Exception as e:
            import traceback
            logger.warning(f"自动抓取异常: {traceback.format_exc()[-400:]}")
            ap["error"] = repr(e)
            self._auto_pick_finish(f"异常: {e!r}")

    def _auto_pick_advance(self, ap):
        """状态机：每个 phase 里等够时间/等运动走完再进下一相。"""
        ph = ap["phase"]
        el = time.time() - ap["t"]
        side = ap["side"]

        def nxt(nphase):
            ap["phase"] = nphase
            ap["t"] = time.time()
            ap["sent"] = False

        # "发过了没有"必须用显式标志: 用 `el < 0.05` 判断时, 一帧卡顿
        # (渲染/资产加载)就会让 el 直接跳过窗口 → 指令永远发不出去。
        if not ap.get("sent") and ph in ("open", "measure", "reopen",
                                         "pre", "grasp", "close", "lift"):
            if ph == "open":
                self.set_gripper_position(side=side, position=1000)
            elif ph in ("measure", "close"):
                self.set_gripper_position(side=side, fraction=ap["frac"])
            elif ph == "reopen":
                self.set_gripper_position(side=side, position=1000)
            else:
                self.send_joint_command(ap["qs"][ph], duration=1.8,
                                        joint_names=ap["arm_names"])
            ap["sent"] = True

        if ph == "reset":
            # 先把机器人送回桌面正前方标准位 + 手臂回 home。
            # 用 VR 把机器人开走之后，随机摆放的可达范围就对不上了。
            if not ap.get("reset_robot", True):
                nxt("spawn")
            elif not ap.get("sent"):
                self._reset_robot_to_table()
                ap["sent"] = True
            elif el > 3.5:
                nxt("spawn")

        elif ph == "spawn":
            # 随机挑一个方块放到手臂前方可达处，再走正常抓取流程。
            # **必须先恢复机身高度**：可达范围是按 home 位姿标定的，机身还停在
            # 上一轮降下来的位置时 arm_base 会整体前移 0.2m+，物体就被放到够不着
            # 的地方（实测落到 x=0.667）。
            step = ap.get("spawn_step", 0)
            if step == 0:
                if not ap.get("sent"):
                    self._restore_body_home()
                    ap["sent"] = True
                elif el > 2.5:
                    ap["spawn_step"] = 1
                    ap["sent"] = False
                    ap["t"] = time.time()
            else:
                if not ap.get("sent"):
                    try:
                        info = self._grasp_random_place(
                        side, ap.get("object_name"),
                        forward=ap.get("forward", (0.44, 0.58)))
                        ap["spawn"] = info
                        logger.info(
                            f"自动抓取: 随机放置 {info['name']} → "
                            f"{np.round(info['pos'], 3)} (yaw "
                            f"{np.degrees(info['yaw']):.0f}°)")
                    except Exception as e:
                        ap["error"] = f"随机放置失败: {e!r}"
                        self._auto_pick_finish(ap["error"])
                        return
                    ap["sent"] = True
                elif el > 1.5:
                    nxt("open")

        elif ph == "open":
            if el > 1.2:
                probe = self._grasp_probe(side)
                tcp = 0.5 * (probe["pad_r"] + probe["pad_l"])
                # 随机摆放的物体按名字取(它可能离夹爪很远, 用距离筛会漏)
                obj = (self._grasp_object_by_name(ap["spawn"]["name"])
                       if ap.get("spawn") else
                       self._grasp_find_object(tcp, ap["max_dist"]))
                if obj is None:
                    ap["error"] = "夹爪附近没有可抓物体"
                    self._auto_pick_finish("没找到物体")
                    return
                ap["object"] = obj
                ap["obj_z0"] = float(obj["center"][2])
                w = float(min(obj["size"][0], obj["size"][1])) * 1000.0
                ap["obj_w_mm"] = w
                ap["sep_mm"] = w + gripper_model.PAD_THICKNESS_MM - ap["squeeze"]
                ap["frac"] = gripper_model.fraction_for_sep_mm(ap["sep_mm"])
                logger.info(f"自动抓取目标 {obj['name']} 宽 {w:.1f}mm "
                            f"距离 {obj['dist']:.3f}m")
                nxt("measure")

        elif ph == "measure":
            # 合到**目标闭合角**量出闭合态 TCP（指垫中点随开口漂移 ~11mm）
            if el > 1.8:
                nxt("plan")

        elif ph == "plan":
            probe = self._grasp_probe(side)
            T_ee = probe["T_ee"]
            pad_mid = 0.5 * (probe["pad_r"] + probe["pad_l"])
            tcp_local = T_ee[:3, :3].T @ (pad_mid - T_ee[:3, 3])
            sepv = probe["pad_r"] - probe["pad_l"]
            jaw_local = T_ee[:3, :3].T @ (sepv / (np.linalg.norm(sepv) + 1e-9))
            chain = grasp_planner.load_chain(
                arm_kin.URDF, f"{side}_arm_base_link",
                f"{side}_gripper_base_link")
            limits = np.asarray(probe["arm_limits"], float)
            q0 = np.clip(np.asarray(probe["arm_q"], float),
                         limits[:, 0] + 1e-4, limits[:, 1] - 1e-4)
            # 把 home 位形也当 IK 起点: 手臂若停在很偏的位形(实测 arm_2 贴下限),
            # 多起点会全部收敛到"贴着限位、姿态解不开"的局部解。
            seeds = []
            try:
                jt = getattr(self._vr_teleop, "_jac_table", None) \
                    if self._vr_teleop is not None else None
                hq = np.asarray(jt.get("home_q", []), float) if jt else None
                names_all = list(self.articulation.dof_names)
                if hq is not None and hq.size == len(names_all):
                    seeds.append(hq[[names_all.index(n)
                                     for n in probe["arm_names"]]])
            except Exception:
                pass
            # 物体带随机 yaw → 虎口对准它的面（法向 + 切向 共 4 个候选方向）
            jaw_axes = None
            _yaw = float((ap.get("spawn") or {}).get("yaw", 0.0))
            if abs(_yaw) > 1e-3:
                ca, sa = float(np.cos(_yaw)), float(np.sin(_yaw))
                jaw_axes = [np.array([ca, sa, 0.0]), np.array([-ca, -sa, 0.0]),
                            np.array([-sa, ca, 0.0]), np.array([sa, -ca, 0.0])]
            # **每帧只跑一档机身高度**（生成器）：一次完整规划要跑上百次
            # least_squares，同步跑完会把画面冻住十几秒（用户实测"卡顿很久"）。
            ap["plan_gen"] = grasp_planner.plan_grasp_steps(
                chain, q0, limits, probe["T_ab"], tcp_local, jaw_local,
                np.asarray(ap["object"]["center"], float),
                approach=ap["approach"], lift=ap["lift"], seeds=seeds,
                jaw_axes=jaw_axes)
            ap["plan_best"] = None
            ap["plan_names"] = probe["arm_names"]
            nxt("plan_tick")
            return

        elif ph == "plan_tick":
            gen = ap.get("plan_gen")
            if gen is None:
                self._auto_pick_finish("规划状态丢失")
                return
            try:
                score, qs, info, dz = next(gen)
                if ap["plan_best"] is None or score < ap["plan_best"][0]:
                    ap["plan_best"] = (score, qs, info, dz)
                ap["note"] = f"规划中… 已试机身 {dz:+.2f}m (得分 {score:.0f})"
                return
            except StopIteration:
                pass
            except Exception as e:
                ap["error"] = f"规划异常: {e!r}"
                self._auto_pick_finish(ap["error"])
                return
            ap["plan_gen"] = None
            best = ap["plan_best"]
            if best is None:
                qs, info = None, {}
            else:
                _, qs, info, dz = best
            if qs is None:
                # 可达范围是估的，够不着就把物体往机器人方向挪一点重放
                if ap.get("retry", 0) < 3:
                    ap["retry"] = ap.get("retry", 0) + 1
                    f0, f1 = ap.get("forward", (0.44, 0.58))
                    ap["forward"] = (f0 - 0.06, f1 - 0.06)
                    logger.info(f"自动抓取: 规划失败, 物体挪近重试 "
                                f"(第 {ap['retry']} 次, forward="
                                f"{ap['forward']})")
                    ap["spawn_step"] = 1
                    ap["sent"] = False
                    nxt("spawn")
                    return
                ap["error"] = info.get("error")
                self._auto_pick_finish(f"规划失败: {ap['error']}")
                return
            # descent/ascent 是"数组的列表"(笛卡尔路点), 不是数组
            ap["qs"] = {
                k: (v.tolist() if hasattr(v, "tolist")
                    else [np.asarray(x).tolist() for x in v])
                for k, v in qs.items()}
            ap["arm_names"] = ap.get("plan_names")
            ap["body_dz"] = float(info.get("body_dz", 0.0))
            ap["note"] = ("IK 位置误差 " + "/".join(
                f"{info['infos'][t]['pos_err_mm']:.1f}" for t in
                ("pre", "grasp", "lift")) + "mm, 机身 "
                + f"{ap['body_dz']:+.2f}m")
            nxt("body")

        elif ph == "body":
            # 机身升降(lift_joint 棱柱): 规划选了哪个高度就先移过去。
            # 臂够不到桌面时降机身比硬掰手臂好(用户 2026-10-09 指出)。
            if abs(ap.get("body_dz", 0.0)) < 1e-3:
                nxt("reopen")
            elif not ap.get("sent"):
                names = list(self.articulation.dof_names)
                if "lift_joint" in names:
                    cur = float(self._current_targets()[names.index("lift_joint")])
                    self.send_joint_command([cur + ap["body_dz"]],
                                            duration=1.5,
                                            joint_names=["lift_joint"])
                ap["sent"] = True
            elif self._motion is None and el > 0.3:
                nxt("reopen")

        elif ph == "reopen":
            if el > 1.2:
                nxt("pre")

        elif ph == "pre":
            # 一步到物体上方悬停位(关节空间插值, 2s)。用户 2026-10-09 明确要
            # 回到这个版本 —— 加笛卡尔接近路径反而变成"先往上、再从上面接近"。
            if el > 0.3 and self._motion is None:
                ap["wp"] = 0
                nxt("descent")

        elif ph in ("descent", "ascent"):
            # 笛卡尔直线路点: 关节空间插值会让 TCP 走曲线、斜着蹭到物体
            wps = ap["qs"][ph]
            k = ap.get("wp", 0)
            if not ap.get("sent"):
                self.send_joint_command(wps[k], duration=0.25,
                                        joint_names=ap["arm_names"])
                ap["sent"] = True
            elif el > 0.28 and self._motion is None:
                k += 1
                if k >= len(wps):
                    nxt("close" if ph == "descent" else "settle")
                else:
                    ap["wp"] = k
                    ap["sent"] = False
                    ap["t"] = time.time()

        elif ph == "close":
            if el > 2.2:
                try:
                    gp = self.get_gripper_position(side)
                    pr = self._grasp_link_T(f"{side}_gripper_right_pad_link")[:3, 3]
                    pl = self._grasp_link_T(f"{side}_gripper_left_pad_link")[:3, 3]
                    ap["close_leader"] = gp["angle_rad"]
                    ap["close_sep_mm"] = round(
                        float(np.linalg.norm(pr - pl) * 1000), 2)
                    logger.info(
                        f"自动抓取闭合: 主动关节 {gp['angle_rad']:.4f} "
                        f"指垫中心距 {ap['close_sep_mm']:.2f}mm "
                        f"(目标 {ap['sep_mm']:.2f}mm)")
                except Exception:
                    pass
                # 闭合自检：驱动有稳态误差（力矩上限/刚度），实测指垫会停在
                # 离目标 0.5~1.4mm 处 → 没真正接触 → 一抬就掉。量出来差多少
                # 就往紧里再补多少，最多补 3 次。
                # 只在"差一点点"时补：差太多说明不是驱动稳态误差，而是机构已经
                # 卡住了——再加压只会让指垫反而张开（实测一路压到 51.6mm 时指垫
                # 张开到 64mm）。累计补夹量也设上限。
                short = ap.get("close_sep_mm", 0.0) - ap.get("sep_mm", 0.0)
                added = ap.get("grip_added", 0.0)
                if short > 0.5 and ap.get("grip_retry", 0) < 3 \
                        and added < 3.0:
                    ap["grip_retry"] = ap.get("grip_retry", 0) + 1
                    step = min(short + 0.4, 3.0 - added)
                    ap["grip_added"] = added + step
                    ap["sep_mm"] = ap["sep_mm"] - step
                    ap["frac"] = gripper_model.fraction_for_sep_mm(ap["sep_mm"])
                    logger.info(
                        f"自动抓取: 指垫差 {short:.2f}mm → 补夹 {step:.2f}mm 到中心距 "
                        f"{ap['sep_mm']:.2f}mm (第 {ap['grip_retry']} 次)")
                    ap["sent"] = False
                    ap["t"] = time.time()
                    return
                ap["wp"] = 0
                nxt("ascent")

        elif ph == "settle":
            if el > 1.2:
                # 任务级自检: 抬完看物体有没有真的跟着起来。单次抓取的物理
                # 成功率只有 ~50-70%(指垫差最后零点几毫米没接触), 但"重放重抓"
                # 两三次就能把整体成功率拉上去。
                ok = self._auto_pick_check_lifted(ap)
                if ok is None:
                    self._auto_pick_finish("完成")
                elif ok:
                    ap["lifted_ok"] = True
                    self._auto_pick_finish("完成")
                elif ap.get("task_retry", 0) < 2:
                    ap["task_retry"] = ap.get("task_retry", 0) + 1
                    logger.info(f"自动抓取: 没夹住 → 重放重抓 "
                                f"(第 {ap['task_retry']} 次)")
                    for k in ("spawn", "qs", "plan_gen", "plan_best",
                              "grip_retry", "grip_added", "close_sep_mm",
                              "close_leader", "body_dz"):
                        ap.pop(k, None)
                    ap["object"] = None
                    ap["spawn_step"] = 1
                    ap["sent"] = False
                    nxt("spawn")
                else:
                    ap["error"] = "重试 2 次仍未夹住"
                    self._auto_pick_finish("没夹住")
        else:
            self._auto_pick_finish(f"未知状态 {ph}")

    # ==================== USD 机器人资产 ====================

    def load_robot(self, name: str = None, usd_path: str = None,
                   prim_path: str = "/humanoid", presets: dict = None):
        """
        加载 USD 机器人资产并注册 Articulation（推荐路径）。
        适用 Isaac Asset Transformer 产物（顶层 .usd + configuration/ 分层），
        物理/关节配置以资产内烘焙的 physics/robot 层为准，不做 URDF 转换。

        机器人选择（优先级从高到低）：
          1. 传 usd_path —— 直接加载该文件（prim_path 用传入值，默认 /humanoid）
          2. 传 name    —— 从 presets 取对应预设
          3. 都不传     —— 用 config.json robot.name 指定的默认机器人
        presets 来自 config.json 的 robot.presets（经默认参数合并自动传入）。
        预设条目填 usd_path 或 urdf_path 之一；同时存在时优先 USD
        （物理已烘焙）。urdf_path 走 load_urdf 自动转换，mesh 须为绝对路径
        （package:// 不被导入器解析）。

        :param name: 预设名（robot.presets 的 key）
        :param usd_path: 顶层 USD 文件路径（临时指定，不入库）
        :param prim_path: 场景挂载路径（usd_path 直连时生效）
        :return: {"usd_path": ..., "prim_path": ..., "joint_names": [...]}
        """
        if usd_path:
            return self._load_robot_usd(usd_path, prim_path)

        presets = presets or {}
        if not name:
            raise ValueError(
                "未指定机器人：请在 config.json 的 robot.name 配置默认机器人，"
                f"或请求时传 name/usd_path。可用预设: {sorted(presets) or '（无）'}")
        if name not in presets:
            raise ValueError(
                f"预设 '{name}' 不在 robot.presets 里。可用: {sorted(presets)}")

        preset = presets[name]
        prim_path = preset.get("prim_path", "/humanoid")
        usd_path = preset.get("usd_path")
        if usd_path:
            return self._load_robot_usd(usd_path, prim_path)
        urdf_path = preset.get("urdf_path")
        if urdf_path:
            return self.load_urdf(urdf_path, prim_path=prim_path)
        raise ValueError(
            f"预设 '{name}' 缺少 usd_path/urdf_path，请检查 config.json")

    def _bake_joint_gains(self, usd_path: str, gains_path: str):
        """把 <stem>.gains.json 的按关节增益烘进转换产物 USD（值=USD 级）。

        键 = URDF 关节名，值 = {"stiffness": k, "damping": d, "friction": f?}。
        注意折算：物理表现 damping ≈ USD 值 × 57.3（bake_damping 实测），生成
        gains.json 时须把物理期望值 /57.3。只写已有 drive 属性的关节；无驱动
        的 mimic 跟随关节跳过，由 _fix_gripper_joint_gains 补 friction。
        """
        import json as _json
        from pxr import Usd

        try:
            with open(gains_path) as f:
                gains = _json.load(f)
        except Exception as e:
            logger.warning(f"gains.json 读取失败，跳过增益烘焙: {e}")
            return
        stage = Usd.Stage.Open(usd_path)
        if stage is None:
            logger.warning(f"无法打开转换产物，跳过增益烘焙: {usd_path}")
            return
        fixed = skipped = 0
        for prim in stage.Traverse():
            tname = prim.GetTypeName()
            if tname not in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint"):
                continue
            g = gains.get(prim.GetName())
            if not g:
                continue
            axis = "angular" if tname == "PhysicsRevoluteJoint" else "linear"
            ok = True
            for key in ("stiffness", "damping"):
                if key not in g:
                    continue
                attr = prim.GetAttribute(f"drive:{axis}:physics:{key}")
                if not attr:
                    ok = False
                    break
                attr.Set(float(g[key]))
            if ok:
                if "friction" in g:
                    fa = prim.GetAttribute("physics:jointFriction")
                    if fa:
                        fa.Set(float(g["friction"]))
                fixed += 1
            else:
                skipped += 1
        stage.GetRootLayer().Save()
        logger.info(f"增益已烘焙: {fixed} 关节（无 drive 跳过 {skipped}）<- {os.path.basename(gains_path)}")

    def _load_robot_usd(self, usd_path: str, prim_path: str):
        """USD 资产挂载 + Articulation 注册（load_robot 的实现体）。"""
        from isaacsim.core.prims import Articulation
        from isaacsim.core.utils.stage import add_reference_to_stage
        import omni.usd

        if not os.path.exists(usd_path):
            raise FileNotFoundError(f"USD not found: {usd_path}")

        add_reference_to_stage(usd_path, prim_path)
        logger.info(f"USD 机器人已挂载: {usd_path} -> {prim_path}")
        # 必须在 Articulation 创建前修正增益，PhysX 构建时才会用新参数
        self._fix_gripper_joint_gains(prim_path)

        # ArticulationRootAPI 通常在挂载根上；不在时向子树找
        stage = omni.usd.get_context().get_stage()
        arti_path = prim_path
        top = stage.GetPrimAtPath(prim_path)
        if top.IsValid() and not top.HasAPI("ArticulationRootAPI"):
            for prim in top.GetAllChildren():
                if prim.HasAPI("ArticulationRootAPI"):
                    arti_path = str(prim.GetPath())
                    break

        self.robot_prim_path = arti_path
        # 位置传参：兼容 4.x(prim_path) / 5.x(prim_paths_expr) 两种构造签名
        self.articulation = Articulation(arti_path, name="robot")
        self.world.scene.add(self.articulation)
        if not self.world.is_playing():
            self.world.reset()
        self.articulation.initialize()
        self._ensure_default_environment()
        self._build_mimic_map()
        self._apply_home_pose()
        self._register_static_cameras()
        self._apply_static_camera_offsets()
        self._motion = None

        joint_names = list(self.articulation.dof_names)
        logger.info(f"机器人已加载: {arti_path}，关节数: {len(joint_names)}")
        return {"usd_path": usd_path, "prim_path": arti_path, "joint_names": joint_names}

    def _fix_gripper_joint_gains(self, prim_path: str, min_damping: float = 0.05,
                                 friction: float = 0.05):
        """夹爪连杆联动（mimic 关节）增益修正，仅在 Articulation 创建前调用生效。

        URDF 转换产物常见两类问题，均会让仿真一启动夹爪就剧烈振铃：
          1. 主动驱动关节阻尼过小（欠阻尼，play 即振荡）
          2. mimic 跟随关节零阻尼零摩擦（约束链自身振铃不收敛）
        处理：带 DriveAPI 的关节阻尼低于 min_damping 时提升到 min_damping；
        无驱动的跟随关节写入 jointFriction。资产参数正常时不做修改。
        """
        import omni.usd
        from pxr import Sdf, Usd, UsdPhysics

        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(prim_path)
        if not root.IsValid():
            return

        JOINT_TYPES = ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint")
        fixed_damping, fixed_friction = 0, 0
        for prim in Usd.PrimRange(root):
            if prim.GetTypeName() not in JOINT_TYPES or "gripper" not in prim.GetName():
                continue
            axis = ("linear" if prim.GetTypeName() == "PhysicsPrismaticJoint"
                    else "angular")
            drive = UsdPhysics.DriveAPI.Get(prim, axis)
            if drive:
                damping = drive.GetDampingAttr().Get()
                if damping is None or damping < min_damping:
                    drive.GetDampingAttr().Set(min_damping)
                    fixed_damping += 1
            else:
                attr = prim.GetAttribute("physxJoint:jointFriction")
                if not attr:
                    attr = prim.CreateAttribute("physxJoint:jointFriction",
                                                Sdf.ValueTypeNames.Float)
                if attr.Get() is None or attr.Get() < friction:
                    attr.Set(friction)
                    fixed_friction += 1

        if fixed_damping or fixed_friction:
            logger.info(f"夹爪增益修正: {fixed_damping} 个主动关节提升阻尼至 "
                        f"{min_damping}，{fixed_friction} 个跟随关节写入摩擦 {friction}")

    def _load_config(self):
        """读取 config.json（用于 home_pose/teleop 等默认参数）。"""
        try:
            import json as _json
            cfg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "config.json")
            with open(cfg_path) as f:
                return _json.load(f)
        except Exception as e:
            logger.warning(f"config.json 读取失败（使用默认参数）: {e}")
            return {}

    def _apply_home_pose(self):
        """加载后把 robot.home_pose（config.json）写入关节目标。
        未知关节名跳过并告警——URDF 零位=双臂下垂时，在此配置基础姿态。"""
        pose = (self._config or {}).get("home_pose", {})
        pose = {k: v for k, v in pose.items()
                if not k.startswith("_") and isinstance(v, (int, float))}
        if not pose:
            return
        names = list(self.articulation.dof_names)
        applied = {}
        for name, val in pose.items():
            if name in names:
                applied[name] = float(val)
            else:
                logger.warning(f"home_pose 关节名不存在，已跳过: {name}")
        if not applied:
            return
        try:
            idxs = [names.index(n) for n in applied]
            targets = np.asarray(
                self.articulation.get_joint_positions()).reshape(-1).copy()
            for i, n in zip(idxs, applied):
                targets[i] = applied[n]
            self.articulation.set_joint_position_targets(
                targets[idxs].reshape(1, -1).astype(np.float32),
                joint_indices=idxs)
            logger.info(f"home_pose 已应用: {applied}")
        except Exception as e:
            logger.warning(f"home_pose 应用失败: {e}")

    def set_home_pose(self, pose: dict, save: bool = True):
        """设置并（可选）持久化基础姿态 {"关节名": rad}。"""
        self._require_robot()
        names = list(self.articulation.dof_names)
        unknown = [n for n in pose if n not in names]
        if unknown:
            raise ValueError(f"关节不存在: {unknown}。可用（部分）: "
                             f"{[n for n in names if 'shoulder' in n or 'elbow' in n][:8]}")
        if save:
            import json as _json
            cfg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "config.json")
            with open(cfg_path) as f:
                cfg = _json.load(f)
            cfg["home_pose"] = dict(pose)
            with open(cfg_path, "w") as f:
                _json.dump(cfg, f, ensure_ascii=False, indent=2)
        self._apply_home_pose_config(pose)
        return {"applied": pose}

    def _apply_home_pose_config(self, pose):
        names = list(self.articulation.dof_names)
        idxs = [names.index(n) for n in pose]
        targets = np.asarray(
            self.articulation.get_joint_positions()).reshape(-1).copy()
        for i, n in zip(idxs, pose):
            targets[i] = float(pose[n])
        self.articulation.set_joint_position_targets(
            targets[idxs].reshape(1, -1).astype(np.float32),
            joint_indices=idxs)

    # ---------- VR 遥操作 ----------

    def start_vr_teleop(self, **kwargs):
        """启动 Pico 4 VR 遥操作（需 scripts/teleop/setup_isaacteleop.sh 安装）。
        参数默认值取 config.json robot.teleop，调用传参可覆盖。"""
        self._require_robot()
        if self._vr_teleop is None:
            from isaac_teleop import VRTeleopManager
            self._vr_teleop = VRTeleopManager(self)
        allowed = {"rate_hz", "workspace_scale", "max_step", "deadzone",
                   "grip_open"}
        defaults = {
            key: value for key, value in self._config.get("teleop", {}).items()
            if key in allowed
        }
        defaults.update(kwargs)
        self._vr_grip_open = float(defaults.get("grip_open", 0.7))
        return self._vr_teleop.start(**defaults)

    def stop_vr_teleop(self):
        if self._vr_teleop is None:
            return {"status": "not_running"}
        return self._vr_teleop.stop()

    def vr_teleop_status(self):
        if self._vr_teleop is None:
            return {"running": False}
        return self._vr_teleop.status()

    def vr_axis_calib(self, state: str = "start"):
        """手柄→世界旋转的推杆实测标定: start → 双手柄水平前推30cm → done。

        零假设直接测量(不依赖头显偏航/客户端坐标系约定), 每侧独立解 phi;
        state="clear" 回到头显推算。结果在返回体与 vr_teleop_status.axis_calib。
        """
        if self._vr_teleop is None:
            return {"status": "not_running"}
        return self._vr_teleop.axis_calib(state)

    def vr_push_calib(self, state: str = "start", side: str = "left"):
        """**三推杆标定**: 解出跟踪系→机器人世界系的完整旋转矩阵。

        比 vr_axis_calib 强的地方: 后者只解偏航(且假设客户端是标准 OpenXR
        轴序 —— 实测该假设不成立, "前推"在客户端系里走 +x), 本标定用三次
        已知方向的推杆解出完整 3×3 旋转, 对轴序零假设。

        用法: start → 同一只手依次"前推、左推、上抬"各 ~30cm(每次推完停
        1 秒再收回) → done。结果在返回体与 vr_teleop_status.push_calib。
        """
        if self._vr_teleop is None:
            return {"status": "not_running"}
        return self._vr_teleop.vr_push_calib(state, side)

    # ==================== 相机 ====================

    def _hide_mount_visuals(self, attach_to: str) -> None:
        """隐藏相机挂载连杆的视觉网格与碰撞体（相机不遮挡视线/不被碰撞干扰）。

        本机器人 URDF 导入后 visuals/collisions 是 instanceable prim（网格在
        实例原型里），故对实例 prim 直接设 visibility=invisible（可见性会传入
        实例）；碰撞物理禁用通过在根层对内部引用目标 /colliders/<link> 写
        over 实现（USD 禁止直接编辑实例原型）。任何失败都不影响建相机。
        置 robot.HIDE_MOUNTS = False 可整体关闭（排障用）。
        """
        if not getattr(self, "HIDE_MOUNTS", True):
            return
        try:
            import omni.usd
            from pxr import Sdf, UsdGeom

            seg = attach_to.strip("/").split("/")
            if len(seg) < 2:
                return
            link_path = "/" + "/".join(seg[:2])
            stage = omni.usd.get_context().get_stage()
            link = stage.GetPrimAtPath(link_path)
            if not link.IsValid():
                return
            for child in ("visuals", "collisions"):
                p = link.GetChild(child)
                if p.IsValid():
                    p.GetAttribute("visibility").Set("invisible")
            # 普通结构兜底：子树里的 Gprim 一并隐藏
            for q in Usd.PrimRange(link):
                if q.IsA(UsdGeom.Gprim):
                    q.GetAttribute("visibility").Set("invisible")
            # 碰撞物理禁用（下次物理场景重建/重启时生效）
            try:
                root_layer = stage.GetRootLayer()
                over_path = Sdf.Path(f"/colliders/{seg[1]}")
                if not root_layer.GetPrimAtPath(over_path):
                    Sdf.CreatePrimInLayer(root_layer, over_path)
                ps = root_layer.GetPrimAtPath(over_path)
                ps.specifier = Sdf.SpecifierOver
                a = ps.attributes.get("physics:collisionEnabled")
                if a is None:
                    a = Sdf.AttributeSpec(ps, "physics:collisionEnabled",
                                          Sdf.ValueTypeNames.Bool)
                a.default = False
            except Exception:
                pass
        except Exception:
            pass

    def add_camera(self, prim_path: str, attach_to: str = None, name: str = None,
                   position: list = None, rotation_euler: list = None,
                   resolution: list = None, focal_length: float = 24.0,
                   clipping_range: list = None, intrinsics: dict = None):
        """
        在场景中创建一个相机传感器。

        :param prim_path: 相机 prim 路径。挂载时须在 attach_to 之下，
            如 "/humanoid/head_link/head_camera"
        :param attach_to: 可选，挂载到的 link/prim 路径（如 "/humanoid/head_link"）。
            给出后相机作为其子 prim 随机器人运动，position/rotation_euler 按
            该 link 的本地坐标系解释。注意 URDF 导入需 merge_fixed_joints=False
            （默认）才会保留 link prim
        :param rotation_euler: 欧拉角 [roll, pitch, yaw]（度），None=朝 -Z 默认朝向
        :param resolution: [宽, 高]，默认 [1280, 720]
        :param focal_length: 标称焦距（mm），仅在未提供 intrinsics 时生效
        :param intrinsics: 可选，真实相机标定内参（像素单位），优先于 focal_length:
            {"fx": ..., "cx": ..., "cy": ...}（cy 可省略=画面中心；方形像素假设
            fx≈fy，竖直像元按分辨率宽高比自动同步）。畸变/鱼眼模型暂不支持
        """
        from isaacsim.sensors.camera import Camera
        from isaacsim.core.utils.rotations import euler_angles_to_quat
        from pxr import UsdGeom
        import omni.usd

        resolution = resolution or [1280, 720]
        clipping_range = clipping_range or [0.05, 100.0]
        # 默认名取全路径（仅末段会在多相机同叫 camera 时撞场景注册表重名）
        name = name or prim_path.strip("/").replace("/", "_")

        stage = omni.usd.get_context().get_stage()
        if attach_to is not None:
            parent = stage.GetPrimAtPath(attach_to)
            if not (parent and parent.IsValid()):
                raise ValueError(
                    f"attach_to 目标 '{attach_to}' 在场景中不存在。提示：URDF 导入"
                    f"需 merge_fixed_joints=False 才会保留 link prim")
            prefix = attach_to.rstrip("/")
            if not prim_path.startswith(prefix + "/"):
                raise ValueError(
                    f"挂载相机的 prim_path 必须在 attach_to 之下，"
                    f"如 '{prefix}/{name}'")
        UsdGeom.Camera.Define(stage, prim_path)

        # Isaac 5.x 的 Camera 构造不再收 focal_length/clipping_range —— 建好后用 setter
        cam = Camera(prim_path=prim_path, name=name, resolution=tuple(resolution))
        res_w, res_h = int(resolution[0]), int(resolution[1])
        if intrinsics:
            # 真实内参（像素）→ USD 相机参数。USD 焦距/像元属性单位 = 米×10。
            # 关系式：fx = focal_mm * res_w / APERTURE_MM
            #         cx = res_w/2 − offset_m * res_w / aperture_m
            APERTURE_MM = 20.955   # USD 默认水平像元，保持不变（方形像素约定）
            fx = float(intrinsics["fx"])
            fy = intrinsics.get("fy")
            if fy is not None and abs(fy - fx) / fx > 0.05:
                logger.warning(f"fx({fx:.1f}) 与 fy({fy:.1f}) 相差 >5%，"
                               f"当前按方形像素实现仅取 fx，非方形像素需另行适配")
            cx = float(intrinsics.get("cx", res_w / 2.0))
            cy = float(intrinsics.get("cy", res_h / 2.0))
            focal_mm = fx * APERTURE_MM / res_w
            # 实测标定（2026-09-27，两档焦距线性回归 + 三参考球反解）:
            #   渲染 fx = res_w * focalLength / APERTURE_MM —— USD 属性值即 mm，
            #   渲染器按标准针孔解释（无 inch/×10 换算）。
            #   set_focal_length setter 延迟生效且换算系数不稳定（创建后一段时间
            #   渲染呈 151° 鱼眼），故直接以 USD 属性写 mm 值兜底。
            cam.prim.GetAttribute("focalLength").Set(float(focal_mm))
            off_h_m = (res_w / 2.0 - cx) * (APERTURE_MM / 1000.0) / res_w
            off_v_m = (res_h / 2.0 - cy) * (APERTURE_MM / 1000.0) / res_h
            # apertureOffset 与 aperture 同单位(cm)：米→cm 应 ×100（cx=cy 中心时为 0，非中心主点未实测验证）
            cam.prim.GetAttribute("horizontalApertureOffset").Set(off_h_m * 100.0)
            cam.prim.GetAttribute("verticalApertureOffset").Set(off_v_m * 100.0)
            logger.info(f"相机内参已配置: fx={fx:.1f}px, cx={cx:.1f}, cy={cy:.1f} "
                        f"→ 焦距 {focal_mm:.2f}mm（HFOV "
                        f"{2 * np.degrees(np.arctan(APERTURE_MM / (2 * focal_mm))):.1f}°）")
        elif focal_length:
            cam.prim.GetAttribute("focalLength").Set(float(focal_length))
        cam.set_clipping_range(clipping_range[0], clipping_range[1])
        self.world.scene.add(cam)
        if self.world.is_playing():
            cam.initialize()

        if position is not None or rotation_euler is not None:
            # camera_axes="usd"：[0,0,0] 朝 -Z 的 USD 相机约定（5.x 的默认 world 约定是 Z上X前）
            pos = np.array(position, dtype=np.float32) if position is not None else None
            quat = (euler_angles_to_quat(np.array(rotation_euler, dtype=np.float32), degrees=True)
                    if rotation_euler is not None else None)
            if attach_to is not None:
                cam.set_local_pose(translation=pos, orientation=quat, camera_axes="usd")
            else:
                cam.set_world_pose(position=pos, orientation=quat, camera_axes="usd")

        self._cameras[prim_path] = cam
        if attach_to is not None:
            self._attached_cameras[prim_path] = attach_to
            self._hide_mount_visuals(attach_to)
        logger.info(f"相机已创建: {prim_path}"
                    + (f"（挂载于 {attach_to}，随机器人运动）" if attach_to else "")
                    + f" (分辨率 {resolution}，焦距 {focal_length}mm)")
        return {"prim_path": prim_path, "resolution": resolution, "attach_to": attach_to}

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
        """移动相机。挂载相机（add_camera 带 attach_to）按本地坐标移动，
        其余按世界坐标。rotation_euler: [roll, pitch, yaw]（度）。"""
        from isaacsim.core.utils.rotations import euler_angles_to_quat

        cam = self._cameras.get(prim_path)
        if cam is None:
            raise ValueError(f"相机 '{prim_path}' 不存在")
        pos = np.array(position, dtype=np.float32)
        quat = (euler_angles_to_quat(np.array(rotation_euler, dtype=np.float32), degrees=True)
                if rotation_euler is not None else None)
        if prim_path in self._attached_cameras:
            cam.set_local_pose(translation=pos, orientation=quat, camera_axes="usd")
        else:
            cam.set_world_pose(position=pos, orientation=quat, camera_axes="usd")
        return {"prim_path": prim_path}

    # ==================== ROS2 Bridge ====================

    def setup_ros2_bridge(self, joint_names: list = None,
                          command_topic: str = "/humanoid/joint_command",
                          state_topic: str = "/humanoid/joint_states",
                          node_name: str = "route_manager_robot",
                          joint_limits: dict = None,
                          publish_rate: float = 50.0):
        """
        创建 ROS2 关节控制节点并接入仿真（闭环）：
          - 外部节点发到 command_topic 的 JointState → 仿真主线程施加到 articulation
          - articulation 实际关节状态以 publish_rate Hz 发布到 state_topic
        需要 source ROS2 环境后启动 server；失败会降级不阻塞仿真。

        :param joint_names: 默认取 URDF 导入后的全部 DOF 名（可传子集只控制部分关节）
        :param joint_limits: {joint_name: {"lower": l, "upper": u}}，超限指令会被 clamp
        :param publish_rate: state_topic 发布频率（Hz）
        """
        if joint_names is None:
            self._require_robot()
            joint_names = list(self.articulation.dof_names)

        if self._ros_controller is not None:
            self._ros_controller.shutdown()

        self._ros_controller = Ros2JointController(
            joint_names=joint_names,
            command_topic=command_topic,
            state_topic=state_topic,
            node_name=node_name,
            joint_limits=joint_limits,
        )
        self._state_publish_period = 1.0 / max(publish_rate, 1e-3)
        self._motion = None
        return {
            "initialized": self._ros_controller.is_initialized(),
            "joint_names": joint_names,
            "command_topic": command_topic,
            "state_topic": state_topic,
            "publish_rate": publish_rate,
        }

    def send_joint_command(self, positions: list, duration: float = 0.0,
                           joint_names: list = None):
        """
        下发关节目标到仿真（非阻塞）。positions 顺序与 setup_ros2_bridge
        的 joint_names 一致。

        duration>0 时按 smoothstep 平滑插值，主循环逐帧推进；
        duration=0 立即施加。外部 ROS2 节点则通过 command_topic 下发指令。

        :param joint_names: 只下发这些关节（positions 与之一一对应），其余
            关节沿用**当前指令目标**。控制单条臂/夹爪时必须用它：否则调用方
            往往拿 `/joint_states` 反馈当输入，等于把"当前(下垂)姿态"重新
            写成目标，全身会被重力一点点压塌(2026-10-08 实测)。
        """
        self._require_robot()
        if joint_names is not None:
            names = list(self.articulation.dof_names)
            vals = list(positions)
            if len(vals) != len(joint_names):
                raise ValueError(
                    f"positions 长度 {len(vals)} 与 joint_names "
                    f"长度 {len(joint_names)} 不一致")
            full = self._current_targets()
            missing = [n for n in joint_names if n not in names]
            if missing:
                raise ValueError(f"未知关节: {missing[:4]}")
            for n, v in zip(joint_names, vals):
                full[names.index(n)] = float(v)
            positions = full.tolist()

        if self._ros_controller is None:
            # 无 ROS2 桥的回退：positions 顺序 = articulation dof_names，
            # 直接施加（HTTP/relay 控制路径）
            self._require_robot()
            names = list(self.articulation.dof_names)
            if len(positions) != len(names):
                raise ValueError(
                    f"positions 长度 {len(positions)} 与关节数 {len(names)} "
                    f"不一致（顺序须与 get_joint_names 一致）")
            positions = [float(p) for p in positions]
            if duration > 0:
                start = (self._joint_positions_flat()
                         if self.world.is_playing() else None)
                if start is None or len(start) != len(positions):
                    start = list(self._last_direct_cmd) if hasattr(self, "_last_direct_cmd") else None
                self._motion = {"start": start, "target": list(positions),
                                "t0": time.time(), "duration": float(duration)}
                return {"sent": True, "mode": "interpolating",
                        "duration": duration, "ros": False}
            if self.world.is_playing():
                self._apply_joint_positions(positions)
            else:
                logger.warning("仿真未在运行，指令已记录但未施加")
            self._last_direct_cmd = list(positions)
            return {"sent": True, "mode": "immediate", "ros": False}

        positions = self._ros_controller.clamp_positions(positions)
        if len(positions) != len(self._ros_controller.joint_names):
            raise ValueError(
                f"positions 长度 {len(positions)} 与关节数 "
                f"{len(self._ros_controller.joint_names)} 不一致"
                f"（顺序须与 setup_ros2_bridge 的 joint_names 一致）")

        if duration > 0:
            start = None
            if self.articulation is not None and self.world.is_playing():
                try:
                    start = self._joint_positions_flat()
                except Exception:
                    start = None
            if start is None or len(start) != len(positions):
                start = list(self._ros_controller.get_cmd_positions())
            self._motion = {"start": start, "target": list(positions),
                            "t0": time.time(), "duration": float(duration)}
            return {"sent": True, "mode": "interpolating", "duration": duration}

        if self.articulation is not None and self.world.is_playing():
            self._apply_joint_positions(positions)
        else:
            logger.warning("仿真未在运行，指令已记录但未施加")
        self._ros_controller.record_command(positions)
        return {"sent": True, "mode": "immediate"}

    # ==================== 底盘（skid-steer 速度控制） ====================

    def set_base_velocity(self, vx: float = 0.0, wz: float = 0.0):
        """底盘速度指令（vx 前进 m/s，wz 左转 rad/s）。

        WASD 键盘等客户端需以 >=10Hz 持续发送；超过 timeout 秒无更新
        自动刹停（失联保护）。轮子由速度控制专管，关节指令中的轮子
        分量同时被忽略（见 _apply_joint_positions）。
        """
        self._require_robot()
        max_vx = float(self._base_cfg.get("max_vx", 0.6))
        max_wz = float(self._base_cfg.get("max_wz", 1.6))
        vx = max(-max_vx, min(max_vx, float(vx)))
        wz = max(-max_wz, min(max_wz, float(wz)))
        self._base_cmd = {"vx": vx, "wz": wz, "t": time.time()}
        return {"vx": vx, "wz": wz,
                "timeout": float(self._base_cfg.get("timeout", 0.5))}

    def _update_base_velocity(self):
        """底盘速度控制（主循环每帧）：把 (vx, wz) 折算成四轮转速，
        并积分成轮关节位置目标（虚拟速度模式）。

        URDF 转换出的轮关节是位置驱动（stiffness≈203 / damping≈0.08），
        直接下速度目标几乎没有力矩，所以逐帧把位置目标前移 ω*dt；
        目标钳制在真实位置 ±0.6 rad 内，堵转时不会越积越远（力矩封顶）。
        left_sign/right_sign 处理 URDF 左右轮轴镜像（左 0,-1,0 右 0,1,0），
        实测若倒退/自旋改 config.json base_teleop 里的符号即可。
        """
        if self.articulation is None or self._base_cmd is None:
            return
        if not self.world.is_playing():
            return
        cmd = self._base_cmd
        if time.time() - cmd["t"] > float(self._base_cfg.get("timeout", 0.5)):
            # 失联刹停：速度归零并保持（drive 继续锁住位置目标）
            cmd["vx"] = 0.0
            cmd["wz"] = 0.0
        vx, wz = cmd["vx"], cmd["wz"]
        r = float(self._base_cfg.get("wheel_radius", 0.1594))
        tr = float(self._base_cfg.get("track_width", 0.585))
        ls = float(self._base_cfg.get("left_sign", -1.0))
        rs = float(self._base_cfg.get("right_sign", 1.0))
        vl = (vx - wz * tr / 2.0) / r   # 左侧轮目标角速度 rad/s
        vr = (vx + wz * tr / 2.0) / r   # 右侧轮目标角速度 rad/s
        arti = self.articulation
        names = list(arti.dof_names)
        idx, w = [], []
        for n, speed in (("front_left_wheel_joint", vl),
                         ("rear_left_wheel_joint", vl),
                         ("front_right_wheel_joint", vr),
                         ("rear_right_wheel_joint", vr)):
            if n not in names:
                logger.warning(f"底盘控制：找不到关节 {n}，跳过")
                self._base_cmd = None
                return
            idx.append(names.index(n))
            w.append(speed * (ls if "left" in n else rs))
        cur = np.asarray(arti.get_joint_positions()).reshape(-1)
        if np.isnan(cur).any():
            return
        if self._wheel_pos_target is None or len(self._wheel_pos_target) != len(idx):
            self._wheel_pos_target = cur[idx].copy()
        try:
            dt = float(self.world.get_physics_dt())
        except Exception:
            dt = 1.0 / 60.0
        self._wheel_pos_target = (self._wheel_pos_target
                                  + np.asarray(w, dtype=np.float32) * dt)
        self._wheel_pos_target = np.clip(self._wheel_pos_target,
                                         cur[idx] - 0.6, cur[idx] + 0.6)
        tgt = self._wheel_pos_target.reshape(1, -1).astype(np.float32)
        arti.set_joint_position_targets(tgt, joint_indices=idx)

    def get_joint_states(self):
        """查询关节指令/反馈。无 ROS2 桥时 feedback = articulation 实际位置。"""
        if self._ros_controller is None:
            self._require_robot()
            fb = self._joint_positions_flat()
            return {
                "cmd": list(getattr(self, "_last_direct_cmd", [])),
                "feedback": fb,
                "feedback_received": True,
                "ros": False,
            }
        return {
            "cmd": self._ros_controller.get_cmd_positions(),
            "feedback": self._ros_controller.get_feedback_positions(),
            "feedback_received": self._ros_controller.is_feedback_received(),
        }

    # ==================== 内部 ====================

    def _require_robot(self):
        if self.articulation is None:
            raise RuntimeError("机器人未加载，请先调用 load_urdf")

    def _joint_positions_flat(self):
        """关节位置压平为 list（5.x view API 返回 (M, K)，单机器人压成 (K,)）。"""
        return [float(v) for v in
                np.asarray(self.articulation.get_joint_positions()).reshape(-1)]

    def _current_targets(self):
        """当前 50 维**指令目标**（读不到时退化为实测位置）。

        与 `_joint_positions_flat()` 的区别：那个是物理读数，这个是驱动器收到
        的指令，用于"只改我关心的几个关节，别人保持不动"的部分下发。
        """
        names = list(self.articulation.dof_names)
        try:
            t = np.asarray(
                self.articulation._physics_view
                .get_dof_position_targets()).reshape(-1).astype(float)
            if t.size == len(names) and np.isfinite(t).all():
                return t
        except Exception:
            pass
        return np.asarray(self._joint_positions_flat(), dtype=float)

    def _apply_joint_positions(self, positions):
        """把与 dof_names 对齐的位置向量施加到 articulation。

        用 set_joint_position_targets（驱动目标，伺服语义）：此前用的
        set_joint_positions 是瞬移，指令瞬间生效但 drive 目标不变，
        下一物理帧就被 drive 拽回旧位置——外部指令看起来"没生效"。
        底盘速度控制激活时，轮子由 set_base_velocity 专管，指令里的
        轮子分量被忽略，避免两路目标互相覆盖。
        """
        try:
            # dof_names 顺序 = 指令向量顺序（ROS 桥存在时 joint_names 与之一致）
            names = list(self.articulation.dof_names)
            vals = np.asarray(positions, dtype=np.float32).reshape(-1)
            if self._base_cmd is not None:
                keep = [i for i, n in enumerate(names)
                        if n not in _BASE_WHEEL_JOINTS]
            else:
                keep = list(range(len(names)))
            indices = [self.articulation.get_dof_index(names[i])
                       for i in keep]
            # 5.x view API 要求 (M, K)，单机器人 M=1
            self.articulation.set_joint_position_targets(
                vals[keep].reshape(1, -1), joint_indices=indices)
            self._apply_software_mimic(vals)
        except Exception as e:
            logger.error(f"应用关节指令失败: {e}")

    def _process_ros_commands(self):
        """闭环 sim 侧：取 ROS command_topic 最新指令施加到机器人。"""
        if self._ros_controller is None or not self._ros_controller.is_initialized():
            return
        latest = self._ros_controller.get_latest_command()
        if latest is None:
            return
        if self.articulation is None or not self.world.is_playing():
            logger.debug("收到 ROS 关节指令但机器人未就绪，已丢弃")
            return
        self._apply_joint_positions(latest)

    def _advance_motion(self):
        """推进 send_joint_command 的平滑插值（非阻塞，主循环每帧调用）。"""
        m = self._motion
        if m is None:
            return
        elapsed = time.time() - m["t0"]
        t = min(elapsed / m["duration"], 1.0)
        s = smoothstep(t)
        interp = [lerp(a, b, s) for a, b in zip(m["start"], m["target"])]
        if self.articulation is not None and self.world.is_playing():
            self._apply_joint_positions(interp)
        if self._ros_controller is not None:
            self._ros_controller.record_command(interp)
        if t >= 1.0:
            self._motion = None
            logger.info("平滑插值完成")

    def _publish_ros_state(self):
        """把 articulation 实际关节状态发布到 state_topic（限频）。"""
        if self._ros_controller is None or not self._ros_controller.is_initialized():
            return
        if self.articulation is None or not self.world.is_playing():
            return
        now = time.time()
        if now - self._last_state_publish < self._state_publish_period:
            return
        self._last_state_publish = now
        try:
            all_positions = self._joint_positions_flat()
            positions = [all_positions[self.articulation.get_dof_index(n)]
                         for n in self._ros_controller.joint_names]
        except Exception:
            return
        self._ros_controller.publish_joint_states(positions)
