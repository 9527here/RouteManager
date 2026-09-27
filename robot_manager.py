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
        self._mimic_map = None       # 软件 mimic 表 {leader_idx: [(follower_idx, gearing)]}
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

    def step(self, render=True):
        """主循环一帧：ROS2 指令 → 插值推进 → 软件 mimic → 底盘速度 → VR 目标 → 物理步进。"""
        self._process_ros_commands()
        self._advance_motion()
        self._apply_software_mimic()
        self._update_base_velocity()
        if self._vr_teleop is not None:
            self._vr_teleop.apply_pending()   # VR 手柄目标（主线程施加）
        self.world.step(render=render)
        self._publish_ros_state()

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
        except Exception as e:
            # 兜底：仅碰撞平面（无视觉），保证任何 Isaac 版本都可用
            logger.warning(f"GroundPlane 创建失败，退化为纯碰撞平面: {e}")
            from pxr import UsdGeom, UsdPhysics

            UsdGeom.Xform.Define(stage, "/World/ground_plane")
            plane = UsdGeom.Plane.Define(stage, "/World/ground_plane/col")
            UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
        logger.info("已创建默认地平面")

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
        self._mimic_smooth = None
        if self._mimic_map:
            n_followers = sum(len(v) for v in mapping.values())
            logger.info(f"软件 mimic 已启用: {len(mapping)} 个主动关节, "
                        f"{n_followers} 个跟随关节")

    def _apply_software_mimic(self):
        """每帧：跟随关节位置目标 = gearing × 主动关节实际角度。
        目标做指数平滑（防每帧量化跳变喂进刚性驱动引起高频嗡鸣）。"""
        if not self._mimic_map:
            return
        try:
            pos = np.asarray(self.articulation.get_joint_positions()).reshape(-1)
        except Exception:
            return
        if np.isnan(pos).any():
            return   # 求解器已爆，跳过（不喂 NaN）
        if not hasattr(self, "_mimic_smooth") or self._mimic_smooth is None:
            self._mimic_smooth = {}
        alpha = 0.35   # 平滑系数：0.35 ≈ 3~4 帧时间常数，50Hz 下 ~70ms
        for leader_idx, entries in self._mimic_map.items():
            leader_val = float(pos[leader_idx])
            idxs = [i for i, _ in entries]
            raw = np.asarray([leader_val * g for _, g in entries],
                             dtype=np.float32)
            key = tuple(idxs)
            prev = self._mimic_smooth.get(key)
            tgt = raw if prev is None or prev.shape != raw.shape \
                else (prev + alpha * (raw - prev)).astype(np.float32)
            self._mimic_smooth[key] = tgt
            try:
                self.articulation.set_joint_position_targets(
                    tgt.reshape(1, -1), joint_indices=idxs)
            except Exception:
                pass

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
        # 位置传参：兼容 4.x(prim_path) / 5.x(prim_paths_expr) 两种构造签名
        self.articulation = Articulation(staged_path, name="humanoid")
        self.world.scene.add(self.articulation)
        # 确保物理已启动并完成初始化 —— 之后 dof_names / set_joint_positions /
        # ROS2 指令闭环都立即可用
        if not self.world.is_playing():
            self.world.reset()
        self.articulation.initialize()
        self._ensure_default_environment()
        self._build_mimic_map()
        self._apply_home_pose()
        self._motion = None   # 换机器人后旧插值作废

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
        defaults = dict(self._config.get("teleop", {}))
        defaults.pop("_comment", None)
        defaults.pop("arm_joints", None)     # 由 isaac_teleop 内部读取
        defaults.pop("gripper_leader", None)
        defaults.update(kwargs)
        return self._vr_teleop.start(**defaults)

    def stop_vr_teleop(self):
        if self._vr_teleop is None:
            return {"status": "not_running"}
        return self._vr_teleop.stop()

    def vr_teleop_status(self):
        if self._vr_teleop is None:
            return {"running": False}
        return self._vr_teleop.status()

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

    def send_joint_command(self, positions: list, duration: float = 0.0):
        """
        下发关节目标到仿真（非阻塞）。positions 顺序与 setup_ros2_bridge
        的 joint_names 一致。

        duration>0 时按 smoothstep 平滑插值，主循环逐帧推进；
        duration=0 立即施加。外部 ROS2 节点则通过 command_topic 下发指令。
        """
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
