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
import udp_teleop
from ros2_bridge import contract_graph, contract_node
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


# 八关节齐全时为四轮独立转向/驱动；旧四轮模型仍走原差速控制。
# 底盘关节由 set_base_velocity 专管，避免整机位置指令钉住舵角和轮子。
_BASE_WHEEL_JOINTS = ("front_left_wheel_joint", "front_right_wheel_joint",
                      "rear_left_wheel_joint", "rear_right_wheel_joint")
_BASE_STEERING_JOINTS = ("caster_front_left_joint", "caster_rear_left_joint",
                         "caster_front_right_joint", "caster_rear_right_joint")
_BASE_DRIVE_JOINTS = _BASE_STEERING_JOINTS + _BASE_WHEEL_JOINTS

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
        # **必须在 reset 之前**：PhysX 建场景时就把 solver/稳定性参数读进去了，
        # 运行时再改 USD 属性不生效。
        self._config = self._load_config()
        self._ensure_physics_scene()
        self.world.reset()

        self.articulation = None      # URDF 导入后的机器人
        self.robot_prim_path = None
        self._cameras = {}            # prim_path -> Camera sensor
        self._attached_cameras = {}   # prim_path -> attach_to（按本地姿态移动）
        self._ros_controller = None
        self._vr_teleop = None
        render_cfg = self._config.get("render") or {}
        raw_render_interval = os.environ.get(
            "ROUTE_MANAGER_RENDER_INTERVAL",
            render_cfg.get("interval", 5),
        )
        self._render_interval = max(1, int(raw_render_interval))
        self._render_counter = 0
        self._mimic_map = None       # 软件 mimic 表 {leader_idx: [(follower_idx, gearing)]}
        #: URDF 里的 <mimic> 原始表 {joint: (src_joint, multiplier)}，load_urdf 填。
        #: 有了它 _build_mimic_map 就不依赖关节命名，任何夹爪资产都能直接用。
        self._urdf_mimic = None
        #: 夹爪跟随关节的 DOF 索引集合 —— 对外接口(get_joint_names/states)隐藏，
        #: 使用者只需看到每只手 1 个主动关节（物理 DOF 不变）
        self._gripper_followers = set()
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

        #: 世界地面高度（米）。整套场景抬到扫描环境的可站地面上用：
        #: 电力院那份 NuRec 扫描，其"能站的房间"在世界 z≈6.2 处（用户用
        #: `/Xform` 标定）。这个值同时作用于 **地平面 / 机器人初始位姿 /
        #: 自动抓取的桌前复位位姿**，三者必须一致，否则机器人会掉下去或
        #: 复位到地面以下。场景本身（static_scene.usda 里 vla_table 的 z、
        #: dianliyuan_scene 的 translate）由 add_dianliyuan_scene.py 烘焙。
        self._base_z = float((self._config.get("world") or {}).get("base_z", 0.0))

        # XRoboToolkit 遥操作端（placo IK，独立进程）→ UDP 关节目标入口。
        # 见 udp_teleop.py；与进程内 VR 遥操作是**互斥的两条路**，由
        # start_udp_teleop 启动，启动时挂起进程内 VR 对手臂/夹爪的写入。
        self._udp_link = None
        self._udp_cfg = self._config.get("udp_teleop") or {}

        # 进程内 ROS2 契约节点（ros2_bridge/contract_node.py）。
        # 用的是 Isaac 6.2 自带的 py3.12 rclpy —— 所以不需要像
        # scripts/ros2_sim_contract.py 那样另起 py3.10 中继、也不再走 HTTP。
        self._ros2_contract = None
        self._ros2_contract_ref = "base_link"
        self._ros2_contract_period = 1.0 / 50.0
        self._last_ros2_publish = 0.0
        self._udp_last_names = None    # 上一帧下发过的关节名（换关节集时全量重写）

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
        from scripts.physics_view_guard import guard
        if not guard(self, logger):
            # 保持窗口响应，但不再进入mimic/发布器的native读写回退循环。
            self.simulation_app.update()
            return
        self._process_ros_commands()
        # XRoboToolkit 遥操作端（UDP）目标。放在 ROS 之后：两条路都在时 UDP 赢
        # （VR 遥操作是实时的，ROS 指令是任务式的，让实时的覆盖更符合直觉）。
        self._apply_udp_targets()
        # 进程内 ROS2 契约：先 spin（处理 control/* 回调，回调直接写 articulation），
        # 再推进插值 —— 顺序和 `_process_ros_commands` 一致。
        self._service_ros2_contract()
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
        # XRoboToolkit 遥操作端（UDP）的按键事件：左手 X 键 → 自动抓取。
        # 与上面进程内 VR 的 X 键同语义，两条路各自独立。
        self._process_udp_events()
        self._auto_pick_step()
        # mimic 放在 VR 之后: 用**当帧**的主动关节指令算跟随关节, 零滞后。
        # (放在 VR 之前会晚一帧, 夹爪高速开合时能看到跟随指头慢半拍)
        self._apply_software_mimic()
        # 腕部等挂载相机每帧跟随 link 运动(转腕时相机跟着转)；offsets 有缓存,
        # 文件没变就是一次矩阵乘, 开销可忽略。
        self._apply_static_camera_offsets()
        should_render = True if render is None else bool(render)
        if render is None and self._render_interval > 1:
            self._render_counter += 1
            if self._render_counter < self._render_interval:
                should_render = False
            else:
                self._render_counter = 0
        self.world.step(render=should_render)
        self._publish_ros_state()
        self._publish_udp_state()
        self._publish_ros2_contract()

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

    def _disable_cams_prim(self):
        """把 `/World/cams` 整个停用（视口相机菜单里不再列出这 8 路）。

        ⚠️ **必须在机器人加载之前做**。`isaacsim.core.prims.Articulation` 的
        `_on_prim_deletion` 是**无条件** `del self._physics_view` 的（基类
        `XFormPrim._on_prim_deletion` 虽然会按路径过滤，但派生类不看返回值）。
        也就是说**任何 prim 被删/停用**都会把机器人的物理视图干掉，之后
        `get_joint_positions` 之类的调用全部抛
        `'Articulation' object has no attribute '_physics_view'`
        （2026-10-22 实测：相机停用的那一刻机器人就不可控了）。

        救急：`articulation._on_physics_ready(None)` 能把视图补回来。
        """
        if getattr(self, "articulation", None) is not None:
            logger.warning(
                "机器人已加载，跳过 /World/cams 停用 —— 运行时停用 prim 会触发 "
                "PRIM_DELETION 让 articulation 的物理视图失效。"
                "要关相机请在 load_scene 之前设 cameras.auto_register=false。")
            return False
        try:
            import omni.usd
            prim = omni.usd.get_context().get_stage().GetPrimAtPath(
                "/World/cams")
            if prim and prim.IsValid():
                prim.SetActive(False)
                logger.info("静态相机已停用（/World/cams inactive）")
                return True
        except Exception as e:
            logger.warning(f"停用 /World/cams 失败: {e!r}")
        return False

    def _register_static_cameras(self):
        """把静态场景里已有的 Camera prim 包装成可读图传感器。

        **默认不注册**（config.json `cameras.auto_register: false`）：每路相机都
        要建 render product，`camera.initialize()` 那几路会实打实吃 GPU —— 在
        NuRec 背景场景本来就把 GPU 打满的情况下很划算地不加载。

        需要出图时把 `cameras.auto_register` 打开，或用 action `add_camera`
        临时加一路（用完记得删）。
        """
        cfg = self._config.get("cameras") or {}
        if not cfg.get("auto_register", False):
            self._disable_cams_prim()
            logger.info("静态相机自动注册已关闭（config cameras.auto_register）")
            return

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
        """按已标定偏移把静态相机放到对应机器人连杆上。

        step() 每帧调用（相机跟着 link 动），offsets 按 mtime 缓存——文件没改
        就不走磁盘，改动后下一帧自动生效。
        """
        import json
        import omni.usd
        from pxr import Gf

        # 相机没启用时整段跳过：/World/cams 已 inactive，算偏移既无意义
        # 又会因为读 articulation._physics_view 刷警告。
        if not (self._config.get("cameras") or {}).get("auto_register", False):
            return
        offset_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "scripts", "scene", "cam_follow_offsets.json",
        )
        if not os.path.exists(offset_path) or self.articulation is None:
            return
        try:
            mtime = os.stat(offset_path).st_mtime
            if getattr(self, "_cam_offset_cache", None) is None \
                    or self._cam_offset_cache[0] != mtime:
                with open(offset_path, encoding="utf-8") as file:
                    self._cam_offset_cache = (mtime, json.load(file))
            offsets = self._cam_offset_cache[1]
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
                # get_link_transforms() 的四元数是 xyzw（见 _link_pose_table）
                x, y, z, w = quaternion
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
        if self._ros2_contract is not None:
            self._ros2_contract.shutdown()
            self._ros2_contract = None
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
            # 相机停用必须赶在机器人加载之前（见 _disable_cams_prim 的说明：
            # 运行期停用 prim 会让 articulation 的物理视图失效）
            if not (self._config.get("cameras") or {}).get("auto_register",
                                                           False):
                self._disable_cams_prim()
            logger.info("已创建默认地平面 + 静态场景")

    def set_scene_visible(self, visible: bool = True, prim_path: str = None):
        """开关环境背景场景（电力院/泉州3 那类 NuRec 扫描）。

        **为什么需要**：NuRec 扫描是高斯的体渲染，2026-10-19 实测它把 GPU
        打满 —— 场景可见 15~17 FPS、隐藏 **45 FPS**。而且常规画质旋钮
        （渲染分辨率、samplesPerPixel、反射/半透、render_interval）**全部
        无效**，瓶颈在高斯光栅化本身；NuRec 的 `crop` 属性运行时也改不了
        （插件加载后把带 `omni:nurec:*` 的 Volume 换成了自有代理 prim）。
        所以唯一有效的降渲染手段就是**不渲它**。

        场景只当背景、不参与物理也不参与操作，所以做遥操作/数据采集时关掉
        它毫无损失，要用时再开。

        :param prim_path: 默认按 config.json `background_scene.prims` 里列的
            依次找；都没有则跳过。
        """
        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        cands = ([prim_path] if prim_path
                 else (self._config.get("background_scene") or {}).get("prims")
                 or ["/World/dianliyuan_scene", "/World/qz3_scene"])
        hit = []
        for p in cands:
            prim = stage.GetPrimAtPath(p)
            if not prim or not prim.IsValid():
                continue
            img = UsdGeom.Imageable(prim)
            (img.MakeVisible() if visible else img.MakeInvisible())
            hit.append(p)
        if not hit:
            return {"status": "no_scene", "tried": cands}
        logger.info(f"背景场景 {'显示' if visible else '隐藏'}: {hit}")
        return {"status": "ok", "visible": bool(visible), "prims": hit}

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
            self._apply_base_z_static()      # 桌子/物体按 base_z 抬高；必须在机器人之前
            logger.info(f"静态场景已加载: {path}")
        except Exception as e:
            logger.warning(f"静态场景加载失败(不影响机器人): {e}")

    def _apply_base_z_static(self):
        """把静态场景里"放在地面上"的 prim 按 `world.base_z` 整体抬高。

        名单走 config.json `world.raise_prims`（桌子 + 桌上的可抓物体 ——
        物体不一起抬就会浮在桌子外）。`/World/dianliyuan_scene` 那份扫描
        场景**不在名单里**：它的高度由 scripts/scene/add_dianliyuan_scene.py
        直接烘焙进 static_scene.usda，跟这三个数不是一套。

        必须**在机器人之前**做（prim 编辑要在 articulation 建立前，否则物理
        后端失效，见 _load_static_scene 的说明）。
        """
        if not self._base_z:
            return
        import omni.usd
        from pxr import Gf, UsdGeom

        stage = omni.usd.get_context().get_stage()
        prims = (self._config.get("world") or {}).get("raise_prims") or []
        moved = []
        for path in prims:
            p = stage.GetPrimAtPath(path)
            if not p or not p.IsValid():
                continue
            x = UsdGeom.Xformable(p)
            for op in x.GetOrderedXformOps():
                if op.GetOpName() == "xformOp:translate":
                    v = op.Get()
                    op.Set(Gf.Vec3d(float(v[0]), float(v[1]),
                                    float(v[2]) + self._base_z))
                    moved.append(path)
                    break
        if moved:
            logger.info(f"静态场景抬高 {self._base_z} m: {moved}")

    def _apply_base_z(self):
        """把机器人抬到世界地面高度 `world.base_z`（config.json）。

        与 `_create_ground_plane` 的地平面、`_TABLE_POSE` 的复位高度、
        `_apply_base_z_static` 的桌子/物体是**一套, 必须同时改**：
        错开任何一个, 机器人要么从半空掉下来、要么复位到地面以下。

        base_z=0 时是空操作（历史默认）。
        """
        if not self._base_z:
            return
        try:
            self.articulation.set_world_poses(
                positions=np.asarray([[0.0, 0.0, self._base_z]],
                                     dtype=np.float64),
                orientations=np.asarray([[1.0, 0.0, 0.0, 0.0]],
                                        dtype=np.float64))
            logger.info(f"机器人抬到世界地面高度 z={self._base_z}")
        except Exception as e:
            logger.warning(f"设置机器人世界高度失败(继续): {e!r}")

    def _create_ground_plane(self):
        """创建地平面（带视觉网格与碰撞）。幂等：已存在则跳过（重复 load_scene 不再报错）。"""
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath("/World/ground_plane").IsValid():
            # 已存在也要按 base_z 摆一次（幂等），否则第二次 load_scene 会把
            # 高度留在上一次的值上。
            self._set_prim_translate_z("/World/ground_plane", self._base_z)
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
        # 高度：默认 z=0；用 world.base_z 时抬到扫描环境的可站地面。
        # **直接写 xformOp:translate**，不用 set_world_pose —— 后者在 GroundPlane
        # 上实测不生效（2026-10-21：设了 z=6.222，读回仍是 0，机器人直接掉到地面）。
        self._set_prim_translate_z("/World/ground_plane", self._base_z)
        logger.info(f"已创建默认地平面（z={self._base_z}）")

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
                binding = UsdShade.MaterialBindingAPI.Apply(p)
                binding.Bind(mat)
                # 6.2 GroundPlane会给geom单独绑定physics材质（默认restitution=.8）。
                # 普通绑定不会覆盖它，必须明确physics用途，才能兑现本方法的1/1/0设定。
                binding.Bind(mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                             materialPurpose="physics")
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

    # ==================== 外观材质 ====================

    def _fix_visual_materials(self, prim_path: str):
        """让 URDF 里的 `<material><color rgba=...>` 真正显色。

        **问题**：Isaac 的 URDF 导入器其实把颜色读进去了 —— 每个 link 上挂了
        `Looks/material_<颜色hex>`（如 `material_CAD1EE` = 0.792/0.820/0.933 淡蓝）。
        但它同时给每个 mesh 塞了一个**更近**的 `Looks/DefaultMaterial` 绑定。
        按 USD 的就近优先规则，mesh 级永远盖过 link 级 ⇒ URDF 颜色一次都没生效，
        整机渲染成默认白灰（2026-10-12 实测）。

        **修法**：mesh 藏在 `visuals`（instanceable）的原型里，而 **USD 禁止对
        实例原型写入**（`authoring to an instancing prototype is not allowed`）。
        所以先把 `visuals` 去实例化（在根层写 over，不改资产文件），再摘掉 mesh 级
        那个 `DefaultMaterial` 绑定，link 级材质就露出来了。

        只对**有 link 级材质**的 visual 组动手；`visuals` 里本来就没有材质绑定的
        资产（如老的人形 USD）保持原样，行为不变。
        """
        from pxr import Usd, UsdShade
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(prim_path)
        if not root or not root.IsValid():
            return

        def _bound(prim):
            rel = UsdShade.MaterialBindingAPI(prim).GetDirectBindingRel()
            return [str(t) for t in rel.GetTargets()] if rel else []

        deinstanced = unbound = 0
        for prim in Usd.PrimRange(root):
            if prim.GetName() != "visuals":
                continue
            # link 级材质：URDF 颜色绑在 visuals 下面第一层（以 mesh 文件命名的 Xform）。
            # **必须从原型读**：实例开启时 GetAllChildren() 拿到的是实例代理，
            # 代理上的关系读不出来（读到的永远是空），会误判成"没有材质"而跳过。
            # 原型只读不写是允许的，写才报 "authoring to an instancing prototype"。
            src = prim.GetPrototype() if prim.IsInstanceable() else prim
            if not src or not src.IsValid():
                continue
            if not any(_bound(c) for c in src.GetAllChildren()):
                continue
            if prim.IsInstanceable():
                prim.SetInstanceable(False)      # 原型不可写，先展开再改
                deinstanced += 1
            for child in prim.GetAllChildren():
                for node in child.GetAllChildren():
                    for mesh in node.GetAllChildren():
                        if mesh.GetTypeName() != "Mesh":
                            continue
                        b = _bound(mesh)
                        if b and b[0].split("/")[-1] == "DefaultMaterial":
                            UsdShade.MaterialBindingAPI(mesh).GetDirectBindingRel() \
                                .ClearTargets(True)
                            unbound += 1
        if deinstanced or unbound:
            logger.info(f"外观材质已修正: 去实例化 {deinstanced} 个 visuals, "
                        f"摘掉 {unbound} 个遮挡 URDF 颜色的 mesh 级 DefaultMaterial")

    # ==================== 软件 mimic ====================

    def _build_mimic_map(self):
        """构建软件 mimic 表：夹爪跟随关节 = gearing × 主动关节角度。

        倍率优先**从 URDF 的 `<mimic>` 标签读**（`load_urdf` 填 `_urdf_mimic`），
        链式 mimic（如 CT 夹爪的 Left_in→L2→R2）自动展开成对根主动关节的倍率。
        这样任何夹爪资产都不用改代码——单独加载官方 RM65+CT 时关节叫
        `ctag_R2`/`ctag_Left_in`，硬编码人形命名就认不出来了。

        没有 URDF 来源时（走 `load_robot` 加载已转换的 USD）退回人形命名规则：
          *left_main_joint / *left_3_joint → -1.0
          *_1_joint                        → -0.15
          其余                              → +1.0

        PhysX 的 PhysxMimicJointAPI 在该资产上不生效（导入时已用
        `parse_mimic=False` 剥离），由 step() 每帧软件强制执行，确定性有保障。
        """
        names = list(self.articulation.dof_names)
        mapping = self._mimic_map_from_urdf(names)
        if mapping is None:
            mapping = self._mimic_map_from_names(names)
        self._mimic_map = mapping or None
        self._mimic_targets = None
        self._gripper_followers = {i for entries in mapping.values()
                                   for i, _ in entries}
        if self._mimic_map:
            n_followers = sum(len(v) for v in mapping.values())
            logger.info(f"软件 mimic 已启用: {len(mapping)} 个主动关节, "
                f"{n_followers} 个跟随关节")
            self._tune_gripper_drives()
        # 底盘/颈部不依赖 mimic，但同样必须在**加载后**统一驱动参数
        self._tune_wheel_drives()
        self._tune_neck_drives()

    def _mimic_map_from_urdf(self, names):
        """按 URDF `<mimic>` 标签建表（链式展开到根主动关节）。无标签时返回 None。"""
        spec = getattr(self, "_urdf_mimic", None)
        if not spec:
            return None

        def root_of(j, seen=()):
            """沿 mimic 链上溯到根，返回 (根关节名, 累计倍率)。"""
            if j not in spec or j in seen:
                return j, 1.0
            src, mult = spec[j]
            r, m = root_of(src, seen + (j,))
            return r, mult * m

        mapping = {}
        for n in names:
            if n not in spec:
                continue
            leader, mult = root_of(n)
            if leader == n or leader not in names:
                continue          # 自环/根关节不在本机器人里 → 跳过
            mapping.setdefault(names.index(leader), []).append(
                (names.index(n), mult))
        return mapping or None

    @staticmethod
    def _mimic_map_from_names(names):
        """按人形关节命名规则建表（无 URDF 来源时的兜底，见 _build_mimic_map）。"""
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
        return mapping or None

    @staticmethod
    def _parse_urdf_mimic(urdf_path):
        """读 URDF 的 `<mimic joint= multiplier=>`，返回 {关节名: (源关节名, 倍率)}。

        纯 XML 解析，不依赖 Isaac，可在导入前后任意时刻调用。
        """
        import xml.etree.ElementTree as ET

        try:
            root = ET.parse(urdf_path).getroot()
        except Exception as e:
            logger.warning(f"URDF mimic 解析失败(退回命名规则): {e!r}")
            return None
        spec = {}
        for j in root.findall("joint"):
            m = j.find("mimic")
            if m is None or not j.get("name") or not m.get("joint"):
                continue
            try:
                mult = float(m.get("multiplier", 1.0))
            except ValueError:
                mult = 1.0
            spec[j.get("name")] = (m.get("joint"), mult)
        if spec:
            logger.info(f"URDF mimic 表: {len(spec)} 个跟随关节 <- {os.path.basename(urdf_path)}")
        return spec or None

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
            # 人形用 `*_gripper_*`；单独加载官方 RM65+CT 时夹爪关节叫 `ctag_*`
            idx = [i for i, n in enumerate(names)
                   if "gripper" in n or n.startswith("ctag")]
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

    def _tune_wheel_drives(self):
        """给底盘轮关节补上**位置驱动刚度**。

        仅用于没有四个转向关节的旧差速模型；四轮八驱保留原生速度驱动参数。

        为什么必须补：`_update_base_velocity` 是把 (vx, wz) 折算成轮速、再逐帧
        前移**位置目标**的虚拟速度模式。而人形资产烘出来的 `*.gains.json` 里
        四个轮关节是 `stiffness: 0 / damping: 99.7` —— 纯阻尼驱动会忽略位置
        目标，只按 targetVelocity(默认 0) 刹车。后果：`set_base_velocity`
        一路返回成功、轮子却在原地抖，底盘 5 秒只挪 **1.1mm**（实测 2026-10-13）。
        补上 stiffness 后同样指令 6 秒走 **963mm**（≈0.16 m/s）。

        取值来自 `config.json` 的 `base_teleop.wheel_stiffness/wheel_damping`
        （USD 级数值，写进物理视图要 ×57.29578，与 `_bake_joint_gains` 同口径）。
        """
        cfg = self._base_cfg or {}
        ks = float(cfg.get("wheel_stiffness", 203.0))
        kd = float(cfg.get("wheel_damping", 0.08))
        try:
            names = list(self.articulation.dof_names)
            if all(n in names for n in _BASE_DRIVE_JOINTS):
                # 舵轮使用真实速度目标；不再把原生0刚度速度驱动改成位置弹簧。
                return
            idx = [i for i, n in enumerate(names)
                   if n in _BASE_WHEEL_JOINTS]
            if not idx:
                return
            view = self.articulation._physics_view
            k = np.asarray(view.get_dof_stiffnesses()).reshape(-1).copy()
            d = np.asarray(view.get_dof_dampings()).reshape(-1).copy()
            before = round(float(k[idx[0]]), 1)
            if before > 0.0 and abs(before - ks * 57.29578) < 1.0:
                return          # 已是目标值
            k[idx] = ks * 57.29578
            d[idx] = kd * 57.29578
            view.set_dof_stiffnesses(k.reshape(1, -1), [0])
            view.set_dof_dampings(d.reshape(1, -1), [0])
            logger.info(f"底盘轮驱动刚度 {before} → {ks * 57.29578:.1f} "
                        f"(USD {ks:.1f}/{kd:.2f}, {len(idx)} 个轮)")
        except Exception as e:
            logger.warning(f"底盘轮驱动调参失败(底盘可能推不动): {e!r}")

    def _tune_neck_drives(self, zeta: float = 1.5):
        """恢复项目已有的颈部参考阻尼；force资产的真实阻尼比另需惯量标定。

        为什么必须补：URDF 里 `head_{pitch,yaw}_joint` 写的是 `damping=0.5`，
        但烘进 `*.gains.json` 时变成了 **0.0215 / 0.0435**（`_fix_gripper_joint_gains`
        只修名字含 `gripper` 的关节，颈部被漏掉）。阻尼几乎为 0 的后果实测：

          · 手臂一动手臂的反作用就让头**点头 3.5°**（抓取全程 head_pitch 极差
            61.7 mrad），肉眼就是"机身轻微晃动"；底盘只动 0.56mm、躯干 0.1°，
            静止整定后所有 link 摆幅 0.000mm —— 晃的只有头。
          · 补到 ζ=1.5 后同样一段流程 head_pitch 极差降到 **17.6 mrad (1.0°)**。

        旧 `acceleration` 型驱动可用 ζ = d/(2√k)。当前L3整机是 `force` 型，
        这里的 d = ζ·2·√k 仅沿用已做运动/抖动对照的仿真参考，不能把ζ当作
        force驱动的实物阻尼比；刚度、力矩上限与摩擦不随此方法改变。

        顺带修腕部相机转台（`*_camera_rojoint`）：同一个烘焙出来的低阻尼，
        实测抖动中位 1.2 mrad、峰值 13.8 mrad，是除头之外最大的。
        """
        try:
            names = list(self.articulation.dof_names)
            idx = [i for i, n in enumerate(names)
                   if n in ("head_pitch_joint", "head_yaw_joint")
                   or n.endswith("_camera_rojoint")]
            if not idx:
                return
            view = self.articulation._physics_view
            k = np.asarray(view.get_dof_stiffnesses()).reshape(-1).copy()
            d = np.asarray(view.get_dof_dampings()).reshape(-1).copy()
            before = {names[i]: round(float(d[i]), 3) for i in idx}
            changed = 0
            for i in idx:
                want = zeta * 2.0 * float(np.sqrt(max(k[i], 0.0)))
                if want > d[i]:
                    d[i] = want
                    changed += 1
            if not changed:
                return
            view.set_dof_dampings(d.reshape(1, -1), [0])
            logger.info(f"颈部/相机转台参考阻尼 → d={2*zeta:g}√k ({changed} 个关节; "
                        f"之前 {before})")
        except Exception as e:
            logger.warning(f"颈部驱动调参失败(头会晃): {e!r}")

    def _ensure_physics_scene(self):
        """写物理场景级参数（必须在 PhysX 建场景**之前**，运行时改不生效）。

        依据 PhysX 官方 *Articulation Stability Guide*：
          · `enableStabilization` —— 官方 `World` 文档说默认开，但本机实测
            `/physicsScene` 上是 **false**，得自己写。
          · `solverPositionIterationCount` —— "Increasing the number of position
            iterations with the TGS solver **reduces the effective simulation
            timestep**"，是比降 `physics_dt` 便宜的稳定化手段（用户要求不降性能）。
          · `solveArticulationContactLast` —— "gripping 场景下让 dynamic contact
            在求解顺序里靠后…可以减少持续接触穿透"。官方原文给的就是这个属性名。

        值放在 config.json 的 `physics` 段，可以单独关掉。
        """
        cfg = (self._config.get("physics") or {})
        if not cfg.get("enabled", False):
            logger.info("物理场景调参已关闭（config.json physics.enabled=false）")
            return
        it = int(cfg.get("solver_position_iterations", 32))
        spec = {
            "physxScene:enableStabilization":
                bool(cfg.get("enable_stabilization", True)),
            # **没有 `solverPositionIterationCount` 这个属性** —— PhysX 5 的
            # TGS 用的是 min/max 区间（求解器自适应），两个都设成同一个值
            # 就等价于固定迭代数。
            "physxScene:minPositionIterationCount": it,
            "physxScene:maxPositionIterationCount": it,
            # 官方给的属性名（`scenePrim.CreateSolveArticulationContactLastAttr()`），
            # PhysxSceneAPI 里没有现成 accessor，得自己建。
            "physxScene:solveArticulationContactLast":
                bool(cfg.get("solve_articulation_contact_last", True)),
        }
        try:
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            sc = stage.GetPrimAtPath("/physicsScene")
            if not sc.IsValid():
                logger.warning("找不到 /physicsScene，跳过物理场景调参")
                return
            from pxr import PhysxSchema, Sdf
            PhysxSchema.PhysxSceneAPI.Apply(sc)
            done = {}
            for k, v in spec.items():
                a = sc.GetAttribute(k)
                if not a:
                    a = sc.CreateAttribute(k, Sdf.ValueTypeNames.Bool
                                           if isinstance(v, bool)
                                           else Sdf.ValueTypeNames.Int)
                a.Set(v)
                done[k.split(":")[-1]] = v
            logger.info(f"物理场景参数: {done}")
        except Exception as e:
            logger.warning(f"物理场景调参失败: {e!r}")

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

    # ==================== URDF → USD 导入（按 Isaac 版本分派） ====================
    #
    # Isaac 5.1 与 6.2 的导入器是**两套完全不同的 API**：
    #
    #   5.1  `from isaacsim.asset.importer.urdf import _urdf`
    #        两步：parse_urdf(目录, 文件名, cfg) → import_robot(..., dest.usd)
    #        产物是**单个 .usd 文件**，不带 defaultPrim（要自己补写）
    #        有 `parse_mimic` 开关（本项目必须设 False，见下方长注释）
    #
    #   6.2  `from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig`
    #        一步：URDFImporter().import_urdf(cfg) → 返回顶层 .usda 路径
    #        `cfg.usd_path` 是**目录**，产物是 asset-transformer 分层结构：
    #            <dir>/<stem>/<stem>.usda          ← 顶层，带 defaultPrim
    #            <dir>/<stem>/payloads/base.usda
    #            <dir>/<stem>/payloads/Physics/physics.usda   ← 关节/驱动在这层
    #        顶层带 **Physics 变体集**（none/physics/physx/mujoco），默认没选，
    #        必须显式选 `physx` 否则关节属性不参与合成。
    #
    #   6.2 没有 `parse_mimic`。实测它的 mimic 关节**同时带 NewtonMimicAPI 和
    #        自己的 drive**（5.1 的 PhysxMimicJointAPI 是没 drive 的），所以
    #        软件 mimic 仍然写得进去 —— 但原生 NewtonMimicAPI 会同时生效，
    #        两者理论上等价（同一组 multiplier），实测未发现冲突。

    def _reject_if_robot_loaded(self):
        """已经有机器人时拒绝重复加载，并给出可操作的原因。

        为什么不让它"自动卸掉再加载"：两侧都试过，代价太大 —— 5.1 上二次
        URDF 导入会在 `UsdGeomXformable::AddXformOp` 里 abort 把进程带死，
        6.2 上则是 scene 注册表名字冲突。重启服务是唯一可靠的做法。
        """
        if getattr(self, "articulation", None) is None:
            return
        raise RuntimeError(
            "本会话已加载过机器人，不能再加载一次（Isaac 的 scene 注册表按名字"
            f"唯一）。当前已注册: {self.robot_prim_path}。"
            "请重启服务后再加载：pkill -f '[m]ain.py' 然后 ./scripts/run.sh")

    @staticmethod
    def _set_prim_translate_z(path: str, z: float):
        """把某个 prim 的 xformOp:translate 的 z 分量设成 z（其余分量不动）。

        比 `XFormPrim.set_world_pose` 可靠 —— 后者在 GroundPlane 这类包装对象上
        实测不生效（设了 6.222，读回仍是 0）。没有 translate op 时新建一个。
        """
        import omni.usd
        from pxr import Gf, UsdGeom

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            return False
        x = UsdGeom.Xformable(prim)
        for op in x.GetOrderedXformOps():
            if op.GetOpName() == "xformOp:translate":
                v = op.Get()
                op.Set(Gf.Vec3d(float(v[0]), float(v[1]), float(z)))
                return True
        x.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, float(z)))
        return True

    @staticmethod
    def _find_articulation_root(stage, root_path: str, max_depth: int = 4) -> str:
        """在 root_path 子树里找带 ArticulationRootAPI 的 prim（BFS）。

        找不到就返回 root_path 本身 —— 调用方（Articulation 构造）会在根上
        自己再找一次，不会因为这里返回保守值就静默失败。
        """
        root = stage.GetPrimAtPath(root_path)
        if not root or not root.IsValid():
            return root_path
        if root.HasAPI("ArticulationRootAPI"):
            return root_path
        queue = [(root, 0)]
        while queue:
            prim, depth = queue.pop(0)
            if depth >= max_depth:
                continue
            for child in prim.GetAllChildren():
                if child.HasAPI("ArticulationRootAPI"):
                    return str(child.GetPath())
                queue.append((child, depth + 1))
        return root_path

    def _import_urdf_to_usd(self, urdf_path: str, dest_path: str,
                            fix_base: bool, merge_fixed_joints: bool,
                            self_collision: bool) -> str:
        """导入 URDF，返回**可挂载的顶层 USD 文件路径**。"""
        try:
            from isaacsim.asset.importer.urdf import (  # noqa: F401
                URDFImporter, URDFImporterConfig)
            return self._import_urdf_62(urdf_path, dest_path, fix_base,
                                        merge_fixed_joints, self_collision)
        except ImportError:
            return self._import_urdf_51(urdf_path, dest_path, fix_base,
                                        merge_fixed_joints, self_collision)

    def _import_urdf_62(self, urdf_path: str, dest_path: str,
                        fix_base: bool, merge_fixed_joints: bool,
                        self_collision: bool) -> str:
        """Isaac 6.2：URDFImporter + URDFImporterConfig（一步，分层产物）。"""
        from isaacsim.asset.importer.urdf import (URDFImporter,
                                                  URDFImporterConfig)

        # 6.2 的 usd_path 是**目录**。调用方按老习惯传 "...xxx.usd" 时去掉
        # 后缀当目录名，避免与既有的同名 .usd 文件冲突。
        out_dir = dest_path or (os.path.splitext(urdf_path)[0] + "_usd")
        if out_dir.endswith(".usd") or out_dir.endswith(".usda"):
            out_dir = os.path.splitext(out_dir)[0]
        os.makedirs(out_dir, exist_ok=True)

        cfg = URDFImporterConfig()
        cfg.urdf_path = urdf_path
        cfg.usd_path = out_dir
        cfg.fix_base = fix_base
        cfg.merge_fixed_joints = merge_fixed_joints
        cfg.allow_self_collision = self_collision
        top = URDFImporter().import_urdf(cfg)
        if not top or not os.path.exists(top):
            raise RuntimeError(f"URDF 导入失败，产物不存在: {top!r}")
        logger.info(f"URDF 已转换为分层 USD: {top}")

        # 顶层带 Physics 变体集且默认不选 —— 不选 physx 的话关节/驱动属性
        # 根本不参与合成，Articulation 会是个没有驱动目标的空壳。
        try:
            from pxr import Usd
            stage = Usd.Stage.Open(top)
            root = stage.GetDefaultPrim() if stage else None
            if root:
                vs = root.GetVariantSets().GetVariantSet("Physics")
                if vs and vs.GetVariantSelection() != "physx":
                    vs.SetVariantSelection("physx")
                    stage.GetRootLayer().Save()
                    logger.info("已选定 Physics 变体: physx")
        except Exception as e:
            logger.warning(f"选定 Physics 变体失败(继续): {e!r}")

        # **剥掉原生 mimic 约束**（6.2 用 NewtonMimicAPI，5.1 用 PhysxMimicJointAPI）。
        # 本项目的夹爪靠 `_apply_software_mimic` 每帧自己给跟随关节写位置目标；
        # 原生约束同时生效时两套会互相顶 —— 6.2 上实测：带场景加载后夹爪跑到
        # 63 rad、PhysX 刷 Invalid transform，整个仿真发散成 NaN。
        # 5.1 侧对应的开关是 `parse_mimic=False`（见 _import_urdf_51），
        # 6.2 没有这个字段，所以在这里事后剥。
        try:
            from pxr import Usd
            stage2 = Usd.Stage.Open(top)
            stripped = 0
            for prim in stage2.Traverse():
                for api in ("NewtonMimicAPI", "PhysxMimicJointAPI"):
                    if prim.HasAPI(api):
                        prim.RemoveAPI(api)
                        stripped += 1
            if stripped:
                stage2.GetRootLayer().Save()
                logger.info(f"已剥离原生 mimic 约束 {stripped} 处"
                            "（软件 mimic 接管跟随关节）")
        except Exception as e:
            logger.warning(f"剥离原生 mimic 失败(夹爪可能发散): {e!r}")

        # 6.2 的导入器**不覆盖**已存在的产物目录，每次导入在同级建一个新目录
        # （`<stem>`、`<stem>_1`、`<stem>_2` …），每个 ~53 MB。只保留最新 2 个，
        # 否则反复加载会把磁盘堆满（2026-10-21 实测 5 次就 314 MB）。
        try:
            import re
            import shutil
            # top = <out_dir>/<dir>/<stem>.usda —— 要按**目录名**去重，
            # 不是文件名（文件名没有 _N 后缀，之前写错导致清理从不触发）。
            top_norm = os.path.normpath(top)
            parent = os.path.dirname(os.path.dirname(top_norm))
            dir_name = os.path.basename(os.path.dirname(top_norm))
            base = re.sub(r"_\d+$", "", dir_name)
            sibs = sorted(
                (os.path.join(parent, d) for d in os.listdir(parent)
                 if d == base or d.startswith(base + "_")),
                key=os.path.getmtime, reverse=True)
            for old in sibs[2:]:
                shutil.rmtree(old, ignore_errors=True)
                logger.info(f"清理旧导入产物: {old}")
        except Exception as e:
            logger.debug(f"清理旧导入产物失败(不影响加载): {e!r}")
        return top

    def _import_urdf_51(self, urdf_path: str, dest_path: str,
                        fix_base: bool, merge_fixed_joints: bool,
                        self_collision: bool) -> str:
        """Isaac ≤5.1：_urdf 两步流程（产物是单个 .usd，需补 defaultPrim）。"""
        from isaacsim.asset.importer.urdf import _urdf

        if dest_path is None:
            dest_path = os.path.splitext(urdf_path)[0] + ".usd"

        urdf_interface = _urdf.acquire_urdf_interface()
        import_config = _urdf.ImportConfig()
        import_config.fix_base = fix_base
        import_config.merge_fixed_joints = merge_fixed_joints
        import_config.self_collision = self_collision
        import_config.import_inertia_tensor = True
        # **关掉 PhysX 原生 mimic**（导入器默认 True）。夹爪四连杆的跟随关节
        # 在 URDF 里带 <mimic>，默认会给它们挂 PhysxMimicJointAPI —— 那种关节
        # **没有 drive、没有 limit**，而我们的软件 mimic(_apply_software_mimic)
        # 又每帧往它们写位置目标：写不进去 + PhysX 那套在本资产上不稳 ⇒
        # 跟随关节自由乱飞、夹爪散架（2026-10-12 实测左手 left_2 跑到 1.62rad）。
        # 关掉后跟随关节变回**普通驱动关节**，软件 mimic 才真正生效。
        try:
            import_config.parse_mimic = False
        except Exception as e:
            logger.warning(f"关闭 parse_mimic 失败(夹爪可能散架): {e!r}")
        # 位置驱动：可以直接 set_joint_positions 控制关节
        # (Isaac 5.1: 枚举 UrdfJointTargetType.JOINT_DRIVE_POSITION;
        #  旧版是 UrdfJointTarget.DRIVE_ANGLE)
        try:
            import_config.default_drive_type = \
                _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        except AttributeError:
            import_config.default_drive_type = _urdf.UrdfJointTarget.DRIVE_ANGLE

        # Isaac 5.x 两步流程: parse_urdf(目录, 文件名, cfg) 只内存解析;
        # import_robot 落盘 USD 并返回 articulation root
        urdf_dir, urdf_file = os.path.split(os.path.abspath(urdf_path))
        urdf_robot = urdf_interface.parse_urdf(urdf_dir, urdf_file,
                                               import_config)
        # 返回值是文件内的 articulation root 路径；这里用不到 —— 挂载后由
        # _find_articulation_root 在 stage 上现找，两代产物都能覆盖。
        urdf_interface.import_robot(
            urdf_dir, urdf_file, urdf_robot, import_config, dest_path, True)
        # 5.x 产物不带 defaultPrim —— 不补写的话挂载引用会 Unresolved reference。
        # defaultPrim 必须取顶层 Xform（lt10 实测）：指向 articulation root
        # （base_link）的嵌套 defaultPrim 会让 PhysX 展开异常甚至 segfault。
        from pxr import Usd as _Usd
        prod = _Usd.Stage.Open(dest_path)
        if prod is not None and not prod.GetRootLayer().HasDefaultPrim():
            tops = prod.GetPseudoRoot().GetAllChildren()
            if tops:
                prod.SetDefaultPrim(tops[0])
                prod.GetRootLayer().Save()
                logger.info(f"已补写 USD defaultPrim: {tops[0].GetPath()}")
        logger.info(f"URDF 已转换为 USD: {dest_path}")
        return dest_path

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
        from isaacsim.core.prims import Articulation
        from isaacsim.core.utils.stage import add_reference_to_stage

        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF not found: {urdf_path}")

        # 直接调 load_urdf 走同一道闸（它会花几十秒做导入，白跑一遍更糟）
        self._reject_if_robot_loaded()
        dest_path = self._import_urdf_to_usd(
            urdf_path, dest_path, fix_base, merge_fixed_joints, self_collision)
        # FK/IK 的链来源必须跟着实际加载的模型走（臂可能是 6 轴 RM65 重建版，
        # 而 arm_kin 默认指向 7 轴旧 URDF）。不同步会让抓取规划形状对不上、
        # VR IK 被关节顺序安全闸停用。见 arm_kin.set_urdf 的注释。
        try:
            arm_kin.set_urdf(urdf_path)
            logger.info(f"FK 链来源已切换: {urdf_path}")
        except Exception as e:
            logger.warning(f"切换 FK 链来源失败(保持原样): {e}")

        # 记下 URDF 的 <mimic> 表：_build_mimic_map 靠它建软件 mimic，
        # 不依赖关节命名（单独加载官方 RM65+CT 时关节叫 ctag_R2 / ctag_Left_in）
        self._urdf_mimic = self._parse_urdf_mimic(urdf_path)

        # 转换后增益烘焙: URDF 无 stiffness 概念、官方臂无 dynamics, 直转产物
        # damping≈0 会纯弹簧振铃。URDF 同目录 <stem>.gains.json 存在时按关节名
        # 烘 stiffness/damping 进 USD (值=USD 级, 见 scene-friction/bake_damping
        # 记忆: 物理 damping ≈ USD 值 × 57.3, 生成侧已折算)
        gains_path = os.path.splitext(urdf_path)[0] + ".gains.json"
        if os.path.exists(gains_path):
            self._bake_joint_gains(dest_path, gains_path)

        # 统一挂到用户指定的 prim_path，再在子树里找 ArticulationRootAPI。
        # **不能硬编码层级**：5.1 产物外面多包一层 base_link，6.2 的分层产物
        # 外面多包一层机器人名（defaultPrim），两者差一层。
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if not stage.GetPrimAtPath(prim_path).IsValid():
            add_reference_to_stage(dest_path, prim_path)
        staged_path = self._find_articulation_root(stage, prim_path)

        self.robot_prim_path = staged_path
        # 让 URDF 里的 <material> 真正显色（必须在建 Articulation 之前做，
        # 因为去实例化会动 stage；之后再动会把物理视图搞失效）
        self._fix_visual_materials(staged_path)

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
        self._apply_base_z()
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

    def get_joint_names(self, full: bool = False):
        """列出关节名。

        **默认隐藏夹爪的跟随关节**：一只 ctag2f120 夹爪物理上有 12 个可动件
        （四连杆机构），但对外只需要 1 个主动关节——其余由 `_apply_software_mimic`
        每帧按倍率补齐。所以列表里每只手只留 `*_gripper_right_2_joint`。

        **物理 DOF 一个没少**（`num_dofs_total` 是真实值），只是不对外列；
        要看全量传 `full=true`。RViz 那种"一个滑块控制整只夹爪"就是这个口径。
        """
        self._require_robot()
        all_names = list(self.articulation.dof_names)
        hide = self._gripper_followers
        names = (all_names if full
                 else [n for i, n in enumerate(all_names) if i not in hide])
        return {
            "prim_path": self.robot_prim_path,
            "dof_names": names,
            "num_dofs": len(names),
            "dof_names_all": all_names,
            "num_dofs_total": int(self.articulation.num_dof),
            "hidden": len(all_names) - len(names),
        }

    def set_joint_positions(self, positions: list, joint_names: list = None):
        """直接设置关节位置（瞬置语义；平滑控制用 send_joint_command）。"""
        self._require_robot()
        names = list(self.articulation.dof_names)
        if all(n in names for n in _BASE_DRIVE_JOINTS):
            selected = names if joint_names is None else list(joint_names)
            values = np.asarray(positions, dtype=np.float32).reshape(-1)
            if values.size != len(selected):
                raise ValueError('关节位置数量与关节名称数量不一致')
            keep = [i for i, name in enumerate(selected) if name not in _BASE_DRIVE_JOINTS]
            if keep:
                self.articulation.set_joint_positions(values[keep].reshape(1, -1),
                    joint_indices=[names.index(selected[i]) for i in keep])
            return {"status": "ok", "base_joints_ignored": len(selected)-len(keep)}
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
            leader = self._gripper_leader_for(s, names)
            if not leader:
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

    def _gripper_leader_for(self, side, names):
        """找该侧夹爪的主动关节名。

        人形：`_GRIPPER_LEADER` 里写死的 `{side}_gripper_right_2_joint`。
        单臂（官方 RM65+CT 单独加载）：关节叫 `ctag_R2` 之类，命名对不上，
        此时 `_mimic_map` 里**只有一个主动关节**，直接用它（side 参数忽略）。
        """
        leader = self._GRIPPER_LEADER.get(side)
        if leader and leader in names:
            return leader
        if self._mimic_map and len(self._mimic_map) == 1:
            idx = next(iter(self._mimic_map))
            return names[idx]
        return None

    def get_gripper_position(self, side: str = "right"):
        """回读夹爪开口：主动关节角 → rm 单位(1~1000) 与比例。

        side="both" 时返回 {"left": {...}, "right": {...}}（与 set_gripper_position
        的 both 语义对齐，此前会直接报错）。
        """
        if side == "both":
            return {"left": self.get_gripper_position("left"),
                    "right": self.get_gripper_position("right")}
        self._require_robot()
        names = list(self.articulation.dof_names)
        leader = self._gripper_leader_for(side, names)
        if not leader:
            raise ValueError(
                f"未找到夹爪主动关节(side={side})；已知关节: {names}")
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
    #: 机器人底盘在桌面正前方的标准位姿（X 键抓取第一步会送回来）。
    #: z 是**相对世界地面**的 —— 地面被 `world.base_z` 抬高时这里也要跟着抬，
    #: 否则复位会把机器人送到地面以下（见 _apply_base_z）。
    _TABLE_POSE_Z = 0.3295
    _TABLE_QUAT = (1.0, 0.0, 0.0, 0.0)

    def _reset_robot_to_table(self):
        """把机器人底盘送回桌面正前方的标准位姿，手臂回 home。

        用户要求：按 X 抓取前先"复位到桌前"，否则用 VR 把机器人开走之后
        随机摆放的可达范围就对不上了。
        """
        pos = (0.0, 0.0, self._base_z + self._TABLE_POSE_Z)
        quat = self._TABLE_QUAT
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
            # 回 **home_pose** 而不是硬编码 0：0 位是双臂水平前伸，很难看。
            home = self._home_pose_values()
            for s in ("left", "right"):
                for n in [f"{s}_arm_{i}_joint" for i in range(1, 7)] + \
                        [f"{s}_wrist_joint"]:
                    if n in names:
                        q[names.index(n)] = float(home.get(n, 0.0))
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
                  approach: float = 0.25, lift: float = 0.20,
                  squeeze: float = 0.5, max_dist: float = 0.45,
                  randomize: bool = True, reset_robot: bool = True,
                  place_at: list = None, forward: list = None):
        """启动**自动抓取**（非阻塞）。由 step() 逐帧推进。

        流程：**随机放物体** → 张开 → 取物体中心 → 合到目标开口量出闭合态
        TCP → 规划三段轨迹(含机身高度搜索) → 升降机身 → 预备 → 直线下压 →
        闭合 → 直线上抬 →（给了 `place_at` 就继续）→ 规划到放置点 → 下压 →
        张开 → 上抬。

        :param randomize: True=每次先随机挑一个方块放到手臂前方可达处再抓
            （菜单键默认）；False=抓夹爪附近最近的物体。
        :param place_at: [x, y] 世界坐标（桌面上的落点）。给了就抓完**放下**再
            结束；不给就停在"抬起来"的状态。z 由桌面高度 + 物体半高自动算。

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
            "place_at": (list(place_at) if place_at else None),
            # 随机落点的"前方"区间(米, 相对机器人底座)。调大 = 故意放远，
            # 用来验证"远物降机身去抓"这条路径。
            "forward": (tuple(float(v) for v in forward)
                        if forward else (0.44, 0.58)),
            "place_mode": False,
        }
        self._set_vr_suspended(True)
        logger.info(f"自动抓取启动 (side={side}, randomize={randomize}, "
                    f"place_at={place_at})")
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

    @staticmethod
    def _traj_sample(wps, u):
        """在路点序列上按参数 u∈[0,1] 做 Catmull-Rom 插值。

        比"每段线性插值"好在 **C1 连续**：速度在路点处不跳变，所以可以
        整段连续流送，不需要在路点上停下来。
        """
        P = np.asarray(wps, dtype=float)
        n = P.shape[0]
        if n == 1:
            return P[0]
        x = float(np.clip(u, 0.0, 1.0)) * (n - 1)
        i = min(int(np.floor(x)), n - 2)
        f = x - i
        p0 = P[max(i - 1, 0)]
        p1 = P[i]
        p2 = P[i + 1]
        p3 = P[min(i + 2, n - 1)]
        return 0.5 * ((2.0 * p1) + (-p0 + p2) * f
                      + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * f * f
                      + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * f * f * f)

    def _arm_err(self, ap, q_target, ):
        """实测手臂关节角与目标的最大偏差(rad)。inf = 读不到。"""
        try:
            names = list(self.articulation.dof_names)
            q = np.asarray(self.articulation.get_joint_positions()).reshape(-1)
            return max(abs(float(q[names.index(n)]) - float(t))
                       for n, t in zip(ap["arm_names"], q_target))
        except Exception:
            return float("inf")

    def _arm_at_rest(self, ap, tol: float = 0.05):
        """手臂关节**实测**速度是否都接近 0 (rad/s)。

        `self._motion is None` 只说明**指令**插值发完了, 不代表驱动把手臂带到了
        位——下降最后一步尤其明显。用实测速度判断才靠谱。
        """
        try:
            names = list(self.articulation.dof_names)
            v = np.asarray(self.articulation.get_joint_velocities()).reshape(-1)
            return all(abs(float(v[names.index(n)])) < tol
                       for n in ap["arm_names"])
        except Exception:
            return True

    def _place_target(self, ap):
        """放置点的 TCP 目标 = [x, y, 桌面高 + 半个物体高 + 2mm]。

        z 不给调用方算：物体底面要落在桌面上，得知道桌面高度和物体真实高度。
        """
        try:
            import omni.usd
            from pxr import UsdGeom

            x, y = float(ap["place_at"][0]), float(ap["place_at"][1])
            size = np.asarray(ap["object"]["size"], float)
            top = (ap.get("spawn") or {}).get("table_top")
            if top is None:
                st = omni.usd.get_context().get_stage()
                bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                       [UsdGeom.Tokens.default_])
                r = bb.ComputeWorldBound(
                    st.GetPrimAtPath("/World/vla_table")).ComputeAlignedRange()
                if r.IsEmpty():
                    return None
                top = float(np.array(r.GetMax())[2])
            return [x, y, float(top) + float(size[2]) * 0.5 + 0.002]
        except Exception as e:
            logger.warning(f"放置点计算失败: {e!r}")
            return None

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
                                         "pre", "grasp", "close", "lift",
                                         "release"):
            if ph in ("open", "reopen", "release"):
                # release: 放置时张开手指，把物体留在桌面上
                self.set_gripper_position(side=side, position=1000)
            elif ph in ("measure", "close"):
                # close 不在这里一次性写目标 —— 由 close 相位**逐帧渐变**下发
                # (见 close 分支)，否则指垫是一步撞上去的。
                if ph == "measure":
                    self.set_gripper_position(side=side, fraction=ap["frac"])
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
                np.asarray(ap.get("plan_target")
                           or ap["object"]["center"], float),
                approach=ap["approach"], lift=ap["lift"], seeds=seeds,
                jaw_axes=jaw_axes,
                # 路点打密: 默认 6 步时最后一步(落爪)是前几步的 2.5 倍关节运动,
                # 驱动追不上 → 合爪时手还在横移, 把方块扫出虎口(2026-10-13 实测)。
                descent_steps=12, lift_steps=8,
                # **方块离得远时降机身去够**（用户 2026-10-14 指出）。
                # 不降机身硬掰手臂的话，腕部被手臂限位卡住、接近轴只能斜着
                # ——实测"方块稍微远一些就不能垂直抓下去，会有一个角度"。
                # 降机身让肘部有余量，接近轴才拉得回铅垂。
                #
                # 2026-10-13 曾经把这里关成 (0.0,) 是因为**降机身不生效**：
                # `body` 相位只等 `_motion is None`(插值发完)就往下走，而
                # lift 驱动当时还没把机身带到位 → 规划按降了 dz 算、实际没降，
                # TCP 整体高 dz、抓空。现在 body 相位改成等**实测到位**了，
                # 可以重新打开。
                body_offsets=(0.0, -0.06, -0.12, -0.18),
                # 早点收：45 分意味着"位置 0mm + 朝下偏 5.6°"也直接接受，
                # 那就不会再去试更低的机身。压到 20 逼它把朝下误差换小。
                accept_score=20.0)
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
            # 每段轨迹的时长：按笛卡尔行程 / 参考线速度定，夹在 [1.0, 4.0]s。
            # 太短 → 伺服追不上、末端甩；太长 → 慢得让人等。
            ap["traj_T"] = {
                "descent": float(np.clip(ap["approach"] / 0.12, 1.0, 4.0)),
                "ascent": float(np.clip(ap["lift"] / 0.12, 1.0, 4.0)),
            }
            # **机身升降量在生成器的第 4 个返回值里，不在 info 里**：
            # `plan_grasp_steps` yield 的是 (score, qs, info, dz)，而
            # `info` 只由 `_plan_grasp_at` 填了 `infos`（`plan_grasp` 那个
            # 非生成器版本才往 info 里补 `body_dz`）。以前这里读
            # `info.get("body_dz", 0.0)` 永远拿到 0 —— 规划按"机身降了 dz"
            # 解的 IK、实际机身没降，TCP 整体高 dz 抓空。所以当时把
            # body_offsets 关成了 (0.0,)。现在直接取 yield 出来的 dz。
            ap["body_dz"] = float(dz)
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
                    ap["body_tgt"] = cur + float(ap["body_dz"])
                    self.send_joint_command([ap["body_tgt"]], duration=1.5,
                                            joint_names=["lift_joint"])
                ap["sent"] = True
            elif self._motion is None and el > 0.3:
                # **必须等机身"真的"到位**，不能只看 `_motion is None`（那只说明
                # 插值发完了）。2026-10-13 就是因为这里没等，降机身的指令跟不上、
                # 规划按"降了 dz"算而实际没降 → TCP 整体高 dz、抓空，于是把
                # body_offsets 关成了 (0.0,)。现在判据换成**实测 lift_joint 位置**，
                # 降机身才能重新启用（用户 2026-10-14 要求远物降机身去抓）。
                arrived = True
                try:
                    names = list(self.articulation.dof_names)
                    q = np.asarray(
                        self.articulation.get_joint_positions()).reshape(-1)
                    arrived = abs(float(q[names.index("lift_joint")])
                                  - float(ap["body_tgt"])) < 0.005
                except Exception:
                    pass
                if arrived or el > 4.0:
                    nxt("reopen")

        elif ph == "reopen":
            if el > 1.2:
                nxt("pre")

        elif ph == "pre":
            # 一步到物体上方悬停位(关节空间插值, 2s)。用户 2026-10-09 明确要
            # 回到这个版本 —— 加笛卡尔接近路径反而变成"先往上、再从上面接近"。
            #
            # **必须等手臂"真的"到位，不能只看 `_motion is None`**：那只是说
            # 插值发完了，伺服还在追。实测 `pre` 插值刚结束那一帧，实测关节角
            # 与目标最多差 **0.277 rad (16°)**；此时就开下降，等于让手臂在竖直
            # 下降的同时横向甩过去 —— 指垫扫到方块把它推走(实测 TCP 横向摆
            # 85mm，方块被推 160mm)，最后在离方块 10cm 的空气里合爪。
            # 用实测关节角判到位(与 `preclose` 同一套判据)，超时 3.5s 兜底。
            if el > 0.3 and self._motion is None:
                if el > 3.5 or self._arm_err(
                        ap, np.asarray(ap["qs"]["pre"], float)) < 0.02:
                    ap["wp"] = 0
                    nxt("descent")

        elif ph in ("descent", "ascent"):
            # **整段连续流送**，不再"发一个路点 → 等它停稳 → 再发下一个"。
            #
            # 旧做法为什么抖：`send_joint_command(duration=0.25)` 走的是
            # `_advance_motion` 的 **smoothstep 插值**，而 smoothstep 在两端
            # 速度都是 0 —— 每个路点都是一次"加速→减速→停死"。实测下降段
            # TCP 速度在 6~450mm/s 之间以 ~2.5Hz 来回摆(12 个路点 = 停 12 次),
            # 肉眼看就是"下夹爪的时候在抖"。
            #
            # 现在：路点之间用 Catmull-Rom(C1 连续)插值，整段用**一条** smoothstep
            # 时间律(只在起点/终点速度为 0)，每帧把参考点直接写进伺服目标。
            # 速度连续、中间不停，TCP 仍然沿着规划的笛卡尔直线走。
            wps = ap["qs"][ph]
            if not ap.get("sent"):
                # 清掉上一段遗留的 smoothstep 插值: 否则它会继续按旧目标写
                # 伺服目标，和本段的流送打架(位置被来回拽 = 又一种抖)。
                self._motion = None
                ap["sent"] = True
                ap["t"] = time.time()
                return
            T = float(ap["traj_T"][ph])
            u = min(el / T, 1.0)
            q = self._traj_sample(wps, smoothstep(u))
            self.send_joint_command(q.tolist(), duration=0.0,
                                    joint_names=ap["arm_names"])
            if u >= 1.0 and (el > T + 2.0
                             or self._arm_err(ap, np.asarray(wps[-1], float))
                             < 0.03):
                # 走完 ≠ 手臂停了: 合爪前还有 preclose 等实测速度归零
                nxt("preclose" if ph == "descent" else "settle")

        elif ph == "preclose":
            # 等**实测**关节速度真的接近 0 再合爪(超时兜底 1.5s)
            if el > 1.5 or (el > 0.35 and self._arm_at_rest(ap)):
                nxt("release" if ap.get("place_mode") else "close")

        elif ph == "release":
            # 放置：下压到位后张开手指，把物体留在桌面上，再抬起
            if el > 1.6 and self._arm_at_rest(ap):
                ap["placed"] = True
                ap["wp"] = 0
                ap["sent"] = False
                nxt("ascent")

        elif ph == "close":
            # **缓慢合爪**：把开口比例从**当前实测值**按 smoothstep 渐变到目标
            # 值，每帧下发一个中间值。
            #
            # 为什么要渐变：`set_gripper_position` 是把主动关节目标**一步**写到
            # 位，驱动会立刻以 maxVelocity(4 rad/s) 把指垫甩过去撞在物体上。
            # 夹爪驱动是 acceleration 型且刚度已提到 100000(等效 ~280 N·m/rad)，
            # 这一下的反作用力矩直接作用在腕部 —— 整条手臂被拽得**大幅度晃一下**
            # (用户 2026-10-14 实测："合爪准备抓取的时候会大幅度晃一下")。
            # 渐变之后加速度有界、接触是"压上去"而不是"撞上去"。
            #
            # f0 每帧读**实测**角度反查，所以重试补夹时会从当前开口继续收，
            # 不会退回全开。
            T = float(ap.get("close_T", 1.0))
            if el < T:
                try:
                    names = list(self.articulation.dof_names)
                    cur = float(np.asarray(self.articulation.get_joint_positions())
                                .reshape(-1)[names.index(
                                    self._GRIPPER_LEADER[side])])
                    f0 = gripper_model.fraction_for_leader(cur)
                except Exception:
                    f0 = 1.0
                self.set_gripper_position(
                    side=side, fraction=f0 + (float(ap["frac"]) - f0)
                    * smoothstep(el / T))
                return
            if el > T + 1.2:
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
                if ap.get("place_mode"):
                    # 放置段走完(张开 + 抬起 + 停稳) → 收工
                    ap["note"] = "已抓取并放下"
                    self._auto_pick_finish("完成（已放下）")
                    return
                # 任务级自检: 抬完看物体有没有真的跟着起来。单次抓取的物理
                # 成功率只有 ~50-70%(指垫差最后零点几毫米没接触), 但"重放重抓"
                # 两三次就能把整体成功率拉上去。
                ok = self._auto_pick_check_lifted(ap)
                if ok is None:
                    self._auto_pick_finish("完成")
                elif ok:
                    ap["lifted_ok"] = True
                    # 夹住了 → 如果给了放置点，接着走"放下"这一段：
                    # 复用同一套 pre/descent/ascent 规划，只是目标换成放置点、
                    # 收尾从"合爪"换成"张开"。
                    if ap.get("place_at") and not ap.get("placed"):
                        tgt = self._place_target(ap)
                        if tgt is not None:
                            ap["place_mode"] = True
                            ap["plan_target"] = tgt
                            for k in ("qs", "plan_gen", "plan_best",
                                      "grip_retry", "grip_added", "wp"):
                                ap.pop(k, None)
                            logger.info(f"自动抓取: 已夹住 → 转到放置点 "
                                        f"{np.round(tgt, 3).tolist()}")
                            nxt("plan")
                            return
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
        # 同一会话不能重复加载机器人：Isaac 的 scene 注册表按名字唯一，二次
        # `world.scene.add` 直接抛 "name is not unique"；5.1 上更糟 —— URDF
        # 导入器会在 AddXformOp 上 abort 把进程带死。这里提前拦成一句人话。
        self._reject_if_robot_loaded()
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
        # USD 路径拿不到 URDF 的 <mimic> 表 → 清掉上一次的，退回命名规则建表
        self._urdf_mimic = None
        self._fix_visual_materials(prim_path)
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
        self._apply_base_z()
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

    def _home_pose_values(self):
        """config.json 的 `home_pose` 里**当前机器人能用的**关节值 (dict)。

        `home_pose` 是按臂型标定的（7 轴旧臂 vs 6 轴 RM65 运动学不同，
        同一组角度摆出来完全不是一个姿势），所以**整侧校验**：一侧的
        `arm_1..6 + wrist` 必须**全部**存在才采用，缺一个就整侧跳过——
        宁可保持零位，也不要拿 7 轴的值去摆 6 轴的臂。
        """
        pose = (self._config or {}).get("home_pose", {}) or {}
        pose = {k: float(v) for k, v in pose.items()
                if not k.startswith("_") and isinstance(v, (int, float))}
        if not pose or self.articulation is None:
            return {}
        names = set(self.articulation.dof_names)
        out = {}
        for side in ("left", "right"):
            keys = [f"{side}_arm_{i}_joint" for i in range(1, 7)]
            if f"{side}_wrist_joint" in pose:
                keys.append(f"{side}_wrist_joint")
            if all(k in names for k in keys) and all(k in pose for k in keys):
                out.update({k: pose[k] for k in keys})
            elif any(k in pose for k in keys):
                logger.warning(
                    f"home_pose: {side} 臂的关节对不上当前机器人"
                    f"(缺 {[k for k in keys if k not in names]})，整侧跳过")
        return out

    def _apply_home_pose(self):
        """加载后把 `home_pose`（config.json）写入关节目标。
        未知关节名跳过并告警——URDF 零位=双臂水平前伸时，在此配置基础姿态。"""
        applied = self._home_pose_values()
        if not applied:
            return
        try:
            names = list(self.articulation.dof_names)
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

    # ==================== ROS2 契约（/<robot>/sim/<instance>/...） ====================
    #
    # 给上层运行时用的一套标准化仿真接口，目标是**同一套话题既能驱仿真、又能
    # 接真机**。手臂类话题走独立中继进程（scripts/ros2_sim_contract.py，系统
    # python3.10 + rclpy）；相机走 Isaac 内 OmniGraph（见 setup_ros2_cameras）。
    #
    # 为什么手臂不直接在 Isaac 里发：本 env 是 py3.12，Humble 的 rclpy 是
    # py3.10，进程内 import 不了。中继那边每周期只调一次 get_sim_state，
    # 消息量几十个 float，HTTP 完全够用。

    def _contract_cfg(self) -> dict:
        return self._config.get("ros2_contract") or {}

    def get_contract_meta(self):
        """中继启动时拉一次的静态信息（关节名、夹爪、末端 link、相机内参）。

        一次性返回，避免中继起来时发一串 HTTP 把单任务队列占满。
        """
        cfg = self._contract_cfg()
        robot = cfg.get("robot", "iasscsim_robot")
        role = cfg.get("role", "sim")
        base = f"/{robot}/{role}"

        dofs = list(self.articulation.dof_names) if self.articulation else []
        sides = {}
        for side in ("left", "right"):
            arm = [f"{side}_arm_{i}_joint" for i in range(1, 7)]
            arm = [n for n in arm if n in dofs]
            sides[side] = {
                "topic": f"{base}/{side}_arm",
                "arm_joints": arm,
                "gripper_joint": self._GRIPPER_LEADER.get(side),
                "flange_link": f"{side}_gripper_base_link",
                "tcp_links": [f"{side}_gripper_right_pad_link",
                              f"{side}_gripper_left_pad_link"],
            }

        return {
            "robot": robot,
            "role": role,
            "base_topic": base,
            "groups": cfg.get("groups") or {},
            "sides": sides,
            "cameras": self.get_camera_specs(),
            "joint_count": len(dofs),
            "errors": [] if dofs else ["机器人未加载"],
        }

    def get_camera_specs(self):
        """契约里 4 路相机的 prim / 分辨率 / 内参（供中继发 CameraInfo 兜底）。"""
        import omni.usd
        from pxr import UsdGeom

        cfg = (self._contract_cfg().get("cameras") or {})
        mapping = cfg.get("map") or {}
        if not mapping:
            mapping = {
                "left_hand_camera": {"prim": "left_wrist_gemini_335lg",
                                     "resolution": [1280, 800]},
                "right_hand_camera": {"prim": "right_wrist_gemini_335lg",
                                      "resolution": [1280, 800]},
                "chest_camera": {"prim": "chest_gemini_335lg",
                                 "resolution": [1280, 800]},
                "head_camera": {"prim": "head_zed_x", "resolution": [960, 600]},
            }
        stage = omni.usd.get_context().get_stage()
        out = {}
        base = f"/{self._contract_cfg().get('robot', 'iasscsim_robot')}/" \
               f"{self._contract_cfg().get('role', 'sim')}"
        for role_name, spec in mapping.items():
            prim_name = spec.get("prim")
            w, h = (spec.get("resolution") or [1280, 800])[:2]
            entry = {"prim": prim_name, "width": int(w), "height": int(h),
                     "frame_id": f"{role_name}_optical_link",
                     "image_topic": f"{base}/{role_name}/color/image_raw",
                     "info_topic": f"{base}/{role_name}/color/camera_info"}
            prim = stage.GetPrimAtPath(f"/World/cams/{prim_name}") if prim_name \
                else None
            entry["prim_valid"] = bool(prim and prim.IsValid())
            if entry["prim_valid"]:
                try:
                    cam = UsdGeom.Camera(prim)
                    focal = float(cam.GetFocalLengthAttr().Get())
                    ap = float(cam.GetHorizontalApertureAttr().Get()) or 20.955
                    fx = float(w) * focal / ap
                    entry.update({"focal_length_mm": round(focal, 6),
                                  "horizontal_aperture_mm": round(ap, 6),
                                  "fx": round(fx, 4), "fy": round(fx, 4),
                                  "cx": round(w / 2.0, 4),
                                  "cy": round(h / 2.0, 4)})
                except Exception as e:
                    entry["error"] = str(e)[:80]
            out[role_name] = entry
        return out

    def _link_pose_table(self):
        """一次读全所有 link 的世界位姿 → {name: (pos, quat_wxyz)}。"""
        art = self.articulation
        if art is None:
            return {}
        lt = np.asarray(art._physics_view.get_link_transforms()).reshape(-1, 7)
        table = {}
        # 6.2 的 Articulation 暴露的是 `body_names`（不是 link_names）。
        # 顺序与 get_link_transforms() 的行一一对应。
        for i, name in enumerate(getattr(art, "body_names", None) or []):
            if i >= lt.shape[0]:
                break
            p = np.asarray(lt[i][:3], float)
            # **TensorAPI 的 link transform 是 pos(3) + quat(4, xyzw)**，不是
            # wxyz。当 wxyz 读会让朝向整体错掉而位置仍然正确 —— 表现为"拿锚点
            # 对得上了、一动关节就差 90°"。实测依据：同一批位形按 xyzw 解出的
            # 基座常量变换在样本间只差 0.1°，按 wxyz 解差 55.8°；isaac_teleop
            # 的 `x, y, z, w = q` 注释也一致。这里统一重排成 wxyz 再入库，
            # 下游（_quat_wxyz_to_matrix / ROS2 契约）不必各自处理。
            q = np.asarray(lt[i][3:], float)[[3, 0, 1, 2]]      # xyzw -> wxyz
            n = float(np.linalg.norm(q))
            table[name] = (p, q / n if n > 1e-9 else q)
        return table

    @staticmethod
    def _quat_wxyz_to_matrix(q):
        w, x, y, z = q
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ])

    def _pose_in_ref(self, table, name, ref, ref_inv):
        """把 link 世界位姿表达在 ref 系里；ref 缺失时退回世界系。"""
        if name not in table:
            return None
        p, q = table[name]
        if ref is None or ref_inv is None:
            return {"position": [round(float(v), 6) for v in p],
                    "orientation": [round(float(v), 6) for v in q]}
        T = np.eye(4)
        T[:3, :3] = self._quat_wxyz_to_matrix(q)
        T[:3, 3] = p
        T = ref_inv @ T
        qq = self._matrix_to_quaternion(T[:3, :3])
        return {"position": [round(float(v), 6) for v in T[:3, 3]],
                "orientation": [round(float(v), 6) for v in qq]}

    def get_sim_state(self, ref_link: str = "base_link",
                      with_velocity: bool = True,
                      with_effort: bool = False):
        """中继每周期唯一的反馈入口：一次 HTTP 拿全。

        包含全量关节（名字/位置/速度/力矩）、当前驱动目标、双臂的夹爪状态与
        flange/tcp 位姿，以及仿真实时状态。**只读，不改仿真**。
        """
        self._require_robot()
        art = self.articulation
        names = list(art.dof_names)
        positions = [float(v) for v in
                     np.asarray(art.get_joint_positions()).reshape(-1)]
        out = {
            "sim_time": float(getattr(self.world, "current_time", 0.0) or 0.0),
            "playing": bool(self.world.is_playing()),
            "ref_link": ref_link,
            "joint_names": names,
            "positions": positions,
            "cmd_positions": [float(v) for v in self._current_targets()],
            "errors": [],
        }
        if with_velocity:
            try:
                out["velocities"] = [float(v) for v in np.asarray(
                    art.get_joint_velocities()).reshape(-1)]
            except Exception as e:
                out["velocities"] = []
                out["errors"].append(f"velocity: {e}")
        if with_effort:
            try:
                out["efforts"] = [float(v) for v in np.asarray(
                    art.get_measured_joint_efforts()).reshape(-1)]
            except Exception as e:
                out["efforts"] = []
                out["errors"].append(f"effort: {e}")

        table = self._link_pose_table()
        ref_inv = None
        if ref_link and ref_link in table:
            p, q = table[ref_link]
            T = np.eye(4)
            T[:3, :3] = self._quat_wxyz_to_matrix(q)
            T[:3, 3] = p
            ref_inv = np.linalg.inv(T)
        elif ref_link and ref_link != "world":
            out["errors"].append(f"ref_link '{ref_link}' 不存在，退回世界系")
            ref_link = None

        sides = {}
        for side in ("left", "right"):
            flange = f"{side}_gripper_base_link"
            pad_r = f"{side}_gripper_right_pad_link"
            pad_l = f"{side}_gripper_left_pad_link"
            entry = {"flange": self._pose_in_ref(table, flange, ref_link,
                                                 ref_inv)}
            # TCP = 两指垫中点；姿态沿用法兰（TCP 没有独立 link）
            if pad_r in table and pad_l in table:
                mid = 0.5 * (table[pad_r][0] + table[pad_l][0])
                if ref_inv is not None:
                    mid = (ref_inv @ np.append(mid, 1.0))[:3]
                fl = entry.get("flange")
                entry["tcp"] = {"position": [round(float(v), 6) for v in mid],
                                "orientation": fl["orientation"] if fl else
                                [0.0, 0.0, 0.0, 1.0]}
            else:
                entry["tcp"] = None
            try:
                g = self.get_gripper_position(side=side)
                entry["gripper"] = {k: g[k] for k in
                                    ("angle_rad", "opening_mm", "fraction",
                                     "position") if k in g}
            except Exception as e:
                entry["gripper"] = None
                out["errors"].append(f"gripper[{side}]: {e}")
            sides[side] = entry
        out["sides"] = sides
        return out

    def move_j(self, side: str = "left", positions: list = None,
               duration: float = 1.0, joint_names: list = None):
        """点到点关节目标（单臂 6 轴）。带限幅，走 send_joint_command 的插值。

        只下发这一侧臂关节 —— **不能让调用方拿反馈当输入全量下发**，
        那会把躯干/升降/头写成当前姿态，重力会把身体一点点压塌（2026-10-08）。
        """
        self._require_robot()
        if side not in ("left", "right"):
            raise ValueError(f"side 必须是 left/right，收到 {side!r}")
        names = list(positions and joint_names or
                     [f"{side}_arm_{i}_joint" for i in range(1, 7)])
        if joint_names:
            allowed = {f"{side}_arm_{i}_joint" for i in range(1, 7)}
            bad = [n for n in joint_names if n not in allowed]
            if bad:
                raise ValueError(f"move_j 只接受 {side} 臂关节，收到 {bad[:3]}")
            names = list(joint_names)
        vals = [float(v) for v in (positions or [])]
        if len(vals) != len(names):
            raise ValueError(f"positions({len(vals)}) 与 joint_names"
                             f"({len(names)}) 长度不一致")

        # 限幅：运行时真实限位（资产可能与 URDF 不同，见 isaac_teleop 的教训）
        dofs = list(self.articulation.dof_names)
        lim = np.asarray(self.articulation.get_dof_limits()).reshape(-1, 2)
        clamped = []
        for k, (n, v) in enumerate(zip(names, vals)):
            if n not in dofs:
                raise ValueError(f"未知关节 {n}")
            i = dofs.index(n)
            lo, hi = float(lim[i][0]), float(lim[i][1])
            nv = min(max(v, lo), hi)
            if abs(nv - v) > 1e-9:
                clamped.append([n, round(v, 4), round(nv, 4)])
            vals[k] = nv

        self.send_joint_command(vals, duration=float(duration),
                                joint_names=names)
        return {"sent": True, "side": side,
                "mode": "interpolating" if duration > 0 else "immediate",
                "duration": float(duration), "joint_names": names,
                "clamped": clamped}

    def setup_ros2_cameras(self, enabled: bool = True,
                           graph_path: str = None, fps: float = None,
                           publish_clock: bool = False):
        """建/删相机 OmniGraph（`/<robot>/sim/<cam>/color/{image_raw,camera_info}`）。

        **前置条件**：`/World/cams` 得有相机 prim。`config.json` 的
        `cameras.auto_register` 默认 false，那时 `/World/cams` 被
        `SetActive(False)` —— 本方法会自动把它重新激活（不动别的）。

        :param enabled: False = 删图，关掉相机话题
        :param fps: 相机发布频率，写进 `omni:sensor:tickRate`；默认读
            `ros2_contract.cameras.fps`
        """
        cfg = self._contract_cfg()
        cam_cfg = cfg.get("cameras") or {}
        graph_path = graph_path or cam_cfg.get("graph_path") or \
            "/ROS2_SimContract"
        if not enabled:
            return contract_graph.remove_camera_graph(graph_path)

        # cameras.auto_register=false 时 `/World/cams` 是 inactive，此时
        # `prim.IsValid()` 为 False，get_camera_specs() 会把它们全过滤掉。
        # 先激活再取规格（**激活** 不是删除，不会触发 PRIM_DELETION）。
        try:
            import omni.usd
            cams = omni.usd.get_context().get_stage().GetPrimAtPath(
                "/World/cams")
            if cams and cams.IsValid() and not cams.IsActive():
                cams.SetActive(True)
                logger.info("已重新激活 /World/cams（ROS2 相机话题需要）")
        except Exception as e:
            logger.warning(f"激活 /World/cams 失败: {e!r}")

        specs = self.get_camera_specs()
        cameras = {}
        for role, s in specs.items():
            if not s.get("prim_valid"):
                # 场景没加载或相机被停用 —— 交给调用方决定要不要报错
                continue
            cameras[role] = {
                "prim_path": f"/World/cams/{s['prim']}",
                "width": s["width"], "height": s["height"],
                "frame_id": s["frame_id"],
                "topic": f"{role}/color/image_raw",
                "info_topic": f"{role}/color/camera_info",
            }
        if not cameras:
            return {"ok": False, "graph_path": graph_path, "cameras": {},
                    "errors": ["没有任何可用相机 prim —— 先 load_scene，"
                               "并确认 cameras.auto_register / cams 未被停用"]}

        robot = cfg.get("robot", "iasscsim_robot")
        role = cfg.get("role", "sim")
        res = contract_graph.build_camera_graph(
            graph_path=graph_path, cameras=cameras,
            node_namespace=f"/{robot}/{role}",
            fps=float(fps if fps is not None
                      else (cam_cfg.get("fps") or 15.0)),
            publish_clock=bool(publish_clock))
        res["ok"] = not res["errors"]
        logger.info(f"ROS2 相机图: {res.get('ok')} "
                    f"{list(res.get('cameras', {}))} {res['errors']}")
        return res

    def setup_ros2_contract(self, enabled: bool = True, rate_hz: float = None,
                            ref_link: str = None):
        """启停**进程内** ROS2 契约节点（`/<robot>/sim/<instance>/...`）。

        与 `scripts/ros2_sim_contract.py`（py3.10 中继 + HTTP）是同一套话题的
        两种实现 —— 二选一，别同时开。进程内这条没有 HTTP 往返，反馈能跑到
        物理帧率；中继那条留给「Isaac 环境里 rclpy 不可用」的兜底场景。

        :param rate_hz: 反馈发布频率（默认 50）；控制回调是每帧都处理的，不受此限
        :param ref_link: flange/tcp 的参考系（默认 `base_link`）
        """
        cfg = self._contract_cfg()
        if not enabled:
            if self._ros2_contract is not None:
                self._ros2_contract.shutdown()
                self._ros2_contract = None
                logger.info("ROS2 契约节点已关闭")
            return {"running": False}

        if self._ros2_contract is not None:
            return {"running": True, "already": True,
                    "node": self._ros2_contract._node.get_name()
                    if self._ros2_contract._node else None}

        rate_hz = float(rate_hz if rate_hz is not None
                        else (cfg.get("state_hz") or 50.0))
        self._ros2_contract_ref = ref_link or "base_link"

        self._require_robot()
        handlers = {
            "send_joint_command": self.send_joint_command,
            "move_j": self.move_j,
            "set_gripper_position": self.set_gripper_position,
        }
        try:
            # 传 **meta** 而不是 cfg：sides（夹爪关节名 / flange / tcp link）
            # 是运行时从 articulation 推导的，配置里没有。
            self._ros2_contract = contract_node.ContractNode(
                self.get_contract_meta(), handlers)
        except Exception as e:
            logger.error(f"ROS2 契约节点创建失败: {e!r}")
            self._ros2_contract = None
            return {"running": False, "error": f"{type(e).__name__}: {e}"}

        self._ros2_contract_period = 1.0 / max(rate_hz, 1e-3)
        self._last_ros2_publish = 0.0
        return {"running": True, "rate_hz": rate_hz,
                "base": self._ros2_contract.base}

    def ros2_contract_status(self):
        """查进程内契约节点状态。"""
        node = self._ros2_contract
        if node is None:
            return {"running": False}
        return {"running": True,
                "node": node._node.get_name() if node._node else None,
                "base": node.base,
                "rate_hz": round(1.0 / max(self._ros2_contract_period, 1e-9), 2),
                "ref_link": self._ros2_contract_ref,
                "publishers": len(node.pub)}

    def _service_ros2_contract(self):
        """主线程：处理一次 ROS2 控制回调。

        回调就在这一句里同步执行（`spin_once` 跑在主线程），所以回调里直接调
        `send_joint_command` / `move_j` / `set_gripper_position` 是安全的 ——
        不需要缓冲、不需要锁。
        """
        if self._ros2_contract is None or self.articulation is None:
            return
        self._ros2_contract.spin_once(0.0)

    def _publish_ros2_contract(self):
        """主线程：按 `rate_hz` 把 `get_sim_state()` 发布到 feedback/*。"""
        node = self._ros2_contract
        if node is None or self.articulation is None:
            return
        if not self.world.is_playing():
            return
        now = time.time()
        if now - self._last_ros2_publish < self._ros2_contract_period:
            return
        self._last_ros2_publish = now
        try:
            st = self.get_sim_state(ref_link=self._ros2_contract_ref,
                                    with_velocity=True)
            node.publish(st, ref_link=self._ros2_contract_ref)
        except Exception as e:
            logger.warning(f"ROS2 契约发布失败: {e!r}")

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

        if self._ros_controller is None or not self._ros_controller.is_initialized():
            # 无可用 ROS2 桥的回退：positions 顺序 = articulation dof_names，
            # 直接施加（HTTP/relay 控制路径）。
            # **必须连 is_initialized() 一起判**：Isaac 6.2 是 py3.12、Humble
            # rclpy 是 py3.10，setup_ros2_bridge 会留下一个构造失败的
            # `_ros_controller`（非 None 但 initialized=False），只判 None 会
            # 误走 ROS 路径，指令凭空消失。
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

    # ==================== 底盘（四轮八驱；兼容旧四轮差速模型） ====================

    def _get_swerve_model(self):
        names = list(self.articulation.dof_names)
        if not all(name in names for name in _BASE_DRIVE_JOINTS):
            return None
        cached = getattr(self, "_swerve_model", None)
        if cached is not None and getattr(self, "_swerve_articulation", None) is self.articulation:
            return cached
        from swerve_base import SwerveModel
        # load_urdf及当前L2工位会同步arm_kin.URDF；不靠旧wheel_radius/track_width猜几何。
        source = (self._base_cfg or {}).get("swerve_urdf") or arm_kin.URDF
        model = SwerveModel.from_urdf(source)
        limits = np.asarray(self.articulation.get_dof_limits()).reshape(-1, 2)
        model.set_steering_limits([limits[names.index(module.steering_joint)] for module in model.modules])
        self._swerve_model, self._swerve_articulation = model, self.articulation
        return model

    def get_base_drive_info(self):
        """读取实际底盘结构、几何、限位和参考规格标记，不下发运动。"""
        self._require_robot()
        model = self._get_swerve_model()
        if model is None:
            return {"mode": "skid_steer_legacy", "supports_lateral": False}
        result = model.metadata()
        try:
            model.solve(0., .1, 0.)
            result["supports_lateral"] = True
        except ValueError:
            result["supports_lateral"] = False
        result["steering_reference"] = getattr(self, "_swerve_reference", None)
        result["control_active"] = self._base_cmd is not None
        result["last_control"] = getattr(self, "_swerve_last_control", None)
        result["last_verified_stop"] = getattr(self, "_swerve_last_stop", None)
        result["parking_status"] = getattr(self, "_swerve_last_parking", None)
        result["parking_verified"] = bool(self._base_cmd is None and
            (getattr(self, "_swerve_last_parking", None) or {}).get('parking_verified',False))
        return result

    def park_base(self):
        """显式前向驻车：停轮后把四舵回到0，再验证真实停稳；不固定基座。"""
        self._require_robot()
        model = self._get_swerve_model()
        if model is None:
            raise RuntimeError('当前机器人不具备可回正驻车的四轮八驱结构')
        if self._base_cmd is not None and self._base_cmd.get('parking'):
            return dict(getattr(self,'_swerve_last_parking',{}) or
                        {'phase':'braking','parking_verified':False})
        names = list(self.articulation.dof_names)
        q = np.asarray(self.articulation.get_joint_positions()).reshape(-1)
        si = [names.index(m.steering_joint) for m in model.modules]
        target = model.solve_cardinal(.1,0.,q[si])['steering']
        if np.max(abs(target)) > 1e-6:
            raise RuntimeError('当前URDF转向零位不是前向，需重新确认驻车参考')
        last = getattr(self,'_swerve_last_parking',None) or {}
        stopped = getattr(self,'_swerve_last_stop',None)
        pose = self._swerve_base_pose()
        if (self._base_cmd is None and last.get('parking_verified') and stopped is not None
                and np.max(abs(q[si])) < np.deg2rad(.5)
                and np.max(abs(pose[:2]-np.asarray(stopped['base_pose'])[:2])) < .002
                and abs((pose[2]-stopped['base_pose'][2]+np.pi)%(2*np.pi)-np.pi) < .01):
            return dict(last)
        if getattr(self,'_swerve_drive_state',None) is None:
            self._engage_swerve_drive(model)
        state = self._swerve_drive_state
        state['parking_preposition'] = None
        state['parking_stop_window'] = None
        state['cardinal_axis'],state['cardinal_preposition'] = None,None
        state['rotation_preposition'] = None
        state['rotation_applied_speed'] = 0.
        state['brake_sent'] = False
        self._base_cmd = {'vx':0.,'vy':0.,'wz':0.,'t':time.time(),'parking':True}
        self._swerve_last_parking = {'phase':'braking','parking_verified':False,
            'steering_targets_rad':[0.]*4,'reference_only':True,
            'message':'正在停轮并回正四个舵角，尚未完成驻车'}
        return dict(self._swerve_last_parking)

    def configure_swerve_base(self, reference_steering_limit_deg: float = None):
        """会话内应用用户授权的参考舵角；不写回源URDF/USD，原值留在返回元数据中。"""
        self._require_robot()
        model = self._get_swerve_model()
        if model is None:
            raise RuntimeError('当前机器人不具备4转向+4驱动的完整底盘')
        if self._base_cmd is not None:
            raise RuntimeError('请先停止底盘并等待停稳，再调整参考舵角')
        if reference_steering_limit_deg is None:
            return self.get_base_drive_info()
        value = float(reference_steering_limit_deg)
        if not np.isfinite(value) or not 0 < value <= 180.:
            raise ValueError('参考舵角应在0～180度之间')
        from pxr import Usd, UsdPhysics
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.robot_prim_path)
        joints = {prim.GetName(): prim for prim in Usd.PrimRange(root) if prim.IsA(UsdPhysics.Joint)}
        names = list(self.articulation.dof_names)
        indices = [names.index(m.steering_joint) for m in model.modules]
        original = np.asarray(self.articulation.get_dof_limits()).copy()
        original = original.reshape(1, len(names), 2)
        before_usd = {m.steering_joint: [joints[m.steering_joint].GetAttribute('physics:lowerLimit').Get(),
                      joints[m.steering_joint].GetAttribute('physics:upperLimit').Get()] for m in model.modules}
        if not hasattr(self, '_swerve_reference_backup'):
            self._swerve_reference_backup = {'runtime_limits_rad': original[0, indices].tolist(),
                                             'usd_limits_deg': before_usd}
        changed = original.copy()
        changed[0, indices, :] = [-np.deg2rad(value), np.deg2rad(value)]
        # Session layer只覆盖当前仿真会话，关闭场景即失效，不污染源资产。
        try:
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                for module in model.modules:
                    joint = joints[module.steering_joint]
                    joint.GetAttribute('physics:lowerLimit').Set(-value)
                    joint.GetAttribute('physics:upperLimit').Set(value)
            self.articulation._physics_view.set_dof_limits(changed.astype(np.float32), np.array([0], dtype=np.uint32))
            readback = np.asarray(self.articulation.get_dof_limits()).reshape(-1, 2)[indices]
            if not np.allclose(readback, changed[0, indices], atol=1e-6):
                raise RuntimeError('参考舵角张量读回不一致')
        except Exception:
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                for name, bounds in before_usd.items():
                    joints[name].GetAttribute('physics:lowerLimit').Set(bounds[0])
                    joints[name].GetAttribute('physics:upperLimit').Set(bounds[1])
            self.articulation._physics_view.set_dof_limits(original.astype(np.float32), np.array([0], dtype=np.uint32))
            raise
        model.set_steering_limits(readback)
        self._swerve_reference = {'steering_limit_deg': value, 'calibrated_to_user_hardware': False,
            'source': '用户授权的仿真参考规格，待实测', 'layer': 'session',
            'original': self._swerve_reference_backup}
        return self.get_base_drive_info()

    def _engage_swerve_drive(self, model):
        names = list(self.articulation.dof_names)
        wi = [names.index(m.wheel_joint) for m in model.modules]
        si = [names.index(m.steering_joint) for m in model.modules]
        view = self.articulation._physics_view
        stiffness = np.asarray(view.get_dof_stiffnesses()).reshape(-1).copy()
        damping = np.asarray(view.get_dof_dampings()).reshape(-1)
        if np.any(stiffness[si] <= 0.):
            raise RuntimeError('当前转向关节缺少位置伺服刚度，请先确认舵机参考参数')
        if np.any(damping[wi] <= 0.):
            raise RuntimeError('当前滚轮缺少速度伺服阻尼，请先确认驱动参考参数')
        # 只在已有位置弹簧时临时切换控制模式；原生速度驱动不改阻尼/力矩/摩擦。
        self._swerve_drive_state = {'wheel_indices': wi, 'steering_indices': si,
            'saved_stiffness': stiffness[wi].copy(), 'gain_override': bool(np.any(stiffness[wi] != 0.)),
            'brake_since': None, 'brake_sent': False,
            'cardinal_axis': None, 'cardinal_preposition': None,
            'rotation_preposition': None, 'rotation_applied_speed': 0.}
        self._swerve_stop_window = None
        if self._swerve_drive_state['gain_override']:
            stiffness[wi] = 0.
            view.set_dof_stiffnesses(stiffness.reshape(1, -1), [0])

    def set_base_velocity(self, vx: float = 0.0, wz: float = 0.0, vy: float = 0.0,
                          steering_targets=None):
        """底盘局部速度：vx前进、vy向左(m/s)，wz逆时针(rad/s)；保留旧(vx,wz)位置参数。

        客户端需持续刷新，超过timeout自动刹停。只有8关节齐全且舵角限位可行才走舵轮模式。
        steering_targets可固定纯车身X/Y单轴平移的4个舵角(rad)，上层仍负责环境扫掠；
        旋转、混合速度、旧差速或停止不接受该参数。省略时保留既有选支行为。
        """
        self._require_robot()
        requested = np.asarray([vx, vy, wz], dtype=float)
        if not np.isfinite(requested).all():
            raise ValueError('底盘速度必须为有限数')
        cfg = self._base_cfg or {}
        vx = float(np.clip(vx, -float(cfg.get('max_vx', .6)), float(cfg.get('max_vx', .6))))
        vy = float(np.clip(vy, -float(cfg.get('max_vy', cfg.get('max_vx', .6))), float(cfg.get('max_vy', cfg.get('max_vx', .6)))))
        wz = float(np.clip(wz, -float(cfg.get('max_wz', 1.6)), float(cfg.get('max_wz', 1.6))))
        model = self._get_swerve_model()
        requested_steering = None
        if steering_targets is not None:
            if model is None or model.cardinal_axis(vx,vy,wz) is None:
                raise ValueError('固定舵角仅用于四轮八驱的纯前后或纯左右平移；停止时请省略steering_targets')
            try:
                requested_steering = np.asarray(steering_targets,dtype=float).copy()
            except (TypeError,ValueError) as error:
                raise ValueError('固定舵角需要4个有限弧度值') from error
            # 在改动当前任务/驻车状态前拒绝超限、非平行或形状错误的提示。
            requested_steering = model.solve_cardinal(vx,vy,None,requested_steering)['steering'].copy()
        old_parking = self._base_cmd is not None and self._base_cmd.get('parking')
        if old_parking or any(abs(v)>1e-12 for v in (vx,vy,wz)):
            self._swerve_last_parking = {'phase':'cancelled' if old_parking else 'inactive',
                'parking_verified':False,'message':'驻车已被普通底盘速度指令取消'}
            state = getattr(self,'_swerve_drive_state',None)
            if state is not None:
                state['parking_preposition'],state['parking_stop_window'] = None,None
                if old_parking:state['brake_sent'] = False
        solution = None
        if model is not None:
            names = list(self.articulation.dof_names)
            q = np.asarray(self.articulation.get_joint_positions()).reshape(-1)
            actual = [q[names.index(m.steering_joint)] for m in model.modules]
            axis = model.cardinal_axis(vx,vy,wz)
            state = getattr(self, '_swerve_drive_state', None) or {}
            locked = state.get('cardinal_preposition') if state.get('cardinal_axis') == axis else None
            target = requested_steering if requested_steering is not None else (locked.target if locked is not None else None)
            solution = (model.solve_cardinal(vx,vy,actual,target)
                        if axis is not None else model.solve(vx,vy,wz,actual))
            vx, vy, wz = map(float, solution['executed_twist'])
            if (vx == 0. and vy == 0. and wz == 0. and self._base_cmd is None
                    and getattr(self, '_swerve_drive_state', None) is None):
                wi = [names.index(m.wheel_joint) for m in model.modules]
                si = [names.index(m.steering_joint) for m in model.modules]
                last = getattr(self, '_swerve_last_stop', None)
                pose = self._swerve_base_pose()
                if (last is not None and np.max(abs(pose[:2]-np.asarray(last['base_pose'])[:2])) < .002
                        and abs((pose[2]-last['base_pose'][2]+np.pi) % (2*np.pi)-np.pi) < .01
                        and np.max(abs(q[wi]-np.asarray(last['wheel_positions']))) < .015
                        and np.max(abs(q[si]-np.asarray(last['steering_positions']))) < .01):
                    # 仍在上一次真实停稳的允许变化范围内，不因速度微振重复唤醒整机。
                    return {'vx': 0., 'vy': 0., 'wz': 0., 'timeout': float(cfg.get('timeout', .5)),
                            'mode': 'four_wheel_independent_steering_drive', 'stopped': True,
                            'steering_reference': getattr(self, '_swerve_reference', None)}
            if getattr(self, '_swerve_drive_state', None) is None:
                self._engage_swerve_drive(model)
        elif vy != 0.:
            raise ValueError('当前旧差速底盘不支持横移vy')
        engage = self._base_cmd is None
        self._base_cmd = {'vx': vx, 'vy': vy, 'wz': wz, 't': time.time()}
        if requested_steering is not None:
            self._base_cmd['steering_targets'] = requested_steering.tolist()
        if engage and (vx != 0.0 or vy != 0.0 or wz != 0.0):
            # 按当前stage/真实刚体查询sleep，只唤醒睡着的实体。即使把同一组
            # 速度写回去也属于状态修改；原生wake避免在已醒且抖动时再次触碰它。
            from scripts.physics_wake import wake_if_sleeping
            wake = wake_if_sleeping(self)
            self._swerve_last_wake = wake
            if not wake['supported'] or wake['sleep_after'] is not False:
                self._base_cmd = None
                raise RuntimeError('底盘原生唤醒未验证，未下发速度状态fallback: ' + str(wake))
        result = {'vx': vx, 'vy': vy, 'wz': wz, 'timeout': float(cfg.get('timeout', .5)),
                  'mode': 'four_wheel_independent_steering_drive' if model is not None else 'skid_steer_legacy'}
        if solution is not None:
            result.update(steering_targets_rad=solution['steering'].tolist(),
                          wheel_velocity_targets_rad_s=solution['wheel_velocity'].tolist(),
                          steering_targets_locked=requested_steering is not None,
                          steering_reference=getattr(self, '_swerve_reference', None))
        return result

    def _swerve_base_pose(self):
        position, quaternion = self._link_pose_table()['base_link']
        w, x, y, z = quaternion
        yaw = np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        return np.array([position[0], position[1], yaw], dtype=float)

    def _swerve_base_ypr(self):
        _, quaternion = self._link_pose_table()['base_link']
        w,x,y,z = quaternion
        return np.array([np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)),
                         np.arcsin(np.clip(2*(w*y-z*x),-1.,1.)),
                         np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y))],dtype=float)

    def _update_swerve_parking(self, model, state, q, velocity):
        from swerve_base import SteeringPrealignment,BaseSettlingWindow
        arti = self.articulation
        wi,si = state['wheel_indices'],state['steering_indices']
        now = float(self.world.current_time)
        transition = state.get('parking_preposition')
        if transition is None:
            rate = min(float((self._base_cfg or {}).get('steering_preposition_rate_rad_s',.6)),
                       min(m.steering_max_velocity_rad_s for m in model.modules))
            transition = SteeringPrealignment(q[si],np.zeros(4),now,
                max_rate_rad_s=rate,angle_tolerance_rad=np.deg2rad(.5))
            state['parking_preposition'] = transition
            state['parking_ypr_start'] = self._swerve_base_ypr().copy()
        pose = self._swerve_base_pose()
        command,alignment = transition.update(now,q[si],q[wi],pose)
        arti.set_joint_velocity_targets(np.zeros((1,4),dtype=np.float32),joint_indices=wi)
        arti.set_joint_position_targets(command.reshape(1,-1).astype(np.float32),joint_indices=si)
        verification = None
        if alignment['ready']:
            if state.get('parking_stop_window') is None:
                state['parking_stop_window'] = BaseSettlingWindow()
            verification = state['parking_stop_window'].update(now,pose,q[si],q[wi])
        else:
            state['parking_stop_window'] = None
        done = bool(verification and verification['settled'] and np.max(abs(q[si]))<np.deg2rad(.5))
        status = {'phase':'parked' if done else ('verifying' if alignment['ready'] else alignment['stage']),
            'parking_verified':done,'steering_targets_rad':[0.]*4,
            'steering_actual_rad':q[si].tolist(),'steering_zero_error_rad':float(np.max(abs(q[si]))),
            'alignment':alignment,'verification':verification,
            'base_ypr_start_rad':state['parking_ypr_start'].tolist(),'base_ypr_rad':self._swerve_base_ypr().tolist(),
            'wheel_velocity_targets_rad_s':[0.]*4,'elapsed_s':now-transition.started,
            'calibrated_to_user_hardware':False,
            'message':'四轮已回正并验证停稳' if done else '四轮回正驻车中，滚轮速度保持0'}
        self._swerve_last_parking = status
        self._swerve_last_control = dict(status,motion_kind='parking')
        if done:
            if state['gain_override']:
                arti.set_joint_position_targets(q[wi].reshape(1,-1).astype(np.float32),joint_indices=wi)
                stiffness = np.asarray(arti._physics_view.get_dof_stiffnesses()).reshape(-1).copy()
                stiffness[wi] = state['saved_stiffness']
                arti._physics_view.set_dof_stiffnesses(stiffness.reshape(1,-1),[0])
            self._swerve_last_stop = {'base_pose':pose.tolist(),'wheel_positions':q[wi].tolist(),
                'steering_positions':q[si].tolist(),'sim_time':now,'verification':verification,
                'parking_verified':True}
            status['verified_at_sim_time'] = now
            self._base_cmd,self._wheel_pos_target,self._swerve_drive_state = None,None,None
            self._swerve_stop_window = None
        elif now-transition.started > max(12.,transition.duration+8.):
            # 有界失败后保持当下舵角并转普通停轮，不能把超时误报驻车完成。
            arti.set_joint_position_targets(q[si].reshape(1,-1).astype(np.float32),joint_indices=si)
            self._base_cmd = {'vx':0.,'vy':0.,'wz':0.,'t':time.time()}
            state['brake_sent'] = False
            status.update(phase='failed',message='四轮回正驻车超时，已停止，驻车未验收')
            self._swerve_last_control = dict(status,motion_kind='parking')

    def _update_swerve_velocity(self, model, cmd):
        arti = self.articulation
        state = getattr(self, '_swerve_drive_state', None)
        if state is None:
            self._engage_swerve_drive(model)
            state = self._swerve_drive_state
        wi, si = state['wheel_indices'], state['steering_indices']
        q = np.asarray(arti.get_joint_positions()).reshape(-1)
        velocity = np.asarray(arti.get_joint_velocities()).reshape(-1)
        if not np.isfinite(q[wi+si]).all() or not np.isfinite(velocity[wi+si]).all():
            arti.set_joint_velocity_targets(np.zeros((1, 4), dtype=np.float32), joint_indices=wi)
            state['rotation_preposition'] = None
            state['rotation_applied_speed'] = 0.
            raise RuntimeError('底盘8关节反馈异常，已撤除轮速目标')
        if cmd.get('parking'):
            self._update_swerve_parking(model,state,q,velocity)
            return
        stopping = all(abs(cmd.get(axis, 0.)) < 1e-12 for axis in ('vx', 'vy', 'wz'))
        if stopping:
            if not state['brake_sent']:
                arti.set_joint_velocity_targets(np.zeros((1, 4), dtype=np.float32), joint_indices=wi)
                # 停车一次性保持实测舵角，避免继续追旧目标的满力矩反扭矩；不是打舵回零。
                arti.set_joint_position_targets(q[si].reshape(1, -1).astype(np.float32), joint_indices=si)
                state['brake_sent'] = True
                # 中途停止后再次旋转也须重新验证停轮，不能沿用上次放行状态。
                state['rotation_preposition'] = None
                state['rotation_applied_speed'] = 0.
            # 不猛打舵回零；用位置窗口确认真实停稳，不能把目标0当作实际静止。
            # 原生高kd下瞬时轮速有0.09～0.207rad/s微振，实际1s根漂移仅约0.1mm。
            # 阈值是经现场数据核对的仿真判据，非真机规格或按压力标定。
            now = float(self.world.current_time)
            if getattr(self, '_swerve_stop_window', None) is None:
                from swerve_base import BaseSettlingWindow
                self._swerve_stop_window = BaseSettlingWindow()
                state['brake_since'] = now
            pose = self._swerve_base_pose()
            verification = self._swerve_stop_window.update(now, pose, q[si], q[wi])
            state['settling_verification'] = verification
            self._swerve_last_control = {'steering_actual_rad': q[si].tolist(),
                'wheel_velocity_targets_rad_s': [0.]*4, 'wheel_velocity_actual_rad_s': velocity[wi].tolist(),
                'stopping': True, 'settling_verification': verification}
            if verification['settled']:
                if state['gain_override']:
                    arti.set_joint_position_targets(q[wi].reshape(1, -1).astype(np.float32), joint_indices=wi)
                    k = np.asarray(arti._physics_view.get_dof_stiffnesses()).reshape(-1).copy()
                    k[wi] = state['saved_stiffness']
                    arti._physics_view.set_dof_stiffnesses(k.reshape(1, -1), [0])
                self._swerve_last_stop = {'base_pose': pose.tolist(), 'wheel_positions': q[wi].tolist(),
                    'steering_positions': q[si].tolist(), 'sim_time': now, 'verification': verification}
                self._base_cmd = None
                self._wheel_pos_target = None
                self._swerve_drive_state = None
                self._swerve_stop_window = None
            return
        state['brake_sent'], state['brake_since'] = False, None
        self._swerve_stop_window = None
        axis = model.cardinal_axis(cmd['vx'],cmd.get('vy',0.),cmd['wz'])
        if axis is not None:
            state['rotation_preposition'] = None
            state['rotation_applied_speed'] = 0.
            from swerve_base import SteeringPrealignment
            now = float(self.world.current_time)
            transition = state.get('cardinal_preposition')
            selected_target = None
            if cmd.get('steering_targets') is not None:
                selected_target = model.solve_cardinal(cmd['vx'],cmd.get('vy',0.),q[si],cmd['steering_targets'])['steering']
            target_changed = (transition is not None and selected_target is not None
                              and not np.array_equal(selected_target,transition.target))
            if state.get('cardinal_axis') != axis or transition is None or target_changed:
                solution = model.solve_cardinal(cmd['vx'],cmd.get('vy',0.),q[si],selected_target)
                # 四舵同步限速；工位当前参考1.2rad/s，未配置时仍用0.6，另受URDF限速。
                reference_rate = float((self._base_cfg or {}).get('steering_preposition_rate_rad_s',.6))
                model_rate = min(m.steering_max_velocity_rad_s for m in model.modules)
                transition = SteeringPrealignment(q[si],solution['steering'],now,
                                                  max_rate_rad_s=min(reference_rate,model_rate))
                state['cardinal_axis'],state['cardinal_preposition'] = axis,transition
                state['prealign_base_ypr_start'] = self._swerve_base_ypr().copy()
                state['prealign_base_pose_start'] = self._swerve_base_pose().copy()
                state['cardinal_applied_speed'] = 0.
                state['cardinal_last_update_time'] = now
            solution = model.solve_cardinal(cmd['vx'],cmd.get('vy',0.),q[si],transition.target)
            pose = self._swerve_base_pose()
            ypr = self._swerve_base_ypr()
            command,verification = transition.update(now,q[si],q[wi],pose)
            dt = max(0.,min(now-state.get('cardinal_last_update_time',now),.10))
            state['cardinal_last_update_time'] = now
            axis_index = 0 if axis=='x' else 1
            requested_speed = float(solution['executed_twist'][axis_index])
            acceleration = float((self._base_cfg or {}).get('cardinal_acceleration_m_s2',.15))
            if not np.isfinite(acceleration) or acceleration <= 0.:
                raise ValueError('单轴平移参考加速度必须为有限正数')
            if verification['ready']:
                previous = state.get('cardinal_applied_speed',0.)
                applied_speed = float(previous+np.clip(requested_speed-previous,-acceleration*dt,acceleration*dt))
            else:
                applied_speed = 0.
            # 高层在对舵期间可能已请求巡航速度；真正放行后仍必须从0斜坡，避免骤起。
            state['cardinal_applied_speed'] = applied_speed
            speeds = solution['wheel_velocity']*(applied_speed/requested_speed) if abs(requested_speed)>1e-12 else np.zeros(4)
            executed_twist = np.zeros(3)
            executed_twist[axis_index] = applied_speed
            arti.set_joint_position_targets(command.reshape(1,-1).astype(np.float32),joint_indices=si)
            arti.set_joint_velocity_targets(speeds.reshape(1,-1).astype(np.float32),joint_indices=wi)
            delta_yaw = float((ypr[0]-state['prealign_base_ypr_start'][0]+np.pi)%(2*np.pi)-np.pi)
            self._swerve_last_control = {'motion_kind':'cardinal_translation','cardinal_axis':axis,
                'phase':verification['phase'],'steering_targets_rad':solution['steering'].tolist(),
                'steering_targets_locked':cmd.get('steering_targets') is not None,
                'steering_command_rad':command.tolist(),'steering_actual_rad':q[si].tolist(),
                'rolling_plane_heading_rad_mod_pi':solution['rolling_plane_heading_rad_mod_pi'].tolist(),
                'actual_rolling_plane_heading_rad_mod_pi':[
                    float((m.heading_zero+m.steering_sign*value+np.pi/2)%np.pi-np.pi/2)
                    for m,value in zip(model.modules,q[si])],
                'commanded_wheel_ground_velocity_base_m_s':[
                    (speed*m.radius_m*m.wheel_sign*np.array([
                        np.cos(m.heading_zero+m.steering_sign*value),
                        np.sin(m.heading_zero+m.steering_sign*value)])).tolist()
                    for m,value,speed in zip(model.modules,solution['steering'],speeds)],
                'steering_equivalence_note':'±90关节角配合反向轮速表示同一滚动平面；平移方向以轮胎地面速度向量为准',
                'wheel_velocity_targets_rad_s':speeds.tolist(),'wheel_velocity_actual_rad_s':velocity[wi].tolist(),
                'alignment_scale':1. if verification['ready'] else 0.,'prealignment':verification,
                'requested_twist':[cmd['vx'],cmd.get('vy',0.),cmd['wz']],
                'executed_twist':executed_twist.tolist(),
                'reference_acceleration_m_s2':acceleration,
                'prealign_base_ypr_start_rad':state['prealign_base_ypr_start'].tolist(),
                'base_ypr_rad':ypr.tolist(),'base_yaw_drift_rad':delta_yaw,
                'prealign_base_pose_start':state['prealign_base_pose_start'].tolist()}
            return
        state['cardinal_axis'],state['cardinal_preposition'] = None,None
        pure_rotation = abs(cmd['vx']) <= 1e-12 and abs(cmd.get('vy',0.)) <= 1e-12
        if pure_rotation:
            from swerve_base import SteeringPrealignment
            now = float(self.world.current_time)
            solution = model.solve(0.,0.,cmd['wz'],q[si])
            transition = state.get('rotation_preposition')
            if transition is None:
                reference_rate = float((self._base_cfg or {}).get('steering_preposition_rate_rad_s',.6))
                model_rate = min(m.steering_max_velocity_rad_s for m in model.modules)
                transition = SteeringPrealignment(q[si],solution['steering'],now,
                                                  max_rate_rad_s=min(reference_rate,model_rate))
                state['rotation_preposition'] = transition
                state['rotation_applied_speed'] = 0.
                state['rotation_last_update_time'] = now
                state['rotation_ypr_start'] = self._swerve_base_ypr().copy()
                state['rotation_pose_start'] = self._swerve_base_pose().copy()
            # 极限之外的等价舵角在别的转向范围内可能换支；新目标仍须重新停轮对舵。
            if np.max(abs(solution['steering']-transition.target)) > 1e-6:
                reference_rate = float((self._base_cfg or {}).get('steering_preposition_rate_rad_s',.6))
                model_rate = min(m.steering_max_velocity_rad_s for m in model.modules)
                transition = SteeringPrealignment(q[si],solution['steering'],now,
                                                  max_rate_rad_s=min(reference_rate,model_rate))
                state['rotation_preposition'] = transition
                state['rotation_applied_speed'] = 0.
                state['rotation_last_update_time'] = now
            pose = self._swerve_base_pose()
            ypr = self._swerve_base_ypr()
            command,verification = transition.update(now,q[si],q[wi],pose)
            dt = max(0.,min(now-state.get('rotation_last_update_time',now),.10))
            state['rotation_last_update_time'] = now
            requested_speed = float(solution['executed_twist'][2])
            acceleration = float((self._base_cfg or {}).get('max_yaw_acceleration_rad_s2',.35))
            if not np.isfinite(acceleration) or acceleration <= 0.:
                arti.set_joint_velocity_targets(np.zeros((1,4),dtype=np.float32),joint_indices=wi)
                state['rotation_preposition'] = None
                state['rotation_applied_speed'] = 0.
                raise ValueError('原地旋转参考角加速度必须为有限正数')
            if verification['ready']:
                previous = state.get('rotation_applied_speed',0.)
                applied_speed = float(previous+np.clip(requested_speed-previous,-acceleration*dt,acceleration*dt))
            else:
                applied_speed = 0.
            # 只有真实停轮、切线轮向同步轨迹结束且反馈稳定后，才从0开始旋转。
            state['rotation_applied_speed'] = applied_speed
            speeds = solution['wheel_velocity']*(applied_speed/requested_speed) if abs(requested_speed)>1e-12 else np.zeros(4)
            arti.set_joint_position_targets(command.reshape(1,-1).astype(np.float32),joint_indices=si)
            arti.set_joint_velocity_targets(speeds.reshape(1,-1).astype(np.float32),joint_indices=wi)
            phase = 'rotating' if verification['ready'] else 'prealign'
            verification = dict(verification,phase=phase)
            self._swerve_last_control = {'motion_kind':'in_place_rotation','phase':phase,
                'steering_targets_rad':transition.target.tolist(),
                'steering_command_rad':command.tolist(),'steering_actual_rad':q[si].tolist(),
                'wheel_velocity_targets_rad_s':speeds.tolist(),'wheel_velocity_actual_rad_s':velocity[wi].tolist(),
                'alignment_scale':1. if verification['ready'] else 0.,'prealignment':verification,
                'requested_twist':[cmd['vx'],cmd.get('vy',0.),cmd['wz']],
                'executed_twist':[0.,0.,applied_speed],
                'reference_yaw_acceleration_rad_s2':acceleration,
                'prealign_base_ypr_start_rad':state['rotation_ypr_start'].tolist(),
                'prealign_base_pose_start':state['rotation_pose_start'].tolist(),
                'base_ypr_rad':ypr.tolist(),
                'base_yaw_change_rad':float((ypr[0]-state['rotation_ypr_start'][0]+np.pi)%(2*np.pi)-np.pi)}
            return
        state['rotation_preposition'] = None
        state['rotation_applied_speed'] = 0.
        solution = model.solve(cmd['vx'], cmd.get('vy', 0.), cmd['wz'], q[si])
        angle_error = float(np.max(abs(solution['steering']-q[si])))
        # 大角度转向先停轮对舵；小偏差按投影降速，避免尚未对准时横擦地面。
        alignment = max(0., float(np.cos(angle_error))) if angle_error < .12 else 0.
        speeds = solution['wheel_velocity']*alignment
        arti.set_joint_position_targets(solution['steering'].reshape(1, -1).astype(np.float32), joint_indices=si)
        arti.set_joint_velocity_targets(speeds.reshape(1, -1).astype(np.float32), joint_indices=wi)
        self._swerve_last_control = {'steering_targets_rad': solution['steering'].tolist(),
            'steering_actual_rad': q[si].tolist(), 'wheel_velocity_targets_rad_s': speeds.tolist(),
            'wheel_velocity_actual_rad_s': velocity[wi].tolist(), 'alignment_scale': alignment,
            'executed_twist': solution['executed_twist'].tolist()}

    def _update_base_velocity(self):
        """底盘速度控制（主循环每帧）：四轮八驱走转向位置+原生轮速目标。

        旧差速模型将(vx,wz)折算四轮转速并积分位置目标，以下说明仅限旧模型。

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
            # 失联刹停：速度归零
            cmd["vx"] = 0.0
            cmd["wz"] = 0.0
        if time.time() - cmd["t"] > float(self._base_cfg.get("timeout", 0.5)):
            cmd["vy"] = 0.0
        model = self._get_swerve_model()
        if model is not None:
            self._update_swerve_velocity(model, cmd)
            return
        vx, wz = cmd["vx"], cmd["wz"]

        # **速度已经是 0 就彻底撒手**，别再每帧写轮子位置目标。
        #
        # 为什么：`set_joint_position_targets` 会**一直把整个 articulation
        # 唤醒**，而唤醒状态下这台机器人有 0.3~4 mrad 的宽带微振、link 摆幅
        # 1~4mm（实测 2026-10-14）。以前 `set_base_velocity` 一旦被调用过，
        # `_base_cmd` 就永久非空 → 每帧都在写 → 机器人**永远睡不着** →
        # 用户看到的"静止待机也在抖"。
        #
        # 实测对照（同一场景，只改这一条）：
        #   调过一次 set_base_velocity(0,0) 后：head_pitch 抖动 4.006 mrad、
        #                                      轮子 0.38 mrad、|v| 峰 2.0 rad/s
        #   清掉 _base_cmd 后：                  全部 0.000，完全静止
        # 与轮子刚度无关（刚度 0 / 203 都是 ~3.4~4.1 mrad）。
        if vx == 0.0 and wz == 0.0:
            # 只"停止写目标"，**不要**把轮子刚度也松开 —— 松开后轮子会自由
            # 空转（实测位置极差 12.5 rad ≈ 两圈），机器人变成会滚的，更糟。
            self._wheel_pos_target = None
            self._base_cmd = None
            return
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

    def get_joint_states(self, full: bool = False):
        """查询关节指令/反馈。无 ROS2 桥时 feedback = articulation 实际位置。

        默认与 `get_joint_names` **同口径**（隐藏夹爪跟随关节），两者的
        `dof_names[i]` ↔ `feedback[i]` 一一对应；`full=true` 给全量。
        """
        if self._ros_controller is None or not self._ros_controller.is_initialized():
            # 同 send_joint_command：未初始化的 controller 要当作没有
            self._require_robot()
            fb = self._joint_positions_flat()
            cmd = list(getattr(self, "_last_direct_cmd", []))
            ros = False
        else:
            fb = list(self._ros_controller.get_feedback_positions())
            cmd = list(self._ros_controller.get_cmd_positions())
            ros = True
        hide = self._gripper_followers
        n_all = len(self.articulation.dof_names) if self.articulation else 0

        def _filt(v):
            # 只在长度对得上整机 DOF 时过滤（ROS 桥可能只订了子集）
            if full or len(v) != n_all:
                return v
            return [x for i, x in enumerate(v) if i not in hide]

        return {
            "cmd": _filt(cmd),
            "feedback": _filt(fb),
            "feedback_received": (True if not ros
                                  else self._ros_controller.is_feedback_received()),
            "ros": ros,
            "hidden": (0 if full or not n_all else len(hide)),
        }

    # ==================== 内部 ====================

    def _require_robot(self):
        if self.articulation is None:
            raise RuntimeError("机器人未加载，请先调用 load_urdf")
        from scripts.physics_view_guard import guard
        if not guard(self, logger):
            raise RuntimeError(self._physics_view_fault)

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
            if all(name in names for name in _BASE_DRIVE_JOINTS):
                protected = _BASE_DRIVE_JOINTS
            elif self._base_cmd is not None:
                protected = _BASE_WHEEL_JOINTS
            else:
                protected = ()
            keep = [i for i, n in enumerate(names) if n not in protected]
            if not keep:
                return
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

    # ==================== XRoboToolkit 遥操作端（UDP） ====================
    #
    # md 那套链路的仿真侧：遥操作端在 `xr-robotics` env 里跑 placo IK，每帧把
    # 50 个关节的目标位置用 UDP 发过来；这里只负责收下并施加。
    #
    # 为什么不用进程内的 isaac_teleop.py：那是 CloudXR + isaacteleop 的另一套
    # VR 端软件，与本链路互斥。两条路都写 articulation 会互相覆盖，所以
    # start_udp_teleop 会把进程内 VR 挂起（`set_suspended`），stop 时恢复。

    def start_udp_teleop(self, port: int = None, host: str = None,
                         status_port: int = None, status_hz: float = None,
                         suspend_vr: bool = True):
        """开 UDP 遥操作入口。参数缺省取 config.json 的 `udp_teleop` 段。

        :param port: 接收关节目标的端口（遥操作端 `--isaac-udp-port`）
        :param status_port: 状态回发端口（0=不回发）；契约桥监听它
        :param suspend_vr: 是否挂起进程内 VR 遥操作（默认是，避免两路抢控制权）
        """
        cfg = self._udp_cfg
        port = int(port if port is not None else cfg.get("port", 63900))
        host = str(host if host is not None else cfg.get("host", "127.0.0.1"))
        status_port = int(status_port if status_port is not None
                          else cfg.get("status_port", 0))
        status_hz = float(status_hz if status_hz is not None
                          else cfg.get("status_hz", 10.0))

        if self._udp_link is not None and self._udp_link.is_initialized():
            if self._udp_link.port == port:
                return {"status": "already_running", **self._udp_link.status()}
            self.stop_udp_teleop()          # 换端口 = 先关再开

        try:
            link = udp_teleop.UdpTargetLink(
                port=port, host=host, status_port=status_port,
                status_hz=status_hz).start()
        except OSError as e:
            logger.error(f"UDP 遥操作入口绑定失败: {e}")
            return {"status": "error", "reason": f"bind {host}:{port} 失败: {e}"}

        self._udp_link = link
        self._udp_last_names = None
        if suspend_vr and self._vr_teleop is not None:
            self._vr_teleop.set_suspended(True)
        logger.info(f"UDP 遥操作入口启动: udp://{host}:{port}"
                    f"（进程内 VR 遥操作{'已挂起' if suspend_vr else '保持' }）")
        return {"status": "started", "suspend_vr": bool(suspend_vr),
                **link.status()}

    def stop_udp_teleop(self, resume_vr: bool = True):
        """关 UDP 遥操作入口，恢复进程内 VR 遥操作。"""
        if self._udp_link is None:
            return {"status": "not_running"}
        self._udp_link.stop()
        self._udp_link = None
        self._udp_last_names = None
        if resume_vr and self._vr_teleop is not None:
            self._vr_teleop.set_suspended(False)
        return {"status": "stopped", "resume_vr": bool(resume_vr)}

    def udp_teleop_status(self):
        """读 UDP 遥操作入口状态（含帧龄，用来判断遥操作端是不是挂了）。"""
        if self._udp_link is None:
            return {"running": False, "config": dict(self._udp_cfg)}
        return self._udp_link.status()

    def _apply_udp_targets(self):
        """主线程：把最新一帧 UDP 目标施加到 articulation。

        语义与 `run_isaac_udp_target.py` 一致 —— 收到就一直保持，丢包时机器人
        停在原地（位置驱动目标本来就保持），不会弹回零位。遥操作端挂没挂由
        `udp_teleop_status().frame_age_s` 判断，不在这里自作主张。

        底盘速度控制激活时轮子分量被忽略（与 `_apply_joint_positions` 同一条
        规则）：md 那边把非臂关节都钉在 0 并一并广播，不排除轮子的话
        `set_base_velocity` 会被每帧覆盖。
        """
        link = self._udp_link
        if link is None or self.articulation is None:
            return
        # 自动抓取进行中：丢掉手柄目标，否则用户一握握把就把手臂从规划轨迹上
        # 拽走。（进程内 VR 那条路是靠 `set_suspended` 挡的，这里同理。）
        if self._auto_pick is not None:
            return
        latest = link.get_latest()
        if latest is None or not self.world.is_playing():
            return
        names, positions, _t = latest
        try:
            dofs = list(self.articulation.dof_names)
            keep, idxs, vals = [], [], []
            for n, v in zip(names, positions):
                if n not in dofs:
                    continue                      # 遥操作端多发的关节，忽略
                protected = (_BASE_DRIVE_JOINTS if all(name in dofs for name in _BASE_DRIVE_JOINTS)
                             else (_BASE_WHEEL_JOINTS if self._base_cmd is not None else ()))
                if n in protected:
                    continue                      # 底盘8关节归set_base_velocity专管
                keep.append(n)
                idxs.append(self.articulation.get_dof_index(n))
                vals.append(float(v))
            if not idxs:
                return
            # 5.x view API 要求 (M, K)，单机器人 M=1
            self.articulation.set_joint_position_targets(
                np.asarray(vals, dtype=np.float32).reshape(1, -1),
                joint_indices=idxs)
            # 软件 mimic 不用在这里显式跑：step() 里的 `_apply_software_mimic()`
            # 紧接着会以"指令目标"(= 刚写下去的这一帧)重算跟随关节。md 那边已经
            # 把 12 个夹爪关节全发过来了，主动关节值相同 → 算出的跟随值也相同，
            # 写的是同一批目标，不会打架。
            if self._udp_last_names is None or keep != self._udp_last_names:
                logger.info(f"UDP 遥操作接管 {len(keep)} 个关节"
                            f"（首帧: {keep[:6]}{' …' if len(keep) > 6 else ''}）")
                self._udp_last_names = list(keep)
        except Exception as e:
            logger.error(f"施加 UDP 关节目标失败: {e}")

    def _process_udp_events(self):
        """主线程：处理遥操作端发来的按键事件。

        与进程内 VR 那条路的左手 X 键语义完全一致（见 isaac_teleop.py 的
        `consume_menu_request`）：边沿触发的**一次性**请求，在这里启动自动抓取。

        `auto_pick` 必须在仿真主线程调用 —— 它的插值靠 `step()` 推进，从 HTTP
        协程里同步等会死锁（见 auto-pick-menu-key 的说明）。
        """
        link = self._udp_link
        if link is None:
            return
        for payload, _t in link.take_events():
            kind = payload.get("event")
            if kind != "auto_pick":
                logger.warning(f"未知 UDP 事件，忽略: {kind!r}")
                continue
            side = str(payload.get("side") or "left")
            try:
                res = self.auto_pick(side=side)
                logger.info(f"UDP 事件触发自动抓取(side={side}): {res}")
            except Exception as e:
                logger.warning(f"UDP 事件处理异常: {e!r}")

    def _publish_udp_state(self):
        """主线程：把仿真实测状态回发给遥操作端（限频，失败静默）。"""
        link = self._udp_link
        if link is None or self.articulation is None:
            return
        if not self.world.is_playing():
            return
        try:
            dofs = list(self.articulation.dof_names)
            q = self._joint_positions_flat()
            v = [float(x) for x in np.asarray(
                self.articulation.get_joint_velocities()).reshape(-1)]
            base = None
            try:
                p, quat = self.articulation.get_world_pose()
                base = [float(x) for x in (*np.asarray(p).reshape(-1),
                                           *np.asarray(quat).reshape(-1))]
            except Exception:
                pass
            link.record_state(dofs, q, v, base)
        except Exception:
            pass
