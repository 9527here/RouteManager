"""isaac_teleop.py — Pico 4 Ultra VR 遥操作（Isaac Teleop 集成）。

架构（VR → 仿真，进程内）：
    Pico 4 Ultra (WebXR client) --CloudXR--> CloudXR runtime（本机服务）
        → TeleopSession（isaacteleop 1.5.144rc1）
        → ControllersSource 原始输出（grip 位姿 + 扳机/侧握）
        → 工作区缩放 → 手臂关节速率控制 + 夹爪开合
        → 直接施加到 articulation（默认 50 Hz 循环线程）

API 依据（已核对安装的 wheel 源码）：
    isaacteleop.teleop_session_manager: TeleopSession/TeleopSessionConfig
    isaacteleop.retargeting_engine.deviceio_source_nodes.ControllersSource
      输出 controller_left/right = OptionalTensorGroup，键 ControllerInputIndex：
      GRIP_POSITION(3)/GRIP_ORIENTATION(4,xyzw)/GRIP_IS_VALID/
      TRIGGER_VALUE(0~1)/SQUEEZE_VALUE(0~1) 等；控制器未跟踪时 is_none()
    isaacteleop.cloudxr.CloudXRLauncher: 拉起/附着 CloudXR runtime + WSS 代理

说明：
  - v1 使用"控制器位移 → 关节速率"映射（不依赖 IK retargeter，适配任意
    自制机械臂）；retargeters 管线（SE3 绝对位姿 IK）留作 v2。
  - 首次 start 会下载 CloudXR runtime 到 ~/.cloudxr（需外网，走代理）。
"""
import argparse
import json
import logging
import os
import threading
import time

import numpy as np

import gripper_model

logger = logging.getLogger(__name__)


class VRTeleopManager:
    """进程内 VR 遥操作会话管理器。

    由 RobotManager 持有；/execute 通过 start_vr_teleop/stop_vr_teleop/
    vr_teleop_status 三个动作调用。
    """

    #: 侧握接合后这段时间内的手柄位移**不计入**目标(持续重锚)。
    #: 手指捏握把会把手柄带偏几厘米, 吸收掉才不会"一按握把臂就飞"。
    CLUTCH_SETTLE_S = 0.30

    #: 腕部跟随一阶低通系数(每帧向目标朝向靠拢的比例)。太小跟不上翻腕,
    #: 太大则手柄姿态噪声被 IK 放大成逐帧乱扭。
    WRIST_ALPHA = 0.15

    def __init__(self, robot):
        self._robot = robot          # RobotManager
        self._session = None
        self._launcher = None
        self._thread = None
        self._stop_evt = threading.Event()
        self._status = {"running": False, "frames": 0, "last_error": None,
                        "last_controller": None}
        self._hands = {}             # side -> {"prev": np.array 或 None}
        self._pending = {"grip": {}, "arm": {}, "base": {}, "head": {}, "torso": {}}  # VR 线程→主线程 目标缓冲
        self._params = {}
        self._grip = {}
        self._arms = {}
        self._lock = threading.Lock()
        self._viz = None            # isaacteleop.viz VizSession(kXr)
        self._head_layer = None     # 头部相机画面 QuadLayer
        self._head_cam = None       # 流送源相机对象
        self._torch = None
        self._frame_i = 0
        self._arm_target = {}       # side -> 自维护的手臂关节目标(不依赖读 view)
        self._head_joints = {}
        self._head_ref = None
        self._apply_err_logged = False
        # --- 手臂运动学(FK + 数值雅可比; 取代原来的预计算雅可比表) ---
        self._kin = {}              # side -> URDF 关节链
        self._kin_lim = {}          # side -> (n,2) 关节限位(rad)
        self._arm_base_world = {}   # side -> (R_ab, t_ab) 手臂基座世界位姿
        self._base_idx = {"left": None, "right": None}   # link 索引缓存
        self._base_upd_n = 0
        self._lim_from_sim = False  # 关节限位是否已从仿真读过
        self._posture_ref = {}      # side -> 零空间姿态参考(接合瞬间的位形)

    # ---------- 配置 ----------

    def _load_teleop_cfg(self):
        try:
            with open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "config.json")) as f:
                return json.load(f).get("teleop") or {}
        except Exception:
            return {}

    # ---------- 启停 ----------

    def start(self, rate_hz: float = 50.0, workspace_scale: float = 0.7,
              max_step: float = 0.08, deadzone: float = 0.01,
              grip_open: float = 0.7):
        """启动 VR 遥操作。

        :param rate_hz: 控制循环频率
        :param workspace_scale: VR 位移 → 关节增量缩放
        :param max_step: 每帧关节目标最大变化（rad），平滑限幅
        :param deadzone: 位置死区（m），抑制手抖
        :param grip_open: 夹爪 leader 全开角度（rad），扳机 1.0 → 此角度
        """
        if self._session is not None:
            raise RuntimeError("VR 遥操作已在运行，先 stop_vr_teleop")

        robot = self._robot
        robot._require_robot()
        names = list(robot.articulation.dof_names)

        tele_cfg = self._load_teleop_cfg()
        self._grip = {
            "left": tele_cfg.get("gripper_leader", {}).get("left",
                    "left_gripper_right_2_joint"),
            "right": tele_cfg.get("gripper_leader", {}).get("right",
                     "right_gripper_right_2_joint"),
        }
        for k, v in self._grip.items():
            if v not in names:
                raise ValueError(f"夹爪 leader {v} 不在关节列表中")

        self._arms = {}
        for side in ("left", "right"):
            jn = tele_cfg.get("arm_joints", {}).get(side)
            if not jn:
                # 7 个: arm_1..6 + 转腕关节。**必须与 FK 链一致**
                # (URDF: arm_base → arm_1..6 → wrist → gripper_base)。
                # 原来只取 arm_1..6, 导致 IK 的 FK 链(7)与下发列表(6)不一致 ——
                # 而且被排除的转腕正是之前卡死不动的那一个。
                jn = [n for n in names
                      if n.startswith(side) and "gripper" not in n
                      and (any(k in n for k in
                               ("arm_1", "arm_2", "arm_3", "arm_4",
                                "arm_5", "arm_6"))
                           or n == f"{side}_wrist_joint")]
            missing = [n for n in jn if n not in names]
            if missing:
                raise ValueError(f"{side} 手臂关节不存在: {missing}")
            self._arms[side] = jn
        if not any(self._arms.values()):
            raise ValueError("未识别出手臂关节，请在 config.json "
                             "teleop.arm_joints 手工指定")

        self._params = dict(rate_hz=rate_hz, workspace_scale=workspace_scale,
                            max_step=max_step, deadzone=deadzone,
                            grip_open=grip_open)
        self._vr_frame_interval = max(
            1, int(tele_cfg.get("vr_frame_interval", 2)))
        self._head_signs = {
            "yaw": float(tele_cfg.get("head_yaw_sign", 1.0)),
            "pitch": float(tele_cfg.get("head_pitch_sign", 1.0)),
        }

        # ---- 构建 isaacteleop 会话 ----
        from isaacteleop.cloudxr import CloudXRLauncher
        from isaacteleop.retargeting_engine.deviceio_source_nodes import (
            ControllersSource, HeadSource)
        from isaacteleop.retargeting_engine.interface.output_combiner import (
            OutputCombiner)
        from isaacteleop.teleop_session_manager import (
            TeleopSession, TeleopSessionConfig)

        src = ControllersSource("controllers")
        head_src = HeadSource("head")
        # 直通管线：不做 IK，原始控制器/头部张量直接进 step() 结果
        pipeline = OutputCombiner({
            "controller_left": src.output(src.LEFT),
            "controller_right": src.output(src.RIGHT),
            "head": head_src.output("head"),
        })
        # 头部跟踪同步: 头显姿态 → 机器人 head yaw/pitch 关节(现场匹配 dof 名)
        self._head_joints = {}
        try:
            _dn = list(self._robot.articulation.dof_names)
            for nm in _dn:
                low = nm.lower()
                if "head" in low and "yaw" in low:
                    self._head_joints["yaw"] = nm
                elif "head" in low and "pitch" in low:
                    self._head_joints["pitch"] = nm
            logger.info(f"头部跟踪关节: {self._head_joints}")
        except Exception as e:
            logger.warning(f"头部关节匹配失败: {e}")
        self._head_ref = None      # 校准基准 (yaw0, pitch0)
        self._head_yaw_track = None   # 头显在跟踪系的偏航(每帧更新)
        self._base_yaw_world = None   # 机器人底座世界偏航(主线程缓存)
        # 手柄→世界旋转的"推杆实测标定"(action: vr_axis_calib):
        # 实测值优先于头显推算(phi 不依赖头显/手柄系一致性假设)
        self._phi_fixed = {}      # {side: phi(rad)} 实测标定值
        # 三推杆标定解出的**完整旋转矩阵**(跟踪系→机器人世界系)。
        # 设了它就完全取代 Q=Rz(phi)·P —— 不再假设客户端轴约定。
        self._calib_R = None
        self._calib3 = None       # 三推杆标定进行中
        self._calib = None        # 标定进行中 {p0, yaw_r0, t0}
        self._last_ctrl_pos = {}  # {side: np3} 最新手柄位置(标定采样用)
        # 标定值持久化: 跟踪零点由头显 App 会话决定, 服务重启不影响它,
        # 所以存盘复用可靠(可疑时重标一次即可, vr_axis_calib start/done)
        self._calib_file = None
        try:
            import os as _osc
            self._calib_file = _osc.path.join(
                _osc.getcwd(), "scripts", "scene", "axis_calib.json")
            with open(self._calib_file) as _f:
                _cj = json.load(_f)
            _pf = _cj.get("phi") or {}
            if "left" in _pf and "right" in _pf:
                self._phi_fixed = {k: float(v) for k, v in _pf.items()}
                logger.info(f"已加载上次标定 phi={self._phi_fixed} "
                            f"(ts={_cj.get('ts')})")
            _R = _cj.get("R")
            if _R is not None:
                _Ra = np.asarray(_R, float)
                if _Ra.shape == (3, 3):
                    self._calib_R = _Ra
                    logger.info(f"已加载三推杆旋转矩阵 R (ts={_cj.get('ts')})")
        except Exception:
            pass

        # ---- IK 数据: Jacobian 表(scripts/scene/build_jac_table.py 生成) ----
        import os as _os
        self._jac_table = None
        self._ik_state = {}
        self._ik_smooth = {}    # IK 目标速率限制平滑状态 {side: pos3}
        # 手柄位移 → 末端位移的比例。**1:1 才跟手**(智元 geniesim 的手臂
        # 也是 1:1, 只有腰部是 0.5); 原来 0.5 导致"手柄动了很大幅度, 仿真里
        # 只走一半"。右摇杆按下切到 0.4(精细) / 1.0(正常)。
        self._ik_scale = 1.0
        self._R_base_inv = None
        self._base_pos = None
        self._arm_limits = {}
        self._q_all = None      # 全 50 维关节目标(全量写入, 避免读 backend)
        self._wheel_target = None
        self._torso_target = None   # [lift, torso_pitch] 自维护目标(右摇杆)
        self._btn_prev = 0.0        # 复位键(A/X)边沿检测
        self._x_prev = 0.0          # 左手 X 键边沿检测(自动抓取)
        self._menu_req = None       # 菜单键请求时间戳(由 step() 取走)
        self._suspended = False     # 自动抓取期间挂起 VR 手臂/夹爪写入
        self._ctrl_dbg = {}
        _tab = _os.path.join(_os.getcwd(), "scripts", "scene", "jac_table.json")
        try:
            with open(_tab) as f:
                self._jac_table = json.load(f)
            bp = self._jac_table.get("base_quat")
            if bp:
                w, x, y, z = bp
                R = np.array([
                    [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),
                     2 * (x * z + w * y)],
                    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z),
                     2 * (y * z - w * x)],
                    [2 * (x * z - w * y), 2 * (y * z + w * x),
                     1 - 2 * (x * x + y * y)]])
                self._R_base_inv = R.T
                self._base_pos = np.asarray(
                    self._jac_table.get("base_pos", [0, 0, 0]), float)
            logger.info(f"IK 表已加载: {self._jac_table.get('n_samples')} 采样")
            # 全量写仅作为"物理视图失效"时的兜底: 视图健康时用带索引写
            # (全量写会把轮子/caster 也锁到 home 值 → 底盘被伺服拉扯 → 上下抖)
            if os.environ.get("VR_FULL_WRITE", "0") == "1":
                hq = self._jac_table.get("home_q")
                if hq:
                    self._q_all = np.asarray(hq, dtype=np.float32)
        except Exception as e:
            logger.warning(f"IK 表未加载(将退回速率映射): {e}")
            self._jac_table = None
        # 关节限位走 USD(不依赖物理视图)
        try:
            import omni.usd as _ou
            _stage = _ou.get_context().get_stage()
            for side in ("left", "right"):
                for nm in self._arms.get(side, []):
                    p = _stage.GetPrimAtPath(f"/humanoid/joints/{nm}")
                    lo = p.GetAttribute("physics:lowerLimit").Get()
                    up = p.GetAttribute("physics:upperLimit").Get()
                    if lo is not None and up is not None:
                        self._arm_limits[nm] = (float(np.deg2rad(lo)),
                                                float(np.deg2rad(up)))
        except Exception as e:
            logger.warning(f"关节限位读取失败: {e}")

        args = argparse.Namespace(
            cloudxr_install_dir=os.path.expanduser("~/.cloudxr"),
            launch_cloudxr_runtime=True,
            cloudxr_device_profile="Quest3",   # PICO 系用 Quest3 档案
            cloudxr_env_config=None,
            accept_eula=True,
            launch_wss_proxy=None,
        )
        self._launcher = CloudXRLauncher.launch_context(args, accept_eula=True)
        self._launcher.__enter__()   # 拉起/附着 CloudXR runtime + WSS 代理

        # ---- 视频图层:VizSession(kXr)持有 OpenXR 会话,头显里显示机器人相机。
        # CloudXR 流送的是 XR 应用提交的图层;不提交图层头显就是黑屏。
        # VizSession 失败只损失画面,手柄控制不受影响。 ----
        # 先选流送源相机(图层分辨率要跟它一致, 否则 submit 尺寸不匹配=黑屏)
        self._head_cam = None
        try:
            for pat in ("head_zed_x", "tv_cam", "teleop_view"):
                for k, v in self._robot._cameras.items():
                    if pat in k:
                        self._head_cam = v
                        break
                if self._head_cam is not None:
                    break
        except Exception:
            pass
        _cam_w, _cam_h = 1280, 800
        try:
            _res = self._head_cam.get_resolution()
            _cam_w, _cam_h = int(_res[0]), int(_res[1])
        except Exception:
            pass

        # ---- 相机跟随: 用**文档安装链常数偏移**(与机器人位姿无关) ----
        # 旧实现从 USDA 读"烘焙时旧场景的世界位姿"算 offset(旧场景机器人 yaw≈8.9°,
        # 新场景姿态不同 → offset 错 → 相机被推到错误位置). 改为:
        #   offset = T(xyz) @ R(rpy) @ R_opt @ R_cam   (相对父连杆, 常数)
        #   M_cam = M_link(父连杆) @ offset
        # 每帧写 /World/cams/<nm> 的 xformOp(已验证不会让物理后端失效).
        self._cam_follow = []
        self._cam_offset = None
        self._head_link_idx = None
        self._cam_prim_path = None
        self._stage = None
        try:
            import omni.usd as _ou2
            self._stage = _ou2.get_context().get_stage()
            _art = self._robot.articulation
            try:
                self._base_link_idx = int(_art.get_link_index("base_link"))
            except Exception:
                self._base_link_idx = None
            # 末端连杆索引(主线程缓存实测 EE 位姿, 锚点不再靠 Jacobian 外推估)
            self._ee_link_idx = {}
            for _sd, _ln in (("left", "left_gripper_base_link"),
                             ("right", "right_gripper_base_link")):
                try:
                    self._ee_link_idx[_sd] = int(_art.get_link_index(_ln))
                except Exception:
                    pass
            self._ee_pose_cache = {}
            # 躯干无 yaw 关节: torso_pitch_link 相对底座的水平方向 = 物理前向
            # (head_pitch 会随头转, 不能用)
            try:
                self._torso_fwd_idx = int(
                    _art.get_link_index("torso_pitch_link"))
            except Exception:
                self._torso_fwd_idx = None

            def _rR(r, p, y):
                cr, sr = np.cos(r), np.sin(r)
                cp, sp = np.cos(p), np.sin(p)
                cy, sy_ = np.cos(y), np.sin(y)
                return (np.array([[cy, -sy_, 0], [sy_, cy, 0], [0, 0, 1]])
                        @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
                        @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))

            def _Tf(R, t):
                M = np.eye(4)
                M[:3, :3] = R
                M[:3, 3] = t
                return M

            _Ro = _rR(np.deg2rad(-90), 0, np.deg2rad(-90))
            _Rc = _rR(np.deg2rad(180), 0, 0)
            # 相机跟随: 偏移来自 register_cameras.py 静止实测生成的
            # scripts/scene/cam_follow_offsets.json —— 位置=挂载连杆 mesh
            # bbox 中心, 朝向=渲染验证约定(col0=光轴, col1=图像上),
            # off = inv(M_父连杆_rest) @ M_相机目标_rest。
            # 旧实现(XformCache 链/aim_base 换算)因 fabric 不同步 +
            # base_link 轴约定与直觉不符, 相机落后 8cm 且朝向偏 90°(用户实测)。
            try:
                import os as _os2
                with open(_os2.path.join(_os2.getcwd(), "scripts", "scene",
                                         "cam_follow_offsets.json")) as _f:
                    _offs = json.load(_f)
            except Exception:
                _offs = {}
            for _nm, _d in _offs.items():
                path = f"/World/cams/{_nm}"
                if not self._stage.GetPrimAtPath(path).IsValid():
                    continue
                try:
                    idx = int(_art.get_link_index(_d["parent"]))
                    self._cam_follow.append(
                        (path, idx, np.asarray(_d["off"], float)))
                except Exception:
                    continue
            # tv_cam: 相对 base_link 的 look-at 常数(第三人称, 保留)
            if self._stage.GetPrimAtPath("/World/cams/tv_cam").IsValid():
                eye = np.array([-1.6, 0.0, 1.9])
                d = np.array([1.05, 0.0, 0.75]) - eye
                d /= np.linalg.norm(d)
                pit = float(-np.arcsin(np.clip(d[2], -1, 1)))
                yawc = float(np.arctan2(d[1], d[0]))
                off = _Tf(_rR(0.0, pit, yawc), eye) @ _Tf(_Ro, [0, 0, 0]) \
                    @ _Tf(_Rc, [0, 0, 0])
                self._cam_follow.append(
                    ("/World/cams/tv_cam",
                     int(_art.get_link_index("base_link")), off))
            logger.info(
                f"相机跟随已启用: {len(self._cam_follow)} 路(文档链常数偏移)")
        except Exception as e:
            logger.warning(f"相机跟随初始化失败(相机保持固定): {e}")
            self._cam_follow = []

        # ---- 视频图层:VizSession(kXr)持有 OpenXR 会话,头显里显示机器人相机。
        # CloudXR 流送的是 XR 应用提交的图层;不提交图层头显就是黑屏。
        # VizSession 失败只损失画面,手柄控制不受影响。 ----
        self._viz = None
        self._head_layer = None
        oxr = None
        try:
            import isaacteleop.viz as viz
            vcfg = viz.VizSessionConfig()
            vcfg.mode = viz.DisplayMode.kXr
            vcfg.app_name = "RouteManagerVR"
            vcfg.xr_system_wait_seconds = 60   # 等头显 CONNECT,避免无限阻塞
            # 关键:TeleopSession 共享本 instance 时不会再自建(自建时才用它自己
            # 算出的扩展列表),所以这里必须把 deviceio 需要的 OpenXR 扩展全部
            # 启用(清单取自 _deviceio_session .so 的字符串表)。缺一个就会在
            # 运行时报 "Failed to get <fn> function pointer"。
            vcfg.required_extensions = [
                "XR_KHR_convert_timespec_time",
                "XR_NVX1_action_context",
                "XR_NVX1_push_tensor",
                "XR_NVX1_tensor_data",
                "XR_NV_opaque_data_channel",
                "XR_EXT_hand_tracking",
                "XR_BD_body_tracking",
                "XR_MNDX_xdev_space",
            ]
            self._viz = viz.VizSession.create(vcfg)
            lcfg = viz.QuadLayerConfig()
            lcfg.name = "head_cam"
            lcfg.resolution = viz.Resolution(_cam_w, _cam_h)
            # native OpenXR quad 必须带 placement(否则 submit 每帧报
            # "requires Config::placement to be set"):头显初始位置正前方
            # 1.5m 处挂一块 1.6×1.0m 画面(1920×1200 = 16:10)
            lcfg.placement = viz.QuadLayerPlacement(
                viz.Pose3D((0.0, 0.0, -1.5), (1.0, 0.0, 0.0, 0.0)),
                [1.6, 1.0])
            self._head_layer = self._viz.add_quad_layer(lcfg)
            oxr = self._viz.get_oxr_handles()
            if oxr is not None:
                # get_oxr_handles 返回裸 uint64 元组; TeleopSession 的 C++ run()
                # 需要 core::OpenXRSessionHandles 对象
                from isaacteleop.oxr import OpenXRSessionHandles
                oxr = OpenXRSessionHandles(*oxr)
            logger.info(f"VizSession(kXr) 就绪, oxr={oxr is not None}")
        except Exception as e:
            logger.warning(f"VizSession 创建失败,无头显画面: {e}")
            self._viz = None
            self._head_layer = None

        try:
            import torch
            self._torch = torch
        except Exception:
            self._torch = None

        config = TeleopSessionConfig(
            app_name="RouteManagerVR",
            trackers=[],             # 由管线自动发现（ControllersSource 自带）
            pipeline=pipeline,
            plugins=[],
            oxr_handles=oxr,         # 共享 VizSession 的 OpenXR 会话(None 则自建)
        )
        try:
            session = TeleopSession(config)
            session.__enter__()
            self._session = session
        except Exception:
            # 失败路径同样要清掉 VizSession 与 launcher(否则 OpenXR/Vulkan
            # 资源泄漏,每次重试都加重 GPU 负担)
            if self._viz is not None:
                try:
                    self._viz.destroy()
                except Exception:
                    pass
                self._viz = None
                self._head_layer = None
            self._launcher.__exit__(None, None, None)
            self._launcher = None
            raise

        self._stop_evt.clear()
        self._hands = {s: {"prev": None} for s in ("left", "right")}
        with self._lock:
            self._pending = {"grip": {}, "arm": {}, "base": {}, "head": {}, "torso": {}}
        self._status = {"running": True, "frames": 0, "last_error": None,
                        "last_controller": None}
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="vr_teleop_loop")
        self._thread.start()
        logger.info(f"VR 遥操作已启动: rate={rate_hz}Hz, "
                    f"arms={self._arms}, grip={self._grip}")
        return {"status": "started", "arms": self._arms,
                "gripper": self._grip,
                "note": "Pico 浏览器连 https://nvidia.github.io/IsaacTeleop/client"}

    def stop(self):
        if self._thread is not None:
            self._stop_evt.set()
            self._thread.join(timeout=3.0)
            self._thread = None
        if self._session is not None:
            try:
                self._session.__exit__(None, None, None)
            except Exception as e:
                logger.warning(f"TeleopSession 退出异常: {e}")
            self._session = None
        if self._viz is not None:
            try:
                self._viz.destroy()
            except Exception as e:
                logger.warning(f"VizSession 销毁异常: {e}")
            self._viz = None
            self._head_layer = None
        if self._launcher is not None:
            try:
                self._launcher.__exit__(None, None, None)
            except Exception as e:
                logger.warning(f"CloudXR launcher 退出异常: {e}")
            self._launcher = None
        self._status["running"] = False
        logger.info("VR 遥操作已停止")
        return {"status": "stopped"}

    def status(self):
        return dict(self._status)

    # ---------- 控制循环 ----------

    def _loop(self):
        dt = 1.0 / self._params["rate_hz"]
        while not self._stop_evt.is_set():
            t0 = time.monotonic()
            try:
                self._tick()
            except Exception as e:
                self._status["last_error"] = str(e)
                logger.error(f"遥操作循环异常: {e}")
            sleep = dt - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)

    def _tick(self):
        """一次 VR 数据 → 写入待施加缓冲（不在本线程碰 Kit/关节 API）。

        线程安全：articulation/set_joint_position_targets 必须在仿真主线程
        调用（Kit 场景 API 非线程安全，曾导致进程段错误崩溃）。本线程只做
        isaacteleop session.step() + 手柄数据读取，把目标写进 _pending，
        由 RobotManager.step() 在主线程调用 apply_pending() 施加。
        """
        session = self._session
        if session is None:
            return

        result = session.step()
        self._status["frames"] += 1

        from isaacteleop.retargeting_engine.tensor_types import (
            ControllerInputIndex as CII)

        ctrl_view = {}
        pending = {"grip": {}, "arm": {}, "base": {}, "head": {}, "torso": {}}

        # ---- 头部跟踪: 头显朝向 → 相对基准的 yaw/pitch ----
        try:
            from isaacteleop.retargeting_engine.tensor_types import (
                HeadInputIndex as HII)
            hg = result.get("head")
            _none = hg.is_none if hg is not None else True
            if callable(_none):
                _none = _none()      # 手柄组是方法, 头组可能是属性
            if hg is not None and not _none:
                if bool(hg[HII.IS_VALID]) and bool(hg[HII.IS_TRACKED]):
                    hq = np.asarray(hg[HII.ORIENTATION], dtype=np.float64)
                    if hq.size == 4:
                        x, y, z, w = hq
                        # OpenXR: X 右 / Y 上 / Z 后 → 前向 = R 的第三列取负:
                        #   (-2(xz+wy), -2(yz-wx), -(1-2(x²+y²)))
                        # 曾误写成第三行(等价于共轭四元数, yaw/pit 同时反号),
                        # 是"左转→仿真右转"反复调符号治不好的根因。
                        fx = -(2 * (x * z + w * y))
                        fy = -(2 * (y * z - w * x))
                        fz = -(1 - 2 * (x * x + y * y))
                        yaw = float(np.arctan2(fx, -fz))          # 右转为正
                        pit = float(np.arcsin(np.clip(fy, -1, 1)))  # 上为正
                        self._head_yaw_track = yaw   # 跟踪系偏航(手臂映射自标定用)
                        if self._head_ref is None:
                            self._head_ref = (yaw, pit)   # 首帧为基准
                        # head_yaw_joint 与 OpenXR 的“右转为正”保持同号；
                        # 若更换机器人后左右反向，用 config.json 的符号翻转。
                        dy = float(np.clip(yaw - self._head_ref[0],
                                           -1.0, 1.0))
                        # URDF head_pitch axis≈(0,-1,0): +=抬头 → 同号
                        dp = float(np.clip(pit - self._head_ref[1],
                                           -0.8, 0.8))
                        if self._head_joints:
                            pending["head"] = {
                                "yaw": dy * self._head_signs["yaw"],
                                "pitch": dp * self._head_signs["pitch"],
                            }
        except Exception as e:
            self._status["last_error"] = f"head: {e}"

        for side, key in (("left", "controller_left"),
                          ("right", "controller_right")):
            grp = result.get(key)
            ctrl_view[side] = None
            if grp is None:
                self._ctrl_dbg[side] = "no_group"
                self._hands[side]["prev"] = None
                continue
            try:
                # is_none 在不同张量组上可能是方法也可能是属性(bool), 兼容两者
                _none = grp.is_none
                if callable(_none):
                    _none = _none()
                if _none:
                    self._ctrl_dbg[side] = "none"
                    self._hands[side]["prev"] = None
                    continue
                def _g(idx, default=0.0):
                    """逐项读取并兜底: 某些字段在旧版 schema 里可能越界,
                    不能因为一项缺失废掉整个手柄解析。"""
                    try:
                        return grp[idx]
                    except Exception:
                        return default
                pos = np.asarray(_g(CII.GRIP_POSITION, None),
                                 dtype=np.float64)
                trig = float(_g(CII.TRIGGER_VALUE, 0.0))
                sq = float(_g(CII.SQUEEZE_VALUE, 0.0))
                prim = float(_g(CII.PRIMARY_CLICK, 0.0))
                sec = float(_g(CII.SECONDARY_CLICK, 0.0))
                tclick = float(_g(CII.THUMBSTICK_CLICK, 0.0))
                menu = float(_g(CII.MENU_CLICK, 0.0))
                valid = bool(_g(CII.GRIP_IS_VALID, False))
                thumb = (float(_g(CII.THUMBSTICK_X, 0.0)),
                         float(_g(CII.THUMBSTICK_Y, 0.0)))
                _ori = _g(CII.GRIP_ORIENTATION, None)
                ori = (np.asarray(_ori, dtype=np.float64)
                       if _ori is not None else None)
            except Exception as e:
                self._ctrl_dbg[side] = "ERR:" + repr(e)[:90]
                self._hands[side]["prev"] = None
                continue
            # ---- grip 位姿无效时用 aim 位姿兜底 ----
            # 客户端同时送 grip(手握) 和 aim(指向射线) 两套位姿, 实测
            # grip_is_valid 大量时间为 false(手部追踪丢失/静止不更新), 而
            # aim 通常一直有效。没有这一步 → _tick 整帧跳过 → 手柄"完全没
            # 反应"、标定采样为 0(2026-10-08 实测 n_samples=0)。
            _aim_v = bool(_g(CII.AIM_IS_VALID, False))
            _aim_p = _g(CII.AIM_POSITION, None)
            _aim_o = _g(CII.AIM_ORIENTATION, None)
            _aim_ps = (np.asarray(_aim_p, dtype=np.float64)
                       if _aim_p is not None else None)
            _dbg = self._status.setdefault("pose_src", {})
            _dbg[side] = {"grip_valid": bool(valid), "aim_valid": _aim_v,
                          "src": "grip"}
            if (not valid or pos.size != 3) and _aim_v \
                    and _aim_ps is not None and _aim_ps.size == 3:
                pos = _aim_ps
                if _aim_o is not None and np.asarray(_aim_o).size == 4:
                    ori = np.asarray(_aim_o, dtype=np.float64)
                valid = True
                _dbg[side]["src"] = "aim"
            if not valid or pos.size != 3:
                self._ctrl_dbg[side] = ("invalid" if not valid
                                        else f"size{pos.size}")
                self._hands[side]["prev"] = None
                continue
            self._ctrl_dbg[side] = "ok"
            self._last_ctrl_pos[side] = pos.copy()   # 推杆实测标定采样用
            self._calib_sample(side, pos)
            self._calib3_sample(side, pos, sq)       # 三推杆标定采样
            ctrl_view[side] = {"pos": pos, "trigger": trig, "thumb": thumb,
                               "squeeze": sq, "ori": ori, "primary": prim,
                               "secondary": sec, "tclick": tclick,
                               "menu": menu}
            # 输入峰值记录: 用户任意时刻操作, 事后读峰值/计数即可判定
            # "客户端没送数据" vs "阈值/逻辑问题"(不用再同步盯着按)
            pk = self._status.setdefault("input_peak", {}).setdefault(side, {})
            pk["squeeze_max"] = round(max(pk.get("squeeze_max", 0.0), sq), 2)
            pk["trigger_max"] = round(max(pk.get("trigger_max", 0.0), trig), 2)
            pk["thumb_max"] = round(max(pk.get("thumb_max", 0.0),
                                        abs(thumb[0]), abs(thumb[1])), 2)

            # 夹爪: 官方 geniesim 语义 gripper = 1 - indexTrig(松=全开, 扣=闭),
            # 行程 × gripper; 之前方向反了(扣扳机=张开)
            _frac = 1.0 - min(max(trig, 0.0), 1.0)
            # 比例 → 主动关节角走实测标定表(与 set_gripper_position 同刻度);
            # 线性 ×grip_open 在中间段能差 5mm 开口。
            pending["grip"][side] = gripper_model.leader_for_fraction(_frac)
            # 遥测: 扳机→开口比例(0=全闭 1=全开)。真机 rm 单位 =
            # 1 + frac*999, 与 RouteManager.set_gripper_position 同一刻度。
            # 扣扳机时这里不动 = 客户端没送 trigger, 不是映射问题。
            self._status.setdefault("grip_cmd", {})[side] = round(_frac, 3)

        # 标定结算的兜底检查: 结算不能只挂在"位姿有效的帧"上——用户推完
        # 摘头显后位姿失效, 采样不再执行, done 标志永远等不到结算(实测)。
        _cal = getattr(self, "_calib", None)
        if _cal and (_cal.get("finalize_now") or time.time() > _cal["t_end"]):
            self._calib_finalize()

        # 手臂: 有 IK 表时走"侧握按住 = 绝对位姿 IK"(官方 control_trigger 模式),
        # 松手冻结; 无表则退回速率映射。
        # 单点异常不能拖垮整帧: _ik_tick 抛异常时 pending 也要提交, 否则
        # 夹爪/底盘/头部全跟着失效(2026-10-08: 腕部跟随里一个 list 运算
        # 抛异常 → 整个 _tick 中断 → "握着握把什么都不动")
        if self._jac_table is not None:
            try:
                self._ik_tick(ctrl_view, pending)
            except Exception as e:
                self._status["last_error"] = f"ik_tick: {e}"
        else:
            try:
                self._arm_rate_tick(ctrl_view, pending)
            except Exception as e:
                self._status["last_error"] = f"arm_rate: {e}"

        # 底盘:左摇杆 y → 前进(vx), x → 转向(wz);死区内归零。
        # set_base_velocity 自带限幅(0.6 m/s / 1.6 rad/s)与 0.5s 超时刹停,
        # 摇杆回中即发 0,松手安全。
        lc = ctrl_view.get("left")
        if lc is not None:
            x, y = lc["thumb"]
            dz = self._params.get("base_deadzone", 0.15)
            vx = 0.0 if abs(y) < dz else float(np.clip(y, -1, 1)) * 0.6
            wz = 0.0 if abs(x) < dz else -float(np.clip(x, -1, 1)) * 1.6
            pending["base"] = {"vx": vx, "wz": wz}

        # 复位键(智元 keyOne 语义): A/X 上升沿 → 头部基准、IK 锚点、躯干目标
        # 全部重新锚定; 底盘刹停。轮子积分器**不清零**(清零会让轮位目标跳变)。
        # 主键按左右**分开**取: 右手 A = 复位, 左手 X = 自动抓取
        # (菜单键是 Pico 系统键会退回桌面, 用不了)
        _btn = 0.0            # 右主键(A)
        _xkey = 0.0           # 左主键(X)
        _btn2 = 0.0
        _tc_l = 0.0
        _tc_r = 0.0
        for _s in ("left", "right"):
            _c = ctrl_view.get(_s)
            if _c:
                if _s == "left":
                    _xkey = float(_c.get("primary", 0.0))
                    _tc_l = float(_c.get("tclick", 0.0))
                else:
                    _btn = float(_c.get("primary", 0.0))
                    _tc_r = float(_c.get("tclick", 0.0))
                _btn2 = max(_btn2, float(_c.get("secondary", 0.0)))
        _bc = self._status.setdefault("btn_count",
                                      {"ax": 0, "by": 0, "stick_l": 0,
                                       "stick_r": 0, "x": 0})
        if _btn > 0.5 and self._btn_prev <= 0.5:
            _bc["ax"] += 1
            self._head_ref = None          # 下一帧以当前头显朝向为新基准
            self._ik_state = {}            # 手臂重新锚定(再按侧握从当前位继续)
            self._ik_smooth = {}           # 平滑状态重同步, 防复位后追旧目标
            self._torso_target = None      # 躯干目标重新锚定
            pending["base"] = {"vx": 0.0, "wz": 0.0}
            pending["arm_home"] = True     # 复位=全回: 手臂也回 home(用户明确
                                           # 期望"按复位机械臂回 home", 智元
                                           # keyOne=reset 同语义)
            pending.pop("torso", None)
        self._btn_prev = _btn
        # B/Y: 双臂回 home(标定中心姿态)——可见的硬动作, 便于确认按钮生效
        if _btn2 > 0.5 and getattr(self, "_btn2_prev", 0.0) <= 0.5:
            _bc["by"] += 1
            pending["arm_home"] = True
            # 清离合锚点: 若此刻还握着侧握, 旧锚点产生的 ik 目标会在同一帧
            # 把刚回 home 的手臂立刻拉回去(用户实测"复位对机械臂无效")。
            # 清掉后仍握持的一侧下一帧重新锚定(从 home 位继续跟手)。
            self._ik_state = {}
            self._ik_smooth = {}
        self._btn2_prev = _btn2
        # 左手 **X 键**: 边沿触发一次"自动抓取"请求(由 RobotManager.step 取走)。
        # 原来用菜单键, 但那是 Pico 系统键, 按下去直接退回桌面。
        # 抬起沿才置位, 防止按住时每帧重复触发。
        if _xkey > 0.5 and getattr(self, "_x_prev", 0.0) <= 0.5:
            self._menu_req = time.time()
            _bc["x"] += 1
        self._x_prev = _xkey
        # 右摇杆按下: 精细/正常速度切换; 左摇杆按下: 底盘刹停
        if _tc_r > 0.5 and getattr(self, "_tcr_prev", 0.0) <= 0.5:
            _bc["stick_r"] += 1
            self._ik_scale = 0.4 if self._ik_scale > 0.6 else 1.0
        self._tcr_prev = _tc_r
        if _tc_l > 0.5 and getattr(self, "_tcl_prev", 0.0) <= 0.5:
            _bc["stick_l"] += 1
            pending["base"] = {"vx": 0.0, "wz": 0.0}
        self._tcl_prev = _tc_l

        # 躯干(智元"右摇杆→腰"语义): 头部已由头显直驱, 右摇杆空闲 →
        # y = lift 升降(棱柱 [-0.6, 0.05]m, 电机 0.1m/s → 每帧 2mm),
        # x = torso_pitch 俯仰(软限位 ±0.5rad, 硬件 ±1.57)。
        rc = ctrl_view.get("right")
        if rc is not None:
            x, y = rc["thumb"]
            dz = self._params.get("base_deadzone", 0.15)
            if abs(x) > dz or abs(y) > dz:
                if self._torso_target is None:
                    tt = [0.0, 0.0]
                    try:
                        nms = list(self._robot.articulation.dof_names)
                        if self._q_all is not None:
                            for _k, _jn in ((0, "lift_joint"),
                                            (1, "torso_pitch_joint")):
                                if _jn in nms:
                                    tt[_k] = float(
                                        self._q_all[nms.index(_jn)])
                    except Exception:
                        pass
                    self._torso_target = tt
                # 用户 2026-09-29 指定: 右摇杆 **前后(y)=俯仰**, **左右(x)=升降**
                # (此前是前后=升降/左右=俯仰, 按用户要求对调)
                self._torso_target[0] = float(np.clip(   # lift: 左右轴
                    self._torso_target[0]
                    + (x if abs(x) > dz else 0.0) * 0.002, -0.6, 0.05))
                # 官方 waist.yaml 限位 [-0.0175, 1.5708](只能前俯); 软限收紧防顶死
                self._torso_target[1] = float(np.clip(   # pitch: 前后轴
                    self._torso_target[1]
                    + (y if abs(y) > dz else 0.0) * 0.008, -0.017, 0.8))
                pending["torso"] = {"lift": self._torso_target[0],
                                    "pitch": self._torso_target[1]}

        with self._lock:
            self._pending = pending
        self._status["ctrl_dbg"] = dict(getattr(self, "_ctrl_dbg", {}))
        # 按钮原始值诊断: 用户按了没反应时, 看这里是 0(客户端没送)还是 1(逻辑没接)
        self._status["last_buttons"] = {
            s: (None if v is None else
                {"A" if s == "right" else "X":
                    round(float(v.get("primary", 0.0)), 2),
                 "BY": round(float(v.get("secondary", 0.0)), 2),
                 "stick": round(float(v.get("tclick", 0.0)), 2),
                 "menu": round(float(v.get("menu", 0.0)), 2)})
            for s, v in ctrl_view.items()}
        self._status["ik_scale"] = self._ik_scale
        self._status["last_controller"] = {
            s: (None if v is None else
                {"pos": v["pos"].round(3).tolist(),
                 "trigger": round(v["trigger"], 2),
                 "squeeze": round(v["squeeze"], 2),
                 "thumb": [round(v["thumb"][0], 2), round(v["thumb"][1], 2)]})
            for s, v in ctrl_view.items()}

    def _viz_tick(self):
        """视频提交与 XR 合成——**必须主线程**(Kit 渲染 API 非线程安全,
        VR 线程调用曾导致进程段错误)。按 vr_frame_interval 同步降频。"""
        # 相机跟随机器人头部(第一视角): 每帧把相机挪到 head_pitch_link + 固定偏移
        if self._cam_follow:
            try:
                from pxr import Gf as _Gf
                lt = np.asarray(
                    self._robot.articulation._physics_view
                    .get_link_transforms()).reshape(-1, 7)
                # 底座世界偏航缓存(主线程读张量; VR 线程只消费缓存值)
                # 机器人物理前向 yaw: base_link 系 X 静止时指世界 -X(URDF 约定
                # 坑, 实测 frame yaw=π), 不能直接取系 yaw; 用 base→torso_pitch
                # 水平方向(躯干无 yaw 关节, 与系约定/头转都无关)
                bi = getattr(self, "_base_link_idx", None)
                hi = getattr(self, "_torso_fwd_idx", None)
                if bi is not None and hi is not None \
                        and bi < lt.shape[0] and hi < lt.shape[0]:
                    _d = np.asarray(lt[hi][:3], float) \
                        - np.asarray(lt[bi][:3], float)
                    if _d[0] ** 2 + _d[1] ** 2 > 1e-6:
                        self._base_yaw_world = float(
                            np.arctan2(_d[1], _d[0]))
                # 末端实测位姿缓存(IK 锚点用, 零估计误差)
                for _sd, _ei in getattr(self, "_ee_link_idx", {}).items():
                    if _ei is not None and _ei < lt.shape[0]:
                        _me = np.eye(4)
                        _me[:3, :3] = self._q2R(lt[_ei][3:])
                        _me[:3, 3] = np.asarray(lt[_ei][:3], float)
                        self._ee_pose_cache[_sd] = _me
                # 末端实测位置进 status(主线程写): 排障时能直接看手臂是否
                # 已顶到工作空间边界("/python 不回显 result"这点踩过坑)
                self._status["ee_cache"] = {
                    _s: [round(float(v), 3) for v in _m[:3, 3]]
                    for _s, _m in self._ee_pose_cache.items()}
                for _path, _idx, _off in self._cam_follow:
                    v7 = lt[_idx]
                    w, x, y, z = np.asarray(v7[3:], float)
                    _n = max(np.linalg.norm([w, x, y, z]), 1e-9)
                    w, x, y, z = w / _n, x / _n, y / _n, z / _n
                    R = np.array([
                        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),
                         2 * (x * z + w * y)],
                        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z),
                         2 * (y * z - w * x)],
                        [2 * (x * z - w * y), 2 * (y * z + w * x),
                         1 - 2 * (x * x + y * y)]])
                    M_link = np.eye(4)
                    M_link[:3, :3] = R
                    M_link[:3, 3] = np.asarray(v7[:3], float)
                    M_cam = M_link @ _off
                    qq = np.asarray(self._R2q(M_cam[:3, :3]), float)
                    t = M_cam[:3, 3]
                    p = self._stage.GetPrimAtPath(_path)
                    p.GetAttribute("xformOp:translate").Set(
                        _Gf.Vec3d(float(t[0]), float(t[1]), float(t[2])))
                    p.GetAttribute("xformOp:orient").Set(
                        _Gf.Quatd(float(qq[0]), float(qq[1]), float(qq[2]),
                                  float(qq[3])))
            except Exception as e:
                self._status["last_error"] = f"cam_follow: {e}"
                self._cam_follow = []      # 失败一次就不再重试
        if self._viz is None:
            return
        try:
            submit_frame = False
            if (self._head_layer is not None and self._head_cam is not None
                    and self._torch is not None):
                self._frame_i += 1
                if self._frame_i % self._vr_frame_interval == 0:
                    rgb = self._head_cam.get_rgba()
                    if rgb is not None and rgb.size:
                        t = self._torch.from_numpy(
                            np.ascontiguousarray(rgb)).cuda()
                        self._head_layer.submit(t)
                        submit_frame = True
            if submit_frame or self._head_layer is None:
                self._viz.render()
        except Exception as e:
            self._status["last_error"] = f"viz: {e}"

    # ---------- 手臂: 速率映射(无 IK 表时的回退) ----------

    def _arm_rate_tick(self, ctrl_view, pending):
        """控制器位移 → 前三个臂关节增量(相对上次位置)。"""
        for side in ("left", "right"):
            c = ctrl_view[side]
            prev = self._hands[side]["prev"]
            if c is not None:
                self._hands[side]["prev"] = c["pos"]
            if c is None or prev is None:
                continue
            d = c["pos"] - prev
            if float(np.linalg.norm(d)) < self._params["deadzone"]:
                continue
            jn = self._arms[side]
            if not jn:
                continue
            deltas = []
            for i in range(len(jn)):
                raw = d[i] if i < 3 else 0.0
                raw *= self._params["workspace_scale"]
                deltas.append(max(-self._params["max_step"],
                                  min(self._params["max_step"], raw)))
            pending["arm"][side] = {"names": jn, "deltas": deltas}

    # ---------- IK: 侧握按住 = 绝对位姿控制(官方 control_trigger 模式) ----------

    def _table_nn(self, side, q):
        """Jacobian 表里按 6 维关节角最近的采样。"""
        best, bd = None, 1e9
        for s in self._jac_table["samples"]:
            if s["side"] != side:
                continue
            d = float(np.linalg.norm(np.asarray(s["q"], float) - q))
            if d < bd:
                bd, best = d, s
        return best

    # ---- 四元数 / 位姿工具 ----

    @staticmethod
    def _q2R(q):
        w, x, y, z = np.asarray(q, float)
        n = max(float(np.linalg.norm([w, x, y, z])), 1e-9)
        w, x, y, z = w / n, x / n, y / n, z / n
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),
             2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z),
             2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x),
             1 - 2 * (x * x + y * y)]])

    @staticmethod
    def _R2q(R):
        tr = float(np.trace(R))
        if tr > 0:
            s = np.sqrt(tr + 1) * 2
            return [0.25 * s, (R[2, 1] - R[1, 2]) / s,
                    (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            return [(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                    (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
        if i == 1:
            s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            return [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                    0.25 * s, (R[1, 2] + R[2, 1]) / s]
        s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        return [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[1, 2] + R[2, 1]) / s, 0.25 * s]

    def _ctrl_T(self, c):
        """手柄位姿 → 4×4(世界系; 无姿态数据时旋转取单位阵)。"""
        T = np.eye(4)
        if c.get("ori") is not None and np.asarray(c["ori"]).size == 4:
            T[:3, :3] = self._q2R(c["ori"])
        T[:3, 3] = np.asarray(c["pos"], float)
        return T

    # ---------- 手臂运动学(FK 取代预计算雅可比表) ----------

    def _ensure_kin(self, side):
        """惰性加载 URDF 手臂链 + 限位。"""
        if side in self._kin:
            return self._kin[side]
        try:
            import arm_kin
            ch = arm_kin.load_chain(arm_kin.URDF, f"{side}_arm_base_link",
                                    f"{side}_gripper_base_link")
            # 安全闸: FK 链的关节顺序必须与下发顺序(self._arms)一致,
            # 否则 IK 解会被写到别的关节上 —— 宁可不动。
            names_kin = arm_kin.joint_names(ch)
            names_arm = list(self._arms.get(side, []))
            if names_arm and names_arm != names_kin:
                raise RuntimeError(
                    f"关节顺序不一致: 下发 {names_arm} vs FK {names_kin}")
            self._kin[side] = ch
            self._kin_lim[side] = arm_kin.joint_limits(ch)
        except Exception as e:
            self._status["last_error"] = f"kin[{side}]: {e}"
            self._kin[side] = None
        return self._kin[side]

    def _update_arm_base(self, force=False):
        """主线程: 从实时 link 变换刷新手臂基座世界位姿(FK 的基准)。

        必须主线程读(Kit 张量非线程安全); VR 线程只消费缓存值。
        """
        self._base_upd_n += 1
        if not force and self._base_upd_n % 20:
            return
        try:
            art = self._robot.articulation
            lt = np.asarray(
                art._physics_view.get_link_transforms()).reshape(-1, 7)
            for side in ("left", "right"):
                nm = f"{side}_arm_base_link"
                if self._base_idx.get(side) is None:
                    self._base_idx[side] = int(art.get_link_index(nm))
                i = self._base_idx[side]
                p = np.asarray(lt[i][:3], float)
                q = np.asarray(lt[i][3:], float)   # xyzw
                x, y, z, w = q
                R = np.array([
                    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
                     2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
                     2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w),
                     1 - 2 * (x * x + y * y)]])
                self._arm_base_world[side] = (R, p)
        except Exception as e:
            self._status["last_error"] = f"arm_base: {e}"
        # 限位用**仿真里实际生效**的那份(资产与 URDF 不一致: 实测
        # left_arm_2 URDF ±130° 但资产只到 +78°, 用 URDF 的会解出够不着的
        # 目标, 表现为"指令发了臂不动/停住")。只需读一次。
        if not self._lim_from_sim:
            try:
                art = self._robot.articulation
                dn = list(art.dof_names)
                dl = np.asarray(art.get_dof_limits()).reshape(-1, 2)
                for side in ("left", "right"):
                    jn = list(self._arms.get(side, []))
                    if jn and all(n in dn for n in jn):
                        self._kin_lim[side] = np.asarray(
                            [dl[dn.index(n)] for n in jn], dtype=float)
                self._lim_from_sim = True
            except Exception as e:
                self._status["last_error"] = f"lim: {e}"

    def _arm_q_now(self, side, names_all=None):
        """手臂关节的**实测**角度(IK 的起点)。主线程调用。"""
        try:
            art = self._robot.articulation
            names_all = list(names_all or art.dof_names)
            pos = np.asarray(art.get_joint_positions()).reshape(-1)
            return np.asarray(
                [pos[names_all.index(n)] for n in self._arms.get(side, [])],
                dtype=float)
        except Exception as e:
            self._log_apply_err(f"arm_q_now[{side}]: {e}")
            return None

    def _ee_T(self, side, q):
        """EE 位姿(世界系) —— 用 FK 精确算(原来是查表+一阶外推, 差到厘米级)。"""
        ch = self._ensure_kin(side)
        ab = self._arm_base_world.get(side)
        T = np.eye(4)
        if ch is None or ab is None or q is None:
            return T
        try:
            import arm_kin
            p, R = arm_kin.ee_world(ch, np.asarray(q, float), ab[0], ab[1])
            T[:3, :3] = R
            T[:3, 3] = p
        except Exception as e:
            self._status["last_error"] = f"ee_T[{side}]: {e}"
        return T

    def _ee_estimate(self, side, q):
        """(兼容)只返回 EE 位置。"""
        return self._ee_T(side, q)[:3, 3]

    # ---------- IK: 侧握按住 = 绝对位姿控制(官方 control_trigger 模式) ----------

    def _ik_tick(self, ctrl_view, pending):
        """**位置定末端**(用户指定 2026-09-28): 握离合期间用手柄**当前位置**
        直接决定末端目标, 不做逐帧增量积分:
            pos_tgt = EE_anchor + Q · scale · (p_ctrl − p_ctrl_anchor)
            R_tgt   = Q · R_ctrl · R_ctrl_anchor⁻¹ · Qᵀ · R_EE_anchor
        其中 Q = Rz(phi)·P, P 为 OpenXR→机器人轴置换, phi 为偏航自标定。
        旧逐帧 dT 积分在手柄姿态噪声下随机游走(用户实测"完全是随机的");
        位置式零漂移——手移到哪目标就在哪, 松手冻结, 重握从当前位置重锚无跳变。
        phi = 握瞬间的 跟踪系→世界 偏航自标定(_track_phi, 免手标矩阵);
        scale 握瞬间锁定(握持中切档不跳变, 下次握生效)。"""
        dbg = {}
        for side in ("left", "right"):
            c = ctrl_view[side]
            st = self._ik_state.setdefault(
                side, {"on": False, "anchor_ctrl": None, "T_ee0": None})
            if c is None:
                st["on"] = False
                st["anchor_ctrl"] = None
                continue
            # 离合阈值 0.6 接合 / 0.4 释放(迟滞)。
            # 历史: 0.5 → 0.8 是因为"握紧过程中手自然后拉被当成位移"；
            # 2026-10-08 实测日志显示 0.8 偏高(用户按侧握常常只到 0.3~0.6,
            # 表现为"按住握把机械臂不动")。正确解法不是抬高阈值, 而是
            # 下面的 **settle 窗口**吸收掉握紧瞬间的手柄位移。
            sq = float(c["squeeze"])
            on = sq > (0.4 if st["on"] else 0.6)
            dbg[side] = {"on": bool(on),
                         "squeeze": round(float(c["squeeze"]), 2)}
            if not on:
                st["on"] = False
                st["anchor_ctrl"] = None
                st["t_engage"] = None
                continue
            T_ctrl = self._ctrl_T(c)
            _now = time.monotonic()
            if not st["on"]:
                # 按下瞬间: 锚定手柄位姿 + **实测** EE 位姿 + 偏航标定(不产生运动)
                st["on"] = True
                st["t_engage"] = _now
                st["anchor_ctrl"] = T_ctrl
                _live = getattr(self, "_ee_pose_cache", {}).get(side)
                st["T_ee0"] = (_live.copy() if _live is not None else
                               self._ee_T(side, self._arm_target.get(side)))
                # 接合快照(掉线后仍可查): 飞臂问题对着这组数据分析
                self._status.setdefault("ik_engage", {})[side] = {
                    "ee0": [round(float(v), 3) for v in st["T_ee0"][:3, 3]],
                    "ee0_src": "live" if _live is not None else "table",
                    "ctrl0": [round(float(v), 3) for v in T_ctrl[:3, 3]],
                    "squeeze": round(sq, 2)}
                # phi 优先取"推杆实测标定"(vr_axis_calib, 对手柄自身位移
                # 反解, 零假设); 未标定时回退头显偏航推算(依赖头显/手柄
                # 坐标系一致 + 握瞬间头朝正前, 两个假设都未实测验证过)
                if side in getattr(self, "_phi_fixed", {}):
                    st["phi"] = float(self._phi_fixed[side])
                    st["phi_src"] = "calib"
                else:
                    st["phi"] = self._track_phi()
                    st["phi_src"] = "head"
                st["scale"] = float(self._ik_scale)
                st["R_ctrl0"] = T_ctrl[:3, :3].copy()   # 腕部跟随的转动基准
                st["q_sm"] = None
                continue
            # settle: 接合后的 CLUTCH_SETTLE_S 内**持续重锚**, 不算位移。
            # 手指捏握把的动作会把手柄带偏几厘米, 不吸收掉就变成
            # "一按握把机械臂自动往身后飞"(用户反复反馈的现象)。
            # 用重锚而不是"固定前若干帧的位移", 用户捏多久都不会漏。
            if _now - float(st.get("t_engage") or _now) < self.CLUTCH_SETTLE_S:
                st["anchor_ctrl"] = T_ctrl
                dbg[side] = {"on": True, "settle": True,
                             "squeeze": round(float(c["squeeze"]), 2)}
                continue
            if st["anchor_ctrl"] is None or st["T_ee0"] is None:
                st["anchor_ctrl"] = T_ctrl
                continue
            ph = float(st.get("phi", 0.0))
            sc = float(st.get("scale", self._ik_scale))
            cph, sph = np.cos(ph), np.sin(ph)
            d_track = T_ctrl[:3, 3] - st["anchor_ctrl"][:3, 3]
            if self._calib_R is not None:
                # 三推杆标定出的**完整旋转矩阵**(数据说话, 零轴序假设)
                Qr = self._calib_R
            else:
                # 回退: Q = Rz(偏航自标定) @ P(假设 OpenXR 轴: 前=-Z/左=-X/上=+Y)。
                # ⚠️ 该假设 2026-10-08 实测**不成立**(客户端"前推"走 +x),
                # 所以强烈建议先做一次 vr_push_calib。
                Qr = np.array([[cph, -sph, 0.0], [sph, cph, 0.0],
                               [0, 0, 1.0]]) @ \
                    np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
            dp = Qr @ (d_track * sc)
            # ---- 腕部跟随(2026-10-08 用户要求打开) ----
            # 手柄相对接合时刻的转动 dR(跟踪系) → 共轭映射到世界系 →
            # 乘到接合瞬间的 EE 朝向。手柄姿态逐帧噪声大, 不滤波会让 IK
            # 姿态乱扭(这是当初锁死朝向的原因), 所以用 nlerp 一阶低通。
            # 注意 _R2q 返回的是 **list**, 必须转 np 数组再做插值/取反,
            # 否则 `-list` / `float*list` 抛异常 → 整个 _tick 中断 →
            # pending 永不提交 → "握着握把什么都不动"(2026-10-08 踩过)
            _q_des = np.asarray(self._R2q(st["T_ee0"][:3, :3]), dtype=float)
            _Rc0 = st.get("R_ctrl0")
            if _Rc0 is not None and c.get("ori") is not None:
                dR = T_ctrl[:3, :3] @ _Rc0.T
                _q_des = np.asarray(
                    self._R2q(Qr @ dR @ Qr.T @ st["T_ee0"][:3, :3]),
                    dtype=float)
            _q_prev = st.get("q_sm")
            if _q_prev is not None:
                if float(np.dot(_q_prev, _q_des)) < 0.0:
                    _q_des = -_q_des          # 四元数双覆盖: 取同半球再插值
                _q_sm = (1.0 - self.WRIST_ALPHA) * _q_prev \
                    + self.WRIST_ALPHA * _q_des
                n_ = float(np.linalg.norm(_q_sm))
                _q_sm = _q_sm / n_ if n_ > 1e-9 else _q_des
            else:
                _q_sm = _q_des
            st["q_sm"] = _q_sm
            _tq_out = _q_sm
            pos_tgt = st["T_ee0"][:3, 3] + dp
            # 朝向: 腕部跟随(上面算出的低通后四元数 _q_sm); 手柄无姿态数据时
            # 退回"锁定接合瞬间朝向"。
            pending.setdefault("ik", {})[side] = {
                "pos": [float(v) for v in pos_tgt],
                "quat": [float(v) for v in _tq_out]}
            # 管线环形缓冲(最近 60 帧): 手位移→映射→目标点 全量留档,
            # 掉线后仍可读, 供"往哪推→臂往哪走"的离线定位
            _pl = self._status.setdefault("pipe", [])
            _pl.append([round(float(v), 3) for v in
                        (d_track.tolist() + dp.tolist() + pos_tgt.tolist())]
                       + [side])
            del _pl[:-60]
            dbg[side] = {"on": True, "phi": round(ph, 2), "scale": sc,
                         "phi_src": st.get("phi_src", "?"),
                         # dpt=原始跟踪系位移(Q 之前): "往X推臂往Y走"类
                         # 反馈可直接解出残差旋转, 不用再猜
                         "dpt": [round(float(v), 3) for v in d_track],
                         "dp": [round(float(v), 3) for v in dp],
                         "tgt": [round(float(v), 3) for v in pos_tgt]}
        self._status["ik_dbg"] = dbg

    def _track_phi(self):
        """跟踪系→世界系的偏航偏移 = 机器人底座朝向 - 操作者头朝向。

        握离合瞬间锁定: 此后手柄"往前推"= 机器人前方, 与操作者站在哪个
        方向无关(等价于智元配置里手标的 l/r_transform_matrix, 免标定)。
        头显数据缺失时退化为 0(跟踪系=世界系假设)。
        """
        yu = self._head_yaw_track
        yr = self._base_yaw_world
        if yu is None or yr is None:
            return 0.0
        # 符号经离线双场景推导: 头右转 a → yaw_u=+a 且手柄位移在 F 系已含
        # Rz(-a), 需 Rz(+a) 抵消("朝哪推=机器人前"); 底座转 b 时需 Rz(+b)
        # ("我的前=机器人转后的前")。两场景统一为 phi = yaw_r + yaw_u。
        return float(yr + yu)

    # ---------- 手柄→世界旋转: 推杆实测标定(零假设, 优先于头显推算) ----------

    def axis_calib(self, state: str = "start"):
        """推杆实测标定(peak-hold 版): start 后开 12s 窗口, VR 线程逐帧记录
        每只手相对起点的**水平位移峰值**, 窗口到期自动结算 → 结果进
        vr_teleop_status.axis_calib。用户推出去、顿一下、收回即可。

        为什么不用 start/done 两次调用取端点差: 实测单任务队列常被 10Hz
        relay 轮询占满, done 迟到 15~40s, 用户早已放手 → 两次标定都读到
        ~0 位移被拒。峰值采样对"放手早"和"HTTP 迟到"都免疫。

        数学: 管线 dp = Rz(phi)·P·d_track, 要求水平前推映射到机器人前方
        (yaw_r) → phi = yaw_r − atan2(−dx, −dz)。对客户端坐标系零假设
        (P 是代码里固定的), 头显数据不参与; 左右独立解出后取平均
        (房间偏置共模保留, 双手推杆弧线差模抵消, 实测 ±14° → <1°)。
        done=立即结算; clear=清除标定回到头显推算; result=读上次结果。
        """
        if state == "clear":
            self._phi_fixed = {}
            self._calib = None
            self._status["axis_calib"] = {"state": "cleared"}
            return {"status": "cleared"}

        if state == "start":
            p0 = {s: v.copy() for s, v in
                  getattr(self, "_last_ctrl_pos", {}).items() if v is not None}
            if "left" not in p0 or "right" not in p0:
                return {"status": "error",
                        "reason": "手柄数据未就绪(头显未连接或无跟踪)",
                        "have": sorted(p0.keys())}
            self._status["axis_calib"] = {"state": "recording"}
            self._calib = {"p0": p0, "peak": {},
                           "yaw_r0": self._base_yaw_world,
                           "t_end": time.time() + 12.0,
                           "finalize_now": False}
            return {"status": "recording", "window_s": 12.0,
                    "hint": "双手柄水平向前推~30cm顿一下再收回; 12s后自动结算"}

        if state == "done":
            cal = getattr(self, "_calib", None)
            if cal:
                # 只置旗, 结算由 VR 线程执行(标定状态单线程读写, 免竞态)
                cal["finalize_now"] = True
                return {"status": "finalizing",
                        "hint": "下一帧结算, 读 vr_teleop_status.axis_calib"}
            return self._status.get(
                "axis_calib", {"status": "error", "reason": "未先调 start"})

        if state == "result":
            return self._status.get("axis_calib", {"status": "no_data"})

        return {"status": "error", "reason": f"未知 state: {state}"}

    def vr_push_calib(self, state: str = "start", side: str = "left"):
        """**三推杆标定**: 解出完整旋转矩阵 R(跟踪系 → 机器人世界系)。

        用法: start → 用**同一只手**依次"前推、左推、上抬"各 ~30cm
        (每次推完停 1 秒) → done → 结算并存盘。

        为什么需要: 管线里 dp = Q·scale·d_track, Q 原来是手写的
        `Rz(phi)·P`, 其中 P 假设客户端是标准 OpenXR(前=-Z/上=+Y/右=+X)。
        2026-10-08 实测该假设**是错的** —— 用户"往前推"时手柄在客户端系里
        主要走 +x, 于是被当成"往右", 前后完全反的。与其继续猜轴序, 不如
        让数据说话。

        数学: 三次位移峰值方向 u1,u2,u3(跟踪系) ↔ 期望世界方向
        v1=+x(前), v2=+y(左), v3=+z(上)。令 U=[u1 u2 u3]、V=[v1 v2 v3],
        R = V·Uᵀ, 再用 SVD 投影到最近的正交旋转(去噪)。对客户端轴约定
        **零假设**。
        """
        if state == "clear":
            self._calib_R = None
            self._calib3 = None
            self._status["push_calib"] = {"state": "cleared"}
            return {"status": "cleared"}

        if state == "start":
            self._calib3 = {"side": side, "buf": [],
                            "t_end": time.time() + 180.0}
            self._status["push_calib"] = {"state": "recording", "side": side}
            return {"status": "recording", "side": side, "window_s": 180.0,
                    "hint": "依次: ①向前推 ~30cm 停一下 ②向左推 ~30cm 停一下 "
                            "③向上抬 ~30cm 停一下; 然后调 done"}

        if state == "done":
            out = self._calib3_finalize()
            return out or {"status": "error", "reason": "未先调 start"}

        if state == "result":
            return self._status.get("push_calib", {"status": "no_data"})
        return {"status": "error", "reason": f"未知 state: {state}"}

    def _calib3_sample(self, side, pos, sq=0.0):
        """VR 线程: 记录三推杆标定期间的原始手柄轨迹 + 握把值。

        用**握把**定界而不是"推完停一下": 客户端在手柄静止时把位姿标成
        invalid(样本直接消失), 停顿根本采不到; 而且手柄重新被追踪时会位置
        突跳(实测 0.7~1.0m), 靠停顿切分会被突跳污染。握把边沿天然把每次
        "按住→推→松开"分开, 与用户习惯一致。
        """
        c3 = self._calib3
        if not c3 or side != c3["side"]:
            return
        c3["buf"].append((time.time(), np.asarray(pos, float).copy(),
                          float(sq)))
        if len(c3["buf"]) > 8000:
            del c3["buf"][:2000]
        if time.time() > c3["t_end"]:
            self._calib3_finalize()

    def _calib3_finalize(self):
        c3, self._calib3 = self._calib3, None
        if not c3:
            return None
        buf = c3["buf"]
        # 分段: 按**握把**边沿切(见 _calib3_sample 的说明)。每次
        # "按住握把→推→松开"= 一次推杆; 段内取相对段首的最大位移方向。
        # 顺带丢掉单样本突跳(>0.10m/样本 = 手柄重锚, 不是手在动)。
        pushes, cur = [], []
        prev_p = None
        for t, p, sq in buf:
            jump = (prev_p is not None
                    and float(np.linalg.norm(p - prev_p)) > 0.10)
            prev_p = p
            on = sq > 0.5
            if on and (not cur or not jump):
                cur.append((t, p))
            elif on and jump:
                if cur:
                    pushes.append(cur)
                cur = [(t, p)]
            elif cur:
                pushes.append(cur)
                cur = []
        if cur:
            pushes.append(cur)
        exc = []
        for run in pushes:
            if len(run) < 3:
                continue
            p0 = run[0][1]
            ds = np.asarray([p - p0 for _, p in run], float)
            mags = np.linalg.norm(ds, axis=1)
            i = int(np.argmax(mags))
            if float(mags[i]) > 0.12:
                exc.append({"p0": p0, "dir": ds[i].copy(),
                            "mag": float(mags[i]),
                            "t": run[i][0], "n": len(run)})
        exc.sort(key=lambda e: e["t"])
        out = {"state": "done", "side": c3["side"],
               "n_samples": len(buf),
               "n_excursions": len(exc),
               "mag_m": [round(e["mag"], 3) for e in exc[:4]]}
        if buf:
            _arr = np.asarray([p for _, p, _sq in buf], float)
            out["t_span_s"] = round(buf[-1][0] - buf[0][0], 1)
            out["extent_m"] = [round(float(v), 3)
                               for v in (_arr.max(0) - _arr.min(0))]
        if len(exc) < 3:
            out.update(status="rejected",
                       reason=f"只识别到 {len(exc)} 次推杆(需 3 次, 每次>12cm); "
                              f"请前/左/上各推一次, 每次推完停 1 秒")
            self._status["push_calib"] = out
            return out
        U = np.column_stack([e["dir"] / e["mag"] for e in exc[:3]])
        V = np.eye(3)                     # 前=+x, 左=+y, 上=+z
        try:
            Uo = np.linalg.qr(U)[0]       # 正交化测量方向
            A = V @ Uo.T
            P_, S_, Q_ = np.linalg.svd(A)
            d = float(np.linalg.det(P_ @ Q_))
            R = P_ @ np.diag([1.0, 1.0, d]) @ Q_
        except Exception as e:
            out.update(status="error", reason=f"解算失败: {e}")
            self._status["push_calib"] = out
            return out
        res = [round(float(np.degrees(np.arccos(np.clip(
            np.dot(R @ (e["dir"] / e["mag"]), V[:, i]), -1, 1))))
            for i, e in enumerate(exc[:3]))]
        out.update(R=[[round(float(v), 5) for v in row] for row in R],
                   residual_deg=res,
                   axis_ok=[bool(r < 30.0) for r in res])
        if not all(out["axis_ok"]):
            out.update(status="rejected",
                       reason=f"三次推杆方向不自洽(残差 {res}° > 30°), "
                              f"请按 前/左/上 的顺序重推")
        else:
            self._calib_R = R
            out["status"] = "ok"
        self._status["push_calib"] = out
        try:
            cj = {}
            if self._calib_file and os.path.exists(self._calib_file):
                with open(self._calib_file) as f:
                    cj = json.load(f)
            if out["status"] == "ok":
                cj["R"] = out["R"]
            cj["ts"] = time.strftime("%Y-%m-%d %H:%M:%S")
            cj["push3"] = out
            with open(self._calib_file, "w") as f:
                json.dump(cj, f, ensure_ascii=False, indent=1)
        except Exception as e:
            self._status["last_error"] = f"calib3 save: {e}"
        return out

    def _calib_sample(self, side, pos):
        """VR 线程: 标定窗口内记录该手水平位移峰值, 到期/置旗即结算。"""
        cal = self._calib
        if not cal:
            return
        p0 = cal.get("p0", {}).get(side)
        if p0 is None:
            return
        d = np.asarray(pos, float) - p0
        horiz = float(np.hypot(d[0], d[2]))     # 跟踪系水平面 = XZ
        pk = cal["peak"].get(side)
        if pk is None or horiz > pk["horiz"]:
            cal["peak"][side] = {"d": d.copy(), "horiz": horiz}
        if cal.get("finalize_now") or time.time() > cal["t_end"]:
            self._calib_finalize()

    def _calib_finalize(self):
        cal, self._calib = self._calib, None
        if not cal:
            return
        out = {"state": "done", "sides": {}}
        yr = self._base_yaw_world
        if yr is None:
            yr = cal.get("yaw_r0")
        if yr is None:
            out.update(status="error", reason="base_yaw 未就绪")
            self._status["axis_calib"] = out
            return
        ok_any = False
        for s in ("left", "right"):
            pk = cal.get("peak", {}).get(s)
            if pk is None:
                out["sides"][s] = {"ok": False, "reason": "无数据"}
                continue
            d = pk["d"]
            horiz = pk["horiz"]
            vert = float(d[1])                  # 跟踪系上 = +Y
            rec = {"horiz_m": round(horiz, 3), "vert_m": round(vert, 3),
                   "yaw_r_deg": round(float(np.degrees(yr)), 1)}
            if horiz < 0.15 or abs(vert) > 0.25:
                rec.update(ok=False,
                           reason="峰值水平位移<0.15m或垂直漂移>0.25m, 请水平前推")
            else:
                phi = float(yr - np.arctan2(-d[0], -d[2]))
                phi = float(np.arctan2(np.sin(phi), np.cos(phi)))   # 归一
                self._phi_fixed[s] = phi
                rec.update(ok=True,
                           phi_deg=round(float(np.degrees(phi)), 1),
                           phi_head_deg=round(
                               float(np.degrees(self._track_phi())), 1))
                ok_any = True
            out["sides"][s] = rec
        out["status"] = "ok" if ok_any else "rejected"
        oks = [s for s in ("left", "right")
               if out["sides"].get(s, {}).get("ok")]
        if len(oks) == 2:
            avg = float(np.mean([self._phi_fixed[s] for s in oks]))
            for s in oks:
                self._phi_fixed[s] = avg
            out["phi_avg_deg"] = round(float(np.degrees(avg)), 2)
            out["note"] = "两侧取平均; 峰值采样, 推出后收回即可"
        self._status["axis_calib"] = out
        try:
            import json as _json
            if not self._calib_file:
                raise RuntimeError("calib 路径未初始化")
            with open(self._calib_file, "w") as _f:
                _json.dump({"phi": self._phi_fixed,
                            "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
                            "sides": out.get("sides", {})}, _f,
                           ensure_ascii=False, indent=1)
        except Exception as e:
            self._status["last_error"] = f"calib save: {e}"

    def _dls_ik(self, side, q, tgt_pos, tgt_quat=None, iters=8, lam=0.05,
                step_max=0.12, w_rot=0.6, max_total=0.06,
                q_ref=None, w_posture=0.15):
        """阻尼最小二乘(位置 3DOF + 可选朝向): 目标 EE(世界系) → 新关节角。

        `max_total` 是**单次调用**(=单物理帧)允许的关节角变化上限(rad, 各关节
        范数)。没有它时, 手臂在奇异位形附近(如 home 的 T-pose: 手臂完全伸直、
        arm_2..6≈0)会被 DLS 一步甩出 —— step_max=0.12 × iters=8 = 单帧最多
        0.96 rad, 实测"一按离合 q4 瞬间跳 0.71 rad"就是它。目标点本身每帧只
        走 ≤8mm(见 apply_pending 的速率限制), 所以限速不影响跟手, 只把"甩"
        变成几帧内平滑过去。
        """
        q0 = np.asarray(q, float).copy()
        q = q0.copy()
        ch = self._ensure_kin(side)
        ab = self._arm_base_world.get(side)
        if ch is None or ab is None:
            return q                      # 运动学不可用 → 不动, 不乱动
        import arm_kin
        n_chain = len(arm_kin.joint_names(ch))
        if q.size != n_chain:
            # 长度不符说明调用方的关节列表与 FK 链不同步 —— 返回原值(不动),
            # 绝不把解写到别的关节上。
            self._status["last_error"] = (
                f"ik[{side}] 维度不符: q={q.size} chain={n_chain}")
            return q0
        R_ab, t_ab = ab
        lim = self._kin_lim.get(side)
        tgt_pos = np.asarray(tgt_pos, float)
        for _ in range(iters):
            # 每个关节一行: J[i] = [vx vy vz wx wy wz] (对末端原点精确)
            J = arm_kin.jacobian(ch, q, R_ab, t_ab)          # (n,6)
            p_cur, R_cur = arm_kin.ee_world(ch, q, R_ab, t_ab)
            err_p = tgt_pos - p_cur
            if tgt_quat is not None:
                R_t = self._q2R(tgt_quat)
                Re = R_cur.T @ R_t
                err_r = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0],
                                  Re[1, 0] - Re[0, 1]]) * 0.5
                err = np.concatenate([err_p, w_rot * err_r])
                Jp = J.T                                  # (6,n)
            else:
                err = err_p
                Jp = J[:, :3].T                           # (3,n)
            if float(np.linalg.norm(err)) < 1e-3:
                break
            # 自适应阻尼(Chan 式): 条件数越差阻尼越大, 奇异点附近自动"软下来"
            try:
                sv = np.linalg.svd(Jp, compute_uv=False)
                w = float(sv[-1] / max(sv[0], 1e-9))     # 1=良态, 0=奇异
                lam_eff = lam * (1.0 + 30.0 * max(0.0, 0.05 - w) / 0.05)
            except Exception:
                lam_eff = lam
            Jpinv = Jp.T @ np.linalg.inv(
                Jp @ Jp.T + (lam_eff ** 2) * np.eye(Jp.shape[0]))
            dq = Jpinv @ err
            # 零空间姿态回归: 7 自由度做 3 自由度位置任务, 有 4 维零空间;
            # 只走最小范数步会**在零空间里缓慢漂移**(实测转腕关节 q7 从 17°
            # 缠到 171°、q1 漂到 +29°, 机械臂"绕圈")。加一项把零空间分量
            # 拉回参考位形(接合瞬间的位形), 任务方向不受影响。
            if q_ref is not None:
                N = np.eye(Jp.shape[1]) - Jpinv @ Jp
                dq = dq + N @ (w_posture * (np.asarray(q_ref, float) - q))
            n = float(np.linalg.norm(dq))
            if n > step_max:
                dq = dq * (step_max / n)
            q = q + dq
            if lim is not None:
                # 用**真实限位**留 0.02rad 余量(原来是 20% 余量, 直接把关节
                # 顶死在人造边界上: 实测 q4 停在 59.2°、q6 停在 -72.0° 不动)
                q = np.clip(q, lim[:, 0] + 0.02, lim[:, 1] - 0.02)
        # 单帧总步长闸: 奇异点附近 DLS 会给出巨步, 按范数等比缩回
        d = q - q0
        n = float(np.linalg.norm(d))
        if n > max_total:
            q = q0 + d * (max_total / n)
        return q


    # ---------- 主线程施加（RobotManager.step() 调用） ----------

    def set_suspended(self, flag: bool):
        """挂起/恢复 VR 对手臂与夹爪的写入（自动抓取期间用，避免两边抢控制权）。

        只挡手臂/夹爪/躯干；头部跟踪与视频合成照常。
        """
        self._suspended = bool(flag)

    def consume_menu_request(self) -> bool:
        """取走"左手 **X** 键按下"的请求（一次性，主线程调用）。

        名字保留为 menu_request 是历史原因：最早想用菜单键，但那是 Pico
        系统键、按下去直接退回桌面，改成 X 键（左手 primary_click）。
        """
        if self._menu_req is None:
            return False
        self._menu_req = None
        return True

    def apply_pending(self):
        """把 VR 线程缓存的关节目标施加到 articulation（仅主线程调用）。

        手臂目标**自维护**(累加不自读): VR 会话启动后读关节位置的 tensor
        view 可能被 invalidate(日志刷 getDofPositions 错误且每帧异常),
        自维护目标 + 每侧独立 try 保证控制不被单点异常打断。
        """
        self._viz_tick()            # 主线程做视频提交(线程安全)
        self._update_arm_base()     # 主线程刷新 FK 基准(手臂基座世界位姿)
        with self._lock:
            pending = self._pending
            self._pending = {"grip": {}, "arm": {}, "base": {}, "head": {}, "torso": {}}
        if self._robot.articulation is None:
            return
        if getattr(self, "_suspended", False):
            # 自动抓取进行中: 丢弃手柄目标(否则用户一握握把就把手臂拽回去)。
            # 同时清掉离合锚点, 恢复后从当前位重新锚定。
            self._ik_state = {}
            self._ik_smooth = {}
            return
        arti = self._robot.articulation
        names_all = list(arti.dof_names)

        # 全量写入路径: 带 joint_indices 的部分写会先读 backend 全量目标,
        # 而 prim 编辑后 backend 已失效 → 必失败。维护一份 50 维目标数组,
        # 各分支只改自己的分量, 最后一次性写入(不需要读)。
        if self._q_all is not None:
            touched = False
            touched = False
            # 底盘:摇杆 → 四轮位置目标积分(全量写路径, 不读 backend)。
            # 位置驱动轮按 ω·dt 逐帧前移目标, 摇杆回中即停。
            if pending.get("base") and self._q_all is not None:
                try:
                    names_b = list(arti.dof_names)
                    vx = float(pending["base"]["vx"])
                    wz = float(pending["base"]["wz"])
                    r = 0.1594
                    tr = 0.585
                    vl = (vx - wz * tr / 2.0) / r
                    vr = (vx + wz * tr / 2.0) / r
                    try:
                        dt = float(self._robot.world.get_physics_dt())
                    except Exception:
                        dt = 1.0 / 60.0
                    if self._wheel_target is None:
                        self._wheel_target = np.zeros(4, dtype=float)
                    self._wheel_target = self._wheel_target + np.asarray(
                        [vl, vl, vr, vr], dtype=float) * dt
                    # 轮位目标要**绕回 [-pi, pi]**: 它按 ω·dt 无限累加, 久了就超出
                    # PhysX 的 setDriveTarget 允许范围 [-2Pi, 2Pi], 会每帧刷
                    # "only supports target angle in range [-2Pi, 2Pi]" 报错。
                    # 轮子是连续转的, 取模不影响位置伺服的等效效果。
                    self._wheel_target = (self._wheel_target + np.pi) % (2 * np.pi) - np.pi
                    for k, (n, ls) in enumerate((
                            ("front_left_wheel_joint", -1.0),
                            ("rear_left_wheel_joint", -1.0),
                            ("front_right_wheel_joint", 1.0),
                            ("rear_right_wheel_joint", 1.0))):
                        if n in names_b:
                            self._q_all[names_b.index(n)] = float(
                                self._wheel_target[k] * ls)
                    touched = True
                except Exception as e:
                    self._log_apply_err(f"base: {e}")
            for side, target in pending["grip"].items():
                nm = self._grip.get(side)
                if nm and nm in names_all:
                    self._q_all[names_all.index(nm)] = float(target)
                    touched = True
            hd = pending.get("head")
            if hd:
                for key, val in hd.items():
                    nm = getattr(self, "_head_joints", {}).get(key)
                    if nm and nm in names_all:
                        self._q_all[names_all.index(nm)] = float(val)
                        touched = True
            tr = pending.get("torso")
            if tr:
                for key, jn in (("lift", "lift_joint"),
                                ("pitch", "torso_pitch_joint")):
                    if jn in names_all:
                        self._q_all[names_all.index(jn)] = float(tr[key])
                        touched = True
            if pending.get("arm_home") and self._jac_table is not None:
                hq = np.asarray(self._jac_table.get("home_q", []), float)
                if hq.size == len(names_all):
                    for side in ("left", "right"):
                        jn = self._arms.get(side, [])
                        if jn:
                            idxs = [names_all.index(n) for n in jn]
                            self._arm_target[side] = hq[idxs].copy()
                            self._q_all[idxs] = hq[idxs].astype(np.float32)
                    touched = True
            # 零空间姿态参考: 接合瞬间记下位形, 松手清掉(见 _dls_ik 的 q_ref)
            _ikp = pending.get("ik") or {}
            for _s in ("left", "right"):
                if _s in _ikp:
                    if self._posture_ref.get(_s) is None:
                        _qq = self._arm_q_now(_s, names_all)
                        if _qq is not None:
                            self._posture_ref[_s] = _qq
                else:
                    self._posture_ref.pop(_s, None)
            for side, tgt in _ikp.items():
                try:
                    jn = self._arms.get(side, [])
                    if not jn:
                        continue
                    idxs = [names_all.index(n) for n in jn]
                    # IK 起点**必须用实测关节角**: 用自维护的 _arm_target 时,
                    # 只要别的东西动过臂(HTTP 指令 / A,X 回 home / 限位截断 /
                    # 重力), 这份状态就和真臂脱节, 下一帧从错误位形出发 → IK
                    # 解到完全不同的分支 → 机械臂"甩到别处"(实测: 目标只降
                    # 6cm, 臂却转到 q1=111° 的另一个解)。
                    q = self._arm_q_now(side, names_all)
                    if q is None or len(q) != len(jn):
                        q = self._arm_target.get(side)
                    if q is None or len(q) != len(jn):
                        q = self._q_all[idxs].astype(float).copy()
                    if isinstance(tgt, dict):
                        _p = np.asarray(tgt["pos"], float)
                        # 目标速率限制(~0.9m/s @60Hz): 1:1 映射下要跟得上正常手速, 又不能让
                        # 手柄突变把臂猛拽出去
                        sm = self._ik_smooth.setdefault(side, _p.copy())
                        _d = _p - sm
                        _nd = float(np.linalg.norm(_d))
                        if _nd > 0.015:
                            _d = _d * (0.015 / _nd)
                        sm = sm + _d
                        self._ik_smooth[side] = sm
                        _tq = tgt.get("quat")
                        q_new = self._dls_ik(
                            side, q, sm,
                            np.asarray(_tq, float) if _tq else None,
                            q_ref=self._posture_ref.get(side))
                    else:
                        q_new = self._dls_ik(side, q, np.asarray(tgt, float),
                                             q_ref=self._posture_ref.get(side))
                    self._arm_target[side] = q_new
                    self._q_all[idxs] = q_new.astype(np.float32)
                    touched = True
                except Exception as e:
                    self._log_apply_err(f"ik[{side}]: {e}")
            for side, arm in pending["arm"].items():
                try:
                    idxs = [names_all.index(n) for n in arm["names"]]
                    base = self._arm_target.get(side)
                    if base is None:
                        base = self._q_all[idxs].astype(float).copy()
                    tgt = base + np.asarray(arm["deltas"], dtype=float)
                    self._arm_target[side] = tgt
                    self._q_all[idxs] = tgt.astype(np.float32)
                    touched = True
                except Exception as e:
                    self._log_apply_err(f"arm[{side}]: {e}")
            if touched:
                try:
                    arti.set_joint_position_targets(
                        self._q_all.reshape(1, -1).astype(np.float32))
                except Exception as e:
                    self._log_apply_err(f"apply_all: {e}")
            return

        # 回退路径(无 IK 表/无 home_q): 逐项带索引写
        # 底盘: 摇杆 → 四轮位置目标积分(按索引写)。**必须在这条回退路径里
        # 也实现**: 原来底盘只写在 _q_all 分支内, 而 _q_all 默认不初始化
        # (VR_FULL_WRITE=1 才建), 导致默认配置下"摇杆推了底盘不动"
        # (2026-09-29 用户实测: pending.base 有值但 wheel_target 恒为 None)。
        if pending.get("base"):
            try:
                vx = float(pending["base"]["vx"])
                wz = float(pending["base"]["wz"])
                r = 0.1594
                tr = 0.585
                vl = (vx - wz * tr / 2.0) / r
                vr = (vx + wz * tr / 2.0) / r
                try:
                    dt = float(self._robot.world.get_physics_dt())
                except Exception:
                    dt = 1.0 / 60.0
                if self._wheel_target is None:
                    self._wheel_target = np.zeros(4, dtype=float)
                    self._wheel_target = self._wheel_target + np.asarray(
                        [vl, vl, vr, vr], dtype=float) * dt
                    # 轮位目标要**绕回 [-pi, pi]**: 它按 ω·dt 无限累加, 久了就超出
                    # PhysX 的 setDriveTarget 允许范围 [-2Pi, 2Pi], 会每帧刷
                    # "only supports target angle in range [-2Pi, 2Pi]" 报错。
                    # 轮子是连续转的, 取模不影响位置伺服的等效效果。
                    self._wheel_target = (self._wheel_target + np.pi) % (2 * np.pi) - np.pi
                for k, (n, ls) in enumerate((
                        ("front_left_wheel_joint", -1.0),
                        ("rear_left_wheel_joint", -1.0),
                        ("front_right_wheel_joint", 1.0),
                        ("rear_right_wheel_joint", 1.0))):
                    if n in names_all:
                        arti.set_joint_position_targets(
                            np.asarray([[float(self._wheel_target[k] * ls)]],
                                       dtype=np.float32),
                            joint_indices=[names_all.index(n)])
            except Exception as e:
                self._log_apply_err(f"base(fallback): {e}")
        # 躯干(右摇杆): 同样必须在回退路径里施加(与底盘同一原因:
        # 原来只写在 _q_all 分支, 而该分支默认不启用 → 右摇杆推了没反应)
        tr_p = pending.get("torso")
        if tr_p:
            for key, jn in (("lift", "lift_joint"),
                            ("pitch", "torso_pitch_joint")):
                if jn not in names_all:
                    continue
                try:
                    arti.set_joint_position_targets(
                        np.asarray([[float(tr_p[key])]], dtype=np.float32),
                        joint_indices=[names_all.index(jn)])
                except Exception as e:
                    self._log_apply_err(f"torso[{key}]: {e}")
        # 回 home(B/Y 或 A/X): 同上, 补到回退路径(否则"按了复位手臂不回位")
        if pending.get("arm_home") and self._jac_table is not None:
            try:
                hq = np.asarray(self._jac_table.get("home_q", []), float)
                if hq.size == len(names_all):
                    for side in ("left", "right"):
                        jn = self._arms.get(side, [])
                        if not jn:
                            continue
                        idxs = [names_all.index(n) for n in jn]
                        self._arm_target[side] = hq[idxs].copy()
                        arti.set_joint_position_targets(
                            hq[idxs].reshape(1, -1).astype(np.float32),
                            joint_indices=idxs)
            except Exception as e:
                self._log_apply_err(f"arm_home(fallback): {e}")
        for side, target in pending["grip"].items():
            try:
                idx = names_all.index(self._grip[side])
                arti.set_joint_position_targets(
                    np.asarray([[target]], dtype=np.float32),
                    joint_indices=[idx])
            except Exception as e:
                self._log_apply_err(f"grip[{side}]: {e}")
        hd = pending.get("head")
        if hd:
            for key, val in hd.items():
                nm = getattr(self, "_head_joints", {}).get(key)
                if not nm:
                    continue
                try:
                    idx = names_all.index(nm)
                    arti.set_joint_position_targets(
                        np.asarray([[float(val)]], dtype=np.float32),
                        joint_indices=[idx])
                except Exception as e:
                    self._log_apply_err(f"head[{key}]: {e}")
        for side, tgt in (pending.get("ik") or {}).items():
            try:
                jn = self._arms.get(side, [])
                if not jn:
                    continue
                idxs = [names_all.index(n) for n in jn]
                # 同上: IK 起点用**实测关节角**(原来是写死的 6 维 T-pose 常量,
                # 手臂链变 7 关节后越界抛异常被吞 → "目标在动、臂不动";
                # 用自维护 _arm_target 则会在臂被别的东西动过后"甩到别的解")
                q = self._arm_q_now(side, names_all)
                if q is None or len(q) != len(jn):
                    q = self._arm_target.get(side)
                if q is None or len(q) != len(jn):
                    continue
                if isinstance(tgt, dict):
                    _tq = tgt.get("quat")
                    q_new = self._dls_ik(
                        side, q, np.asarray(tgt["pos"], float),
                        np.asarray(_tq, float) if _tq else None,
                        q_ref=self._posture_ref.get(side))
                else:
                    q_new = self._dls_ik(side, q, np.asarray(tgt, float),
                                         q_ref=self._posture_ref.get(side))
                self._arm_target[side] = q_new
                arti.set_joint_position_targets(
                    q_new.reshape(1, -1).astype(np.float32),
                    joint_indices=idxs)
            except Exception as e:
                self._log_apply_err(f"ik[{side}]: {e}")
        for side, arm in pending["arm"].items():
            try:
                idxs = [names_all.index(n) for n in arm["names"]]
                base = self._arm_target.get(side)
                if base is None:
                    base = np.asarray(
                        [-1.2, 0.0, 0.0] if side == "left"
                        else [1.2, 0.0, 0.0], dtype=np.float32)
                tgt = base + np.asarray(arm["deltas"], dtype=np.float32)
                self._arm_target[side] = tgt
                arti.set_joint_position_targets(
                    tgt.reshape(1, -1).astype(np.float32),
                    joint_indices=idxs)
            except Exception as e:
                self._log_apply_err(f"arm[{side}]: {e}")

    def _log_apply_err(self, msg):
        """施加失败只提示一次(视图失效会每帧复现,避免刷屏拖垮主循环)。"""
        if not getattr(self, "_apply_err_logged", False):
            self._apply_err_logged = True
            logger.warning(f"施加 VR 目标失败(后续同类错误不再重复): {msg}")
