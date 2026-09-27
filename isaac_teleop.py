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

logger = logging.getLogger(__name__)


class VRTeleopManager:
    """进程内 VR 遥操作会话管理器。

    由 RobotManager 持有；/execute 通过 start_vr_teleop/stop_vr_teleop/
    vr_teleop_status 三个动作调用。
    """

    def __init__(self, robot):
        self._robot = robot          # RobotManager
        self._session = None
        self._launcher = None
        self._thread = None
        self._stop_evt = threading.Event()
        self._status = {"running": False, "frames": 0, "last_error": None,
                        "last_controller": None}
        self._hands = {}             # side -> {"prev": np.array 或 None}
        self._pending = {"grip": {}, "arm": {}}  # VR 线程→主线程 的目标缓冲
        self._params = {}
        self._grip = {}
        self._arms = {}
        self._lock = threading.Lock()
        self._viz = None            # isaacteleop.viz VizSession(kXr)
        self._head_layer = None     # 头部相机画面 QuadLayer
        self._head_cam = None       # 流送源相机对象
        self._torch = None
        self._frame_i = 0

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
                jn = [n for n in names
                      if n.startswith(side) and "gripper" not in n
                      and any(k in n for k in
                              ("arm_1", "arm_2", "arm_3", "arm_4",
                               "arm_5", "arm_6"))][:6]
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

        # ---- 构建 isaacteleop 会话 ----
        from isaacteleop.cloudxr import CloudXRLauncher
        from isaacteleop.retargeting_engine.deviceio_source_nodes import (
            ControllersSource)
        from isaacteleop.retargeting_engine.interface.output_combiner import (
            OutputCombiner)
        from isaacteleop.teleop_session_manager import (
            TeleopSession, TeleopSessionConfig)

        src = ControllersSource("controllers")
        # 直通管线：不做 IK，原始控制器张量直接进 step() 结果
        pipeline = OutputCombiner({
            "controller_left": src.output(src.LEFT),
            "controller_right": src.output(src.RIGHT),
        })

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
        self._viz = None
        self._head_layer = None
        oxr = None
        try:
            import isaacteleop.viz as viz
            vcfg = viz.VizSessionConfig()
            vcfg.mode = viz.DisplayMode.kXr
            vcfg.app_name = "RouteManagerVR"
            vcfg.xr_system_wait_seconds = 60   # 等头显 CONNECT,避免无限阻塞
            self._viz = viz.VizSession.create(vcfg)
            lcfg = viz.QuadLayerConfig()
            lcfg.name = "head_cam"
            lcfg.resolution = viz.Resolution(1920, 1200)
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

        # 头部相机作为流送源
        self._head_cam = None
        try:
            for k, v in self._robot._cameras.items():
                if "head_zed_x" in k:
                    self._head_cam = v
                    break
        except Exception:
            pass
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
            self._launcher.__exit__(None, None, None)
            self._launcher = None
            raise

        self._stop_evt.clear()
        self._hands = {s: {"prev": None} for s in ("left", "right")}
        with self._lock:
            self._pending = {"grip": {}, "arm": {}}
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

        # ---- 视频帧提交:头相机 RGBA 上 GPU → QuadLayer → XR 合成。
        # 隔帧提交(~25fps),render 每帧驱动 XR 帧循环。 ----
        if self._viz is not None:
            try:
                if (self._head_layer is not None and self._head_cam is not None
                        and self._torch is not None):
                    self._frame_i += 1
                    if self._frame_i % 2 == 0:
                        rgb = self._head_cam.get_rgba()
                        if rgb is not None and rgb.size:
                            t = self._torch.from_numpy(
                                np.ascontiguousarray(rgb)).cuda()
                            self._head_layer.submit(t)
                self._viz.render()
            except Exception as e:
                self._status["last_error"] = f"viz: {e}"
        result = session.step()
        self._status["frames"] += 1

        from isaacteleop.retargeting_engine.tensor_types import (
            ControllerInputIndex as CII)

        ctrl_view = {}
        pending = {"grip": {}, "arm": {}}

        for side, key in (("left", "controller_left"),
                          ("right", "controller_right")):
            grp = result.get(key)
            ctrl_view[side] = None
            if grp is None:
                self._hands[side]["prev"] = None
                continue
            try:
                if grp.is_none():
                    self._hands[side]["prev"] = None
                    continue
                pos = np.asarray(grp[CII.GRIP_POSITION], dtype=np.float64)
                trig = max(float(grp[CII.TRIGGER_VALUE]),
                           float(grp[CII.SQUEEZE_VALUE]))
                valid = bool(grp[CII.GRIP_IS_VALID])
            except Exception:
                self._hands[side]["prev"] = None
                continue
            if not valid or pos.size != 3:
                self._hands[side]["prev"] = None
                continue
            ctrl_view[side] = {"pos": pos, "trigger": trig}

            # 夹爪：扳机 0~1 → leader 0~grip_open
            pending["grip"][side] = min(max(trig, 0.0), 1.0) \
                * self._params["grip_open"]

        # 手臂：控制器位移 → 关节增量（相对上次位置）
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
            # 位移 xyz 依次驱动前三个臂关节（其余保持不动）
            deltas = []
            for i in range(len(jn)):
                raw = d[i] if i < 3 else 0.0
                raw *= self._params["workspace_scale"]
                deltas.append(max(-self._params["max_step"],
                                  min(self._params["max_step"], raw)))
            pending["arm"][side] = {"names": jn, "deltas": deltas}

        with self._lock:
            self._pending = pending
        self._status["last_controller"] = {
            s: (None if v is None else
                {"pos": v["pos"].round(3).tolist(),
                 "trigger": round(v["trigger"], 2)})
            for s, v in ctrl_view.items()}

    # ---------- 主线程施加（RobotManager.step() 调用） ----------

    def apply_pending(self):
        """把 VR 线程缓存的关节目标施加到 articulation（仅主线程调用）。"""
        with self._lock:
            pending = self._pending
            self._pending = {"grip": {}, "arm": {}}
        if self._robot.articulation is None:
            return
        arti = self._robot.articulation
        names_all = list(arti.dof_names)
        try:
            for side, target in pending["grip"].items():
                idx = names_all.index(self._grip[side])
                arti.set_joint_position_targets(
                    np.asarray([[target]], dtype=np.float32),
                    joint_indices=[idx])
            for side, arm in pending["arm"].items():
                cur = np.asarray(arti.get_joint_positions()).reshape(-1)
                idxs = [names_all.index(n) for n in arm["names"]]
                tgt = cur[idxs] + np.asarray(arm["deltas"], dtype=np.float32)
                arti.set_joint_position_targets(
                    tgt.reshape(1, -1).astype(np.float32),
                    joint_indices=idxs)
        except Exception as e:
            logger.warning(f"施加 VR 目标失败: {e}")
