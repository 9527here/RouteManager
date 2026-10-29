#!/usr/bin/env python3
"""ros2_sim_contract.py — `/<robot>/sim/<instance>/...` 契约中继。

**必须用系统 python3.10 跑**（Isaac 进程是 py3.12，Humble 的 rclpy 是 py3.10，
不能同进程）：

    source /opt/ros/humble/setup.bash
    python3 scripts/ros2_sim_contract.py

或者用封装好的 `scripts/start_ros2_contract.sh`。

## 它做什么

    Isaac (HTTP :18765)                本进程 (rclpy, py3.10)
      get_sim_state  ────────────────>  按关节组切分 → feedback/joint_states
                                        夹爪状态 → gripper_joint_state / gripper_status
                                        flange/tcp → PoseStamped
      move_j / send_joint_command <──  control/joint_states、control/move_j
      set_gripper_position      <──  control/gripper

相机（`<cam>/color/image_raw` + `camera_info`）**不经过这里** —— 由 Isaac 内的
OmniGraph 原生发布（见 `robot_manager.setup_ros2_cameras`），避免每帧大图跨进程。
`--camera-info` 只是兜底：万一组件没自动发内参，用本进程补发。

## 线程模型

rclpy 的 spin 只跑订阅回调，回调里**只写缓存、不做 HTTP**（HTTP 会阻塞 spin，
把 move_j 和夹爪指令一起拖慢）。真正的 HTTP 在两个守护线程里：

  · `_state_loop` —— 按 `--state-hz` 轮询 `get_sim_state`，拿不到就丢这一帧
  · `_cmd_loop`   —— 按 `--command-hz` 把缓存里的最新指令转发出去（409 重试）

指令一律 **latest-wins**：积压的旧目标在几秒后突然执行比丢指令更危险。
"""
import argparse
import json
import threading
import time
import urllib.error
import urllib.request

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Float32, Float32MultiArray
from trajectory_msgs.msg import JointTrajectory

URL = "http://127.0.0.1:18765"
_HTTP_LOCK = threading.Lock()
_last_warn = {}


def warn_throttled(key: str, msg: str, period: float = 5.0):
    now = time.monotonic()
    if now - _last_warn.get(key, 0.0) >= period:
        _last_warn[key] = now
        print(f"[contract] {msg}", flush=True)


def http_action(action: str, parameters: dict = None, timeout: float = 1.0,
                tries: int = 1, retry_sleep: float = 0.05):
    """POST /execute。409（服务器忙）按 tries 重试，其它错误直接返回 None。

    状态轮询用 `tries=1`（丢帧无所谓，下一周期再来）；控制指令用 `tries>1`。
    """
    body = json.dumps({"action": action,
                       "parameters": parameters or {}}).encode()
    last = None
    for k in range(max(1, tries)):
        try:
            with _HTTP_LOCK:
                req = urllib.request.Request(
                    URL + "/execute", data=body,
                    headers={"Content-Type": "application/json"})
                with urllib.request.urlopen(req, timeout=timeout) as resp:
                    return json.loads(resp.read())
        except urllib.error.HTTPError as e:
            last = e
            if e.code != 409:
                warn_throttled(f"{action}:{e.code}",
                               f"{action} HTTP {e.code}: {e.reason}")
                return None
            time.sleep(retry_sleep * (1 + k * 0.5))
        except Exception as e:      # 连接失败 / 超时
            last = e
            time.sleep(retry_sleep * (1 + k * 0.5))
    warn_throttled(f"{action}:fail", f"{action} 失败: {last}")
    return None


class SimContract(Node):
    def __init__(self, args):
        super().__init__("iasscsim_robot_sim_contract")
        self.args = args
        self.meta = None
        self.pub = {}
        self.pending = {}
        self.lock = threading.Lock()
        self.stop = threading.Event()

        self._wait_for_meta()
        base = self.meta.get("base_topic", f"/{args.robot}/{args.role}")
        self.base = base
        self._init_publishers()
        self._init_subscriptions()

        self.state_thread = threading.Thread(target=self._state_loop,
                                             daemon=True)
        self.cmd_thread = threading.Thread(target=self._cmd_loop, daemon=True)
        self.state_thread.start()
        self.cmd_thread.start()
        print(f"[contract] 契约已启动: {base}", flush=True)

    # ---------------- 启动 ----------------

    def _wait_for_meta(self):
        deadline = time.monotonic() + self.args.meta_timeout
        while time.monotonic() < deadline:
            r = http_action("get_contract_meta", timeout=5.0, tries=1)
            d = (r or {}).get("data") if r else None
            if d and not d.get("errors"):
                self.meta = d
                return
            warn_throttled("meta", "等 get_contract_meta（机器人还没加载？）")
            time.sleep(1.0)
        raise SystemExit("拿不到 get_contract_meta —— 先 load_scene + load_robot")

    def _qos(self, depth=10, reliable=True):
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=depth,
            reliability=(ReliabilityPolicy.RELIABLE if reliable
                         else ReliabilityPolicy.BEST_EFFORT),
            durability=DurabilityPolicy.VOLATILE)

    def _init_publishers(self):
        q = self._qos(10)
        q1 = self._qos(1)
        for group in self.meta["groups"]:
            self.pub[f"{group}/fb"] = self.create_publisher(
                JointState, f"{self.base}/{group}/feedback/joint_states", q)
        for side, s in self.meta["sides"].items():
            ns = f"{self.base}/{side}_arm"
            self.pub[f"{side}/gjs"] = self.create_publisher(
                JointState, f"{ns}/feedback/gripper_joint_state", q)
            self.pub[f"{side}/gst"] = self.create_publisher(
                Float32MultiArray, f"{ns}/feedback/gripper_status", q)
            self.pub[f"{side}/flange"] = self.create_publisher(
                PoseStamped, f"{ns}/feedback/flange_pose", q)
            self.pub[f"{side}/tcp"] = self.create_publisher(
                PoseStamped, f"{ns}/feedback/tcp_pose", q)
        if self.args.camera_info:
            for role, c in (self.meta.get("cameras") or {}).items():
                self.pub[f"cam/{role}"] = self.create_publisher(
                    CameraInfo, c["info_topic"], q1)
        self._q1 = q1

    def _init_subscriptions(self):
        # 命令用 depth=1 + latest-wins：积压的旧目标晚几秒执行更危险
        q1 = self._qos(1)
        for group in self.meta["groups"]:
            self.create_subscription(
                JointState, f"{self.base}/{group}/control/joint_states",
                lambda msg, g=group: self._on_joint(g, msg), q1)
        for side in self.meta["sides"]:
            ns = f"{self.base}/{side}_arm"
            self.create_subscription(
                JointTrajectory, f"{ns}/control/move_j",
                lambda msg, s=side: self._on_move_j(s, msg), q1)
            self.create_subscription(
                Float32, f"{ns}/control/gripper",
                lambda msg, s=side: self._on_gripper(s, msg), q1)
            self.pending[side] = {"move": None, "gripper": None}
        for group in self.meta["groups"]:
            self.pending.setdefault(group, {})
            self.pending[group]["joint"] = None

    # ---------------- 订阅回调（只写缓存） ----------------

    def _on_joint(self, group, msg: JointState):
        names = list(msg.name) or list(self.meta["groups"].get(group, []))
        if len(names) != len(msg.position):
            warn_throttled(f"joint:{group}",
                           f"{group} control/joint_states 名/值长度不符")
            return
        with self.lock:
            self.pending[group]["joint"] = {"positions": list(msg.position),
                                            "joint_names": names}

    def _on_move_j(self, side, msg: JointTrajectory):
        if not msg.points:
            return
        if len(msg.points) > 1:
            warn_throttled(f"move_j:{side}",
                           f"{side} move_j 收到多点，只取最后一点")
        pt = msg.points[-1]
        names = list(msg.joint_names) or list(
            self.meta["sides"][side]["arm_joints"])
        if len(names) != len(pt.positions):
            warn_throttled(f"move_j:{side}",
                           f"{side} move_j 名/值长度不符")
            return
        dur = (pt.time_from_start.sec +
               pt.time_from_start.nanosec * 1e-9)
        if dur <= 0:
            dur = self.args.default_move_duration
        with self.lock:
            self.pending[side]["move"] = {"positions": list(pt.positions),
                                          "joint_names": names,
                                          "duration": float(dur)}

    def _on_gripper(self, side, msg: Float32):
        with self.lock:
            self.pending[side]["gripper"] = float(msg.data)

    # ---------------- 控制线程 ----------------

    def _cmd_loop(self):
        period = 1.0 / max(1e-3, self.args.command_hz)
        while not self.stop.is_set():
            with self.lock:
                snap = {k: dict(v) for k, v in self.pending.items()}
                for k in self.pending:
                    for kk in self.pending[k]:
                        self.pending[k][kk] = None
            for side in self.meta["sides"]:
                mv = snap.get(side, {}).get("move")
                if mv:
                    http_action("move_j",
                                {"side": side,
                                 "positions": mv["positions"],
                                 "joint_names": mv["joint_names"],
                                 "duration": mv["duration"]},
                                timeout=3.0, tries=10)
                gp = snap.get(side, {}).get("gripper")
                if gp is not None:
                    http_action("set_gripper_position",
                                {"side": side,
                                 "position": int(round(gp))},
                                timeout=2.0, tries=10)
            for group in self.meta["groups"]:
                j = snap.get(group, {}).get("joint")
                if not j:
                    continue
                http_action("send_joint_command",
                            {"positions": j["positions"],
                             "joint_names": j["joint_names"]},
                            timeout=2.0, tries=10)
            self.stop.wait(period)

    # ---------------- 状态线程 ----------------

    def _state_loop(self):
        period = 1.0 / max(1e-3, self.args.state_hz)
        idle = max(0.0, self.args.state_idle)
        backoff = 1.0
        while not self.stop.is_set():
            t0 = time.monotonic()
            r = http_action("get_sim_state",
                            {"ref_link": self.args.ref_link,
                             "with_velocity": True},
                            timeout=0.7, tries=1)
            st = (r or {}).get("data") if r else None
            if st and st.get("joint_names"):
                backoff = 1.0
                try:
                    self._publish_all(st)
                except Exception as e:
                    warn_throttled("publish", f"发布失败: {e!r}")
            else:
                # 拿不到（409/超时）就退让，别把单任务队列一直占着 ——
                # 否则外部脚本的 HTTP 调用会持续 409。
                backoff = min(backoff * 2.0, 8.0)
            elapsed = time.monotonic() - t0
            # **按占空比睡**，而不是固定周期。RouteManager 是单任务队列，
            # 一次 get_sim_state 在带 NuRec 背景 + 4 路相机时要 ~150ms；
            # 若按 10Hz 的 100ms 周期跑，循环会 100% 占用队列，外部脚本
            # （pick_object / 用户自己的 HTTP 调用）全部 409。
            # 目标：本进程最多占 `duty` 的时间，其余留给别人。
            duty = max(0.05, min(0.95, self.args.state_duty))
            wait = max(elapsed * (1.0 / duty - 1.0), idle)
            wait = min(wait, 1.0)          # 别睡太久，掉线要能尽快恢复
            if st is None:
                wait *= backoff
            self.stop.wait(wait)

    def _stamp(self, sim_time):
        from builtin_interfaces.msg import Time
        t = Time()
        t.sec = int(sim_time)
        t.nanosec = int((sim_time - int(sim_time)) * 1e9)
        return t

    def _publish_all(self, st):
        names = st["joint_names"]
        pos = st["positions"]
        vel = st.get("velocities") or []
        cmd = st.get("cmd_positions") or []
        idx = {n: i for i, n in enumerate(names)}
        stamp = self._stamp(st.get("sim_time", 0.0))

        for group, joints in self.meta["groups"].items():
            have = [j for j in joints if j in idx]
            if not have or f"{group}/fb" not in self.pub:
                continue
            ii = [idx[j] for j in have]
            m = JointState()
            m.header.stamp = stamp
            m.name = have
            m.position = [float(pos[i]) for i in ii]
            if vel:
                m.velocity = [float(vel[i]) for i in ii]
            self.pub[f"{group}/fb"].publish(m)


        for side, s in st.get("sides", {}).items():
            ns_g = f"{side}/gjs"
            if ns_g in self.pub and s.get("gripper"):
                g = s["gripper"]
                m = JointState()
                m.header.stamp = stamp
                m.name = [self.meta["sides"][side]["gripper_joint"]]
                m.position = [float(g.get("angle_rad", 0.0))]
                self.pub[ns_g].publish(m)

                a = Float32MultiArray()
                a.data = [float(g.get("position", 0.0)),
                          float(g.get("opening_mm", 0.0)),
                          float(g.get("fraction", 0.0)),
                          float(g.get("angle_rad", 0.0))]
                d = a.data
                a.layout.dim = []
                self.pub[f"{side}/gst"].publish(a)

            for kind, key in (("flange", "flange"), ("tcp", "tcp")):
                pose = s.get(key)
                if not pose:
                    continue
                p = PoseStamped()
                p.header.stamp = stamp
                p.header.frame_id = self.args.ref_link
                p.pose.position.x, p.pose.position.y, p.pose.position.z = \
                    pose["position"]
                q = pose["orientation"]        # wxyz
                p.pose.orientation.x, p.pose.orientation.y, \
                    p.pose.orientation.z, p.pose.orientation.w = q[1], q[2], q[3], q[0]
                self.pub[f"{side}/{kind}"].publish(p)

        if self.args.camera_info:
            for role, c in (self.meta.get("cameras") or {}).items():
                if f"cam/{role}" not in self.pub or "fx" not in c:
                    continue
                m = CameraInfo()
                m.header.stamp = stamp
                m.header.frame_id = c["frame_id"]
                m.width = c["width"]
                m.height = c["height"]
                m.distortion_model = "plumb_bob"
                m.d = [0.0] * 5
                fx, fy, cx, cy = c["fx"], c["fy"], c["cx"], c["cy"]
                m.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
                m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                m.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
                self.pub[f"cam/{role}"].publish(m)

    def shutdown(self):
        self.stop.set()
        for t in (self.state_thread, self.cmd_thread):
            t.join(timeout=1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="iasscsim_robot")
    ap.add_argument("--role", default="sim")
    ap.add_argument("--state-hz", type=float, default=10.0)
    ap.add_argument("--command-hz", type=float, default=20.0)
    ap.add_argument("--state-idle", type=float, default=0.03,
                    help="每轮状态轮询后至少留出的空档(秒)")
    ap.add_argument("--state-duty", type=float, default=0.4,
                    help="本进程最多占用 RouteManager 单任务队列的比例(0~1)，其余留给外部 HTTP 调用")
    ap.add_argument("--default-move-duration", type=float, default=1.0)
    ap.add_argument("--ref-link", default="base_link",
                    help="flange/tcp 参考系；'world' 表示世界系")
    ap.add_argument("--camera-info", action="store_true",
                    help="兜底：由本进程发 CameraInfo（OmniGraph 没发时用）")
    ap.add_argument("--meta-timeout", type=float, default=120.0)
    args = ap.parse_args()

    rclpy.init()
    node = None
    try:
        node = SimContract(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()