"""udp_teleop.py — XRoboToolkit 遥操作端 → RouteManager 仿真的 UDP 关节目标入口。

md（VR 机械臂遥操作复刻指南）那套链路：

    PICO 头显 ──> PC Service(:63901) ──> xrobotoolkit_sdk
                                            │
                              placo IK（xr-robotics env，独立进程）
                                            │  UDP {"names":[...], "positions":[...]}
                                            ▼
                                    UdpTargetLink  ← 本文件
                                            │  后台线程只存"最新一帧"
                                            ▼
                              RobotManager.step()（主线程施加到 articulation）

**线程约定与 `ros2_bridge/` 完全一致**：接收线程只更新缓冲，物理操作全部在主线程
`step()` 里做 —— 这是这个项目里唯一被验证过线程安全的写法。本模块因此顶层**不 import
任何 Isaac 模块**，可被 `standalone_server.py --demo` 之类的假执行器直接使用。

与 md 里 `run_isaac_udp_target.py` 的差别（都是有意为之）：
  · 那边自己起 SimulationApp + 开 flat USD；这边是 RouteManager 的仿真，所以只做
    收包与缓冲，物理施加交给 `RobotManager`。
  · 那边用 `apply_action(ArticulationAction(...))`；这边沿用 RouteManager 已有的
    `set_joint_position_targets`（同一个"驱动目标"语义，见 `_apply_joint_positions`）。
  · 丢包/超时策略：md 那边收到一帧就一直保持；这边默认也是保持，但把"帧龄"暴露在
    status 里，调用方能据此判断遥操作端是不是挂了。
"""
import collections
import json
import logging
import socket
import threading
import time

logger = logging.getLogger(__name__)

#: 收包缓冲大小。md 那套一帧 50 个关节 ≈ 1.5 KB，64 KB 留足余量。
_RECV_BUF = 1 << 16


class UdpTargetLink:
    """UDP 关节目标接收端（后台线程只存最新帧，主线程取用）。"""

    def __init__(self, port: int = 63900, host: str = "127.0.0.1",
                 status_port: int = 0, status_hz: float = 10.0):
        self.port = int(port)
        self.host = str(host)
        #: 状态回发端口（0 = 不回发）。md 那套的契约桥监听 63901，
        #: 收到 {names, positions, velocities, base_pose} 用来做闭环校验。
        self.status_port = int(status_port or 0)
        self._status_period = 1.0 / max(1.0, float(status_hz))
        self._sock = None
        self._status_sock = None
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        #: 最新一帧指令 (names, positions, 收到时刻)。主线程取用。
        self._latest = None
        #: 遥操作端发来的**事件**（按键触发，如左手 X 键 → 自动抓取）。
        #: 与关节帧同一条 UDP 通道，靠有没有 "event" 字段区分。
        #: 用队列不用单槽：按键是离散事件，连按两下不该丢第二下。
        self._events = collections.deque(maxlen=16)
        self._status_out = None
        self._last_status_t = 0.0
        self._stats = {"frames": 0, "dropped": 0, "errors": 0,
                       "events": 0, "last_event": None,
                       "last_error": None, "started_at": None,
                       "last_rx": None}

    # ---------------- 生命周期 ----------------

    def start(self):
        if self._sock is not None:
            return self
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.setblocking(False)
        self._sock = sock
        if self.status_port:
            self._status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._recv_loop, name="udp-teleop", daemon=True)
        self._thread.start()
        self._stats["started_at"] = time.time()
        logger.info(f"UDP 遥操作入口已监听 udp://{self.host}:{self.port}"
                    + (f"，状态回发 → :{self.status_port}"
                       if self.status_port else ""))
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        for s in (self._sock, self._status_sock):
            try:
                if s is not None:
                    s.close()
            except Exception:
                pass
        self._sock = None
        self._status_sock = None
        with self._lock:
            self._latest = None
        logger.info("UDP 遥操作入口已停止")

    def is_initialized(self) -> bool:
        return self._sock is not None

    # ---------------- 接收线程 ----------------

    def _recv_loop(self):
        """只做两件事：drain 到只剩最新一帧、解析后存进缓冲。**不碰物理**。"""
        while not self._stop.is_set():
            latest = None
            # 一次把积压的包全部读掉：遥操作端 100 Hz，仿真主循环 60 Hz，
            # 每帧都会积压 1~2 包。只保留最新的，旧包没有价值。
            while True:
                try:
                    latest = self._sock.recvfrom(_RECV_BUF)[0]
                except BlockingIOError:
                    break
                except OSError as e:
                    if not self._stop.is_set():
                        self._stats["errors"] += 1
                        self._stats["last_error"] = f"recv: {e}"
                    break
            if latest is not None:
                self._ingest(latest)
            else:
                # 没包就睡一下，别空转烧核（非阻塞 socket 没有超时可用）
                self._stop.wait(0.002)

    def _ingest(self, raw: bytes):
        try:
            payload = json.loads(raw)
            # 事件包（按键触发）与关节帧共用一条通道：有 "event" 就是事件。
            if isinstance(payload, dict) and "event" in payload:
                self._events.append((dict(payload), time.time()))
                self._stats["events"] += 1
                self._stats["last_event"] = payload.get("event")
                self._stats["last_rx"] = time.time()
                return
            names = [str(n) for n in payload["names"]]
            positions = [float(v) for v in payload["positions"]]
            if len(names) != len(positions):
                raise ValueError(f"names/positions 长度不符: "
                                 f"{len(names)} vs {len(positions)}")
            if not all(p == p and abs(p) != float("inf") for p in positions):
                raise ValueError("positions 含非有限值")
        except Exception as e:
            self._stats["dropped"] += 1
            self._stats["last_error"] = f"parse: {e}"
            return
        with self._lock:
            self._latest = (names, positions, time.time())
        self._stats["frames"] += 1
        self._stats["last_rx"] = time.time()

    # ---------------- 主线程接口 ----------------

    def get_latest(self):
        """取最新一帧 (names, positions, 收到时刻)；没收到过返回 None。

        **不消费**（不像 ROS 桥的 get_latest_command 取走即清）：位置目标每帧
        都要重写一次才不会被别的控制路径覆盖，而且丢包时保持上一帧目标正是
        我们要的语义（遥操作端挂了，机器人停在原地而不是弹回零位）。
        """
        with self._lock:
            return self._latest

    def frame_age(self):
        """最新一帧的年龄（秒）；从未收到过返回 None。"""
        latest = self.get_latest()
        return None if latest is None else time.time() - latest[2]

    def take_events(self):
        """取走并清空待处理事件（主线程调用）。返回 [(payload, 收到时刻), ...]。"""
        with self._lock:
            out = list(self._events)
            self._events.clear()
        return out

    def record_state(self, names, positions, velocities=None, base_pose=None):
        """主线程：把仿真实测状态回发给遥操作端（限频，失败静默）。"""
        if self._status_sock is None:
            return
        now = time.time()
        if now - self._last_status_t < self._status_period:
            return
        self._last_status_t = now
        try:
            msg = {"names": list(names),
                   "positions": [float(v) for v in positions]}
            if velocities is not None:
                msg["velocities"] = [float(v) for v in velocities]
            if base_pose is not None:
                msg["base_pose"] = [float(v) for v in base_pose]
            self._status_sock.sendto(
                json.dumps(msg).encode(), (self.host, self.status_port))
        except OSError:
            pass

    def status(self) -> dict:
        latest = self.get_latest()
        age = self.frame_age()
        st = dict(self._stats)
        st.update({
            "running": self.is_initialized(),
            "port": self.port,
            "status_port": self.status_port or None,
            "have_frame": latest is not None,
            "frame_age_s": None if age is None else round(age, 3),
            "n_joints": len(latest[0]) if latest else 0,
        })
        if st.get("started_at"):
            st["uptime_s"] = round(time.time() - st["started_at"], 1)
        if st.get("last_rx"):
            st["last_rx_age_s"] = round(time.time() - st["last_rx"], 3)
        return st
