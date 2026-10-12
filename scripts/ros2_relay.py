#!/usr/bin/env python3
"""ros2_relay.py — ROS2 ↔ RouteManager HTTP 中继。

背景：Isaac Sim 的 Python 是 3.11，ROS Humble 的 rclpy C 扩展只支持
3.10，无法在仿真进程内直接跑 rclpy。本脚本用系统 python3（3.10）
运行，把 ROS2 话题与 RouteManager HTTP 接口互转：

    /humanoid/joint_command (JointState) → POST /execute send_joint_command
    轮询 GET  /execute get_joint_states  → /humanoid/joint_states (JointState)
    /cmd_vel (Twist)                     → POST /execute set_base_velocity

用法：
    # 无需 source ROS（系统已装）；若 import 失败再 source /opt/ros/humble/setup.bash
    python3 scripts/ros2_relay.py
    python3 scripts/ros2_relay.py --no-cmd-vel   # 禁用底盘订阅

可用环境变量：
    ROUTE_MANAGER_URL   默认 http://127.0.0.1:18765
    RELAY_RATE          状态发布频率，默认 50 Hz
"""
import argparse
import json
import os
import threading
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

URL = os.environ.get("ROUTE_MANAGER_URL", "http://127.0.0.1:18765")
RATE = float(os.environ.get("RELAY_RATE", "10.0"))  # 服务器单任务队列+VR 渲染慢, 20Hz 仍挤爆


def http_action(action: str, parameters: dict = None, timeout: float = 5.0):
    payload = json.dumps({"action": action,
                          "parameters": parameters or {}}).encode()
    req = urllib.request.Request(
        URL + "/execute", data=payload,
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        out = json.loads(resp.read().decode())
    if isinstance(out, dict) and out.get("detail"):
        raise RuntimeError(str(out["detail"])[:300])
    return out.get("data")


class Relay(Node):
    def __init__(self, with_base: bool = True):
        super().__init__("route_manager_relay")
        self.joint_names = http_action("get_joint_names")["dof_names"]
        self.get_logger().info(f"RouteManager 关节 {len(self.joint_names)} 个")
        self._cmd = {n: 0.0 for n in self.joint_names}
        self._lock = threading.Lock()

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(
            JointState, "/humanoid/joint_command", self._on_command, qos)
        self.pub = self.create_publisher(JointState,
                                         "/humanoid/joint_states", qos)
        self.timer = self.create_timer(1.0 / RATE, self._publish_state)

        topics = ["/humanoid/joint_command", "/humanoid/joint_states"]

        # --- 底盘 cmd_vel ---
        self._with_base = with_base
        if with_base:
            # last-message-wins 缓存，定时器按 RELAY_RATE 节流转发给 HTTP。
            # RouteManager 内部 0.5s 无更新自动刹停，故停止转发即等于停车。
            self._twist_lock = threading.Lock()
            self._last_twist: Twist | None = None
            self.sub_vel = self.create_subscription(
                Twist, "/cmd_vel", self._on_cmd_vel, qos)
            self.vel_timer = self.create_timer(1.0 / RATE, self._forward_cmd_vel)
            topics.append("/cmd_vel")

        self.get_logger().info(
            f"中继已启动: 订阅话题 {topics} → {URL} @ {RATE}Hz")

    # ---------- 底盘回调 ----------

    def _on_cmd_vel(self, msg: Twist):
        """缓存最新消息，由定时器节流转发（避免高频 HTTP 请求）。"""
        with self._twist_lock:
            self._last_twist = msg

    def _forward_cmd_vel(self):
        """按定时器周期把最新 Twist 发给 RouteManager。"""
        with self._twist_lock:
            msg = self._last_twist
        if msg is None:
            return
        # ROS 标准约定：linear.x 前进、angular.z 左转均为正
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)
        try:
            http_action("set_base_velocity", {"vx": vx, "wz": wz})
        except Exception as e:
            self._warn_throttled("vel", f"set_base_velocity 失败: {e}")

    def _on_command(self, msg: JointState):
        """按关节名映射；缺失的关节沿用上次值。"""
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._cmd:
                    self._cmd[name] = float(pos)
            positions = [self._cmd[n] for n in self.joint_names]
        try:
            http_action("send_joint_command", {"positions": positions})
        except Exception as e:
            self._warn_throttled("cmd", f"send_joint_command 失败: {e}")

    def _warn_throttled(self, key, text):
        """同类错误 5s 只报一次(高频轮询下服务器忙时曾刷出 8 万行日志)。"""
        import time as _t
        now = _t.monotonic()
        if not hasattr(self, "_warn_last"):
            self._warn_last = {}
        if now - self._warn_last.get(key, -1e9) > 5.0:
            self.get_logger().warning(text)
            self._warn_last[key] = now

    def _publish_state(self):
        try:
            states = http_action("get_joint_states")
            fb = states.get("feedback") or []
            if len(fb) != len(self.joint_names):
                return
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [float(v) for v in fb]
            self.pub.publish(msg)
        except Exception as e:
            self._warn_throttled("state", f"get_joint_states 失败: {e}")


def main():
    ap = argparse.ArgumentParser(description="ROS2 ↔ RouteManager HTTP 中继")
    ap.add_argument("--cmd-vel", dest="cmd_vel", action="store_true",
                    default=True, help="订阅 /cmd_vel 转发到底盘 (默认)")
    ap.add_argument("--no-cmd-vel", dest="cmd_vel", action="store_false",
                    help="禁用底盘 /cmd_vel 订阅")
    args = ap.parse_args()

    rclpy.init()
    node = Relay(with_base=args.cmd_vel)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
