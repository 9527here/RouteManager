#!/usr/bin/env python3
"""ros2_relay.py — ROS2 ↔ RouteManager HTTP 中继。

背景：Isaac Sim 的 Python 是 3.11，ROS Humble 的 rclpy C 扩展只支持
3.10，无法在仿真进程内直接跑 rclpy。本脚本用系统 python3（3.10）
运行，把 ROS2 话题与 RouteManager HTTP 接口互转：

    /humanoid/joint_command (JointState) → POST /execute send_joint_command
    轮询 GET  /execute get_joint_states  → /humanoid/joint_states (JointState)

用法：
    # 无需 source ROS（系统已装）；若 import 失败再 source /opt/ros/humble/setup.bash
    python3 scripts/ros2_relay.py

可用环境变量：
    ROUTE_MANAGER_URL   默认 http://127.0.0.1:18765
    RELAY_RATE          状态发布频率，默认 50 Hz
"""
import json
import os
import threading
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

URL = os.environ.get("ROUTE_MANAGER_URL", "http://127.0.0.1:18765")
RATE = float(os.environ.get("RELAY_RATE", "50.0"))


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
    def __init__(self):
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
        self.get_logger().info(
            f"中继已启动: /humanoid/joint_command → {URL} → "
            f"/humanoid/joint_states @ {RATE}Hz")

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
            self.get_logger().warning(f"send_joint_command 失败: {e}")

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
            self.get_logger().warning(f"get_joint_states 失败: {e}")


def main():
    rclpy.init()
    node = Relay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
