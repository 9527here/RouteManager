"""
ros2_bridge.controller — 通用 ROS2 关节控制节点
==============================================

发布关节指令、订阅关节状态反馈（话题名可配置，适配任何人形机器人）。
独立线程 spin，主线程（Isaac）随时可安全调用。

用法::

    controller = Ros2JointController(
        joint_names=["joint1", ..., "jointN"],
        command_topic="/humanoid/joint_command",
        state_topic="/humanoid/joint_states",
    )
    controller.send_joint_command([0.0, 0.5, ...])
    controller.get_feedback_positions()

注意：需要 source ROS2 环境后启动（rclpy 可用）。初始化失败会降级为
未连接状态（initialized=False），不阻塞仿真。
"""

import logging
import threading
import time

logger = logging.getLogger("route_manager.ros2")


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def lerp(a, b, t):
    return a + (b - a) * t


def smoothstep(t):
    return t * t * (3.0 - 2.0 * t)


class Ros2JointController:
    def __init__(self, joint_names, command_topic="/humanoid/joint_command",
                 state_topic="/humanoid/joint_states", node_name="robot_controller",
                 joint_limits=None):
        """
        :param joint_names: 关节名列表（顺序即发送顺序）
        :param joint_limits: 可选 {joint_name: {"lower": l, "upper": u}}，超限指令会被 clamp
        """
        self.joint_names = list(joint_names)
        self.joint_limits = joint_limits or {}

        self._node = None
        self._cmd_pub = None
        self._lock = threading.Lock()
        self._initialized = False

        self._cmd_positions = {n: 0.0 for n in self.joint_names}
        self._feedback_positions = {n: 0.0 for n in self.joint_names}
        self._feedback_received = False

        try:
            import rclpy
            from sensor_msgs.msg import JointState

            if not rclpy.ok():
                rclpy.init(args=None)

            self._node = rclpy.create_node(node_name)
            self._cmd_pub = self._node.create_publisher(JointState, command_topic, 10)
            self._node.create_subscription(
                JointState, state_topic, self._joint_state_callback, 10)

            # 独立线程 spin，保证订阅回调持续收到反馈
            self._spin_thread = threading.Thread(
                target=self._spin_loop, daemon=True, name="ros2-spin")
            self._spin_thread.start()

            self._initialized = True
            logger.info(f"ROS2 节点已创建: 发布 {command_topic}，订阅 {state_topic}")
        except Exception as e:
            logger.warning(f"ROS2 初始化失败（将以未连接模式运行）: {e}")

    # ==================== spin ====================

    def _spin_loop(self):
        import rclpy
        while rclpy.ok():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception as e:
                logger.debug(f"spin 异常: {e}")
                time.sleep(0.1)

    def _joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self._feedback_positions and i < len(msg.position):
                self._feedback_positions[name] = msg.position[i]
        if not self._feedback_received:
            self._feedback_received = True
            logger.info("已接收到关节状态反馈")

    # ==================== 发送 ====================

    def send_joint_command(self, positions, duration=0.0, steps=50):
        """
        发送关节指令。
        :param positions: 与 joint_names 等长的位置列表 (rad)
        :param duration: >0 时做 smoothstep 平滑插值（阻塞到完成）
        """
        if not self._initialized:
            logger.warning("ROS2 未初始化，指令未发送")
            return

        positions = self._clamp_positions(positions)

        if duration > 0:
            start = [self._cmd_positions.get(n, 0.0) for n in self.joint_names]
            dt = duration / steps
            for step in range(steps + 1):
                t = smoothstep(step / steps)
                interp = [lerp(s, e, t) for s, e in zip(start, positions)]
                self._publish(interp, blocking_sleep=dt)
        else:
            self._publish(positions)

    def _publish(self, positions, blocking_sleep=None):
        try:
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Header

            msg = JointState()
            msg.header = Header()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.name = list(self.joint_names)
            msg.position = list(positions)

            with self._lock:
                self._cmd_pub.publish(msg)
                for i, name in enumerate(self.joint_names):
                    self._cmd_positions[name] = msg.position[i]

            if blocking_sleep is not None:
                time.sleep(blocking_sleep)
        except Exception as e:
            logger.error(f"发送关节指令失败: {e}")

    def _clamp_positions(self, positions):
        clamped = []
        for name, p in zip(self.joint_names, positions):
            limit = self.joint_limits.get(name)
            if limit:
                p = clamp(p, limit["lower"], limit["upper"])
            clamped.append(float(p))
        return clamped

    # ==================== 查询 ====================

    def get_cmd_positions(self):
        return [self._cmd_positions.get(n, 0.0) for n in self.joint_names]

    def get_feedback_positions(self):
        return [self._feedback_positions.get(n, 0.0) for n in self.joint_names]

    def is_feedback_received(self):
        return self._feedback_received

    def is_initialized(self):
        return self._initialized

    # ==================== 释放 ====================

    def shutdown(self):
        try:
            if self._node is not None:
                self._node.destroy_node()
                self._node = None
            self._initialized = False
            logger.info("ROS2 节点已销毁")
        except Exception as e:
            logger.error(f"shutdown 异常: {e}")
