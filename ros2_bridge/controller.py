"""
ros2_bridge.controller — 通用 ROS2 关节控制节点（仿真闭环）
==========================================================

闭环设计（sim 作为 ROS2 参与者）::

    外部 ROS2 节点 ──publish──> command_topic ──> [本节点订阅]
                                                  │ 暂存最新指令（线程安全）
    RobotManager.step()（仿真主线程）<──取用───────┘
        │ 施加到 articulation
        └──publish_joint_states──> state_topic <──[本节点订阅]── 回读反馈

  - 指令回调运行在 rclpy spin 线程，只更新"最新指令"缓冲；真正的物理
    操作由仿真主线程在 step() 里完成，线程安全。
  - HTTP 的 send_joint_command 不走 ROS hop，直接进插值状态机
    （见 robot_manager.py），经 record_command 记入 cmd 缓冲。

用法::

    controller = Ros2JointController(
        joint_names=["joint1", ..., "jointN"],
        command_topic="/humanoid/joint_command",
        state_topic="/humanoid/joint_states",
    )
    latest = controller.get_latest_command()      # 主线程取最新外部指令
    controller.publish_joint_states([...])        # 发布关节状态反馈
    controller.send_joint_command([0.0, 0.5])     # 纯 ROS 路径（发布到话题）

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
        :param joint_names: 关节名列表（顺序即指令/状态向量顺序）
        :param joint_limits: 可选 {joint_name: {"lower": l, "upper": u}}，超限指令会被 clamp
        """
        self.joint_names = list(joint_names)
        self.joint_limits = joint_limits or {}

        self._node = None
        self._cmd_pub = None
        self._state_pub = None
        self._lock = threading.Lock()   # 保护 _cmd_positions / _latest_command
        self._initialized = False

        self._cmd_positions = {n: 0.0 for n in self.joint_names}
        self._feedback_positions = {n: 0.0 for n in self.joint_names}
        self._feedback_received = False
        self._command_received = False
        self._latest_command = None     # 最新外部指令（对齐 joint_names 顺序）

        try:
            import rclpy
            from sensor_msgs.msg import JointState

            if not rclpy.ok():
                rclpy.init(args=None)

            self._node = rclpy.create_node(node_name)
            self._cmd_pub = self._node.create_publisher(JointState, command_topic, 10)
            self._state_pub = self._node.create_publisher(JointState, state_topic, 10)
            self._node.create_subscription(
                JointState, command_topic, self._command_callback, 10)
            self._node.create_subscription(
                JointState, state_topic, self._joint_state_callback, 10)

            # 独立线程 spin，保证订阅回调持续收到消息
            self._spin_thread = threading.Thread(
                target=self._spin_loop, daemon=True, name="ros2-spin")
            self._spin_thread.start()

            self._initialized = True
            logger.info(f"ROS2 节点已创建: 指令 {command_topic}（订阅→仿真执行），"
                        f"状态 {state_topic}（仿真→发布，同时回读作反馈）")
        except Exception as e:
            logger.warning(f"ROS2 初始化失败（将以未连接模式运行）: {e}")

    # ==================== spin ====================

    def _spin_loop(self):
        import rclpy
        while rclpy.ok() and self._node is not None:
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception as e:
                logger.debug(f"spin 异常: {e}")
                time.sleep(0.1)

    # ==================== 回调（spin 线程） ====================

    def _command_callback(self, msg):
        """外部指令 → 暂存最新值（对齐 joint_names 顺序），仿真主线程取用。"""
        with self._lock:
            for i, name in enumerate(msg.name):
                if name in self._cmd_positions and i < len(msg.position):
                    self._cmd_positions[name] = msg.position[i]
            # 消息里缺失的关节沿用上次指令值
            self._latest_command = [self._cmd_positions.get(n, 0.0)
                                    for n in self.joint_names]
        if not self._command_received:
            self._command_received = True
            logger.info("已接收到外部关节指令")

    def _joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self._feedback_positions and i < len(msg.position):
                self._feedback_positions[name] = msg.position[i]
        if not self._feedback_received:
            self._feedback_received = True
            logger.info("已接收到关节状态反馈")

    # ==================== 指令通道 ====================

    def get_latest_command(self):
        """取走最新外部指令（对齐 joint_names 顺序）；无则返回 None。
        在仿真主线程调用。"""
        with self._lock:
            cmd = self._latest_command
            self._latest_command = None
            return cmd

    def record_command(self, positions):
        """记录经其他路径（HTTP 直连/插值中间值）下发的指令，供查询。"""
        with self._lock:
            for name, p in zip(self.joint_names, positions):
                self._cmd_positions[name] = p

    def clamp_positions(self, positions):
        clamped = []
        for name, p in zip(self.joint_names, positions):
            limit = self.joint_limits.get(name)
            if limit:
                p = clamp(p, limit["lower"], limit["upper"])
            clamped.append(float(p))
        return clamped

    def send_joint_command(self, positions, duration=0.0, steps=50):
        """纯 ROS 路径：把指令发布到 command_topic（外部监听者可见）。
        duration>0 时阻塞式 smoothstep 插值 —— 驱动仿真请用
        RobotManager.send_joint_command（非阻塞）。"""
        if not self._initialized:
            logger.warning("ROS2 未初始化，指令未发送")
            return

        positions = self.clamp_positions(positions)

        if duration > 0:
            with self._lock:
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

    # ==================== 状态通道 ====================

    def publish_joint_states(self, positions, velocities=None, efforts=None):
        """发布关节实际状态到 state_topic（仿真主线程按 publish_rate 调用）。"""
        if not self._initialized:
            return
        try:
            from sensor_msgs.msg import JointState

            msg = JointState()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.name = list(self.joint_names)
            msg.position = list(positions)
            if velocities is not None:
                msg.velocity = list(velocities)
            if efforts is not None:
                msg.effort = list(efforts)
            self._state_pub.publish(msg)
        except Exception as e:
            logger.error(f"发布关节状态失败: {e}")

    # ==================== 查询 ====================

    def get_cmd_positions(self):
        with self._lock:
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
                node, self._node = self._node, None   # 先置 None 让 spin 线程退出
                node.destroy_node()
            self._initialized = False
            logger.info("ROS2 节点已销毁")
        except Exception as e:
            logger.error(f"shutdown 异常: {e}")
