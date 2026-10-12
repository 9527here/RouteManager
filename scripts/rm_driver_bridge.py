#!/usr/bin/env python3
"""rm_driver_bridge.py — 在仿真里实现睿尔曼 rm_driver 的话题接口。

目的: 让上层程序**一行不改**就能既驱仿真、又驱真机 RM65。
真机侧由官方 rm_driver 提供这些话题; 仿真侧没有机械臂控制器, 所以由本节点
把同名话题桥接到 RouteManager 的 HTTP 执行器。

实现的话题(每条臂一个 namespace, 默认 /left_arm/rm_driver、/right_arm/rm_driver):

  入(命令)  {ns}/movej_canfd_custom_cmd  rm_ros_interfaces/Jointposcustom
                joint[6] 弧度, follow/expand/dof/trajectory_mode/radio
                (透传流; 官方要求连续发点, 本桥按 RATE 节流转发)
  入(命令)  {ns}/movej_cmd               rm_ros_interfaces/Movej
                joint[6] 弧度, speed, block, trajectory_connect, dof (带规划移动)
  入(命令)  {ns}/set_gripper_position_cmd rm_ros_interfaces/Gripperset
                position 1~1000 ↔ 开口 0~70mm (1000=全开)
  入(命令)  {ns}/move_stop_cmd            std_msgs/Empty (急停: 冻结当前位姿 + 底盘刹停)
  出(反馈)  /joint_states                 sensor_msgs/JointState (仿真 50 DOF 原名)
  出(反馈)  {ns}/joint_states             sensor_msgs/JointState (joint1..6, rm 风格)

用法(source 过 ROS 与 rm_ros_interfaces 之后):
    python3 scripts/rm_driver_bridge.py                 # 默认双臂 + 标准映射
    python3 scripts/rm_driver_bridge.py --left-ns /rm_driver --single-left

关节映射(默认): rm 的 joint1..6 → 仿真的 {side}_arm_1..6_joint
参数见 --help; 环境变量 ROUTE_MANAGER_URL / RM_BRIDGE_RATE 可覆盖。
"""
import argparse
import json
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

from rm_ros_interfaces.msg import Jointposcustom, Movej, Gripperset

URL = "http://127.0.0.1:18765"
RATE = 10.0          # 转发频率(Hz): 仿真单任务队列敏感, 别照搬真机的 100~200Hz
GRIPPER_OPEN_POS = 1000   # rm 语义: 1000 = 全开
GRIPPER_CLOSE_POS = 1


def http_action(action, parameters=None, timeout=5.0, tries=25):
    """HTTP 调用带 409 重试: 仿真单任务队列常被 relay 轮询占满(实测启动即 409)。"""
    import urllib.error
    payload = json.dumps({"action": action,
                          "parameters": parameters or {}}).encode()
    for i in range(tries):
        req = urllib.request.Request(
            URL + "/execute", data=payload,
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                out = json.loads(resp.read().decode())
            if isinstance(out, dict) and out.get("detail"):
                raise RuntimeError(str(out["detail"])[:200])
            return out.get("data")
        except urllib.error.HTTPError as e:
            if e.code == 409 and i < tries - 1:
                time.sleep(0.15)
                continue
            raise


class ArmBridge:
    """一条臂: 收 rm 风格命令 → 落到仿真关节; 反馈用同一份状态发布。"""

    def __init__(self, node: Node, ns: str, side: str, joint_names_all,
                 sim_prefix: str, gripper_joint: str, qos):
        self.node = node
        self.ns = ns.rstrip("/")
        self.side = side
        self.sim_joints = [f"{sim_prefix}_{i}_joint" for i in range(1, 7)]
        self.rm_joints = [f"joint{i}" for i in range(1, 7)]
        self.gripper_joint = gripper_joint
        self._lock = threading.Lock()
        self._target = None          # {sim_joint_name: value}
        self._grip = None            # 0..1 (1=全开)
        node.create_subscription(Jointposcustom, f"{self.ns}/movej_canfd_custom_cmd",
                                 self._on_canfd, qos)
        node.create_subscription(Movej, f"{self.ns}/movej_cmd", self._on_movej, qos)
        node.create_subscription(Gripperset, f"{self.ns}/set_gripper_position_cmd",
                                 self._on_gripper, qos)
        node.create_subscription(Empty, f"{self.ns}/move_stop_cmd",
                                 self._on_stop, qos)
        self.pub_arm = node.create_publisher(JointState, f"{self.ns}/joint_states", qos)

    # ---- 命令回调(只暂存, 由定时器统一发 HTTP: 与真机驱动同样的"最新指令优先") ----
    def _on_canfd(self, msg: Jointposcustom):
        self._set_arm(list(msg.joint)[:6])

    def _on_movej(self, msg: Movej):
        self._set_arm(list(msg.joint)[:6])

    def _set_arm(self, q):
        if len(q) < 6:
            return
        with self._lock:
            self._target = {n: float(v) for n, v in zip(self.sim_joints, q)}

    def _on_gripper(self, msg: Gripperset):
        pos = float(msg.position)
        frac = (pos - GRIPPER_CLOSE_POS) / float(GRIPPER_OPEN_POS - GRIPPER_CLOSE_POS)
        frac = min(max(frac, 0.0), 1.0)
        with self._lock:
            self._grip = frac

    def _on_stop(self, _msg):
        # 急停: 冻结(目标=当前实测) + 底盘刹停
        try:
            st = http_action("get_joint_states")
            fb = st.get("feedback") or []
            names = self.node.joint_names_all
            with self._lock:
                self._target = {n: float(fb[names.index(n)])
                                for n in self.sim_joints if n in names}
            http_action("set_base_velocity", {"vx": 0.0, "wz": 0.0})
            self.node.get_logger().warn(f"{self.ns} 收到 move_stop: 冻结当前位姿")
        except Exception as e:
            self.node.get_logger().warning(f"move_stop 处理失败: {e}")

    # 注: _on_stop 冻结的是"这条臂的 6 个关节"，由 _cycle 用 joint_names
    # 部分下发，不会波及其他关节。

    # ---- 周期任务: 合并目标 → 一次 HTTP 全量下发 ----
    def take_and_clear(self):
        with self._lock:
            tgt, grip = self._target, self._grip
            self._target, self._grip = None, None
        return tgt, grip


class RmDriverBridge(Node):
    def __init__(self, arms, rate):
        super().__init__("rm_driver_bridge_sim")
        self.joint_names_all = http_action("get_joint_names")["dof_names"]
        self.get_logger().info(f"仿真关节 {len(self.joint_names_all)} 个")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.arms = []
        for side, ns, grip in arms:
            self.arms.append(ArmBridge(self, ns, side, self.joint_names_all,
                                       f"{side}_arm", grip, qos))
            self.get_logger().info(f"桥接 {ns}  → 仿真 {side}_arm_1..6_joint")
        # 单一定时器: 每个周期只取一次状态, 同时用于"下发+发布"
        # (仿真单任务队列敏感; 之前两个 20Hz timer = 40 次/秒 轮询, 争不到槽)
        self.create_timer(1.0 / rate, self._cycle)

    def _cycle(self):
        """一个周期: 取一次状态 → 发布反馈 + (有变化时)合并下发一次。"""
        try:
            fb = http_action("get_joint_states").get("feedback") or []
        except Exception as e:
            self.get_logger().warning(f"取状态失败: {e}", throttle_duration_sec=5.0)
            return
        if len(fb) != len(self.joint_names_all):
            return
        self._publish_states(fb)
        for arm in self.arms:
            tgt, grip = arm.take_and_clear()
            if tgt:
                # 只写这条臂的 6 个关节(joint_names 参数)。整向量下发会把
                # /joint_states 的实测值当成指令重写全身(含升降/躯干/头),
                # 结果是身体被重力一帧帧压塌、并和 VR 遥操作抢控制权。
                jn = [n for n in arm.sim_joints if n in self.joint_names_all]
                vals = [tgt[n] for n in jn]
                try:
                    http_action("send_joint_command",
                                {"positions": vals, "joint_names": jn})
                except Exception as e:
                    self.get_logger().warning(
                        f"{arm.ns} 下发失败: {e}", throttle_duration_sec=5.0)
            if grip is not None:
                # 夹爪走睿尔曼语义(position 1~1000): 仿真侧展开成 12 个
                # 关节(含 mimic 跟随), 真机侧由 rm_driver 直接执行同一指令
                try:
                    http_action("set_gripper_position",
                                {"side": arm.side,
                                 "position": int(round(1 + grip * 999))})
                except Exception as e:
                    self.get_logger().warning(
                        f"{arm.ns} 夹爪下发失败: {e}",
                        throttle_duration_sec=5.0)

    def _publish_states(self, fb):
        try:
            stamp = self.get_clock().now().to_msg()
            # JointState(header=None, ...) 会抛 "header must be a sub message":
            # 必须先建空消息再逐字段赋值(反馈此前一直没发出去)
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = list(self.joint_names_all)
            msg.position = [float(v) for v in fb]
            self._pub_all.publish(msg)
            for arm in self.arms:
                m2 = JointState()
                m2.header.stamp = stamp
                m2.name = list(arm.rm_joints)
                m2.position = [float(fb[self.joint_names_all.index(n)])
                               for n in arm.sim_joints
                               if n in self.joint_names_all]
                arm.pub_arm.publish(m2)
        except Exception as e:
            self.get_logger().warning(
                f"发布状态失败: {e}", throttle_duration_sec=5.0)


def main():
    global URL
    ap = argparse.ArgumentParser(description="仿真侧 rm_driver 话题桥")
    ap.add_argument("--left-ns", default="/left_arm/rm_driver")
    ap.add_argument("--right-ns", default="/right_arm/rm_driver")
    ap.add_argument("--single-left", action="store_true",
                    help="只桥左臂(真机单臂时用)")
    ap.add_argument("--rate", type=float, default=RATE)
    ap.add_argument("--url", default=URL)
    args = ap.parse_args()

    URL = args.url
    arms = [("left", args.left_ns, "left_gripper_left_main_joint")]
    if not args.single_left:
        arms.append(("right", args.right_ns, "right_gripper_right_main_joint"))

    rclpy.init()
    node = RmDriverBridge(arms, args.rate)
    # 全局 /joint_states(仿真原名) 由节点持有
    from rclpy.qos import QoSProfile
    node._pub_all = node.create_publisher(JointState, "/joint_states",
                                          QoSProfile(depth=10))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()