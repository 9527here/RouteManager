"""contract_node.py — 在 Isaac 进程内跑的 ROS2 契约节点。

和旧的 `scripts/ros2_sim_contract.py`（独立 py3.10 中继 + HTTP）相比，这条是
**进程内**的：rclpy 直接用 Isaac 6.2 自带的 **py3.12** 版本，状态和指令都不再
跨进程，实测能把手臂反馈从 5~8 Hz 提到物理帧率。

为什么能这么做：Isaac 6.2 的 `isaacsim.ros2.core` 扩展里打包了 py312 的 rclpy
和全套消息包：

    isaacsim/exts/isaacsim.ros2.core/humble/rclpy/{rclpy,sensor_msgs,std_msgs,...}
    isaacsim/exts/isaacsim.ros2.core/humble/lib/librcl_action.so ...

- Python 侧：把上面的 `humble/rclpy` 加进 `sys.path` 即可（消息包也在同一个目录里）。
- 动态库侧：**不需要动 `LD_LIBRARY_PATH`** —— `run.sh` 已经保留了
  `/opt/ros/humble/lib`，系统 Humble 的 `librcl_action.so` 与 Isaac 自带的
  rclpy 版本匹配，实测能直接加载。（`run.sh` 只摘掉 `.../lib/*-linux-gnu`
  那层给 py3.10 用的扩展，不影响这里。）

**线程模型**：所有 ROS 回调都跑在仿真主线程上 —— `RobotManager.step()` 里调用
`spin_once()`，回调在那一刻同步执行，因此回调里**可以安全地直接操作
articulation**，不需要任何锁，也不会有"回调线程碰物理"的隐患。
这是本项目唯一被验证过的线程安全写法（和 `ros2_bridge/controller.py` 的
"回调只写缓冲"不同，那里是因为 spin 在独立线程）。

**顶层不 import rclpy / omni / pxr**（项目硬性约束：必须在 SimulationApp 之后
才能 import Isaac 相关模块；rclpy 的路径也要等环境就绪后才确定）。
"""
import glob
import logging
import os
import sys

logger = logging.getLogger("route_manager.ros2_contract")

#: 已把 Isaac 自带的 rclpy 路径装进 sys.path
_ROS_PATH_READY = False


def ensure_ros_python_path() -> bool:
    """把 Isaac 自带的 py3.12 rclpy 加进 `sys.path`（幂等）。

    先试直接 import —— Isaac 启用 `isaacsim.ros2.bridge` 后可能已经把路径铺好了。
    失败则从 `isaacsim.ros2.core/<distro>/rclpy` 找一份塞进去，并**清掉
    sys.modules 里半死不活的 rclpy**（第一次 import 会留下一个缺 C 扩展的
    rclpy 模块，不清理的话第二次 import 直接命中缓存还是坏的那个）。
    """
    global _ROS_PATH_READY
    if _ROS_PATH_READY:
        return True
    try:
        import rclpy  # noqa: F401
        _ROS_PATH_READY = True
        return True
    except Exception:
        pass

    cands = []
    try:
        import isaacsim
        root = os.path.dirname(os.path.dirname(os.path.abspath(isaacsim.__file__)))
        cands += sorted(glob.glob(os.path.join(
            root, "exts", "isaacsim.ros2.core", "*", "rclpy")))
    except Exception:
        pass
    # 兜底：按已知安装位置猜一次（不同 env 名也能命中）
    cands += sorted(glob.glob(os.path.join(
        os.path.dirname(os.path.dirname(sys.executable)),
        "lib", "python*", "site-packages", "isaacsim", "exts",
        "isaacsim.ros2.core", "*", "rclpy")))

    for path in cands:
        if not os.path.isdir(path):
            continue
        for name in [k for k in sys.modules
                     if k == "rclpy" or k.startswith("rclpy.")]:
            del sys.modules[name]
        sys.path.insert(0, path)
        try:
            import rclpy  # noqa: F401
            _ROS_PATH_READY = True
            logger.info(f"ROS2 进程内节点使用 Isaac 自带 rclpy: {path}")
            return True
        except Exception as e:
            logger.debug(f"rclpy 路径试错失败 {path}: {e}")
    logger.warning("找不到可用的 py3.12 rclpy（Isaac 自带的那份）")
    return False


#: 图像用 best-effort（sensor data 惯例）；其余 reliable。8 个字段必填。
_QOS_RELIABLE = 10
_QOS_SENSOR = 5


class ContractNode:
    """`/<robot>/<role>/<instance>/...` 契约的进程内实现。

    话题集合与旧的 py3.10 中继**完全一致**，所以对上层来说换实现是不可见的。
    """

    def __init__(self, meta: dict, handlers: dict):
        """
        :param meta: **`RobotManager.get_contract_meta()` 的返回值**，不是
            `config.json` 的原始段 —— 因为 `sides`（夹爪关节名、flange/tcp
            link）是运行时从 articulation 推导出来的，配置里没有。
        :param handlers: `{"move_j": fn, "send_joint_command": fn,
            "set_gripper_position": fn}` —— 都会在**主线程**被回调调用
        """
        if not ensure_ros_python_path():
            raise RuntimeError("py3.12 rclpy 不可用")

        import rclpy
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float32, Float32MultiArray
        from trajectory_msgs.msg import JointTrajectory

        self._rclpy = rclpy
        self._handlers = handlers
        self._cfg = meta
        self._sides = meta.get("sides") or {}
        self._groups = meta.get("groups") or {}
        robot = meta.get("robot", "iasscsim_robot")
        role = meta.get("role", "sim")
        self.base = meta.get("base_topic") or f"/{robot}/{role}"

        self._qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=_QOS_RELIABLE,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self._qos1 = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self._owns_context = False
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_context = True

        self._node = rclpy.create_node(f"{robot}_{role}_contract")
        self.pub = {}
        self._build_publishers(JointState, Float32MultiArray, PoseStamped)
        self._build_subscriptions(JointState, Float32, JointTrajectory)
        logger.info(f"ROS2 契约节点已创建: {self._node.get_name()} @ {self.base}")

    # ---------------- 端点 ----------------

    def _build_publishers(self, JointState, Float32MultiArray, PoseStamped):
        for group in self._groups:
            self.pub[f"{group}/fb"] = self._node.create_publisher(
                JointState, f"{self.base}/{group}/feedback/joint_states",
                self._qos)
        for side in self._sides:
            ns = f"{self.base}/{side}_arm"
            self.pub[f"{side}/gjs"] = self._node.create_publisher(
                JointState, f"{ns}/feedback/gripper_joint_state", self._qos)
            self.pub[f"{side}/gst"] = self._node.create_publisher(
                Float32MultiArray, f"{ns}/feedback/gripper_status", self._qos)
            self.pub[f"{side}/flange"] = self._node.create_publisher(
                PoseStamped, f"{ns}/feedback/flange_pose", self._qos)
            self.pub[f"{side}/tcp"] = self._node.create_publisher(
                PoseStamped, f"{ns}/feedback/tcp_pose", self._qos)
        # 相机图像与 camera_info **不在这里发** —— 由 Isaac 内的 OmniGraph
        # （setup_ros2_cameras → ROS2CameraHelper + ROS2CameraInfoHelper）原生发布，
        # 图像不跨进程。这里只管手臂/夹爪/位姿。

    def _build_subscriptions(self, JointState, Float32, JointTrajectory):
        for group in self._groups:
            self._node.create_subscription(
                JointState, f"{self.base}/{group}/control/joint_states",
                lambda msg, g=group: self._on_joint(g, msg), self._qos1)
        for side in self._sides:
            ns = f"{self.base}/{side}_arm"
            self._node.create_subscription(
                JointTrajectory, f"{ns}/control/move_j",
                lambda msg, s=side: self._on_move_j(s, msg), self._qos1)
            self._node.create_subscription(
                Float32, f"{ns}/control/gripper",
                lambda msg, s=side: self._on_gripper(s, msg), self._qos1)

    # ---------------- 回调（跑在主线程） ----------------

    def _on_joint(self, group, msg):
        names = list(msg.name) or list(self._groups.get(group, []))
        vals = list(msg.position)
        if len(names) != len(vals):
            logger.warning(f"{group} control/joint_states 名/值长度不符，忽略")
            return
        try:
            self._handlers["send_joint_command"](vals, joint_names=names)
        except Exception as e:
            logger.warning(f"{group} 关节指令失败: {e!r}")

    def _on_move_j(self, side, msg):
        if not msg.points:
            return
        pt = msg.points[-1]
        names = list(msg.joint_names) or list(
            (self._sides.get(side) or {}).get("arm_joints") or [])
        if len(names) != len(pt.positions):
            logger.warning(f"{side} move_j 名/值长度不符，忽略")
            return
        dur = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
        try:
            self._handlers["move_j"](side=side, positions=list(pt.positions),
                                     joint_names=names, duration=dur)
        except Exception as e:
            logger.warning(f"{side} move_j 失败: {e!r}")

    def _on_gripper(self, side, msg):
        try:
            self._handlers["set_gripper_position"](
                side=side, position=int(round(float(msg.data))))
        except Exception as e:
            logger.warning(f"{side} 夹爪指令失败: {e!r}")

    # ---------------- 主线程驱动 ----------------

    def spin_once(self, timeout_sec: float = 0.0):
        """处理一次待处理回调。**必须在仿真主线程调用**。"""
        try:
            self._rclpy.spin_once(self._node, timeout_sec=timeout_sec)
        except Exception as e:
            logger.debug(f"spin_once: {e!r}")

    def _stamp(self, sim_time: float):
        from builtin_interfaces.msg import Time
        t = Time()
        t.sec = int(sim_time)
        t.nanosec = int((sim_time - int(sim_time)) * 1e9)
        return t

    def publish(self, st: dict, ref_link: str = "base_link"):
        """把 `get_sim_state()` 的结果发出去。**必须在仿真主线程调用**。"""
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float32MultiArray

        names = st.get("joint_names") or []
        idx = {n: i for i, n in enumerate(names)}
        pos = st.get("positions") or []
        vel = st.get("velocities") or []
        stamp = self._stamp(st.get("sim_time", 0.0))

        for group, joints in self._groups.items():
            have = [j for j in joints if j in idx]
            key = f"{group}/fb"
            if not have or key not in self.pub:
                continue
            ii = [idx[j] for j in have]
            m = JointState()
            m.header.stamp = stamp
            m.name = have
            m.position = [float(pos[i]) for i in ii]
            if vel:
                m.velocity = [float(vel[i]) for i in ii]
            self.pub[key].publish(m)

        for side, s in (st.get("sides") or {}).items():
            g = s.get("gripper")
            if g and f"{side}/gjs" in self.pub:
                m = JointState()
                m.header.stamp = stamp
                m.name = [(self._sides.get(side) or {}).get("gripper_joint")
                          or f"{side}_gripper_right_2_joint"]
                m.position = [float(g.get("angle_rad", 0.0))]
                self.pub[f"{side}/gjs"].publish(m)

                a = Float32MultiArray()
                a.data = [float(g.get("position", 0.0)),
                          float(g.get("opening_mm", 0.0)),
                          float(g.get("fraction", 0.0)),
                          float(g.get("angle_rad", 0.0))]
                self.pub[f"{side}/gst"].publish(a)

            for kind, key in (("flange", "flange"), ("tcp", "tcp")):
                pose = s.get(key)
                if not pose:
                    continue
                p = PoseStamped()
                p.header.stamp = stamp
                p.header.frame_id = ref_link
                p.pose.position.x, p.pose.position.y, p.pose.position.z = \
                    pose["position"]
                q = pose["orientation"]            # wxyz
                p.pose.orientation.x = q[1]
                p.pose.orientation.y = q[2]
                p.pose.orientation.z = q[3]
                p.pose.orientation.w = q[0]
                self.pub[f"{side}/{kind}"].publish(p)

    # ---------------- 生命周期 ----------------

    def shutdown(self):
        try:
            if self._node is not None:
                self._node.destroy_node()
                self._node = None
            if self._owns_context and self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception as e:
            logger.debug(f"契约节点关闭异常: {e!r}")
        finally:
            self.pub = {}

    def is_running(self) -> bool:
        return self._node is not None
