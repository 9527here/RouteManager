#!/usr/bin/env python3
"""ROS2 订阅端回归验证: 等待 humanoid_v3 图发布的三个话题, 齐了就写 JSON 自退.

订阅 /joint_states(记录关节数与名单) /tf(变换条数) /clock(消息数).
全部拿到 -> 写 /tmp/v3_ros2graph_check.json {ok: true, ...} 并退出 0.
超时(默认 220s, 留 Isaac 无头启动余量) -> 写 {ok: false, ...} 退出 1.

用法:
  source /opt/ros/humble/setup.bash
  timeout 240 python3 scripts/ros2_sub_check.py
"""

import json
import sys
import time

ARGS = sys.argv[1:]
TOPICS_TIMEOUT = float(next((a for a in ARGS if not a.startswith("--")), 220.0))
CTAG = "--ctag" in ARGS
OUT = "/tmp/v3_ros2graph_check_ctag.json" if CTAG else \
    "/tmp/v3_ros2graph_check.json"
MIN_JOINTS = 26  # v3 可动关节: lift+torso+head2+双臂14+castor4+wheels4
GRIP_JOINT = "left_gripper_right_2_joint"   # 夹爪变体指纹
GRIP_MIN_JOINTS = 50  # 26 基座 + 24 夹爪(12/手: 1 驱动 + 11 mimic 跟随)

rep = {"ok": False, "joint_states": None, "joint_count": 0, "tf_transforms": 0,
       "clock_msgs": 0}

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

rclpy.init()
node = Node("v3_graph_check")


def on_js(msg):
    if rep["joint_states"] is None:
        rep["joint_states"] = list(msg.name)
        rep["joint_count"] = len(msg.name)
        node.get_logger().info(f"首条 /joint_states: {len(msg.name)} 关节")


def on_tf(msg):
    rep["tf_transforms"] += len(msg.transforms)


def on_clock(msg):
    rep["clock_msgs"] += 1


node.create_subscription(JointState, "joint_states", on_js, 10)
node.create_subscription(TFMessage, "tf", on_tf, 10)
node.create_subscription(Clock, "clock", on_clock, 10)

t0 = time.time()
done = False
while rclpy.ok() and not done:
    rclpy.spin_once(node, timeout_sec=0.2)
    got_js = rep["joint_states"] is not None
    done = got_js and rep["tf_transforms"] > 0 and rep["clock_msgs"] > 0
    if time.time() - t0 > TOPICS_TIMEOUT:
        break

if done:
    names = rep["joint_states"] or []
    # v3 指纹: config/joint_names.yaml 里的权威命名; 机器上可能同时有别的
    # 机器人发 /joint_states(拼音名), 只认 v3 名单才算通过。
    # 夹爪变体额外要求 50 关节 + 夹爪驱动关节在名单里。
    base_ok = rep["joint_count"] >= MIN_JOINTS \
        and "lift_joint" in names and "left_arm_1_joint" in names
    is_grip = GRIP_JOINT in names
    if is_grip:
        rep["ok"] = base_ok and rep["joint_count"] >= GRIP_MIN_JOINTS
        rep["variant"] = "ctag2f120"
    else:
        rep["ok"] = base_ok and not CTAG
        rep["variant"] = "v3"
    if not rep["ok"]:
        rep["note"] = (
            f"关节数 {rep['joint_count']}, lift_joint={'lift_joint' in names}, "
            f"left_arm_1_joint={'left_arm_1_joint' in names}, "
            f"gripper={is_grip}, --ctag={CTAG} —— 若夹爪图发布却是 v3 关节数, "
            f"检查图层引用的 flat 是否夹爪版; False 基座名说明是别的机器人串台"
        )
rep["elapsed_s"] = round(time.time() - t0, 1)

node.destroy_node()
rclpy.shutdown()
with open(OUT, "w") as f:
    json.dump(rep, f, indent=1, ensure_ascii=False)
print(json.dumps({k: rep[k] for k in ("ok", "joint_count", "tf_transforms", "clock_msgs")}))
sys.exit(0 if rep["ok"] else 1)
