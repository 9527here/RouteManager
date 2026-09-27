#!/usr/bin/env python3
"""Build the ROS2 action graph for humanoid_upper_body_v3 as an INDEPENDENT layer.

产出 usd/ros2_graph.usd: 引用 usd/humanoid_upper_body_v3_flat.usd（不改动它）+
一个 OmniGraph action graph /ROS2_Bridge_humanoid_v3:

  OnPlaybackTick ─┬─ ROS2PublishJointState     (/joint_states, 整个 articulation)
                  ├─ ROS2PublishTransformTree (/tf, 整棵 link 树)
                  └─ ROS2PublishClock         (/clock, use_sim_time 配套)
  ROS2SubscribeJointState (/joint_states 下行指令) → Articulation Controller

迁移约定（夹爪后补时）:
  - 绑定锚点 = articulation 根 prim /humanoid_upper_body_v3/Geometry/base_link,
    新 URDF 重导入后保持 robot 名与结构, 图层零改动;
  - 关节名单留空 = 发布全部关节, 新增夹爪关节自动带上(如节点版本不支持
    空列表, 在 JOINT_LIST 处按 config/joint_names.yaml 追加)。

用法（两进程回归验证）:
  source /opt/ros/humble/setup.bash          # LD_LIBRARY_PATH 必须在父 shell
  # 1) 先起订阅进程（系统 python3.10 + rclpy, 后台, 完成条件满足即自退）
  timeout 240 python3 scripts/ros2_sub_check.py &
  # 2) 再跑建图脚本（Isaac python3.11, 头less, 建+存+play 6s）
  /home/jy/micromamba/envs/isaacsim5.1/bin/python3.11 \
      scripts/build_ros2_graph.py [--no-verify]
结果 JSON: /tmp/v3_ros2graph_build.json + /tmp/v3_ros2graph_check.json
"""

import json
import sys

PKG = "/home/jy/datasets/vla_data/humanbody_urdf/humanoid_upper_body_v3"
# --ctag: 夹爪变体图层 —— 引用指向 ctag flat, 其余零改动
# (robot 名/绑定锚点/图名不变, PublishJointState 无名单 = 自动带上 50 关节)。
CTAG = "--ctag" in sys.argv[1:]
FLAT = f"{PKG}/usd/humanoid_upper_body_v3_ctag2f120_flat.usd" if CTAG else \
    f"{PKG}/usd/humanoid_upper_body_v3_flat.usd"
OUT = f"{PKG}/usd/ros2_graph_ctag2f120.usd" if CTAG else f"{PKG}/usd/ros2_graph.usd"
REPORT = "/tmp/v3_ros2graph_build_ctag.json" if CTAG else \
    "/tmp/v3_ros2graph_build.json"
ARTICULATION = "/humanoid_upper_body_v3/Geometry/base_link"
ROBOT_ROOT = "/humanoid_upper_body_v3"
GRAPH = "/ROS2_Bridge_humanoid_v3"

# 5.1 节点类型名 —— 已从 isaacsim_ros2-5.1.0.0 wheel 的 Template.usda / 本机
# isaacsim.core.nodes 的 OgnIsaacArticulationController.ogn 核实（2026-09-15）:
#   ROS2PublishJointState  v1: inputs:targetPrim(rel,单数)/topicName/queueSize,
#                              无 frameId —— header.frame_id 由节点自定
#   ROS2PublishTransformTree v1: inputs:targetPrims(rel数组)/parentPrim(rel)/topicName
#   ROS2SubscribeJointState  v2: 纯输出(jointNames/positionCommand/velocityCommand/
#                              effortCommand/execOut), 无 articulation 绑定
#   IsaacArticulationController: inputs:robotPath(string)/targetPrim(rel)/
#                              positionCommand/jointNames/execIn
NODES = {
    "tick": "omni.graph.action.OnPlaybackTick",
    "js_pub": "isaacsim.ros2.bridge.ROS2PublishJointState",
    "tf_pub": "isaacsim.ros2.bridge.ROS2PublishTransformTree",
    "clk_pub": "isaacsim.ros2.bridge.ROS2PublishClock",
    "js_sub": "isaacsim.ros2.bridge.ROS2SubscribeJointState",
    "ctrl": "isaacsim.core.nodes.IsaacArticulationController",
}

rep = {"error": None, "node_types": NODES, "graph": GRAPH, "out": OUT}

app = None
try:
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})

    import omni.kit.app
    import omni.usd

    # ---- 1. 启用 ROS2 bridge 扩展（pip 环境缺省不加载） -----------------
    ext_mgr = omni.kit.app.get_app().get_extension_manager()  # Kit 107: snake_case
    for ext in ("isaacsim.ros2.bridge",):
        if not ext_mgr.is_extension_enabled(ext):
            ext_mgr.set_extension_enabled_immediate(ext, True)
    for _ in range(60):
        app.update()
    print("step1: ros2 bridge 扩展已启用", flush=True)

    # ---- 2. 校验节点类型确实已注册 ---------------------------------------
    import omni.graph.core as og

    def _node_type_ok(t):
        # Kit 版本间 registry API 改名多次(104:get_graph_registry/107:get_node_type_registry),
        # 逐个探测; 都不可用时返回 None = 跳过预校验, 交给 og.Controller.edit 兜底报错
        for fn in ("get_node_type_registry", "get_graph_registry"):
            f = getattr(og, fn, None)
            if f is not None:
                try:
                    return f().get_node_type(t) is not None
                except Exception:
                    pass
        return None

    bad_types = [(k, t) for k, t in NODES.items() if _node_type_ok(t) is False]
    rep["registered"] = [t for t in NODES.values() if _node_type_ok(t)]
    rep["missing"] = bad_types
    rep["registry_api"] = [fn for fn in ("get_node_type_registry", "get_graph_registry") if hasattr(og, fn)]
    if bad_types:
        raise RuntimeError(f"节点类型未注册: {bad_types}")
    print(f"step2: 节点类型校验通过 registry_api={rep['registry_api']}", flush=True)

    # ---- 3. 新建 stage, 引用 flat USD（分层, 不动原文件） ----------------
    ctx = omni.usd.get_context()
    ctx.new_stage()
    for _ in range(10):
        app.update()
    stage = ctx.get_stage()
    default_prim = stage.DefinePrim(ROBOT_ROOT)
    default_prim.GetReferences().AddReference(FLAT)
    stage.SetDefaultPrim(default_prim)
    for _ in range(30):
        app.update()

    # ---- 4. og.Controller 建图 -------------------------------------------
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", NODES["tick"]),
                ("PublishJointState", NODES["js_pub"]),
                ("PublishTF", NODES["tf_pub"]),
                ("PublishClock", NODES["clk_pub"]),
                ("SubscribeJointState", NODES["js_sub"]),
                ("Controller", NODES["ctrl"]),
            ],
            keys.SET_VALUES: [
                # PublishJointState 无 frameId 属性(5.1), header.frame_id 由节点处理
                ("PublishJointState.inputs:topicName", "joint_states"),
                ("PublishTF.inputs:topicName", "tf"),
                ("PublishClock.inputs:topicName", "clock"),
                ("SubscribeJointState.inputs:topicName", "joint_states"),
                ("Controller.inputs:robotPath", ARTICULATION),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("SubscribeJointState.outputs:execOut", "Controller.inputs:execIn"),
                ("SubscribeJointState.outputs:positionCommand", "Controller.inputs:positionCommand"),
                ("SubscribeJointState.outputs:jointNames", "Controller.inputs:jointNames"),
            ],
        },
    )
    # articulation 绑定是 target rel, og.Controller 的 SET_VALUES 对 rel 不可靠;
    # OG 节点未赋值的属性不会在 USD 里物化, GetAttribute 拿不到 —— 必须 CreateRelationship
    from pxr import Sdf, Usd

    for node, attr in (
        ("PublishJointState", "inputs:targetPrim"),
        ("PublishTF", "inputs:targetPrims"),
    ):
        p = stage.GetPrimAtPath(f"{GRAPH}/{node}")
        if not p or not p.IsValid():
            raise RuntimeError(f"{GRAPH}/{node} prim 不存在")
        p.CreateRelationship(attr).SetTargets([Sdf.Path(ARTICULATION)])
    for _ in range(10):
        app.update()
    print("step3: rel 绑定完成", flush=True)

    # ---- 5. 保存独立层 ----------------------------------------------------
    stage.GetRootLayer().Export(OUT)
    rep["saved"] = OUT
    print(f"step4: 已保存 {OUT}", flush=True)

    if "--no-verify" in sys.argv:
        rep["ok"] = True
        raise SystemExit(0)

    # ---- 6. play 6 秒供外部订阅进程验证 -----------------------------------
    # Isaac 的 python3.11 不能 import 系统 Humble 的 rclpy(3.10 ABI), 回归验证
    # 由独立的订阅进程完成: scripts/ros2_sub_check.py (系统 python3)。
    # 运行顺序: 先起订阅进程(后台), 再跑本脚本。
    import time

    import omni.timeline

    rep["play"] = {"started": time.time()}
    omni.timeline.get_timeline_interface().play()
    t0 = time.time()
    while time.time() - t0 < 6.0:
        app.update()
    omni.timeline.get_timeline_interface().stop()
    rep["play"]["seconds"] = 6.0
    rep["ok"] = True

except Exception as e:  # noqa: BLE001
    import traceback

    rep["error"] = f"{type(e).__name__}: {e}"
    rep["traceback"] = traceback.format_exc().splitlines()[-8:]
finally:
    # SimulationApp.close() 会硬退出进程, JSON 必须写在 close 之前
    with open(REPORT, "w") as f:
        json.dump(rep, f, indent=1, ensure_ascii=False)
    print(json.dumps({k: rep[k] for k in ("ok", "error") if k in rep}), flush=True)
    if app is not None:
        app.close()
