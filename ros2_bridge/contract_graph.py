"""contract_graph.py — 为 ROS2 sim 契约建相机 OmniGraph。

**顶层不 import omni/graph/pxr/isaacsim**（项目硬性约束：这些必须在
`SimulationApp` 之后才能 import，见 AGENTS.md）。

建出来的图（每路相机 3 个节点，共用一个播放时钟）：

    OnPlaybackTick ──tick──> RP_<cam>.execIn
    RP_<cam> ──execOut──> Img_<cam>.execIn
    RP_<cam> ──execOut──> Info_<cam>.execIn
    RP_<cam>.renderProductPath ──> Img_<cam>.renderProductPath
    RP_<cam>.renderProductPath ──> Info_<cam>.renderProductPath

  节点类型（6.2 实测，见 isaacsim.ros2.nodes/tests/test_camera.py）：
    omni.graph.action.OnPlaybackTick
    isaacsim.core.nodes.IsaacCreateRenderProduct
    isaacsim.ros2.bridge.ROS2CameraHelper      （只发图像）
    isaacsim.ros2.bridge.ROS2CameraInfoHelper  （单独发 CameraInfo）

  **`ROS2CameraHelper` 不会自动派生 camera_info** —— 6.2 里必须再挂一个
  `ROS2CameraInfoHelper`，否则只有图没有内参。

话题拼接（从 Ros2Node 源码确认）：
    最终话题 = nodeNamespace + "/" + topicName
所以这里 `nodeNamespace="/iasscsim_robot/sim"`、
`topicName="<role>/color/image_raw"` →
`/iasscsim_robot/sim/<role>/color/image_raw`。
**不要**同时给非空 namespace 和以 `/` 开头的绝对 topic，会拼出双层前缀。
"""

#: 图像传感器用 best-effort（ROS 传感器数据的标准做法）。8 个字段**必填**。
SENSOR_QOS = ('{"history":"keepLast","depth":5,"reliability":"bestEffort",'
              '"durability":"volatile","deadline":0.0,"lifespan":0.0,'
              '"liveliness":"systemDefault","leaseDuration":0.0}')

#: CameraInfo 体积小，用 reliable。
INFO_QOS = ('{"history":"keepLast","depth":10,"reliability":"reliable",'
            '"durability":"volatile","deadline":0.0,"lifespan":0.0,'
            '"liveliness":"systemDefault","leaseDuration":0.0}')


def build_camera_graph(graph_path: str, cameras: dict, node_namespace: str,
                       image_qos: str = SENSOR_QOS,
                       info_qos: str = INFO_QOS, fps: float = 15.0,
                       publish_clock: bool = False) -> dict:
    """建（或重建）相机图。

    :param graph_path: 图的 prim 路径，如 `/ROS2_SimContract`
    :param cameras: `{role: {"prim_path", "width", "height", "frame_id",
        "topic"}}`；`topic` 是相对话题名（如 `left_hand_camera/color/image_raw`）
    :param node_namespace: 统一命名空间，如 `/iasscsim_robot/sim`
    :param fps: 写进相机 prim 的 `omni:sensor:tickRate`
    :return: `{"graph_path", "cameras": {role: {...}}, "errors": []}`
    """
    import omni.graph.core as og
    import omni.usd
    import usdrt
    from pxr import Sdf, UsdGeom

    result = {"graph_path": graph_path, "cameras": {}, "errors": [],
              "node_types_missing": []}

    # 先确认节点类型都注册了 —— 缺 isaacsim.ros2.nodes 扩展时这里会空
    reg = None
    try:
        for fn in ("get_node_type_registry", "get_graph_registry"):
            if hasattr(og, fn):
                reg = getattr(og, fn)()
                break
    except Exception:
        reg = None
    needed = ["omni.graph.action.OnPlaybackTick",
              "isaacsim.core.nodes.IsaacCreateRenderProduct",
              "isaacsim.ros2.bridge.ROS2CameraHelper",
              "isaacsim.ros2.bridge.ROS2CameraInfoHelper"]
    if reg is not None:
        for t in needed:
            try:
                if reg.get_node_type(t) is None:
                    result["node_types_missing"].append(t)
            except Exception:
                result["node_types_missing"].append(t)
    if result["node_types_missing"]:
        result["errors"].append(
            "节点类型未注册：" + ", ".join(result["node_types_missing"]) +
            "（检查 sim_env.py 是否启用了 isaacsim.ros2.nodes）")
        return result

    stage = omni.usd.get_context().get_stage()
    cams_root = stage.GetPrimAtPath("/World/cams")
    if not cams_root or not cams_root.IsValid():
        result["errors"].append("/World/cams 不存在（load_scene 没跑？）")
        return result
    if not cams_root.IsActive():
        # cameras.auto_register=false 时会被 SetActive(False)，图拿不到相机 prim
        cams_root.SetActive(True)

    # 常量输入（每个节点都一样）
    const_inputs = [
        ("Img_{i}.inputs:type", "rgb"),
        ("Img_{i}.inputs:nodeNamespace", node_namespace),
        ("Info_{i}.inputs:nodeNamespace", node_namespace),
        ("Img_{i}.inputs:qosProfile", image_qos),
        ("Info_{i}.inputs:qosProfile", info_qos),
    ]

    nodes = [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick")]
    values = []
    connects = []
    for i, (role, spec) in enumerate(cameras.items()):
        prim_path = spec["prim_path"]
        if not stage.GetPrimAtPath(prim_path).IsValid():
            result["errors"].append(f"{role}: prim 不存在 {prim_path}")
            continue
        w = int(spec.get("width", 1280))
        h = int(spec.get("height", 800))
        frame_id = spec.get("frame_id", f"{role}_optical_link")
        topic = spec.get("topic", f"{role}/color/image_raw")
        info_topic = spec.get("info_topic", f"{role}/color/camera_info")

        nodes += [(f"RP_{i}", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                  (f"Img_{i}", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                  (f"Info_{i}", "isaacsim.ros2.bridge.ROS2CameraInfoHelper")]
        values += [
            (f"RP_{i}.inputs:cameraPrim", [usdrt.Sdf.Path(prim_path)]),
            (f"RP_{i}.inputs:width", w),
            (f"RP_{i}.inputs:height", h),
            (f"Img_{i}.inputs:topicName", topic),
            (f"Img_{i}.inputs:frameId", frame_id),
            (f"Info_{i}.inputs:topicName", info_topic),
            (f"Info_{i}.inputs:frameId", frame_id),
        ]
        for pat, val in const_inputs:
            values.append((pat.format(i=i), val))
        connects += [
            ("OnPlaybackTick.outputs:tick", f"RP_{i}.inputs:execIn"),
            (f"RP_{i}.outputs:execOut", f"Img_{i}.inputs:execIn"),
            (f"RP_{i}.outputs:execOut", f"Info_{i}.inputs:execIn"),
            (f"RP_{i}.outputs:renderProductPath",
             f"Img_{i}.inputs:renderProductPath"),
            (f"RP_{i}.outputs:renderProductPath",
             f"Info_{i}.inputs:renderProductPath"),
        ]
        # 发布频率由相机 prim 的 tickRate 控制（helper 的 frameSkipCount 已废弃）
        try:
            prim = stage.GetPrimAtPath(prim_path)
            attr = prim.GetAttribute("omni:sensor:tickRate")
            if not attr:
                attr = prim.CreateAttribute("omni:sensor:tickRate",
                                            Sdf.ValueTypeNames.Float)
            attr.Set(float(fps))
        except Exception as e:
            result["errors"].append(f"{role}: 设置 tickRate 失败 {e}")

        result["cameras"][role] = {
            "prim_path": prim_path, "width": w, "height": h,
            "frame_id": frame_id,
            "image_topic": f"{node_namespace.rstrip('/')}/{topic}",
            "info_topic": f"{node_namespace.rstrip('/')}/{info_topic}",
        }

    if publish_clock:
        nodes.append(("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"))
        values += [("Clock.inputs:topicName", "clock"),
                   ("Clock.inputs:nodeNamespace", "")]
        connects.append(("OnPlaybackTick.outputs:tick", "Clock.inputs:execIn"))

    try:
        # 旧图先删掉，保证幂等
        old = stage.GetPrimAtPath(graph_path)
        if old and old.IsValid():
            stage.RemovePrim(graph_path)
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: nodes,
                og.Controller.Keys.SET_VALUES: values,
                og.Controller.Keys.CONNECT: connects,
            },
        )
    except Exception as e:
        result["errors"].append(f"建图失败: {type(e).__name__}: {e}")
    return result


def remove_camera_graph(graph_path: str) -> dict:
    """删掉相机图（切换/关闭相机话题时用）。"""
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(graph_path)
    if prim and prim.IsValid():
        stage.RemovePrim(graph_path)
        return {"removed": True, "graph_path": graph_path}
    return {"removed": False, "graph_path": graph_path}