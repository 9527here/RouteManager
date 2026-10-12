"""隐藏 7 个相机挂载连杆的 visuals/collisions(静态场景流程专用)。

原隐藏逻辑挂在 add_camera() 里(robot_manager._hide_mount_visuals), 但静态场景
流程不再调用 add_camera(相机已在 static_scene.usda 里), 所以需要本脚本显式执行。
机器人加载后执行安全(改已有 prim 属性/根层 over 不会让物理后端失效)。
"""
import json

import omni.usd
from pxr import Sdf, Usd

LINKS = [
    "head_zed_x_link",
    "head_surround_left_link",
    "head_surround_right_link",
    "head_surround_rear_link",
    "chest_gemini_335lg_link",
    "left_wrist_gemini_335lg_link",
    "right_wrist_gemini_335lg_link",
]

stage = omni.usd.get_context().get_stage()
root_layer = stage.GetRootLayer()
out = {}

for ln in LINKS:
    rec = {}
    link_prim = stage.GetPrimAtPath(f"/humanoid/{ln}")
    if not link_prim.IsValid():
        out[ln] = "LINK_MISSING"
        continue
    # 1) 实例 prim 的 visibility(可见性会传入实例)
    for child in ("visuals", "collisions"):
        p = link_prim.GetChild(child)
        if p.IsValid():
            p.GetAttribute("visibility").Set("invisible")
            rec[child] = "invisible"
        else:
            rec[child] = "NONE"
    # 2) Gprim 兜底(万一有非实例化的网格)
    ng = 0
    for p in Usd.PrimRange(link_prim):
        if p.GetTypeName() in ("Mesh", "Cube", "Sphere", "Cylinder"):
            p.GetAttribute("visibility").Set("invisible")
            ng += 1
    rec["gprims"] = ng
    # 3) 碰撞禁用改为**离线烘焙**进机器人 USD(见 bake_collision_disable.py):
    #    运行时在根层 CreatePrimInLayer 写 over 算"新增 prim" → 会让物理后端失效.
    rec["collision_over"] = "baked_offline"
    out[ln] = rec

with open("/tmp/hide_mounts.json", "w") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("OK")