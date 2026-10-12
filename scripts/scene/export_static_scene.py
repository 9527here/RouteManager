"""把当前场景的静态内容(相机/桌子/物体/参考球)导出为 static_scene.usda。

相机从 /humanoid/<link>/... 迁移到 /World/cams/<name> 并**烘焙世界位姿**——
静态场景必须在机器人之前加载(避免运行时 prim 编辑使物理后端失效),
而那时 /humanoid 还不存在, 故相机不能再挂机器人链上。
导出后: load_scene 在加载机器人前 reference 本文件。
"""
import json
import os

import numpy as np
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom

stage = omni.usd.get_context().get_stage()
xc = UsdGeom.XformCache(Usd.TimeCode.Default())
src = stage.GetRootLayer()
dst_path = os.path.join(os.getcwd(), "scripts", "scene", "static_scene.usda")
out = {"copied": {}, "cams": {}}

CAMS7 = [
    ("head_zed_x", "/humanoid/head_zed_x_link/head_zed_x_optical_link/"
                   "head_zed_x_cam_v2"),
    ("head_surround_left", "/humanoid/head_surround_left_link/"
                           "head_surround_left_optical_link/"
                           "head_surround_left_cam_v2"),
    ("head_surround_right", "/humanoid/head_surround_right_link/"
                            "head_surround_right_optical_link/"
                            "head_surround_right_cam_v2"),
    ("head_surround_rear", "/humanoid/head_surround_rear_link/"
                           "head_surround_rear_optical_link/"
                           "head_surround_rear_cam_v2"),
    ("chest_gemini_335lg", "/humanoid/chest_gemini_335lg_link/"
                           "chest_gemini_335lg_optical_link/"
                           "chest_gemini_cam_v2"),
    ("left_wrist_gemini_335lg", "/humanoid/left_wrist_gemini_335lg_link/"
                                "left_wrist_gemini_335lg_optical_link/"
                                "left_wrist_gemini_cam_v2"),
    ("right_wrist_gemini_335lg", "/humanoid/right_wrist_gemini_335lg_link/"
                                 "right_wrist_gemini_335lg_optical_link/"
                                 "right_wrist_gemini_cam_v2"),
]
WORLD_PRIMS = [
    ("/World/vla_table", "/World/vla_table"),
    ("/World/top_view_cam", "/World/top_view_cam"),
    ("/World/teleop_view_cam", "/World/teleop_view_cam"),
    ("/World/ref_red", "/World/ref_red"),
    ("/World/ref_green", "/World/ref_green"),
    ("/World/ref_blue", "/World/ref_blue"),
]

if os.path.exists(dst_path):
    os.remove(dst_path)
dst = Sdf.Layer.CreateNew(dst_path)

for sp, dp in WORLD_PRIMS:
    try:
        Sdf.CopySpec(src, Sdf.Path(sp), dst, Sdf.Path(dp))
        out["copied"][dp] = "ok"
    except Exception as e:
        out["copied"][dp] = "ERR:" + repr(e)[:100]

# 相机: 拷到 /World/cams/<name> 并写世界位姿
world_pose = {}
for nm, leaf in CAMS7:
    sp = f"/World/cams/{nm}"
    try:
        if not stage.GetPrimAtPath(leaf).IsValid():
            out["cams"][nm] = "SRC_MISSING"
            continue
        Sdf.CopySpec(src, Sdf.Path(leaf), dst, Sdf.Path(sp))
        M = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(leaf))
        world_pose[nm] = (M.ExtractTranslation(),
                          M.ExtractRotationQuat())
        out["cams"][nm] = "copied"
    except Exception as e:
        out["cams"][nm] = "ERR:" + repr(e)[:100]

dst.Save()

# 打开导出层, 写世界位姿(静态场景没有父链, local = world)
dstage = Usd.Stage.Open(dst_path)
for nm, (t, q) in world_pose.items():
    prim = dstage.GetPrimAtPath(f"/World/cams/{nm}")
    if not prim.IsValid():
        continue
    prim.CreateAttribute("xformOp:translate",
                         Sdf.ValueTypeNames.Double3).Set(
        (float(t[0]), float(t[1]), float(t[2])))
    prim.CreateAttribute("xformOp:orient",
                         Sdf.ValueTypeNames.Quatd).Set(q)
    prim.CreateAttribute("xformOpOrder",
                         Sdf.ValueTypeNames.StringArray).Set(
        ["xformOp:translate", "xformOp:orient"])
    out["cams"][nm] += "+pose"
dstage.Save()

with open("/tmp/export_static.json", "w") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("OK")