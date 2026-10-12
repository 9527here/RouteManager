"""s5b(二分段: 仅 visibility 隐藏): 7 挂载连杆 visuals/collisions 设 invisible。"""
import json

import omni.usd
from pxr import UsdGeom

stage = omni.usd.get_context().get_stage()
LINKS = ["head_zed_x_link", "head_surround_left_link", "head_surround_right_link",
         "head_surround_rear_link", "chest_gemini_335lg_link",
         "left_wrist_gemini_335lg_link", "right_wrist_gemini_335lg_link"]
out = {}
for ln in LINKS:
    r = {}
    for child in ("visuals", "collisions"):
        p = stage.GetPrimAtPath(f"/humanoid/{ln}").GetChild(child)
        if p.IsValid():
            p.GetAttribute("visibility").Set("invisible")
            r[child] = "invisible"
        else:
            r[child] = "NONE"
    out[ln] = r
with open("/tmp/cam_setup_s5b.json", "w") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("OK")
