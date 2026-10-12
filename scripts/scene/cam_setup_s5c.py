"""s5c(二分段: 仅 Sdf over 禁碰撞): 根层 /colliders/<link> physics:collisionEnabled=False。"""
import json

import omni.usd
from pxr import Sdf

stage = omni.usd.get_context().get_stage()
root_layer = stage.GetRootLayer()
LINKS = ["head_zed_x_link", "head_surround_left_link", "head_surround_right_link",
         "head_surround_rear_link", "chest_gemini_335lg_link",
         "left_wrist_gemini_335lg_link", "right_wrist_gemini_335lg_link"]
out = {}
for ln in LINKS:
    try:
        over_path = Sdf.Path(f"/colliders/{ln}")
        if not root_layer.GetPrimAtPath(over_path):
            Sdf.CreatePrimInLayer(root_layer, over_path)
        ps = root_layer.GetPrimAtPath(over_path)
        ps.specifier = Sdf.SpecifierOver
        a = ps.attributes.get("physics:collisionEnabled")
        if a is None:
            a = Sdf.AttributeSpec(ps, "physics:collisionEnabled",
                                  Sdf.ValueTypeNames.Bool)
        a.default = False
        out[ln] = "authored"
    except Exception as e:
        out[ln] = repr(e)[:150]
with open("/tmp/cam_setup_s5c.json", "w") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("OK")
