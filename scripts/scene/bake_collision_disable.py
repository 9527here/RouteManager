"""离线把 7 个相机挂载连杆的碰撞禁用烘焙进机器人 physics.usd。

为什么离线: 运行时在根层给 /colliders/<link> 写 over(CreatePrimInLayer)算
"新增 prim" → 会让物理后端失效(关节读写全废)。烘焙则加载即生效、零运行时编辑。
"""
import os
import shutil

from pxr import Usd, UsdPhysics

P = ("/home/jy/datasets/vla_data/L1_制作资产/载体设备/humanbody_urdf/"
     "humanoid_upper_body_v3/usd/humanoid_upper_body_v3_ctag2f120_cameras_isaac/"
     "configuration/humanoid_upper_body_v3_ctag2f120_cameras_isaac_physics.usd")
BAK = P + ".pre_collision_bak"
if not os.path.exists(BAK):
    shutil.copy2(P, BAK)
    print("backup ->", BAK)

LINKS = [
    "head_zed_x_link",
    "head_surround_left_link",
    "head_surround_right_link",
    "head_surround_rear_link",
    "chest_gemini_335lg_link",
    "left_wrist_gemini_335lg_link",
    "right_wrist_gemini_335lg_link",
]

stage = Usd.Stage.Open(P)
done = []
for ln in LINKS:
    for pth in (f"/colliders/{ln}", f"/colliders/{ln}/mesh_0"):
        p = stage.GetPrimAtPath(pth)
        if not p.IsValid():
            continue
        UsdPhysics.CollisionAPI.Apply(p)          # 幂等
        a = p.GetAttribute("physics:collisionEnabled")
        if not a.IsValid():
            a = p.CreateAttribute("physics:collisionEnabled",
                                  UsdPhysics.Tokens.physicsCollisionEnabled
                                  if hasattr(UsdPhysics, "Tokens") else None)
        a.Set(False)
        done.append(pth)
stage.Save()
print(f"disabled collision on {len(done)} prims:")
for d in done:
    print("  ", d)