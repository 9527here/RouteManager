"""离线把关节驱动阻尼烘焙进机器人 physics.usd(本地运行, 不需要 Isaac 服务)。

解决"底盘动就晃、动机械臂整机晃": URDF 导入的驱动阻尼极小(手臂≈1.09,
轮子≈0.08), 位置伺服欠阻尼振荡。运行时 set_gains 会触发 PhysX
"target angle 超 ±2π"刷错(轮子类关节目标累积), 故改为**离线烘焙**。
"""
import os
import shutil
import sys

from pxr import Usd, UsdPhysics

P = ("/home/jy/datasets/vla_data/L1_制作资产/载体设备/"
     "humanbody_urdf/humanoid_upper_body_v3/usd/"
     "humanoid_upper_body_v3_ctag2f120_cameras_isaac/configuration/"
     "humanoid_upper_body_v3_ctag2f120_cameras_isaac_physics.usd")
BAK = P + ".pre_damping_bak"

if not os.path.exists(BAK):
    shutil.copy2(P, BAK)
    print("backup ->", BAK)

# 目标阻尼: 按关节名分组
# USD 值 × 57.3 ≈ 物理实际值; 目标: 抑制"动起来晃"但不引入伺服抖动
# (实测 arm 烘 20(实际1146) 抖、原值 1.09 晃 → 取中间)
TARGETS = [
    # 实测: kps 155577 / kds 85.9 → 阻尼比 0.0006、数值欠阻尼 9 倍 → 必然振荡.
    # 对策: 降刚度到数值稳定区 + 阻尼匹配 (USD 值 × 57.3 = 实际值)
    ("_arm_", 150.0, 20.0),      # → 实际 kps 8595 / kds 1146 (阻尼比 ~1.5? 见下)
    ("wheel", 100.0, 3.0),
    ("caster", 50.0, 2.0),
    ("lift", 150.0, 20.0),
    ("torso", 150.0, 20.0),
    ("head", 150.0, 10.0),
    ("gripper", 100.0, 5.0),
]   # caster 不处理: 保持原始驱动(被动化会让底盘支撑变差 → 基座抖)

stage = Usd.Stage.Open(P)
changed = {}
for prim in stage.Traverse():
    if not prim.IsA(UsdPhysics.Joint):
        continue
    nm = str(prim.GetName())
    stiff = damp = None
    for key, s_val, d_val in TARGETS:
        if key in nm:
            stiff, damp = s_val, d_val
            break
    if damp is None:
        continue
    a = prim.GetAttribute("drive:angular:physics:damping")
    ka = prim.GetAttribute("drive:angular:physics:stiffness")
    if not a.IsValid():
        continue
    old_d = a.Get()
    old_s = ka.Get() if ka.IsValid() else None
    a.Set(float(damp))
    if ka.IsValid():
        ka.Set(float(stiff))
    changed[nm] = {"old_damping": old_d, "new_damping": damp,
                   "old_stiff": old_s, "new_stiff": stiff}
stage.Save()

print(f"changed {len(changed)} joints")
for k, v in list(changed.items())[:10]:
    print(f"  {k}: kps {v['old_stiff']}->{v['new_stiff']} kds {v['old_damping']}->{v['new_damping']}")