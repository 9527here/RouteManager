"""调关节驱动(阻尼/刚度)——解决"底盘动就晃、动机械臂整机晃"。

原因: URDF 导入的驱动阻尼极小(手臂 damping≈1.09, 轮子≈0.08), 位置伺服在
欠阻尼下振荡, 且手臂快速运动的反作用力让轮式底盘整机晃动。
必须在物理视图健康时执行(load_scene 之后、VR 启动前皆可; 静态场景方案下
视图全程健康)。设置目标: 手臂/轮子/caster/躯干 分别加重阻尼。
"""
import json

import numpy as np

art = executor.articulation  # noqa: F821
dn = list(art.dof_names)
out = {"groups": {}, "before": {}, "after": {}}

# 目标阻尼(经验值: 抑制振荡, 不牺牲跟手性)
GROUPS = {
    "arm": (lambda n: "arm_" in n, 20.0),
    "wheel": (lambda n: "wheel" in n, 3.0),
    "caster": (lambda n: "caster" in n, 5.0),
    "torso_head": (lambda n: "lift" in n or "torso" in n or "head" in n, 15.0),
    "gripper": (lambda n: "gripper" in n, 5.0),
}

try:
    kps, kds = art.get_gains()
    kps = np.asarray(kps, dtype=np.float32).reshape(-1)
    kds = np.asarray(kds, dtype=np.float32).reshape(-1)
    out["before"] = {"kds_min": float(kds.min()), "kds_max": float(kds.max())}
except Exception as e:
    out["before"] = "ERR:" + repr(e)[:120]
    kps = kds = None

if kds is not None and len(kds) == len(dn):
    for tag, (pred, damp) in GROUPS.items():
        idxs = [i for i, n in enumerate(dn) if pred(n)]
        if not idxs:
            continue
        old = [round(float(kds[i]), 3) for i in idxs]
        kds[idxs] = damp
        out["groups"][tag] = {"n": len(idxs), "old_kds": old[:4], "new": damp}
    try:
        art.set_gains(kps=kps.reshape(1, -1), kds=kds.reshape(1, -1))
        out["set_gains"] = "OK"
    except Exception as e:
        out["set_gains"] = "ERR:" + repr(e)[:140]
    try:
        kps2, kds2 = art.get_gains()
        out["after"] = {"kds_min": float(np.asarray(kds2).min()),
                        "kds_max": float(np.asarray(kds2).max())}
    except Exception as e:
        out["after"] = repr(e)[:80]
else:
    out["err"] = f"gains shape mismatch: {None if kds is None else len(kds)} vs {len(dn)}"

with open("/tmp/tune_drives.json", "w") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("OK")