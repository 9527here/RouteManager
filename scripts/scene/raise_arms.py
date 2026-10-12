"""抬臂:把双臂肩关节设到抬起姿态(经 /python POST 在仿真主线程执行)。

必须在 load_scene 之后、**加相机之前**执行——相机 prim 的添加会让 articulation
的物理视图永久失效,届时 set_joint_positions 将无法工作。
dof 名带 _joint 后缀(left_arm_1_joint → -1.2 rad, right_arm_1_joint → +1.2 rad)。
"""
import json
import time

import numpy as np
from isaacsim.core.prims import SingleArticulation

out = {"init": "", "joints": {}}
art = SingleArticulation(prim_path="/humanoid", name="humanoid")  # noqa: F821
try:
    art.initialize()
    for _ in range(10):
        executor.world.step(render=False)  # noqa: F821
    out["init"] = "ok"
except Exception as e:
    out["init"] = "ERR:" + repr(e)[:200]

if out["init"] == "ok":
    try:
        q = np.asarray(art.get_joint_positions())
        dn = art.dof_names
        for i, nm in enumerate(dn):
            if nm.endswith("arm_1_joint"):
                q[i] = -1.2 if nm.startswith("left") else 1.2
                out["joints"][nm] = "set"
        art.set_joint_positions(q)
        for _ in range(40):
            executor.world.step(render=False)  # noqa: F821
        q2 = np.asarray(art.get_joint_positions())
        for i, nm in enumerate(dn):
            if nm.endswith("arm_1_joint"):
                out["joints"][nm] += f" -> {float(q2[i]):.3f}"
    except Exception as e:
        out["joints"]["err"] = repr(e)[:200]

with open("/tmp/raise_v3.json", "w") as f:
    json.dump(out, f, indent=1)
print("OK")