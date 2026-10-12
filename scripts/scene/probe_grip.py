"""夹爪探针: 把两侧夹爪 leader 目标打到 GRIP_TARGET, 然后把全部关节的
位置/目标/限位 dump 到 /tmp/probe_state.json。

用途: 验证 (a) mimic 链是否保持刚性(跟随关节比值是否恒定),
(b) 夹爪行程是否覆盖全开/全闭, (c) 身体(lift/torso/head)是否会塌。

GRIP_TARGET 由外部 sed 替换成浮点数后 POST /python。
"""
import json

import numpy as np

art = executor.articulation  # noqa: F821
names = list(art.dof_names)

for side in ("left", "right"):
    n = f"{side}_gripper_right_2_joint"
    if n in names:
        art.set_joint_position_targets(
            np.asarray([[GRIP_TARGET]], dtype=np.float32),
            joint_indices=[names.index(n)])

pos = np.asarray(art.get_joint_positions()).reshape(-1).astype(float)
try:
    tgt = np.asarray(
        art._physics_view.get_dof_position_targets()).reshape(-1).astype(float)
except Exception:
    tgt = np.full(len(names), np.nan)
try:
    lim = np.asarray(art.get_dof_limits()).reshape(-1, 2).astype(float)
except Exception:
    lim = np.full((len(names), 2), np.nan)

# 指垫间距(物理开口宽度 mm): 两指 pad 连杆中心距, 最直观的"夹爪开合"证据
pads = {}
try:
    p, _q = art.get_world_poses()
    p = np.asarray(p).reshape(-1, 3).astype(float)
    for side in ("left", "right"):
        a, b = f"{side}_gripper_right_pad_link", f"{side}_gripper_left_pad_link"
        ia, ib = art.get_link_index(a), art.get_link_index(b)
        pads[side] = float(np.linalg.norm(p[int(ia)] - p[int(ib)]) * 1000.0)
except Exception as e:
    pads["err"] = repr(e)[:120]

with open("/tmp/probe_state.json", "w") as f:
    json.dump({"target_requested": GRIP_TARGET,
               "pads_mm": pads,
               "names": names,
               "pos": pos.tolist(),
               "tgt": tgt.tolist(),
               "lim": lim.tolist()}, f)