"""标定手臂 Jacobian 表: 采样肩/肘关节网格, 记录 EE 位姿 + Jacobian。

必须在 **load_scene 之后、任何 prim 编辑(加相机/桌子)之前** 执行——物理视图
在 prim 编辑后永久失效, 届时 get_jacobians/get_world_poses 均不可用。
输出 /tmp/jac_table.json 供运行时 DLS IK(isaac_teleop.py)使用。
"""
import json

import numpy as np

art = executor.articulation  # noqa: F821
out = {"arms": {}, "samples": []}
dn = list(art.dof_names)
lim = np.asarray(art.get_dof_limits()).reshape(-1, 2)   # (N,2) 弧度
out["n_dof"] = len(dn)

# EE link: 手掌基座(随腕刚性, 手柄映射到"手掌"最自然)
EE = {"left": "left_gripper_base_link", "right": "right_gripper_base_link"}
ARM = {}
for side in ("left", "right"):
    idxs = [i for i, n in enumerate(dn)
            if n.startswith(f"{side}_arm_") and n.endswith("_joint")]
    ARM[side] = idxs
    out["arms"][side] = [dn[i] for i in idxs]

ee_idx = {}
for side, ln in EE.items():
    try:
        ee_idx[side] = int(art.get_link_index(ln))
    except Exception as e:
        out["arms"][f"{side}_link_err"] = repr(e)[:100]

# 当前姿态(抬臂 home)作为采样中心
q0 = np.asarray(art.get_joint_positions()).reshape(-1).astype(np.float64)
base_q = q0.copy()

# 肩/肘(前 3 关节)采样: home ± 0.8 rad 三值; 腕(4-6)固定 home
offsets = (-0.8, 0.0, 0.8)
SPAN = 0.9   # 关节限位保护余量(rad)

def _clamped(v, i):
    return float(np.clip(v, lim[i][0] + 0.05, lim[i][1] - 0.05))

n_done = 0
for side in ("left", "right"):
    aidx = ARM[side]
    if not aidx or side not in ee_idx:
        continue
    for d1 in offsets:
        for d2 in offsets:
            for d3 in offsets:
                cfg = base_q.copy()
                vals = [q0[aidx[0]] + d1, q0[aidx[1]] + d2, q0[aidx[2]] + d3]
                for k in range(3):
                    if k < len(aidx):
                        cfg[aidx[k]] = _clamped(vals[k], aidx[k])
                for k in range(3, len(aidx)):
                    cfg[aidx[k]] = _clamped(q0[aidx[k]], aidx[k])
                try:
                    art.set_joint_positions(cfg.reshape(1, -1).astype(np.float32))
                    for _ in range(3):
                        executor.world.step(render=False)  # noqa: F821
                    # EE 世界位姿走 tensor view 的 link transforms((N,7) pos+quat)
                    lt = np.asarray(
                        art._physics_view.get_link_transforms()).reshape(-1, 7)
                    pos = lt[ee_idx[side]][:3]
                    quat = lt[ee_idx[side]][3:]   # wxyz
                    J = np.asarray(art.get_jacobians())
                    Jee = J[0][ee_idx[side]][:, aidx]         # (6, n_arm)
                    out["samples"].append({
                        "side": side,
                        "q": [round(float(cfg[i]), 5) for i in aidx],
                        "pos": [round(float(v), 5) for v in pos],
                        "quat": [round(float(v), 5) for v in quat],
                        "J": np.round(Jee, 6).tolist(),
                    })
                    n_done += 1
                except Exception as e:
                    out.setdefault("sample_errs", []).append(repr(e)[:120])

# 恢复抬臂姿态
try:
    art.set_joint_positions(base_q.reshape(1, -1).astype(np.float32))
    for _ in range(30):
        executor.world.step(render=False)  # noqa: F821
    out["restore"] = "ok"
except Exception as e:
    out["restore"] = repr(e)[:120]

out["n_samples"] = n_done
# 全 50 维 home 关节角: 运行时全量写入关节目标的基底(视图失效后无法再读)
out["home_q"] = [round(float(v), 6) for v in base_q]

# 基座世界位姿(手柄位移 → 基座系 转换用; 此刻矩阵可信)
try:
    lt0 = np.asarray(
        art._physics_view.get_link_transforms()).reshape(-1, 7)
    bi = int(art.get_link_index("base_link"))
    out["base_pos"] = [round(float(v), 5) for v in lt0[bi][:3]]
    out["base_quat"] = [round(float(v), 5) for v in lt0[bi][3:]]  # wxyz
except Exception as e:
    out["base_err"] = repr(e)[:120]

import os
# 服务器 cwd = RouteManager 根; __file__ 在 exec 环境下不存在
persist = os.path.join(os.getcwd(), "scripts", "scene", "jac_table.json")
with open(persist, "w") as f:
    json.dump(out, f)
with open("/tmp/jac_table.json", "w") as f:
    json.dump(out, f)
out["persist_path"] = persist
print("OK")