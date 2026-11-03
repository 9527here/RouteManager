#!/usr/bin/env python3
"""标定 URDF FK 与仿真 link 帧之间的常量变换。

背景：URDF 关节定义与资产里 USD 的 `localPos0`/`localRot0` 逐项一致，
但 PhysX 上报的 link 帧相对 URDF link 帧还差一个常量刚体变换（导入器的
frame 约定），表现为"锚点位置对得上、一动就整体转 90°"。

做法：采 N 组 q → 法兰位姿，用 Procrustes 拟合
    p_sim(q) = A @ t_fk(q) + b
    R_sim(q) = A @ R_fk(q) @ B
A 必须是正交阵（同一 A 同时解释位置和朝向）。留出若干组做验证，
残差 < 5mm 就说明"URDF FK + 常量变换"能准确描述这台机器人。

产物: scripts/arm_calib.json  (A / b / B / 各类残差 / 采样位形)

用法: python3 scripts/calibrate_arm.py [--n 10] [--holdout 3]
"""
import argparse
import json
import math
import os
import sys
import time
import urllib.request

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _REPO)
import arm_kin  # noqa: E402

PORT = int(os.environ.get("ROUTE_MANAGER_PORT", "18765"))
BASE = f"http://127.0.0.1:{PORT}"
SIDE = "right"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_calib.json")


def post(action, params=None, timeout=90):
    body = json.dumps({"action": action, "parameters": params or {}}).encode()
    req = urllib.request.Request(f"{BASE}/execute", data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def wait_idle(timeout=120):
    t0 = time.time()
    while time.time() - t0 < timeout:
        with urllib.request.urlopen(f"{BASE}/health", timeout=5) as r:
            if json.loads(r.read().decode()).get("status") == "IDLE":
                return True
        time.sleep(0.3)
    raise SystemExit("服务一直不 IDLE")


def quat_wxyz_to_R(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def sample(chain):
    st = post("get_sim_state", {"ref_link": ""})["data"]
    names, pos = st["joint_names"], st["positions"]
    q = np.array([pos[names.index(n)] for n in arm_kin.joint_names(chain)], float)
    f = st["sides"][SIDE]["flange"]
    return q, np.array(f["position"], float), quat_wxyz_to_R(*f["orientation"])


def move(chain, q, dur=1.8):
    wait_idle()
    post("move_j", {"side": SIDE, "positions": [float(v) for v in q],
                    "duration": dur})
    time.sleep(dur + 2.5)
    wait_idle()


def angle_deg(A, B):
    R = A.T @ B
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))


def fit_rigid(P_model, P_sim):
    """Procrustes: 求 R,t 使 sim ≈ R@model + t。"""
    cm, cs = P_model.mean(0), P_sim.mean(0)
    H = (P_model - cm).T @ (P_sim - cs)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, cs - R @ cm


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--holdout", type=int, default=3)
    ap.add_argument("--amp", type=float, default=0.30, help="随位形幅度(rad)")
    a = ap.parse_args()

    cfg = json.load(open(os.path.join(_REPO, "config.json")))
    preset = cfg["robot"]["presets"][cfg["robot"]["name"]]
    chain = arm_kin.load_chain(preset["urdf_path"], f"{SIDE}_arm_base_link",
                               f"{SIDE}_gripper_base_link")
    lim = arm_kin.joint_limits(chain)
    names = arm_kin.joint_names(chain)

    q_home, _, _ = sample(chain)
    print(f"[home] q={np.round(q_home,3).tolist()}")

    # 确定性扰动序列：对每个关节依次正负偏一点，覆盖不同方向
    rng = np.random.default_rng(0)
    poses = []
    for i in range(a.n):
        q = q_home.copy()
        k = rng.integers(0, len(q))
        q[k] += a.amp * (1 if i % 2 == 0 else -1)
        q = np.clip(q, lim[:, 0] + 0.08, lim[:, 1] - 0.08)
        poses.append(q)

    data = []
    for i, q_t in enumerate(poses):
        move(chain, q_t)
        q, p, R = sample(chain)
        R_fk, t_fk = arm_kin.fk(chain, q)
        data.append({"q": q.tolist(), "p": p.tolist(), "R": R.tolist(),
                     "t_fk": t_fk.tolist(), "R_fk": R_fk.tolist(),
                     "track_err": float(np.abs(q - q_t).max())})
        print(f"  [{i+1}/{len(poses)}] 跟踪误差 {data[-1]['track_err']*1000:.1f}mrad")

    move(chain, q_home)

    # 拟合用前 n-holdout 组，后面留作验证
    fit = data[:-a.holdout] if a.holdout else data
    P_m = np.array([d["t_fk"] for d in fit])
    P_s = np.array([d["p"] for d in fit])
    A, b = fit_rigid(P_m, P_s)

    # 朝向常量 B：R_sim ≈ A @ R_fk @ B
    Bs = []
    for d in fit:
        Bs.append(np.array(d["R_fk"]).T @ A.T @ np.array(d["R"]))
    B = Bs[0]
    for M in Bs[1:]:                       # 在 SO(3) 上取平均（足够用的一阶做法）
        B = B + M
    U, _, Vt = np.linalg.svd(B)
    B = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt

    print()
    print(f"{'#':<4}{'位置残差(mm)':>13}{'朝向残差(°)':>13}   {'集合':<6}")
    print("-" * 42)
    res = []
    for i, d in enumerate(data):
        pm = np.array(d["t_fk"])
        p_pred = A @ pm + b
        dp = float(np.linalg.norm(p_pred - np.array(d["p"]))) * 1000
        Rp = A @ np.array(d["R_fk"]) @ B
        da = angle_deg(Rp, np.array(d["R"]))
        tag = "验证" if (a.holdout and i >= len(data) - a.holdout) else "拟合"
        res.append({"pos_mm": dp, "rot_deg": da, "set": tag})
        print(f"{i:<4}{dp:>13.2f}{da:>13.2f}   {tag}")

    fit_r = [r["pos_mm"] for r in res if r["set"] == "拟合"]
    val_r = [r["pos_mm"] for r in res if r["set"] == "验证"]
    rot_r = [r["rot_deg"] for r in res]
    print()
    print(f"[拟合] 位置残差 max {max(fit_r):.2f}mm  mean {np.mean(fit_r):.2f}mm")
    if val_r:
        print(f"[验证] 位置残差 max {max(val_r):.2f}mm  mean {np.mean(val_r):.2f}mm")
    print(f"[朝向] 残差 max {max(rot_r):.2f}°")
    ok = (not val_r or max(val_r) < 5.0) and max(rot_r) < 2.0
    print("[结论] " + ("模型成立 ✅ —— 常量变换即可描述 URDF FK 与仿真的对应"
                      if ok else "模型不成立 ❌ —— 偏差不是单一常量变换，需查资产"))

    json.dump({"A": A.tolist(), "b": b.tolist(), "B": B.tolist(),
               "joint_names": names, "urdf": preset["urdf_path"],
               "q_ref": q_home.tolist(), "residuals": res,
               "ok": bool(ok), "samples": len(data)},
              open(OUT, "w"), indent=1)
    print(f"[保存] {OUT}")


if __name__ == "__main__":
    main()
