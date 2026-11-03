#!/usr/bin/env python3
"""逐关节扫描：把 arm_kin 的预测位移和真机实测位移摆在一起比。

用途：区分两类根因——
  * 每个关节的误差**方向一致**（同一旋转量） -> 全局基底/上报姿态问题（R_f）
  * 只有**个别关节**误差大                    -> 那个关节的 axis 建模错

每轮流程：回 q0 -> 只把一个关节转 +delta -> 读回实测 -> 比较位移向量。
位移向量只依赖"动了哪个关节"，与锚点求解无关，所以不会被 R_ab 吸收掉。

用法: python3 scripts/anchor_scan.py [--delta 0.25] [--joints 0,1,2,3,4,5]
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
    raise SystemExit("服务一直不 IDLE，中止")


def read_state(chain):
    """返回 (q, R_f, p_f)。"""
    st = post("get_sim_state", {"ref_link": ""})["data"]
    names, pos = st["joint_names"], st["positions"]
    q = np.array([pos[names.index(n)] for n in arm_kin.joint_names(chain)], float)
    f = st["sides"][SIDE]["flange"]
    w, x, y, z = f["orientation"]
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return q, R, np.array(f["position"], float)


def move(q, dur=1.6):
    wait_idle()
    post("move_j", {"side": SIDE, "positions": [float(v) for v in q],
                    "duration": dur})
    time.sleep(dur + 2.0)
    wait_idle()


def rot_angle_deg(u, v):
    nu, nv = np.linalg.norm(u), np.linalg.norm(v)
    if nu < 1e-9 or nv < 1e-9:
        return float("nan")
    c = float(np.clip(np.dot(u, v) / (nu * nv), -1.0, 1.0))
    return math.degrees(math.acos(c))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--delta", type=float, default=0.25)
    ap.add_argument("--joints", default="0,1,2,3,4,5")
    a = ap.parse_args()
    idxs = [int(s) for s in a.joints.split(",") if s.strip() != ""]

    cfg = json.load(open(os.path.join(_REPO, "config.json")))
    preset = cfg["robot"]["presets"][cfg["robot"]["name"]]
    chain = arm_kin.load_chain(preset["urdf_path"], f"{SIDE}_arm_base_link",
                               f"{SIDE}_gripper_base_link")
    names = arm_kin.joint_names(chain)
    lim = arm_kin.joint_limits(chain)

    q0, R_f, p_f = read_state(chain)
    R_fk, t_fk = arm_kin.fk(chain, q0)
    R_ab = R_f @ R_fk.T
    t_ab = p_f - R_ab @ t_fk
    p0, _ = arm_kin.ee_world(chain, q0, R_ab, t_ab)
    print(f"[base] q0={np.round(q0,3).tolist()}")
    print(f"[base] 锚点残差 {np.linalg.norm(p0 - p_f)*1000:.2f}mm")
    print()
    print(f"{'关节':<18}{'预测位移':<34}{'实测位移':<34}"
          f"{'差(mm)':>9}{'夹角(°)':>9}")
    print("-" * 106)

    rows = []
    for i in idxs:
        delta = a.delta
        if q0[i] + delta > lim[i, 1] - 0.05:
            delta = -delta
        q1 = q0.copy()
        q1[i] += delta

        p_pred, _ = arm_kin.ee_world(chain, q1, R_ab, t_ab)
        d_pred = p_pred - p0

        move(q1)
        _, _, p_a = read_state(chain)
        d_real = p_a - p_f

        err = float(np.linalg.norm(d_pred - d_real))
        # arm_4 / arm_6 绕工具轴转，末端原点本来就不动；对亚毫米向量算夹角
        # 只是噪声（曾把 0.9mm 的抖动报成 91.7°"轴不对"）。位移太小就只比幅值。
        tiny = float(np.linalg.norm(d_pred)) < 0.005
        ang = float("nan") if tiny else rot_angle_deg(d_pred, d_real)
        if tiny:
            flag = "" if err < 0.003 else "  ← 位移异常"
        else:
            flag = "" if ang < 15 else ("  ← 轴不对" if ang < 165 else "  ← 方向反了")
        print(f"{names[i]:<18}{np.round(d_pred,4).tolist()!s:<34}"
              f"{np.round(d_real,4).tolist()!s:<34}{err*1000:>9.1f}{ang:>9.1f}{flag}")
        rows.append((names[i], err, ang))

        move(q0)

    print()
    angs = [r[2] for r in rows if not math.isnan(r[2])]
    if angs:
        spread = max(angs) - min(angs)
        print(f"[判定] 各关节夹角 平均{sorted(angs)[len(angs)//2]:.1f}° "
              f"极差{spread:.1f}°")
        if spread < 20 and min(angs) > 20:
            print("  -> 每个关节都被同一角度旋转：全局基底/上报姿态问题（查 R_f 约定）")
        elif max(angs) > 130:
            print("  -> 存在方向相反的关节：该关节 axis 符号错（URDF 与 USD 不一致）")
        elif any(x > 20 for x in angs):
            print("  -> 只有部分关节偏：这些关节的 axis 建模错")
        else:
            print("  -> 全部吻合 ✅")


if __name__ == "__main__":
    main()
