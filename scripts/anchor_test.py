#!/usr/bin/env python3
"""第二锚点校验：单独转一个关节，比较 arm_kin 预测 vs 真机实际位移。

用来判断「模型错」还是「目标不可达」：
  * 预测与实测差得远 -> arm_kin 的链（关节顺序/轴/端点）与加载的 USD 不一致
  * 对得上           -> 模型没问题，是可达范围或奇异问题

用法: python3 scripts/anchor_test.py [--joint 1] [--delta 0.3]
"""
import argparse
import json
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


def post(action, params=None, timeout=60):
    body = json.dumps({"action": action, "parameters": params or {}}).encode()
    req = urllib.request.Request(f"{BASE}/execute", data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def wait_idle(timeout=90):
    t0 = time.time()
    while time.time() - t0 < timeout:
        with urllib.request.urlopen(f"{BASE}/health", timeout=5) as r:
            if json.loads(r.read().decode()).get("status") == "IDLE":
                return True
        time.sleep(0.3)
    return False


def chain_and_q():
    cfg = json.load(open(os.path.join(_REPO, "config.json")))
    preset = cfg["robot"]["presets"][cfg["robot"]["name"]]
    chain = arm_kin.load_chain(preset["urdf_path"], f"{SIDE}_arm_base_link",
                               f"{SIDE}_gripper_base_link")
    st = post("get_sim_state", {"ref_link": ""})["data"]
    names, pos = st["joint_names"], st["positions"]
    want = arm_kin.joint_names(chain)
    q = np.array([pos[names.index(n)] for n in want], float)
    f = st["sides"][SIDE]["flange"]
    w, x, y, z = f["orientation"]
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return chain, q, R, np.array(f["position"], float), st


def anchor_of(chain, q, p_f, R_f):
    R_fk, t_fk = arm_kin.fk(chain, q)
    R_ab = R_f @ R_fk.T
    t_ab = p_f - R_ab @ t_fk
    return R_ab, t_ab


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", type=int, default=1, help="转第几个关节(0-based)")
    ap.add_argument("--delta", type=float, default=0.3)
    a = ap.parse_args()

    chain, q0, R_f, p_f, st = chain_and_q()
    names = arm_kin.joint_names(chain)
    print(f"[chain] {names}")

    # 1) 锚点
    R_ab, t_ab = anchor_of(chain, q0, p_f, R_f)
    p0, _ = arm_kin.ee_world(chain, q0, R_ab, t_ab)
    print(f"[anchor] 残差={np.linalg.norm(p0 - p_f)*1000:.2f}mm  q={np.round(q0,3).tolist()}")

    # 2) 模型预测：只动一个关节
    q1 = q0.copy()
    q1[a.joint] += a.delta
    p_pred, _ = arm_kin.ee_world(chain, q1, R_ab, t_ab)
    d_pred = p_pred - p0
    print(f"[predict] 关节{names[a.joint]} +{a.delta}rad -> 末端位移 {np.round(d_pred,4).tolist()}")

    # 3) 真机执行
    wait_idle()
    post("move_j", {"side": SIDE, "positions": [float(v) for v in q1], "duration": 2.0})
    time.sleep(4)
    wait_idle()

    # 4) 实测
    chain2, q2, R_f2, p_f2, _ = chain_and_q()
    print(f"[actual ] 关节实测 {np.round(q2,3).tolist()}")
    d_real = p_f2 - p_f
    print(f"[actual ] 末端位移 {np.round(d_real,4).tolist()}")

    err = np.linalg.norm(d_pred - d_real)
    print(f"[result ] 预测-实测 差 {err*1000:.1f}mm  "
          + ("模型对得上 ✅" if err < 0.02 else "模型不一致 ❌"))
    print(f"[result ] 关节跟踪误差 {np.abs(q2 - q1).max()*1000:.1f}mrad")


if __name__ == "__main__":
    main()
