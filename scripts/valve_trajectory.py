#!/usr/bin/env python3
"""阀门拧动轨迹（IK 脚本版）—— 照 piper_lerobot/test/test_valve.py 的动作序列。

不走 RL、不走 Isaac Lab：用 RouteManager 仓库内自带的纯 numpy IK 栈
（arm_kin.py 的 FK/雅可比 + isaac_teleop._dls_ik 的阻尼最小二乘），
算好关节角后通过 HTTP `move_j` 下发。

为什么用这套而不是 Isaac Lab 的 DifferentialIKController：
  PhysX 的雅可比是**关于连杆质心**的、不是 TCP 的（arm_kin.py:3-5），
  直接拿它做位置控制会让末端越走越远 —— 这正是之前在 Isaac Lab 踩的坑。

用法:
  python3 scripts/valve_trajectory.py --probe                 # 先做单点方向校验
  python3 scripts/valve_trajectory.py X Y Z --dry-run         # 只解算不下发
  python3 scripts/valve_trajectory.py X Y Z                   # 执行整套
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


# ---------------------------------------------------------------- HTTP

def _post(action, params=None, timeout=60):
    body = json.dumps({"action": action, "parameters": params or {}}).encode()
    req = urllib.request.Request(f"{BASE}/execute", data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def health():
    with urllib.request.urlopen(f"{BASE}/health", timeout=5) as r:
        return json.loads(r.read().decode())


def wait_idle(timeout=120):
    t0 = time.time()
    while time.time() - t0 < timeout:
        h = health()
        if h.get("status") == "IDLE":
            return True
        time.sleep(0.3)
    return False


# ---------------------------------------------------------------- 运动学

def load_robot_chain():
    cfg = json.load(open(os.path.join(_REPO, "config.json")))
    preset = cfg["robot"]["presets"][cfg["robot"]["name"]]
    urdf = preset.get("urdf_path")
    if not urdf or not os.path.exists(urdf):
        raise SystemExit(f"预设里没有可用的 urdf_path: {preset}")
    return arm_kin.load_chain(urdf, f"{SIDE}_arm_base_link",
                              f"{SIDE}_gripper_base_link")


def anchor(chain):
    """返回 (q_meas, T_ab)：手臂各关节角 + 臂基座在世界系下的位姿。"""
    st = _post("get_sim_state", {"ref_link": ""})["data"]
    names = st["joint_names"]
    pos = st["positions"]
    want = arm_kin.joint_names(chain)
    missing = [n for n in want if n not in names]
    if missing:
        raise SystemExit(f"机器人缺少关节 {missing}，URDF 与资产不匹配")
    q = np.array([pos[names.index(n)] for n in want], float)

    flange = st["sides"][SIDE]["flange"]
    p_f = np.array(flange["position"], float)
    w, x, y, z = flange["orientation"]          # 服务端是 wxyz
    T_f = np.eye(4)
    T_f[:3, :3] = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T_f[:3, 3] = p_f

    T_ab = T_f @ np.linalg.inv(arm_kin.fk(chain, q))
    p_check = arm_kin.ee_world(chain, q, T_ab[:3, :3], T_ab[:3, 3])[:3, 3]
    err = float(np.linalg.norm(p_check - p_f))
    if err > 0.005:
        raise SystemExit(f"锚点自检失败：FK({err*1000:.1f}mm) 与上报法兰位置不符")
    print(f"[anchor] q={np.round(q,3).tolist()} 残差={err*1000:.2f}mm")
    return q, T_ab


def quat_wxyz_to_R(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def euler_to_quat_wxyz(roll, pitch, yaw):
    from scipy.spatial.transform import Rotation
    x, y, z, w = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_quat()
    return (w, x, y, z)


def solve_ik(chain, q0, tgt_pos, tgt_R, T_ab, lim, q_ref,
             iters=60, lam=0.05, step_max=0.12, w_rot=0.6, w_posture=0.15):
    """阻尼最小二乘：世界系目标位姿 → 关节角。返回 (q, 位置残差, 朝向残差°)。"""
    q = np.array(q0, float)
    R_ab, t_ab = T_ab[:3, :3], T_ab[:3, 3]
    for _ in range(iters):
        T = arm_kin.ee_world(chain, q, R_ab, t_ab)
        e_p = tgt_pos - T[:3, 3]
        Re = T[:3, :3].T @ tgt_R
        e_r = 0.5 * np.array([Re[2, 1] - Re[1, 2],
                             Re[0, 2] - Re[2, 0],
                             Re[1, 0] - Re[0, 1]])
        err = np.concatenate([e_p, w_rot * e_r])
        if np.linalg.norm(e_p) < 1e-3 and np.linalg.norm(e_r) < 5e-3:
            break
        J = arm_kin.jacobian(chain, q, R_ab, t_ab)[:3, :]
        J = np.vstack([J, w_rot * arm_kin.jacobian(chain, q, R_ab, t_ab)[3:, :]])
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), err)
        dq += w_posture * (q_ref - q)
        n = np.linalg.norm(dq)
        if n > step_max:
            dq *= step_max / n
        q = np.clip(q + dq, lim[:, 0] + 0.02, lim[:, 1] - 0.02)

    T = arm_kin.ee_world(chain, q, R_ab, t_ab)
    pos_err = float(np.linalg.norm(tgt_pos - T[:3, 3]))
    Re = T[:3, :3].T @ tgt_R
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(Re) - 1) / 2))))
    return q, pos_err, ang


# ---------------------------------------------------------------- 执行

def send(q, duration, dry):
    if dry:
        print(f"  [dry] move_j {np.round(q,3).tolist()} {duration}s")
        return
    wait_idle()
    _post("move_j", {"side": SIDE, "positions": [float(v) for v in q],
                     "duration": duration})


def gripper(pos, dry):
    if dry:
        print(f"  [dry] gripper {pos}")
        return
    wait_idle()
    _post("set_gripper_position", {"side": SIDE, "position": pos})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("x", type=float, nargs="?")
    ap.add_argument("y", type=float, nargs="?")
    ap.add_argument("z", type=float, nargs="?")
    ap.add_argument("--angle", type=float, default=90.0)
    ap.add_argument("--approach", type=float, default=0.12)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--probe", action="store_true",
                    help="单点方向校验：先让末端外移 5cm，确认方向对")
    a = ap.parse_args()

    chain = load_robot_chain()
    lim = arm_kin.joint_limits(chain)
    q, T_ab = anchor(chain)

    if a.probe:
        R = arm_kin.ee_world(chain, q, T_ab[:3, :3], T_ab[:3, 3])[:3, :3]
        p0 = arm_kin.ee_world(chain, q, T_ab[:3, :3], T_ab[:3, 3])[:3, 3]
        tgt = p0 + np.array([0.05, 0.0, 0.0])
        qn, pe, ae = solve_ik(chain, q, tgt, R, T_ab, lim, q)
        print(f"[probe] 目标外移 5cm → 残差 {pe*1000:.1f}mm / {ae:.1f}°")
        if pe > 0.005:
            raise SystemExit("IK 解不出来，先别往下走")
        send(qn, 2.0, False)
        time.sleep(3)
        q2, T2 = anchor(chain)
        pos2 = arm_kin.ee_world(chain, q2, T2[:3, :3], T2[:3, 3])[:3, 3]
        moved = pos2 - p0
        print(f"[probe] 实际位移 {np.round(moved,3).tolist()}  (期望 ≈ [0.05,0,0])")
        print("[probe] 方向 " + ("正确 ✅" if moved[0] > 0.02 else "不对 ❌"))
        return

    if a.x is None:
        raise SystemExit("需要阀门坐标 X Y Z，或用 --probe 做单点校验")

    v = np.array([a.x, a.y, a.z])
    ah = a.approach
    q_pre_R = quat_wxyz_to_R(*euler_to_quat_wxyz(0, math.radians(45), math.radians(2)))
    q_v_R = quat_wxyz_to_R(*euler_to_quat_wxyz(0, math.radians(90), math.radians(2)))
    q_t_R = quat_wxyz_to_R(*euler_to_quat_wxyz(
        0, math.radians(90), math.radians(2) + math.radians(a.angle)))

    stages = [
        ("预接近(45°)", v + [0, 0, ah + 0.20], q_pre_R, None, 2.0),
        ("摆正到上方", v + [0, 0, ah], q_v_R, None, 1.5),
        ("下降到阀门", v + [0, 0, 0.02], q_v_R, None, 1.5),
        ("合爪", v + [0, 0, 0.02], q_v_R, 250, 1.0),
        (f"拧 {a.angle:.0f}°", v + [0, 0, 0.02], q_t_R, None, 2.0),
        ("张开", v + [0, 0, 0.02], q_t_R, 1000, 0.8),
        ("抬升", v + [0, 0, ah], q_t_R, None, 1.5),
    ]

    gripper(1000, a.dry_run)
    for name, pos, R, grip, dur in stages:
        qn, pe, ae = solve_ik(chain, q, pos, R, T_ab, lim, q)
        ok = "OK" if (pe < 0.005 and ae < 3.0) else "!! 残差大"
        print(f"[{name}] 残差 {pe*1000:.1f}mm / {ae:.1f}°  {ok}")
        if pe > 0.02 or ae > 10:
            raise SystemExit(f"{name} 解算失败，中止")
        if grip is not None:
            gripper(grip, a.dry_run)
        send(qn, dur, a.dry_run)
        q = qn
        if not a.dry_run:
            time.sleep(max(dur, 1.0))


if __name__ == "__main__":
    main()
