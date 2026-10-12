#!/usr/bin/env python3
"""pick_object.py — 用 HTTP 指令让机械臂抓桌上的物体（不依赖 VR）。

链路：读 URDF 手臂链 → FK/IK 规划(离线 numpy/scipy) → `send_joint_command`
(带 joint_names 的**部分下发**) → `set_gripper_position`(睿尔曼语义) → 复测物体位置。

安全阀：规划前先用**实测**关节角跑一遍自己的 FK，若与仿真实测 TCP 偏差
> `--fk-tol`（默认 8 mm）就中止——宁可不动，也不要把臂甩出去。

用法：
    python3 scripts/pick_object.py --side left --object cube_c
    python3 scripts/pick_object.py --object cube_a --side right --dry-run
"""
import argparse
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np

URL = os.environ.get("ROUTE_MANAGER_URL", "http://127.0.0.1:18765")
URDF = ("/home/jy/datasets/vla_data/L1_制作资产/载体设备/humanbody_urdf/"
        "humanoid_upper_body_v3/urdf/humanoid_upper_body_v3_ctag2f120.urdf")

# 夹爪标定表集中在 gripper_model.py(实测 leader ↔ 指垫中心距)。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import grasp_planner  # noqa: E402,F401
import gripper_model  # noqa: E402

#: 指垫厚(mm)：开口(内间距) = 中心距 − 该值
PAD_MM = gripper_model.PAD_THICKNESS_MM
#: 主动关节全开角(rad)
GRIP_OPEN_RAD = gripper_model.LEADER_OPEN_RAD


# ---------------------------------------------------------------- HTTP
def http(action, parameters=None, timeout=120.0, tries=60):
    payload = json.dumps({"action": action,
                          "parameters": parameters or {}}).encode()
    for i in range(tries):
        req = urllib.request.Request(URL + "/execute", data=payload,
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                out = json.loads(r.read().decode())
            if isinstance(out, dict) and out.get("detail"):
                if "正忙" in str(out["detail"]):
                    time.sleep(0.3)
                    continue
                raise RuntimeError(str(out["detail"])[:300])
            return out.get("data")
        except urllib.error.HTTPError as e:
            if e.code == 409 and i < tries - 1:
                time.sleep(0.3)
                continue
            raise


def run_in_sim(src_path, out_path, timeout=120.0):
    """把 .py 丢进仿真主线程执行(结果写文件, /python 不回显)。"""
    if os.path.exists(out_path):
        os.remove(out_path)
    code = open(src_path).read()
    payload = json.dumps({"code": code}).encode()
    for i in range(60):
        req = urllib.request.Request(URL + "/python", data=payload,
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                r.read()
            break
        except urllib.error.HTTPError as e:
            if e.code == 409 and i < 59:
                time.sleep(0.3)
                continue
            raise
    for _ in range(80):
        if os.path.exists(out_path) and os.path.getsize(out_path) > 2:
            with open(out_path) as f:
                return json.load(f)
        time.sleep(0.25)
    raise RuntimeError(f"仿真侧没有产出 {out_path}")


# ---------------------------------------------------------------- URDF / FK
def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def _aa(axis, q):
    a = np.asarray(axis, float)
    n = np.linalg.norm(a)
    if n < 1e-12:
        return np.eye(3)
    a = a / n
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)


def _T(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def load_chain(urdf_path, side, root_link, tip_link):
    """从 URDF 取 root_link → tip_link 的关节链(按父子顺序)。"""
    root = ET.parse(urdf_path).getroot()
    joints = {}
    for j in root.findall("joint"):
        o, a = j.find("origin"), j.find("axis")
        joints[j.find("child").get("link")] = {
            "name": j.get("name"), "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "xyz": [float(v) for v in (o.get("xyz") or "0 0 0").split()],
            "rpy": [float(v) for v in (o.get("rpy") or "0 0 0").split()],
            "axis": [float(v) for v in
                     ((a.get("xyz") if a is not None else None) or "0 0 1").split()],
            "limit": (lambda l: [float(l.get("lower", -3.14)),
                                 float(l.get("upper", 3.14))]
                      if l is not None else [-3.14, 3.14])(j.find("limit")),
        }
    chain, link = [], tip_link
    while link != root_link:
        if link not in joints:
            raise RuntimeError(f"URDF 里 {root_link} 到 {tip_link} 不连通({link})")
        chain.append(joints[link])
        link = joints[link]["parent"]
    chain.reverse()
    chain = [j for j in chain if not j["name"].startswith("this_is_a_fake")]
    return chain


def fk(chain, q):
    """列向量约定: 返回 (R, t) 表示 tip 在 root 系里的位姿。"""
    M = np.eye(4)
    qi = 0
    for j in chain:
        M = M @ _T(_rpy(*j["rpy"]), np.asarray(j["xyz"], float))
        if j["type"] != "fixed":
            M = M @ _T(_aa(j["axis"], q[qi]), np.zeros(3))
            qi += 1
    return M[:3, :3], M[:3, 3]


# ---------------------------------------------------------------- 规划
# 规划器在 grasp_planner.py(与 robot_manager 的自动抓取状态机共用同一份)。
# 两处各留一份会让"TCP 取闭合态"这类关键语义再次分叉。
from grasp_planner import plan  # noqa: E402,F401


def jaw_position_for(sep_mm):
    """想要指垫中心距 mm → 睿尔曼 position(1~1000)。走实测标定表。"""
    frac = gripper_model.fraction_for_leader(
        gripper_model.leader_for_sep_mm(sep_mm))
    return int(round(1 + frac * 999))


def jaw_leader_for(sep_mm):
    """想要指垫中心距 mm → 主动关节角(rad)。"""
    return gripper_model.leader_for_sep_mm(sep_mm)


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser(description="HTTP 指令抓取(仿真)")
    ap.add_argument("--side", default="left", choices=["left", "right"])
    ap.add_argument("--object", default="cube_c")
    ap.add_argument("--urdf", default=URDF)
    ap.add_argument("--fk-tol", type=float, default=8.0, help="FK 自检容差(mm)")
    ap.add_argument("--approach", type=float, default=0.12, help="预备高度(m)")
    ap.add_argument("--lift", type=float, default=0.20, help="抬起高度(m)")
    ap.add_argument("--jaw-squeeze", type=float, default=0.5,
                    help="夹持过盈(mm): 指垫净开口 = 物体宽度 - 该值。"
                    "0.5mm 就够(摩擦修好后不需要靠过盈硬挤, 过盈大反而穿模)。")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    print("① 读场景实测(基准变换 / 物体包围盒 / 当前关节角)")
    probe = run_in_sim("/tmp/fk_probe.py", "/tmp/fk_probe.json")
    if "error" in probe:
        print(probe["error"])
        sys.exit(1)
    v = probe[args.side]
    q_meas = np.asarray(v["arm_q"], float)

    def col(m):        # Gf(行向量) → 列向量约定 4x4
        R = np.asarray(m["Rrow"], float).T
        return _T(R, np.asarray(m["t"], float))

    T_ab = col(v["arm_base"])
    bb = probe[args.object]
    lo, hi = np.asarray(bb["min"]), np.asarray(bb["max"])
    obj_c = 0.5 * (lo + hi)
    obj_size = hi - lo
    print(f"   目标 {args.object}: 中心 {obj_c.round(4)} 尺寸 "
          f"{(obj_size * 1000).round(1)}mm  桌面 z={lo[2]:.4f}")

    # --- 夹持目标: 指垫中心距 = 物体宽 + 垫厚 − 过盈 ---
    grasp_w = float(min(obj_size[0], obj_size[1]) * 1000.0)
    jaw_sep = grasp_w + PAD_MM - args.jaw_squeeze
    jaw_leader = jaw_leader_for(jaw_sep)
    print(f"   夹持: 物体宽 {grasp_w:.1f}mm 过盈 {args.jaw_squeeze}mm → "
          f"指垫中心距 {jaw_sep:.1f}mm → 主动关节 {jaw_leader:.4f} rad")

    # --- TCP 取**闭合态**: 指垫中点随开口沿夹爪接近轴漂移 ~11mm, 用张开态
    #     的 TCP 规划会让物体落在指垫外侧(实测闭合后指垫离物体还有 3~4mm) ---
    http("stop_vr_teleop")
    http("set_gripper_position", {"side": args.side,
                                  "fraction": gripper_model.fraction_for_sep_mm(jaw_sep)})
    time.sleep(4.5)      # 夹爪全行程约 4~6s(阻尼 286 限速), 早读会读到运动中值
    pc_all = run_in_sim("/tmp/fk_probe.py", "/tmp/fk_probe.json")
    vc = pc_all[args.side]
    T_ee_c = col(vc["ee"])
    pad_c = 0.5 * (np.asarray(vc["pad_r"]["t"]) + np.asarray(vc["pad_l"]["t"]))
    tcp_local = T_ee_c[:3, :3].T @ (pad_c - T_ee_c[:3, 3])
    sep_c = np.linalg.norm(np.asarray(vc["pad_r"]["t"])
                           - np.asarray(vc["pad_l"]["t"]))
    lead_now = pc_all.get("grip_leader_deg", {}).get(args.side, float("nan"))
    print(f"   闭合态实测: 主动关节 {lead_now:.4f} rad  "
          f"指垫中心距 {sep_c * 1000:.2f}mm  TCP局部 {tcp_local.round(4)}")

    # --- 张开(接近段用张开态), 取当前关节角 + 虎口方向 ---
    http("set_gripper_position", {"side": args.side, "position": 1000})
    time.sleep(4.5)
    v = run_in_sim("/tmp/fk_probe.py", "/tmp/fk_probe.json")[args.side]
    q_meas = np.asarray(v["arm_q"], float)
    T_ee = col(v["ee"])
    pad_r, pad_l = np.asarray(v["pad_r"]["t"]), np.asarray(v["pad_l"]["t"])
    tcp_meas = 0.5 * (pad_r + pad_l)
    tcp_open_local = T_ee[:3, :3].T @ (tcp_meas - T_ee[:3, 3])
    sepv = pad_r - pad_l
    jaw_local = T_ee[:3, :3].T @ (sepv / (np.linalg.norm(sepv) + 1e-9))
    print(f"   张开态: 末端 {T_ee[:3, 3].round(3)} TCP {tcp_meas.round(3)} "
          f"指垫中心距 {np.linalg.norm(sepv) * 1000:.1f}mm  "
          f"虎口方向局部 {jaw_local.round(3)}")

    chain = load_chain(args.urdf, args.side, f"{args.side}_arm_base_link",
                       f"{args.side}_gripper_base_link")
    names = [j["name"] for j in chain if j["type"] != "fixed"]
    limits = np.asarray([j["limit"] for j in chain if j["type"] != "fixed"])
    # 以仿真里**实际烘焙**的限位为准(URDF 与资产不一致: 例如 left_arm_2
    # URDF ±130°, 资产 +78° —— 用 URDF 的会解出顶死在限位上的假解)
    sim_lim = v.get("arm_limits")
    if sim_lim and np.asarray(sim_lim).shape == limits.shape:
        limits = np.asarray(sim_lim, dtype=float)
        print("   关节限位(仿真实测): " +
              ", ".join(f"{n.replace('_joint','')}={np.degrees(l).round(0)}"
                        for n, l in zip(names, limits)))
    print(f"② 手臂链 {len(names)} 关节: {names}")

    # --- FK 自检: 用自己的 FK 复现实测 TCP, 不然不敢下发 ---
    R0, t0 = fk(chain, q_meas)
    tcp_fk = ((T_ab[:3, :3] @ R0) @ tcp_open_local
              + (T_ab[:3, :3] @ t0 + T_ab[:3, 3]))
    err = float(np.linalg.norm(tcp_fk - tcp_meas) * 1000)
    print(f"   FK 自检: 预测 TCP {tcp_fk.round(4)} vs 实测 {tcp_meas.round(4)}"
          f"  →  偏差 {err:.2f} mm")
    if err > args.fk_tol:
        print(f"✗ FK 偏差 {err:.1f}mm > {args.fk_tol}mm，模型与仿真不一致，中止")
        sys.exit(2)

    # --- 三段轨迹 + 笛卡尔直线路点(走 grasp_planner, 与自动抓取共用) ---
    # home 位形也当 IK 起点: 手臂若停在很偏的位形(实测 arm_2 贴下限),
    # 多起点会全部收敛到"贴着限位、姿态解不开"的局部解(朝下误差 30°)。
    _seeds = []
    try:
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "scene", "jac_table.json")) as _f:
            _hq = np.asarray(json.load(_f).get("home_q", []), float)
        _dn = (http("get_joint_names") or {}).get("dof_names") or []
        if _hq.size == len(_dn):
            _seeds.append(_hq[[_dn.index(n) for n in names]])
    except Exception as _e:
        print(f"   (home 起点不可用: {_e!r})")
    qs, pinfo = grasp_planner.plan_grasp(
        chain, q_meas, limits, T_ab, tcp_local, jaw_local, obj_c,
        approach=args.approach, lift=args.lift, seeds=_seeds)
    if qs is None:
        print("✗ " + pinfo["error"])
        sys.exit(3)
    for tag in ("pre", "grasp", "lift"):
        info = pinfo["infos"][tag]
        print(f"③ IK {tag:5s} 位置误差 {info['pos_err_mm']:.1f}mm, "
              f"朝下误差 {info['dir_err_deg']:.1f}°, "
              f"虎口误差 {info['jaw_err_deg']:.1f}° (虎口世界方向 "
              f"{info['jaw_world']}), TCP {info['tcp']}, "
              f"贴限位惩罚 {info['limit_pen']}")
    print(f"   下压路点 {len(qs['descent'])} 段, 抬升路点 {len(qs['ascent'])} 段")

    # 机身升降: 规划会挑一个让手臂姿态误差最小的高度(臂够不到桌面时降机身)
    _bdz = float(pinfo.get("body_dz", 0.0))
    if abs(_bdz) > 1e-3:
        dn = (http("get_joint_names") or {}).get("dof_names") or []
        js = (http("get_joint_states") or {}).get("feedback") or []
        if "lift_joint" in dn and js:
            cur = float(js[dn.index("lift_joint")])
            print(f"   先升降机身 {_bdz:+.3f} m (lift {cur:.3f} → "
                  f"{cur + _bdz:.3f})")
            http("send_joint_command", {"positions": [cur + _bdz],
                                        "joint_names": ["lift_joint"],
                                        "duration": 1.5})
            time.sleep(3.0)
    deg = {t: [round(math.degrees(x), 1) for x in qs[t]]
           for t in ("pre", "grasp", "lift")}
    for t in ("pre", "grasp", "lift"):
        print(f"   {t:5s} deg = {deg[t]}")

    if args.dry_run:
        print("--dry-run: 只规划不下发")
        return

    print("④ 下发(部分写: 只动这条臂的关节, 其余保持不动)")
    http("stop_vr_teleop")            # 防止 VR 与脚本抢控制权
    print("   先张开夹爪(全开 1000)")
    http("set_gripper_position", {"side": args.side, "position": 1000})
    time.sleep(4.5)
    http("send_joint_command", {"positions": list(map(float, qs["pre"])),
                                "joint_names": names, "duration": 2.0})
    time.sleep(3.2)
    # 笛卡尔直线下压: 关节空间插值会让 TCP 走曲线、斜着蹭到物体
    for k, q in enumerate(qs["descent"]):
        http("send_joint_command", {"positions": list(map(float, q)),
                                    "joint_names": names, "duration": 0.25})
        time.sleep(0.35)
    time.sleep(0.8)
    print(f"⑤ 闭合夹爪: 指垫目标中心距 {jaw_sep:.1f}mm(开口 "
          f"{jaw_sep - PAD_MM:.1f}mm) → 主动关节 {jaw_leader:.4f} rad")
    http("set_gripper_position", {"side": args.side,
                                  "fraction": gripper_model.fraction_for_sep_mm(jaw_sep)})
    time.sleep(4.5)
    print("⑥ 抬起(同样走直线路点)")
    for q in qs["ascent"]:
        http("send_joint_command", {"positions": list(map(float, q)),
                                    "joint_names": names, "duration": 0.25})
        time.sleep(0.35)
    time.sleep(1.5)

    print("⑦ 复测")
    after = run_in_sim("/tmp/fk_probe.py", "/tmp/fk_probe.json")
    b2 = after[args.object]
    c2 = 0.5 * (np.asarray(b2["min"]) + np.asarray(b2["max"]))
    dz = float(c2[2] - obj_c[2])
    grip = after["grip_leader_deg"].get(args.side)
    print(f"   物体中心 {obj_c.round(4)} → {c2.round(4)}  (Δz = {dz:+.3f} m)")
    print(f"   夹爪主动关节 {grip:.3f} rad")
    print("✅ 抓起来了" if dz > 0.05 else "❌ 没夹住(物体没被抬起)")


if __name__ == "__main__":
    main()