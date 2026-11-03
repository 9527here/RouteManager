#!/usr/bin/env python3
"""拧阀门：从上方下降 → 夹住手轮盘 → 绕世界 Z 拧 → 松开 → 抬起。

与 valve_trajectory.py 的区别（那份按 piper 的序列写的，且拿法兰当目标）：
  * 目标点是**抓取点(TCP)= 两指垫中点**，用实测的法兰偏移换算
        tcp = p_flange + R_flange @ tcp_off
  * 手轮盘的真实几何从场景里量（mesh 顶点，不用 bbox —— bbox 被子节点
    wheel_joint 的 ±3.4e38 无效 bound 污染过，会读成 109mm 而非 80mm）
  * 拧完直接读手轮关节角验证是否真的转了，而不是只看机械臂动没动

几何前提（2026-10-29 实测）：手轮盘直径 80.0mm、高 32mm、轴=世界 Z；
夹爪全开两指垫间距 88.35mm，单边余量约 4mm。

用法:
  python3 scripts/valve_turn.py --dry-run          # 只解算，看残差
  python3 scripts/valve_turn.py --dry-run --snapshot input.json  # 完全离线，不访问服务

本脚本保留历史端点方案供离线对照，实际执行已禁用。
当前人形操作入口为 valve_operation.py，须经过完整碰撞和真实接触反馈门控。
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
from valve_ik import rotation_angle, solve_ik  # noqa: E402

PORT = int(os.environ.get("ROUTE_MANAGER_PORT", "18765"))
BASE = f"http://127.0.0.1:{PORT}"
SIDE = "right"

#: 抓取点(两指垫中点)在法兰系下的偏移**随夹爪开合变化**（四连杆，闭合 0.2572m、
#: 全开约 0.246m），所以不写常量——每轮从 get_sim_state 的 flange/tcp 现算。
GRIP_OPEN = 1000
GRIP_CLOSE = 820          # ≈71mm，手轮 80mm，留 4.5mm/边 的压紧量
READ_ONLY = False


# ---------------------------------------------------------------- HTTP

def _post(action, params=None, timeout=120):
    if READ_ONLY and action != "get_sim_state":
        raise RuntimeError(f"dry-run 禁止下发 action: {action}")
    body = json.dumps({"action": action, "parameters": params or {}}).encode()
    req = urllib.request.Request(f"{BASE}/execute", data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def _python(code, timeout=180):
    if READ_ONLY and code not in (PROBE_WHEEL, PROBE_ANCHOR):
        raise RuntimeError("dry-run 只允许固定的只读量测代码")
    req = urllib.request.Request(f"{BASE}/python",
                                 data=json.dumps({"code": code}).encode(),
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


# ---------------------------------------------------------------- 场景量测

# 位姿走 **PhysX**（isaacsim.core.prims.RigidPrim），不要用 USD 变换：
# 场景重建后 UsdGeom.XformCache / Xformable().ComputeLocalToWorldTransform()
# 在这个 prim 上都只返回"单位阵 + 0.01 缩放"（穿不过父级），会把盘心读成原点。
# RigidPrim 读到的是仿真真实位姿，而且四元数前后对比就能验证"手轮到底转没转"。
# 半径由 mesh 顶点在局部系量（绕 Z 旋转不改变半径），顶点自带 0.01 缩放。
PROBE_WHEEL = r'''
import json
import numpy as np
import omni.usd
from pxr import Usd, UsdGeom
from isaacsim.core.prims import RigidPrim
st = omni.usd.get_context().get_stage()
link = st.GetPrimAtPath("/World/vla_table/valve_asset/valve_1/wheel_link")
mesh = None
for c in Usd.PrimRange(link):
    if c.IsA(UsdGeom.Mesh):
        mesh = c; break
pts = np.array(UsdGeom.Mesh(mesh).GetPointsAttr().Get(), dtype=float)
# 顶点是未缩放的原始坐标；缩放(0.01)只在 mesh 自己的 xform 里。那个矩阵的
# **缩放是对的、平移是错的**（穿不过父级），所以只取缩放用。
Mm = UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
s = float(Mm.ExtractRotationMatrix().GetRow(0).GetLength())
pts = pts * s
mn, mx = pts.min(0), pts.max(0)
ctr_local = 0.5 * (mn + mx)                  # link 系下的盘心（米）
radius_mm = float(np.linalg.norm(pts[:, :2] - ctr_local[:2], axis=1).max()) * 1000
size_mm = [float(mx[i] - mn[i]) * 1000 for i in range(3)]
p, q = RigidPrim("/World/vla_table/valve_asset/valve_1/wheel_link").get_world_poses()
pos = np.asarray(p).reshape(-1)[:3].astype(float)
quat = np.asarray(q).reshape(-1)[:4].astype(float)      # wxyz
w, x, y, z = quat
R = np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
              [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
              [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
ctr = pos + R @ ctr_local
yaw = float(np.degrees(np.arctan2(R[1,0], R[0,0])))
json.dump({"center": [float(v) for v in ctr],
           "link_world": [float(v) for v in pos],
           "size_mm": size_mm,
           "radius_mm": radius_mm,
           "yaw_deg": yaw}, open("/tmp/valve_geom.json", "w"))
'''


def measure_wheel():
    if os.path.exists("/tmp/valve_geom.json"):
        os.remove("/tmp/valve_geom.json")
    result = _python(PROBE_WHEEL)
    if result.get("status") != "success":
        raise SystemExit(f"手轮量测失败: {result}")
    # /python 的结果不回传，从落盘文件读（仿真进程与本地共享 /tmp）
    with open("/tmp/valve_geom.json") as f:
        g = json.load(f)
    return validate_wheel(g)


def validate_wheel(g):
    # 自检：坐标必须是世界系（z 在 6~8 之间），半径必须是厘米级手轮。
    # 之前 Xformable 拿不到世界变换时这里返回过 8 米半径、局部系中心，
    # 一路算到 IK 才报"残差 7.4 米"，白跑一轮。
    center = np.asarray(g["center"], float)
    size = np.asarray(g["size_mm"], float)
    if (center.shape != (3,) or size.shape != (3,) or
            not np.isfinite(center).all() or not np.isfinite(size).all() or
            np.any(size <= 0) or not math.isfinite(g["yaw_deg"])):
        raise SystemExit("手轮几何输入无效")
    z, r = center[2], g["radius_mm"]
    if not (5.0 < z < 9.0) or not (20.0 < r < 150.0):
        raise SystemExit(
            f"手轮量测不合理：中心 z={z:.3f} 半径={r:.1f}mm —— 变换没取到世界系，"
            "先修 PROBE_WHEEL 再往下走")
    return g


# ---------------------------------------------------------------- 运动学

def load_chain():
    cfg = json.load(open(os.path.join(_REPO, "config.json")))
    preset = cfg["robot"]["presets"][cfg["robot"]["name"]]
    return arm_kin.load_chain(preset["urdf_path"], f"{SIDE}_arm_base_link",
                              f"{SIDE}_gripper_base_link")


def quat_to_R(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


PROBE_ANCHOR = r'''
import json
state = executor.get_sim_state(ref_link="")
p, q = executor._link_pose_table()["right_arm_base_link"]
with open("/tmp/valve_anchor.json", "w") as f:
    json.dump({"state": state, "arm_base": {
        "position": [float(v) for v in p],
        "orientation": [float(v) for v in q]}}, f)
'''


def anchor(chain, snapshot=None):
    """返回 (q, R_ab, t_ab, tcp_off)。tcp_off 是当前夹爪开合状态下抓取点在法兰
    系下的偏移——同一个响应里就有 flange 和 tcp，现算即可。"""
    if snapshot is None:
        # 与法兰同批读独立的臂座，不能用法兰反解基座后再把恒等式当自检。
        if os.path.exists("/tmp/valve_anchor.json"):
            os.remove("/tmp/valve_anchor.json")
        result = _python(PROBE_ANCHOR)
        if result.get("status") != "success":
            raise SystemExit(f"读取锚点失败: {result}")
        with open("/tmp/valve_anchor.json") as f:
            snapshot = json.load(f)
    st = snapshot["state"]
    if st.get("ref_link") not in (None, "", "world"):
        raise SystemExit("锚点输入必须使用世界坐标")
    names, pos = st["joint_names"], st["positions"]
    missing = [n for n in arm_kin.joint_names(chain) if n not in names]
    if missing:
        raise SystemExit(f"机器人缺少关节 {missing}，URDF 与资产不匹配")
    q = np.array([pos[names.index(n)] for n in arm_kin.joint_names(chain)], float)
    side = st["sides"][SIDE]
    f = side["flange"]
    def pose_R(pose):
        quat = np.asarray(pose["orientation"], float)
        norm = np.linalg.norm(quat)
        if quat.shape != (4,) or not np.isfinite(quat).all() or norm < 1e-9:
            raise SystemExit("锚点四元数无效，应为 wxyz")
        return quat_to_R(*(quat / norm))

    R_f = pose_R(f)
    p_f = np.array(f["position"], float)
    tcp_off = R_f.T @ (np.array(side["tcp"]["position"], float) - p_f)
    R_ab = pose_R(snapshot["arm_base"])
    t_ab = np.array(snapshot["arm_base"]["position"], float)
    if not all(np.isfinite(v).all() for v in (q, p_f, t_ab, tcp_off)):
        raise SystemExit("锚点输入含 NaN/无穷值")
    p_check, R_check = arm_kin.ee_world(chain, q, R_ab, t_ab)
    pe, ae = np.linalg.norm(p_check - p_f), rotation_angle(R_check, R_f)
    print(f"[FK独立校验] 实测臂座 → 法兰 {pe*1000:.6f}mm / {ae:.6f}°")
    if pe >= 0.008 or ae >= 3.0:
        raise SystemExit("FK 与独立实测法兰不符，中止：检查 URDF、四元数顺序和参考系")
    return q, R_ab, t_ab, tcp_off


def Rz(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def top_down_reach_bounds(chain, R_ab, t_ab, tcp_off, center, clearance):
    """从 URDF 连杆长度给出位置误差下界，不依赖 IK 是否落入局部极值。

    第一关节原点不随转角变，其余平移长度和是到法兰的保守半径上界。
    +Z 朝下时滚转只转动 tcp_off 的横向分量，扣掉其范数后对任意滚转成立。
    """
    if not chain or any(j["type"] not in ("fixed", "revolute", "continuous")
                        for j in chain):
        return None
    shoulder = R_ab @ np.asarray(chain[0]["xyz"], float) + t_ab
    reach = sum(np.linalg.norm(j["xyz"]) for j in chain[1:])
    bounds = []
    for dz in (clearance, 0.0):
        target = center + np.array([0.0, 0.0, dz + tcp_off[2]])
        lower = np.linalg.norm(target - shoulder) - np.linalg.norm(tcp_off[:2]) - reach
        bounds.append(max(0.0, float(lower)))
    return float(reach), bounds


def solve_multi(chain, tgt_pos, tgt_R, R_ab, t_ab, lim, seeds):
    """多起点求解，返回残差最小的一组。

    单起点 DLS 从"手臂自然下垂"到"法兰 +Z 朝下"这种接近 180° 的姿态差，
    会一路顶到关节限位（实测残差 941mm / 140°）。多起点是廉价且稳妥的解法。
    """
    best = None
    for s in seeds:
        q, pe, ae = solve_ik(chain, s, tgt_pos, tgt_R, R_ab, t_ab, lim)
        score = pe * 1000.0 + ae
        if best is None or score < best[0]:
            best = (score, q, pe, ae)
        # 先尝试上一阶段，收敛就保留这条分支；避免反复搜索后跳到远处的等价解。
        if pe < 5e-4 and ae < math.degrees(3e-3):
            break
    return best[1], best[2], best[3]


# ---------------------------------------------------------------- 执行

def main():
    global READ_ONLY
    ap = argparse.ArgumentParser()
    ap.add_argument("--angle", type=float, default=90.0, help="拧动角度(度)")
    ap.add_argument("--clearance", type=float, default=0.12, help="预接近高度(m)")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--snapshot", help="历史量测 JSON；仅允许 dry-run，完全离线")
    a = ap.parse_args()
    if a.snapshot and not a.dry_run:
        ap.error("--snapshot 只允许与 --dry-run 一起使用，历史状态不能用于执行")
    if not math.isfinite(a.angle) or not math.isfinite(a.clearance) or a.clearance <= 0:
        ap.error("角度必须有限，预接近高度必须为正")
    if not a.dry_run:
        # 旧端点序列没有整身避碰、双垫承载及接触后TCP重规划，不能绕过新运行器。
        ap.error("本入口仅保留 --dry-run；实际操作请用 valve_operation.py 的完整反馈流程")
    READ_ONLY = a.dry_run
    snapshot = None
    if a.snapshot:
        with open(a.snapshot) as f:
            snapshot = json.load(f)
        print(f"[离线快照] {a.snapshot}（历史量测，不代表当前仿真状态）")

    chain = load_chain()
    lim = arm_kin.joint_limits(chain)
    geom = validate_wheel(snapshot["geometry"]) if snapshot is not None else measure_wheel()
    ctr = np.array(geom["center"], float)
    print(f"[手轮] 中心 {np.round(ctr,4).tolist()}  直径 {2*geom['radius_mm']:.1f}mm  "
          f"高 {geom['size_mm'][2]:.1f}mm  初始 yaw {geom['yaw_deg']:.2f}°")

    q, R_ab, t_ab, tcp_off = anchor(chain, snapshot)
    p_fl, R_fl = arm_kin.ee_world(chain, q, R_ab, t_ab)
    tcp0 = p_fl + R_fl @ tcp_off
    print(f"[锚点] 法兰 {np.round(p_fl,4).tolist()}  TCP {np.round(tcp0,4).tolist()}")
    print(f"[锚点] 抓取点相对法兰 {np.round(tcp_off,4).tolist()}  "
          f"({np.linalg.norm(tcp_off)*1000:.1f}mm)")
    print("[TCP] 本轮按量测时的夹爪状态解算；合爪后的偏移与接触尚未验证")

    bounds = top_down_reach_bounds(chain, R_ab, t_ab, tcp_off, ctr, a.clearance)
    impossible = False
    if bounds is not None:
        reach, (pre_bound, grasp_bound) = bounds
        impossible = pre_bound >= 0.005 or grasp_bound >= 0.005
        print(f"[位置可达性] 肩到法兰长度上界 {reach*1000:.3f}mm；任意滚转下 "
              f"预接近残差 ≥{pre_bound*1000:.3f}mm，盘心 ≥{grasp_bound*1000:.3f}mm")
        if impossible:
            print("[位置可达性] 已证明正上方抓取不可达，仅解算首个滚转记录数值残差")

    # 接近姿态：法兰 +Z 指向世界 -Z（从正上方下压，但绕 Z 的滚转是自由的）
    R_down = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

    def flange_for(tcp, R):
        return tcp - R @ tcp_off

    # 多起点种子：当前位形 + 限位内均匀扰动（固定种子，可复现）
    rng = np.random.default_rng(0)
    seeds = [q] + [np.clip(q + rng.uniform(-1.0, 1.0, size=len(q)),
                           lim[:, 0] + 0.05, lim[:, 1] - 0.05)
                   for _ in range(32)]

    # 先定"抓取滚转"：绕世界 Z 转一圈找可行的那个
    R_pre, R_grasp, q_at = None, None, None
    best_roll = None
    for roll_deg in range(0, 360, 30):
        if impossible and roll_deg > 0:
            break
        Rr = Rz(math.radians(roll_deg)) @ R_down
        qa, pe1, ae1 = solve_multi(chain, flange_for(ctr + [0, 0, a.clearance], Rr),
                                   Rr, R_ab, t_ab, lim, seeds)
        qb, pe2, ae2 = solve_multi(chain, flange_for(ctr, Rr),
                                   Rr, R_ab, t_ab, lim, [qa] + seeds)
        ok = pe1 < 0.005 and ae1 < 3.0 and pe2 < 0.005 and ae2 < 3.0
        print(f"[试滚转 {roll_deg:3d}°] 预接近 {pe1*1000:7.2f}mm/{ae1:5.2f}°  "
              f"盘心 {pe2*1000:7.2f}mm/{ae2:5.2f}°  " + ("OK" if ok else "×"))
        if ok:
            best_roll, R_pre, R_grasp, q_at = roll_deg, Rr, Rr, qb
            break
    if best_roll is None:
        if impossible:
            raise SystemExit("正上方抓取几何不可达：目标法兰超过 URDF 链长上界，中止，未下发")
        raise SystemExit("滚转搜索未找到 <5mm / 3° 的解，中止，未下发")
    print(f"\n[选用] 滚转 {best_roll}°\n")

    stages = [
        ("预接近", ctr + [0, 0, a.clearance], R_pre, None, 2.5),
        ("下降到盘心", ctr, R_grasp, None, 2.0),
        ("合爪", ctr, R_grasp, GRIP_CLOSE, 0.8),
        (f"拧 {a.angle:.0f}°", ctr, Rz(math.radians(a.angle)) @ R_grasp, None, 3.0),
        ("张开", ctr, Rz(math.radians(a.angle)) @ R_grasp, GRIP_OPEN, 0.8),
        ("抬起", ctr + [0, 0, a.clearance],
         Rz(math.radians(a.angle)) @ R_grasp, None, 2.0),
    ]

    plan = []
    qc = q
    for name, tcp, R, grip, dur in stages:
        Rf_p = flange_for(tcp, R)
        qn, pe, ae = solve_multi(chain, Rf_p, R, R_ab, t_ab, lim, [qc] + seeds)
        p_actual, R_actual = arm_kin.ee_world(chain, qn, R_ab, t_ab)
        tcp_pe = float(np.linalg.norm(p_actual + R_actual @ tcp_off - tcp))
        ok = pe < 0.005 and tcp_pe < 0.005 and ae < 3.0
        print(f"[{name:<8}] 法兰残差 {pe*1000:7.2f}mm  TCP {tcp_pe*1000:7.2f}mm "
              f" 朝向 {ae:5.2f}°  "
              + ("OK" if ok else "!! 残差大"))
        if not ok:
            raise SystemExit(f"{name} 解算不收敛，中止（先别下发）")
        plan.append((name, qn, grip, dur))
        qc = qn

    if a.dry_run:
        print("\n[dry] 解算全部通过，未下发。")
        return

    print()
    _post("set_gripper_position", {"side": SIDE, "position": GRIP_OPEN})
    for name, qn, grip, dur in plan:
        wait_idle()
        if grip is not None:
            _post("set_gripper_position", {"side": SIDE, "position": grip})
            time.sleep(0.6)
        _post("move_j", {"side": SIDE, "positions": [float(v) for v in qn],
                         "duration": dur})
        time.sleep(dur + 1.2)
        wait_idle()
        print(f"  已完成: {name}")

    geom2 = measure_wheel()
    turned = (geom2["yaw_deg"] - geom["yaw_deg"] + 180.0) % 360.0 - 180.0
    print()
    print(f"[结果] 手轮 yaw {geom['yaw_deg']:.2f}° -> {geom2['yaw_deg']:.2f}°")
    print(f"[结果] 实际拧动 {turned:.1f}° / 目标 {a.angle:.1f}°  "
          + ("✅" if abs(turned) > 0.25 * abs(a.angle) else "❌ 几乎没转"))


if __name__ == "__main__":
    main()
