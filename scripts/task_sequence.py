#!/usr/bin/env python3
"""task_sequence.py — 完整动作演示：底盘前后左右走一圈 → 开到桌前 → 抓取。

链路（全部走 HTTP，不依赖 VR）：

  1. 把底盘摆到"起步位"（桌后 `--start-back` 米，朝向桌子）
  2. 闭环走位：**前进 → 后退 → 左转 → 右转**
  3. 闭环开回桌前标准位姿 `RobotManager._TABLE_POSE`
  4. `auto_pick(reset_robot=False)` 抓桌上最近的可抓物体

为什么是**闭环**：`set_base_velocity` 是把轮子的位置目标逐帧前移
（`_update_base_velocity`），实际速度受打滑/驱动限制，和指令差很多
（实测 vx=0.3 走 1.5s 只前进 5.6cm，后退却有 26cm）。所以每一步都按
**实测底盘位姿**判停，不用"速度×时间"。

用法:
    python3 scripts/task_sequence.py
    python3 scripts/task_sequence.py --side right --start-back 1.2
    python3 scripts/task_sequence.py --no-grasp          # 只走位不抓
    python3 scripts/task_sequence.py --fwd 0.5 --turn-deg 60
"""
import argparse
import json
import math
import os
import sys
import time
import urllib.request

URL = os.environ.get("ROUTE_MANAGER_URL", "http://127.0.0.1:18765")
POSE_FILE = "/tmp/task_sequence_pose.json"

#: 闭环控制周期（秒）。`set_base_velocity` 的失联超时是 0.5s，这里够密。
DT = 0.08
V_FWD = 0.30        # m/s 名义前进速度（实际由闭环修正）
W_TURN = 0.60       # rad/s 名义转向速度
TOL_XY = 0.035      # 到位判定（米）
TOL_YAW = 0.10      # 朝向到位判定（弧度）
YAW_DEAD = 0.10     # 直行时允许的朝向偏差死区（弧度，≈6°）


def post(action, params=None, timeout=600):
    body = {"action": action, "parameters": params or {}}
    req = urllib.request.Request(
        URL + "/execute", data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def _py(code, timeout=120):
    req = urllib.request.Request(
        URL + "/python", data=json.dumps({"code": code}).encode(),
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.status


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------- 底盘接口

_POSE_CODE = (
    "import numpy as np, math, json\n"
    "ex = executor\n"
    "p, q = ex.articulation.get_world_poses()\n"
    "p = np.asarray(p)[0]; q = np.asarray(q)[0]\n"
    "w, x, y, z = [float(v) for v in q]\n"
    "yaw = math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))\n"
    "open(%r, 'w').write(json.dumps("
    "{'x': float(p[0]), 'y': float(p[1]), 'yaw': yaw}))\n" % POSE_FILE)


def base_pose():
    """实测底盘位姿 (x, y, yaw)。"""
    if os.path.exists(POSE_FILE):
        os.remove(POSE_FILE)
    _py(_POSE_CODE)
    for _ in range(40):
        if os.path.exists(POSE_FILE):
            try:
                return json.load(open(POSE_FILE))
            except Exception:
                pass
        time.sleep(0.02)
    raise RuntimeError("读不到底盘位姿")


def send(vx, wz):
    post("set_base_velocity", {"vx": float(vx), "wz": float(wz)}, timeout=30)


def stop():
    for _ in range(3):
        try:
            send(0.0, 0.0)
        except Exception:
            pass
    time.sleep(0.4)


def teleport(x, y, yaw=0.0):
    """把底盘直接摆到指定位姿（走位演示的起点用）。"""
    code = (
        "import numpy as np, json, math\n"
        "ex = executor\n"
        "yaw = %r\n"
        "q = np.array([math.cos(yaw/2.0), 0.0, 0.0, math.sin(yaw/2.0)])\n"
        "ex.articulation.set_world_poses(\n"
        "    positions=np.array([[%r, %r, 0.3295]]),\n"
        "    orientations=np.array([q]))\n"
        "ex._base_cmd = None\n"
        "ex._wheel_pos_target = None\n"
        "open('/tmp/ts_teleport_ok', 'w').write('ok')\n" % (yaw, x, y))
    if os.path.exists("/tmp/ts_teleport_ok"):
        os.remove("/tmp/ts_teleport_ok")
    _py(code)
    time.sleep(0.6)
    return os.path.exists("/tmp/ts_teleport_ok")


def home_arms():
    """把双臂摆回 config.json 的 `home_pose`（自然下垂）。

    加载时 `_apply_home_pose` 已经摆过一次；这里再摆一次是为了**上一次跑完
    留在半空的姿态**也归位。只下发**对得上当前机器人**的那一侧——`home_pose`
    是按 7 轴旧臂标定的，6 轴 RM65 用不了（判定逻辑与 robot_manager 一致）。
    """
    try:
        cfg = json.load(open(os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "config.json")))
    except Exception:
        return
    pose = {k: v for k, v in (cfg.get("home_pose") or {}).items()
            if not k.startswith("_") and isinstance(v, (int, float))}
    if not pose:
        return
    code = (
        "import numpy as np, json\n"
        "ex = executor\n"
        "pose = json.loads(%r)\n"
        "names = set(ex.articulation.dof_names)\n"
        "alln = list(ex.articulation.dof_names)\n"
        "q = ex._current_targets()\n"
        "applied = {}\n"
        "for side in ('left', 'right'):\n"
        "    keys = [f'{side}_arm_{i}_joint' for i in range(1, 7)]\n"
        "    if f'{side}_wrist_joint' in pose:\n"
        "        keys.append(f'{side}_wrist_joint')\n"
        "    if all(k in names and k in pose for k in keys):\n"
        "        for k in keys:\n"
        "            q[alln.index(k)] = float(pose[k]); applied[k] = pose[k]\n"
        "ex._motion = None\n"
        "ex.articulation.set_joint_position_targets("
        "np.asarray(q).reshape(1, -1).astype(np.float32))\n"
        "open('/tmp/ts_home_ok', 'w').write(str(len(applied)))\n"
        % json.dumps(pose))
    if os.path.exists("/tmp/ts_home_ok"):
        os.remove("/tmp/ts_home_ok")
    _py(code)
    time.sleep(1.5)
    n = open("/tmp/ts_home_ok").read() if os.path.exists("/tmp/ts_home_ok") else "?"
    print(f"  手臂回 home: {n} 个关节")


def drive_until(vx, wz, done, timeout, label):
    """按实测位姿闭环走位，直到 done(pose) 为真或超时。"""
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        send(vx, wz)
        p = base_pose()
        last = p
        if done(p):
            stop()
            print(f"  ✓ {label}  ({time.time()-t0:.1f}s)")
            return True
    stop()
    print(f"  ⚠ {label} 超时 {timeout}s，停在 {last}")
    return False


def goto(tx, ty, tyaw=0.0, timeout=120.0):
    """闭环开到 (tx, ty) 并对正 tyaw。

    **不要用比例控制**：底盘实测只有 ~0.12m/s、~0.1rad/s，转向响应慢，
    比例项会让它绕着目标打转（实测停在 y=-0.20、yaw=-47° 收不了）。
    改成"死区 + 离散两拍"：朝向偏差 > `YAW_DEAD` 就原地转，否则直行一步，
    每拍都重新读位姿。慢但一定收敛。
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        p = base_pose()
        dx, dy = tx - p["x"], ty - p["y"]
        if math.hypot(dx, dy) < TOL_XY:
            break
        err = _wrap(math.atan2(dy, dx) - p["yaw"])
        if abs(err) > YAW_DEAD:
            send(0.0, W_TURN * (1.0 if err > 0 else -1.0))
        else:
            send(V_FWD, 0.0)
        time.sleep(DT)
    stop()
    # 对正朝向
    t1 = time.time()
    while time.time() - t1 < 40.0:
        p = base_pose()
        e = _wrap(tyaw - p["yaw"])
        if abs(e) < TOL_YAW:
            break
        send(0.0, W_TURN * (1.0 if e > 0 else -1.0))
        time.sleep(DT)
    stop()
    return base_pose()


# ------------------------------------------------------------------ 抓取

def wait_auto_pick(timeout=300):
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        ap = post("auto_pick_status")["data"]
        line = json.dumps(ap, ensure_ascii=False, default=str)
        if line != last:
            print(f"    [{time.time()-t0:5.1f}s] {line[:240]}", flush=True)
            last = line
        if not ap.get("running"):
            return ap.get("last") or ap
        time.sleep(0.4)
    return {"note": "超时"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", default="left", choices=["left", "right"])
    ap.add_argument("--start-back", type=float, default=1.6,
                    help="起步位在桌后多少米（默认 1.6）")
    ap.add_argument("--fwd", type=float, default=0.7, help="前后各走多少米")
    ap.add_argument("--turn-deg", type=float, default=80.0, help="左右各转多少度")
    ap.add_argument("--place-xy", default="0.55,0.02",
                    help="抓完放下的落点（世界 x,y）；'none' = 不放，只抬起来")
    ap.add_argument("--no-grasp", action="store_true")
    args = ap.parse_args()

    print(f"服务 {URL}")
    print("▶ 1/4 摆到起步位（桌后 %.2fm）+ 手臂回 home（自然下垂）"
          % args.start_back)
    if not teleport(-args.start_back, 0.0, 0.0):
        print("  ⚠ teleport 未确认，继续")
    home_arms()
    p = base_pose()
    print(f"  起步位 x={p['x']:+.3f} y={p['y']:+.3f} yaw={math.degrees(p['yaw']):+.1f}°")

    print("▶ 2/4 前后左右走位")
    p0 = base_pose()
    drive_until(V_FWD, 0.0,
                lambda p: (p["x"] - p0["x"]) >= args.fwd, 90, "前进")
    p1 = base_pose()
    drive_until(-V_FWD, 0.0,
                lambda p: (p1["x"] - p["x"]) >= args.fwd, 90, "后退")
    p2 = base_pose()
    d = math.radians(args.turn_deg)
    drive_until(0.0, W_TURN,
                lambda p: _wrap(p["yaw"] - p2["yaw"]) >= d, 120, "左转")
    p3 = base_pose()
    drive_until(0.0, -W_TURN,
                lambda p: _wrap(p3["yaw"] - p["yaw"]) >= d, 120, "右转")

    print("▶ 3/4 开到桌前标准位姿")
    p = goto(0.0, 0.0, 0.0)
    err = math.hypot(p["x"], p["y"])
    print(f"  到位 x={p['x']:+.3f} y={p['y']:+.3f} "
          f"yaw={math.degrees(p['yaw']):+.1f}°  位置误差 {err*1000:.0f}mm")

    if args.no_grasp:
        print("▶ 4/4 --no-grasp，跳过抓取")
        return
    place = None
    if args.place_xy and args.place_xy.lower() != "none":
        place = [float(v) for v in args.place_xy.split(",")]
    print(f"▶ 4/4 抓取（{args.side} 臂，不复位底盘）"
          + (f" → 放下到 {place}" if place else "（不放）"))
    params = {"side": args.side, "reset_robot": False}
    if place:
        params["place_at"] = place
    r = post("auto_pick", params)
    if r.get("data", {}).get("status") != "started":
        print("  启动失败:", r)
        return
    last = wait_auto_pick()
    print("  结果:", json.dumps(last, ensure_ascii=False))


if __name__ == "__main__":
    main()
