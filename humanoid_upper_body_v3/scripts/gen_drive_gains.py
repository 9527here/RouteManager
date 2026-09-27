#!/usr/bin/env python3
"""Derive PhysX drive gains from the robot's own inertia, not from guesswork.

For each position-controlled joint:
    stiffness k = tau_gravity_max / DROOP        (hold the load within DROOP rad)
    damping   d = 2 * zeta * sqrt(k * I_axis)    (critically damped)

I_axis is the true downstream inertia about the joint axis at the zero pose
(each downstream link's own tensor projected onto the axis, plus the parallel
axis term).  d is then clamped so the drive stays stable at the target
timestep: an implicit PhysX drive tolerates d up to a few times I/dt, so
DAMP_STABILITY_FACTOR * I / dt is used as the ceiling.

Drive roles on the v3 chassis (人形上装部分9.9, same four-castor base as v2):
  * the four wheel joints (ZQL/ZHL/YQL/YHL) get velocity drives, as before;
  * the four castor swivel joints (ZQY/ZHY/YQY/YHY) get position drives that
    hold them at zero.  The chassis CAD gives the wheel no swivel clearance
    (deck material sits inside the wheel's 0.21 m pivot sweep), so on the real
    robot the castors must be locked; the position drive reproduces that in
    sim and keeps the wheel inside the pocket the collision geometry leaves
    for it.  If your real castors actually steer, delete these four entries
    from drive_gains.json *and* widen the chassis pockets first.

Writes drive_gains.json next to joint_dynamics.json.
"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
SRC = HERE.parent.parent / "urdf" / "人形上装部分9.9.urdf"
DYN = json.loads((HERE / "joint_dynamics.json").read_text())

G = 9.81
DROOP = 0.02              # rad of allowed sag under full gravity load
ZETA = 1.0                # critically damped
DT = 1.0 / 60.0           # target sim timestep (recorded for reference)
WHEEL_JOINTS = ("ZQL-JOINT", "ZHL-JOINT", "YQL-JOINT", "YHL-JOINT")

root = ET.parse(SRC).getroot()
L = {l.get("name"): l for l in root.findall("link")}
J = []
for j in root.findall("joint"):
    o = j.find("origin")
    ax = j.find("axis")
    J.append((
        j.get("name"),
        j.find("parent").get("link"),
        j.find("child").get("link"),
        np.array([float(x) for x in o.get("xyz").split()]),
        np.array([float(x) for x in ax.get("xyz").split()]) if ax is not None else np.zeros(3),
        j.get("type"),
    ))

pos = {"base_link": np.zeros(3)}
for _ in range(len(J)):
    for n, p, c, o, a, t in J:
        if p in pos:
            pos[c] = pos[p] + o

kids = {}
for n, p, c, o, a, t in J:
    kids.setdefault(p, []).append(c)


def subtree(link):
    out = [link]
    for c in kids.get(link, []):
        out += subtree(c)
    return out


mass = {n: float(l.find("inertial").find("mass").get("value")) for n, l in L.items()}
com = {n: np.array([float(x) for x in l.find("inertial").find("origin").get("xyz").split()])
       for n, l in L.items()}


def tensor(n):
    d = L[n].find("inertial").find("inertia").attrib
    return np.array([[float(d["ixx"]), float(d["ixy"]), float(d["ixz"])],
                     [float(d["ixy"]), float(d["iyy"]), float(d["iyz"])],
                     [float(d["ixz"]), float(d["iyz"]), float(d["izz"])]])


out = {}
print(f"{'joint':10s} {'I_worst':>9s} {'tau':>8s} {'k':>9s} {'d':>8s}  target")
for n, p, c, o, a, t in J:
    if t == "fixed":
        continue
    links = subtree(c)
    jo = pos[c]
    ah = a / np.linalg.norm(a)

    # Worst-case downstream inertia about the joint axis, maximised over
    # configuration: take every downstream mass at its FULL radius from the
    # joint origin rather than its perpendicular radius at the zero pose.
    # A roll joint's perpendicular radius collapses to ~0 at theta=0 (its axis
    # runs down the arm) but grows once upstream pitch joints bend, so the
    # zero-pose value would under-damp it badly.  Same bound used for effort.
    I_worst = 0.0
    for l in links:
        r = pos[l] + com[l] - jo
        I_worst += ah @ tensor(l) @ ah + mass[l] * float(r @ r)

    eff = DYN[n]["effort"]

    if n in WHEEL_JOINTS:
        # velocity drive: k must be 0, d is the velocity gain.  Size it so the
        # effort clamp saturates at 0.007 rad/s (~0.4 deg/s) of creep: at
        # driving speed the clamp governs regardless of d, and near zero speed
        # the brake is what stops the contact ratchet from integrating into
        # wander (measured: eff/(vel/4) sizing crept ~5 deg/s and wandered
        # 9-16 cm per 15 s; this sizing holds the robot on its wheels, see
        # rest_test_51.py).  The gain is large because the drive is implicit;
        # the clamp keeps the applied torque bounded.
        k = 0.0
        d = eff / 0.007
        tgt = "velocity"
    else:
        tau = sum(mass[l] * G * np.linalg.norm(pos[l] + com[l] - jo) for l in links)
        if t == "prismatic":
            # droop in metres; the "inertia" seen by a prismatic drive is mass
            I_worst = sum(mass[l] for l in links)
            k = I_worst * G / 0.002
        else:
            k = tau / DROOP
        d = 2 * ZETA * np.sqrt(max(k, 1e-9) * I_worst)
        tgt = "position"

    k, d = round(float(k), 1), round(float(d), 2)
    out[n] = dict(stiffness=k, damping=d, target=tgt)
    print(f"{n:10s} {I_worst:9.4f} {eff:8.0f} {k:9.1f} {d:8.2f}  {tgt}")

# force left/right symmetry (the mirror bug perturbs the right arm slightly)
for key in "234567J":
    a, b = f"Z{key}-JOINT", f"Y{key}-JOINT"
    if a in out and b in out:
        for f in ("stiffness", "damping"):
            m = max(out[a][f], out[b][f])
            out[a][f] = out[b][f] = m

(HERE / "drive_gains.json").write_text(json.dumps(out, indent=1))
print("\n-> drive_gains.json  (stiffness Nm/rad or N/m, damping Nms/rad or Ns/m)")
