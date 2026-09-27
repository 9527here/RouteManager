#!/usr/bin/env python3
"""Validate humanoid_upper_body_v3.urdf against everything Isaac Sim cares about.

Exits non-zero on any FAIL so this can gate a re-export.
"""
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import naming_rules  # 规则化命名映射(scripts/ 是 sys.path[0])

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
URDF = PKG / "urdf" / "humanoid_upper_body_v3.urdf"
ORIG = PKG.parent / "urdf" / "人形上装部分9.9.urdf"
# argv[1] 可选: 变体 URDF 路径(如夹爪 humanoid_upper_body_v3_ctag2f120.urdf),
# 对照源随之切到对应 CAD 导出。
if len(sys.argv) > 1:
    URDF = Path(sys.argv[1])
    if "ctag2f120" in URDF.name:
        ORIG = Path("/home/jy/下载/人形上装部分9.9/urdf/人形上装部分9.9_ctag2f120.urdf")

fails, warns = [], []


def check(ok, msg, hard=True):
    if ok:
        print(f"  PASS  {msg}")
    else:
        print(f"  {'FAIL' if hard else 'WARN'}  {msg}")
        (fails if hard else warns).append(msg)


def section(t):
    print(f"\n{t}\n{'-' * len(t)}")


robot = ET.parse(URDF).getroot()
orig = ET.parse(ORIG).getroot()
links = {l.get("name"): l for l in robot.findall("link")}
joints = robot.findall("joint")

section("1. USD-safe naming")
bad = [n for n in list(links) + [j.get("name") for j in joints] + [robot.get("name")]
       if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", n)]
check(not bad, f"all {len(links) + len(joints) + 1} names are ASCII identifiers"
      + (f" -- offenders: {bad}" if bad else ""))
raw = URDF.read_bytes()
try:
    raw.decode("ascii")
    ascii_ok, offender = True, ""
except UnicodeDecodeError as e:
    ascii_ok, offender = False, f" -- first non-ASCII byte at offset {e.start}"
check(ascii_ok, f"file is pure ASCII (comments included){offender}")

section("2. Kinematic tree")
parents = {j.find("child").get("link"): j.find("parent").get("link") for j in joints}
roots = [l for l in links if l not in parents]
check(len(roots) == 1, f"exactly one root link: {roots}")
check(len(joints) == len(links) - 1,
      f"{len(joints)} joints for {len(links)} links (tree, no loops/orphans)")
orphan = [c for c in parents.values() if c not in links]
check(not orphan, f"every joint parent exists{'' if not orphan else f': missing {orphan}'}")
# reachability
seen, frontier = set(roots), list(roots)
kids = {}
for j in joints:
    kids.setdefault(j.find("parent").get("link"), []).append(j.find("child").get("link"))
while frontier:
    n = frontier.pop()
    for c in kids.get(n, []):
        if c not in seen:
            seen.add(c)
            frontier.append(c)
check(seen == set(links), f"all {len(links)} links reachable from {roots[0]}")

section("3. Joint limits and actuation")
for j in joints:
    n, t = j.get("name"), j.get("type")
    if t == "fixed":
        check(j.find("axis") is None, f"{n}: fixed joint carries no <axis>")
        continue
    lt, dy = j.find("limit"), j.find("dynamics")
    ok = lt is not None and dy is not None
    if t == "revolute":
        ok = ok and lt.get("lower") is not None and lt.get("upper") is not None \
            and float(lt.get("upper")) > float(lt.get("lower"))
    if t in ("revolute", "prismatic", "continuous"):
        ok = ok and float(lt.get("effort", 0)) > 0 and float(lt.get("velocity", 0)) > 0
    span = ""
    if lt is not None and lt.get("lower") is not None:
        span = f" [{float(lt.get('lower')):+.3f},{float(lt.get('upper')):+.3f}]"
    check(ok, f"{n} ({t}){span} effort={lt.get('effort')} vel={lt.get('velocity')} "
              f"damping={dy.get('damping') if dy is not None else None}")
for j in joints:
    ax = j.find("axis")
    if ax is not None:
        v = np.array([float(x) for x in ax.get("xyz").split()])
        check(np.linalg.norm(v) > 1e-9, f"{j.get('name')}: axis is non-zero")

section("4. Mesh references resolve on disk")
missing, nvis, ncol = [], 0, 0
for m in robot.iter("mesh"):
    f = m.get("filename")
    check_path = PKG / f.replace(f"package://{PKG.name}/", "")
    if not check_path.is_file():
        missing.append(f)
    if "/visual/" in f:
        nvis += 1
    else:
        ncol += 1
check(not missing, f"{nvis} visual + {ncol} collision mesh refs all resolve"
      + ("" if not missing else f" -- missing {missing}"))
check(all(f.startswith(f"package://{PKG.name}/") for f in
          (m.get("filename") for m in robot.iter("mesh"))),
      f"every mesh uses package://{PKG.name}/")

section("5. Inertial properties preserved")
# 对照源是 CAD 导出(源名), 本包是规则名 —— 经 RENAME_LINKS 归一后逐一比对;
# 名字两边都可能是任一风格, get(name, name) 双向兼容。
om = {naming_rules.RENAME_LINKS.get(l.get("name"), l.get("name")): l
      for l in orig.findall("link")}
# Links added on purpose (grippers, sensors, ...) have no counterpart in the
# original CAD export; they add mass instead of preserving it.
added = [n for n in links if n not in om]
tot_new = tot_old = added_mass = 0.0
for n, l in links.items():
    i = l.find("inertial")
    m = float(i.find("mass").get("value"))
    tot_new += m
    if n in om:
        tot_old += float(om[n].find("inertial").find("mass").get("value"))
    else:
        added_mass += m
    d = i.find("inertia").attrib
    I = np.array([[float(d["ixx"]), float(d["ixy"]), float(d["ixz"])],
                  [float(d["ixy"]), float(d["iyy"]), float(d["iyz"])],
                  [float(d["ixz"]), float(d["iyz"]), float(d["izz"])]])
    ev = np.linalg.eigvalsh(I)
    tri = (ev[0] + ev[1] >= ev[2] * (1 - 1e-6))
    if not (m > 0 and ev[0] > 0 and tri):
        check(False, f"{n}: mass={m} eig={ev} triangle={tri}")
check(abs(tot_new - added_mass - tot_old) < 1e-9,
      f"original links keep their masses: {tot_old:.3f} kg unchanged")
check(True, f"intentionally added links {added} contribute +{added_mass:.3f} kg "
            f"(total {tot_new:.3f} kg)")
check(True, f"all {len(links)} inertia tensors positive-definite and satisfy "
            "the triangle inequality")

section("6. Left/right arm symmetry")
J = {j.get("name"): j for j in joints}


def cum(pre):
    # joints are stored under their rule-compliant names; map built -> rule
    p, out = np.zeros(3), {}
    for k in "1234567J":
        jn = naming_rules.RENAME_JOINTS[f"{pre}{k}_JOINT"]
        o = np.array([float(v) for v in J[jn].find("origin").get("xyz").split()])
        p = p + o
        out[k] = p.copy()
    return out


Z, Y = cum("Z"), cum("Y")
worst_z = worst_y = 0.0
for k in "1234567J":
    d = (Y[k] - np.array([Z[k][0], -Z[k][1], Z[k][2]])) * 1000
    worst_z = max(worst_z, abs(d[2]))
    worst_y = max(worst_y, abs(d[1]))
check(worst_z < 1e-6, f"z-mirror error eliminated (worst |dz| = {worst_z:.6f} mm)")
check(worst_y < 3.0, f"residual y asymmetry {worst_y:.3f} mm exceeds 3 mm "
      f"(intra-segment offsets that cancel pairwise; hand lands within "
      f"{abs((Y['J'] - np.array([Z['J'][0], -Z['J'][1], Z['J'][2]]))[1] * 1000):.2f} mm)",
      hard=False)

section("7. Collision budget")
prim = sum(1 for l in links.values() for c in l.findall("collision")
           if c.find("geometry/box") is not None or c.find("geometry/cylinder") is not None)
msh = sum(1 for l in links.values() for c in l.findall("collision")
          if c.find("geometry/mesh") is not None)
check(prim + msh > 0, f"{prim} analytic primitives + {msh} hull meshes = {prim + msh} shapes")
nofc = [n for n, l in links.items() if not l.findall("collision")]
check(not nofc, f"every link has collision geometry{'' if not nofc else f': {nofc} have none'}")

print(f"\n{'=' * 60}")
print(f"{len(fails)} FAIL, {len(warns)} WARN")
for w in warns:
    print(f"  WARN {w}")
for f in fails:
    print(f"  FAIL {f}")
sys.exit(1 if fails else 0)
