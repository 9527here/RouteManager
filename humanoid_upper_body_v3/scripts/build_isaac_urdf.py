#!/usr/bin/env python3
"""Build an Isaac-Sim-ready URDF from the original SolidWorks export.

Reads (never writes) the original  ../urdf/人形上装部分9.9.urdf  plus the
collision spec produced by gen_collision.py, and emits
  humanoid_upper_body_v3/urdf/humanoid_upper_body_v3.urdf

Every transformation is listed in FIXES below and echoed to stdout so the
diff against the CAD export stays auditable.  Re-run any time the CAD is
re-exported.

Differences vs the v2 pipeline (人形轮系底盘 export), same 29-link robot
re-exported 2026-09-09:
  * SX_Link's frame was re-anchored onto the lift axis (origin moved to
    z=1.1362); XQ's world pose is unchanged - joint origins pass through,
    nothing here rewrites positions (only gen_collision.py had to shift the
    SX_Link slab breaks).
  * the arm chain Z2..Z6/Y2..Y6 now ships real CAD revolute travel, so the
    limit precedence below is  castor-lock design > CAD travel > our derived
    table.  Every CAD <limit> still says effort=0, so effort/velocity always
    come from joint_dynamics.json.
  * the CAD gives the castor swivels a steering range (ZQY/ZHY -90..+30 deg,
    YQY/YHY mirrored) - overridden by the lock design, unchanged from v2:
    the chassis deck sits inside the wheel's 0.21 m pivot sweep, so the real
    castors cannot rotate (see gen_drive_gains.py).
  * 9.9 emits degenerate <limit lower="0" upper="0" .../> blocks on Z7/Y7 -
    a limit block must be REPLACED, not appended to (URDF parsers take the
    first one and would see a locked joint).

Gripper variant (source ..._ctag2f120.urdf, pass its path as argv[1]):
  * same robot + a ctag2f120 two-finger gripper under each wrist (15 joints
    each: 2 drives R2/L2 + 10 mimic followers + 2 fixed pads/mount).  The
    robot/package name stays humanoid_upper_body_v3 so the ROS2 graph layer
    (usd/ros2_graph.usd) migrates unchanged.
  * <mimic> tags are KEPT (correct for ROS2/RViz).  PhysX has no mimic -
    after import the follower joints are independent; closing behaviour is
    solved on the Isaac side (drive-follow / graph multiply), see README.
  * CAD gives the drive joints effort=2 / velocity=1000 rad/s - the velocity
    is an export artifact, clamped to 4; followers carry zeros.  All gripper
    joints get uniform dynamics from GRIPPER_DYN below.
  * collision: the base 29 links reuse the v3 collision spec; gripper links
    are not in the spec and keep their source mesh collision (small parts).
  * new ctag2f120_*.STL meshes are copied into meshes/visual/ on build.
"""
import json
import math
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from naming_rules import RENAME_JOINTS, RENAME_LINKS  # 规则化命名(唯一权威表)

HERE = Path(__file__).resolve().parent
PKG = HERE.parent                      # humanoid_upper_body_v3/
SRC = PKG.parent / "urdf" / "人形上装部分9.9.urdf"
if len(sys.argv) > 1:
    SRC = Path(sys.argv[1])
GRIPPER_MODE = "ctag2f120" in SRC.name
DST = PKG / "urdf" / ("humanoid_upper_body_v3_ctag2f120.urdf" if GRIPPER_MODE
                      else "humanoid_upper_body_v3.urdf")
EXPECTED = (59, 58) if GRIPPER_MODE else (29, 28)
SPEC = json.loads((PKG / "collision_shapes.json").read_text())
DYN = json.loads((HERE / "joint_dynamics.json").read_text())
if GRIPPER_MODE:
    # 夹爪关节动力学(键=源关节名)。 CAD 只给两个驱动关节 effort=2/velocity=1000
    # —— velocity 是导出垃圾值, 钳到 4 rad/s; 其余关节(含 mimic 跟随)全 0,
    # 不补值导入后 PhysX 会振荡。 统一 e=2/v=4/damping 0.1/friction 0.05,
    # 上机后按实测收。
    GRIPPER_DYN = {k: {"effort": 2.0, "velocity": 4.0, "damping": 0.1,
                       "friction": 0.05}
                   for k in RENAME_JOINTS if not k.endswith("_JOINT")}
else:
    GRIPPER_DYN = {}

PKG_NAME = "humanoid_upper_body_v3"

# ---------------------------------------------------------------- joint limits
# Precedence (see module docstring): LOCKED_CASTORS force the lock travel;
# real CAD travel wins where the CAD provides a usable block; the table below
# is the fallback for joints the CAD left unlimited (TZ/TF/ZJ/YJ) or gave a
# degenerate 0..0 block (Z7/Y7) or an inverted one (XQ [3.14, 0]).
#
# Arm chain is a 7-DOF anthropomorphic alternating roll/pitch wrist-partitioned
# arm.  Verified empirically by forward kinematics: a joint is a *pitch* iff
# rotating it changes the shoulder->hand chord length.
#   n=2 +-Y roll   n=3 -+X pitch   n=4 +-Y roll
#   n=5 -+X PITCH == ELBOW         n=6 +-Y roll   n=7 +-X pitch   J +-Y roll
#
# Where the CAD gives no hard stop and the positive direction could not be
# established from geometry, the range is SYMMETRIC on purpose: a wrong-signed
# one-sided limit would pin the joint at full extension, which is far worse
# than allowing some unreachable travel.  Marked [SIGN UNVERIFIED] - narrow
# these against the real hardware before trusting them for planning.
D = math.radians
LIMITS = {
    "XQ_JOINT": (D(-90), D(90)),    # torso lean (pitch about +Y)
    "TZ_JOINT": (D(-90), D(90)),    # head yaw
    "TF_JOINT": (D(-45), D(45)),    # head pitch          [SIGN UNVERIFIED]
    # Castor swivels: the chassis deck sits inside the wheel's pivot sweep, so
    # the real castors cannot rotate - these are lock tolerances, not a range
    # of motion.  gen_drive_gains.py additionally holds them at 0.
    "ZQY_JOINT": (D(-5), D(5)),
    "ZHY_JOINT": (D(-5), D(5)),
    "YQY_JOINT": (D(-5), D(5)),
    "YHY_JOINT": (D(-5), D(5)),
}
ARM_LIMITS = {                      # keyed by the digit/letter after Z or Y
    "2": (D(-170), D(170)),         # shoulder roll   (fallback; CAD: +-178)
    "3": (D(-120), D(120)),         # shoulder pitch  (fallback; CAD: +-130)
    "4": (D(-170), D(170)),         # upper-arm roll  (fallback; CAD: +-135)
    "5": (D(-135), D(135)),         # ELBOW pitch     [SIGN UNVERIFIED]
    "6": (D(-170), D(170)),         # forearm roll    (fallback; CAD: +-128)
    "7": (D(-120), D(120)),         # wrist pitch     (fallback; CAD: degenerate)
    "J": (D(-175), D(175)),         # wrist roll
}
LOCKED_CASTORS = {"ZQY_JOINT", "ZHY_JOINT", "YQY_JOINT", "YHY_JOINT"}
CONTINUOUS = {"ZQL_JOINT", "ZHL_JOINT", "YQL_JOINT", "YHL_JOINT"}   # wheels spin free

MATERIAL = "aluminium_grey"
RGBA = "0.752941 0.752941 0.752941 1.0"

FIXES = []


def note(msg):
    FIXES.append(msg)
    print(msg)


def fmt(v):
    """Trim float noise without losing CAD precision."""
    return f"{v:.9g}"


def xyz(vals):
    return " ".join(fmt(v) for v in vals)


def sub(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def limits_for(name):
    if name in CONTINUOUS:
        return None
    if name in LIMITS:
        return LIMITS[name]
    # fullmatch 防止夹爪关节名(如 ZJ_R2)撞进臂链兜底表
    if re.fullmatch(r"[ZY][0-9J]_JOINT", name):
        return ARM_LIMITS.get(name[1])
    return None


def main():
    tree = ET.parse(SRC)
    robot = tree.getroot()

    # 1 ---------------------------------------------------------- robot name
    note(f"robot name: {robot.get('name')!r} -> {PKG_NAME!r}  (USD prims reject non-ASCII)")
    robot.set("name", PKG_NAME)

    # 2 ------------------------------------------- one named shared material
    # Isaac/USD wants a resolvable material name; 29x name="" collapses badly.
    mat = ET.Element("material", {"name": MATERIAL})
    sub(mat, "color", rgba=RGBA)
    robot.insert(0, mat)
    note(f"materials: 29x name=\"\" -> single shared <material name=\"{MATERIAL}\">")

    # 3 --------------------------------------------------- links: mesh paths
    #    visual  -> meshes/visual/<link>.STL   (copied byte-identical)
    #    collision -> replaced wholesale from collision_shapes.json
    for link in robot.findall("link"):
        ln = link.get("name")

        for vis in link.findall("visual"):
            m = vis.find("geometry/mesh")
            if m is not None:
                old = m.get("filename")
                stl = old.rsplit("/", 1)[-1]
                m.set("filename", f"package://{PKG_NAME}/meshes/visual/{stl}")
            # retarget the inline unnamed material at the shared one
            for old_mat in vis.findall("material"):
                vis.remove(old_mat)
            sub(vis, "material", name=MATERIAL)

        if ln in SPEC:
            # 基座 link: 套 v3 碰撞 spec(解析基元/抽稀壳体)
            for col in link.findall("collision"):
                link.remove(col)
            for shape in SPEC[ln]:
                col = ET.SubElement(link, "collision", {"name": f"{ln}_{shape['name']}"})
                sub(col, "origin",
                    xyz=xyz(shape.get("origin", [0, 0, 0])),
                    rpy=xyz(shape.get("rpy", [0, 0, 0])))
                geo = ET.SubElement(col, "geometry")
                if shape["type"] == "box":
                    sub(geo, "box", size=xyz(shape["size"]))
                elif shape["type"] == "cylinder":
                    sub(geo, "cylinder", radius=fmt(shape["radius"]),
                        length=fmt(shape["length"]))
                else:
                    sub(geo, "mesh",
                        filename=f"package://{PKG_NAME}/meshes/collision/{shape['file']}")
        elif GRIPPER_MODE:
            # 夹爪件不在 spec 里: 保留源网格碰撞(小件, 抽稀不划算), 只把包路径
            # 重定向到本包; 网格与 visual 同一文件, 统一放 meshes/visual/。
            for col in link.findall("collision"):
                m = col.find("geometry/mesh")
                if m is not None:
                    stl = m.get("filename").rsplit("/", 1)[-1]
                    m.set("filename", f"package://{PKG_NAME}/meshes/visual/{stl}")

    note("visual meshes -> package://%s/meshes/visual/*.STL (was package://人形上装部分9.9/)"
         % PKG_NAME)
    nshapes = sum(len(v) for v in SPEC.values())
    if GRIPPER_MODE:
        note(f"collision: base {len(SPEC)} links keep the v3 spec "
             f"({nshapes} shapes); gripper links keep their source mesh "
             "collision (package path redirected)")
    else:
        note(f"collision: 29 full-res visual meshes -> {nshapes} shapes "
             f"(boxes/cylinders/decimated hulls) across {len(SPEC)} links")

    # 4 ------------------------------------------------------------- joints
    joints = {j.get("name"): j for j in robot.findall("joint")}

    for old_name, j in list(joints.items()):
        new_name = old_name.replace("-", "_")
        if new_name != old_name:
            j.set("name", new_name)
        jt = j.get("type")

        # 4a  invalid zero axis on fixed joints
        ax = j.find("axis")
        if jt == "fixed" and ax is not None:
            if all(abs(float(v)) < 1e-12 for v in ax.get("xyz").split()):
                j.remove(ax)
                note(f"{new_name}: removed invalid <axis xyz=\"0 0 0\"/> from a fixed joint")
            continue
        if jt == "fixed":
            continue

        # 4b  limits - precedence: castor lock > usable CAD travel > our table
        # 夹爪关节不在 joint_dynamics.json -> 回退 GRIPPER_DYN 统一值
        dyn = DYN.get(old_name) or GRIPPER_DYN[old_name]
        eff, vel = dyn["effort"], dyn["velocity"]
        lim = limits_for(new_name)
        lt = j.find("limit")
        cad_ok = (lt is not None and lt.get("lower") is not None
                  and float(lt.get("upper")) > float(lt.get("lower")))

        def set_limit(lo, hi):
            """Replace any existing (possibly degenerate) limit block."""
            old = j.find("limit")
            if old is not None:
                j.remove(old)
            return sub(j, "limit", lower=fmt(lo), upper=fmt(hi),
                       effort=fmt(eff), velocity=fmt(vel))

        if new_name in LOCKED_CASTORS:
            set_limit(*lim)
            j.set("type", "revolute")
            note(f"{new_name}: castor locked -> revolute "
                 f"[{math.degrees(lim[0]):+.0f}, {math.degrees(lim[1]):+.0f}] deg "
                 f"(CAD travel overridden - the deck leaves the wheel no swivel "
                 f"clearance), effort {eff:g}, velocity {vel:g}")
        elif cad_ok:
            if jt == "continuous":
                j.set("type", "revolute")
            note(f"{new_name}: CAD travel [{lt.get('lower')}, {lt.get('upper')}] kept"
                 + (" (continuous -> revolute)" if jt == "continuous" else "")
                 + f"; effort 0 -> {eff:g}, velocity -> {vel:g} (CAD emits zeros)")
            lt.set("effort", fmt(eff))
            lt.set("velocity", fmt(vel))
        elif lim is not None:
            cad_txt = (f"[{lt.get('lower')}, {lt.get('upper')}]"
                       if lt is not None and lt.get("lower") is not None else "none")
            lt = set_limit(*lim)
            if jt == "continuous":
                j.set("type", "revolute")
            note(f"{new_name}: {jt} -> revolute with derived limit "
                 f"[{math.degrees(lim[0]):+.0f}, {math.degrees(lim[1]):+.0f}] deg "
                 f"(CAD gave {cad_txt}), effort {eff:g}, velocity {vel:g}")
        else:                                    # drive wheels spin free
            old = j.find("limit")
            if old is not None:
                j.remove(old)
            sub(j, "limit", effort=fmt(eff), velocity=fmt(vel))
            note(f"{new_name}: kept continuous (drive wheel), "
                 f"effort {eff:g} Nm, velocity {vel:g} rad/s")

        # 4c  <dynamics> - every joint needs damping or PhysX oscillates
        sub(j, "dynamics", damping=fmt(dyn["damping"]), friction=fmt(dyn["friction"]))

    note("all %d movable joints: added <dynamics damping friction>"
         % sum(1 for j in robot.findall("joint") if j.get("type") != "fixed"))

    # 5 -------------------------------------------- Y-arm z-mirror correction
    # The two arms are mirrored about the XZ plane, so a Y joint's origin must
    # match its Z counterpart in x and z and negate only y.  The exporter let
    # sign errors slip into z on the right arm (still present in the 9.9
    # export: Y7/YJ carry negated z vs their Z counterparts).  Only z is
    # touched here - the user explicitly chose to leave the skewed joint axes
    # alone.
    for key in "1234567J":
        zj, yj = joints.get(f"Z{key}-JOINT"), joints.get(f"Y{key}-JOINT")
        if zj is None or yj is None:
            continue
        zo = [float(v) for v in zj.find("origin").get("xyz").split()]
        yo = [float(v) for v in yj.find("origin").get("xyz").split()]
        if abs(yo[2] - zo[2]) > 1e-9:
            note(f"Y{key}_JOINT: origin z {yo[2]:+.9f} -> {zo[2]:+.9f} "
                 f"(mirror of Z{key}_JOINT, delta {abs(yo[2]-zo[2])*1000:.3f} mm)")
            yo[2] = zo[2]
            yj.find("origin").set("xyz", xyz(yo))
        if abs(abs(yo[0]) - abs(zo[0])) > 1e-6 or abs(abs(yo[1]) - abs(zo[1])) > 1e-6:
            note(f"  NOTE Y{key}/Z{key} also differ in x/y "
                 f"(x {yo[0]:+.6f} vs {zo[0]:+.6f}, y {yo[1]:+.6f} vs {zo[1]:+.6f}) "
                 f"- left as-is, not part of the z-mirror fix")

    # 5b ------------------------------------------- rule-compliant renaming
    # 用户命名规范 (/home/jy/文档/urdf命名规则): 小写+数字+下划线、语义化、
    # 层级从大到小。  映射表在 scripts/naming_rules.py(三个脚本共用)。
    # 只重命名模型名: link/joint、parent/child 引用、collision 元素名前缀。
    # 网格文件名(TF_0.STL 等)保持源 CAD 命名 —— 有意不动,便于溯源。
    renamed_links = 0
    for link in robot.findall("link"):
        old = link.get("name")
        new = RENAME_LINKS.get(old, old)
        if new != old:
            for col in link.findall("collision"):
                cn = col.get("name") or ""
                if cn.startswith(old + "_"):
                    col.set("name", new + cn[len(old):])
            link.set("name", new)
            renamed_links += 1
    renamed_joints = 0
    for j in robot.findall("joint"):
        old = j.get("name")
        new = RENAME_JOINTS.get(old, old)
        if new != old:
            j.set("name", new)
            renamed_joints += 1
        for ref in ("parent", "child"):
            e = j.find(ref)
            e.set("link", RENAME_LINKS.get(e.get("link"), e.get("link")))
        mim = j.find("mimic")
        if mim is not None:                     # 跟随关节的 mimic 引用同步改名
            mim.set("joint", RENAME_JOINTS.get(mim.get("joint"), mim.get("joint")))
    n_new = (len({l.get("name") for l in robot.findall("link")}),
             len({j.get("name") for j in robot.findall("joint")}))
    if n_new != EXPECTED:
        raise SystemExit(f"rename collision: {n_new[0]} unique link names, "
                         f"{n_new[1]} unique joint names - expected "
                         f"{EXPECTED[0]}/{EXPECTED[1]}")
    note(f"naming rules applied: {renamed_links} links + {renamed_joints} "
         f"joints renamed per /home/jy/文档/urdf命名规则 "
         f"(map: scripts/naming_rules.py; mesh filenames kept as source)")

    # 6 ------------------------------------------------------------- write
    ET.indent(tree, space="  ")
    DST.parent.mkdir(parents=True, exist_ok=True)
    # The file is kept pure ASCII on purpose - Isaac's URDF importer and the
    # USD prim path both choke on the CJK package name, which is why the
    # output goes to a new package name rather than the source one.
    header = ("<?xml version=\"1.0\"?>\n"
              "<!-- Generated by scripts/build_isaac_urdf.py from the original\n"
              "     SolidWorks export in ../urdf/ - do not edit by hand.\n"
              "     See README.md for the rationale behind every change. -->\n")
    body = ET.tostring(robot, encoding="unicode")
    DST.write_text(header + body + "\n", encoding="utf-8")

    # 7 ---------------------------------- copy any still-missing mesh into the pkg
    # 夹爪 ctag2f120_*.STL 只在源包 meshes/(扁平); 包内缺的从源包拷进来。
    # 夹爪碰撞与视觉同文件, 引用统一指 meshes/visual/。
    copied = 0
    for m in robot.iter("mesh"):
        ref = m.get("filename")
        prefix = f"package://{PKG_NAME}/meshes/"
        if not ref.startswith(prefix):
            continue
        rel = ref[len(prefix):]
        dst = PKG / "meshes" / rel
        if dst.is_file():
            continue
        hits = list((SRC.parent.parent / "meshes").glob(f"**/{Path(rel).name}"))
        if not hits:
            raise SystemExit(f"mesh {rel} referenced but not found under "
                             f"{SRC.parent.parent / 'meshes'}")
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(hits[0], dst)
        copied += 1
    if copied:
        note(f"meshes: copied {copied} missing STL file(s) from "
             f"{SRC.parent.parent / 'meshes'} into {PKG / 'meshes'}")

    print(f"\n-> {DST}  ({len(FIXES)} recorded changes)")


if __name__ == "__main__":
    main()
