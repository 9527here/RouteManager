#!/usr/bin/env python3
"""Finalize the imported ctag2f120 gripper-variant USD (gripper twin of
finalize_usd.py - v3 baseline script stays untouched).

Run with:  /home/jy/micromamba/envs/isaacsim5.1/bin/python3.11 \
               scripts/finalize_usd_ctag.py [imported.usda]
(inside SimulationApp - this Kit's pxr is a namespace package scattered across
extension dirs and only usable after app boot).

Takes the Kit importer's output - default: the GUI-import asset
urdf/humanoid_upper_body_v3_ctag2f120/humanoid_upper_body_v3_ctag2f120.usd
(the user's saved usd/humanoid_upper_body_v3_ctag2f120.usda is only a thin
/World wrapper referencing it), argv[1] overrides - and writes
usd/humanoid_upper_body_v3_ctag2f120_flat.usd, with everything
finalize_usd.py does for v3 plus the gripper-specific handling:

  * 59 rigid bodies (29 base + 30 gripper links), 50 movable joints
  * gripper drives: k=30 N*m/rad, c=0.2 N*m*s/rad (STARTING VALUES, tune on
    the real robot), maxForce = URDF effort 2 N*m, target 0 = closed hold
  * limits re-authored from the delivered URDF for all 24 gripper joints
    (belt-and-suspenders: if the import ran WITHOUT "Ignore Mimic" the 5.1
    importer drops the left-hand follower limits to +/-inf)
  * PhysxMimicJointAPI stripped from any joint that carries it - cross-finger
    mimic chains misbehave in 5.1 (measured 2026-09-15), closure happens on
    the command side: followers = multiplier x master (table read from the
    URDF's <mimic> tags, also echoed into the report)
  * mass gate 120.939 kg, everything else identical to finalize_usd.py
    (recenter, materials rescue, flatten, collision normalize, 240 Hz,
    solver 32/4, weld, full self-check)

Kit swallows prints, so the whole report goes to /tmp/v3_finalize_ctag.json.
"""

import json
import sys
from pathlib import Path

ARGS = sys.argv[1:]  # read before SimulationApp boots (Kit may rewrite argv)

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import math  # noqa: E402
import xml.etree.ElementTree as ET  # noqa: E402

import numpy as np  # noqa: E402
import naming_rules  # noqa: E402  (scripts/ is sys.path[0])
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt  # noqa: E402
from pxr import PhysxSchema  # noqa: E402

PKG = Path(__file__).resolve().parent.parent
SRC = Path(ARGS[0]) if ARGS else \
    PKG / "urdf" / "humanoid_upper_body_v3_ctag2f120" / \
    "humanoid_upper_body_v3_ctag2f120.usd"
# flat always lands in usd/ regardless of where the import asset lives
DST = PKG / "usd" / (SRC.stem + "_flat.usd")
URDF = PKG / "urdf" / "humanoid_upper_body_v3_ctag2f120.urdf"
REPORT = Path("/tmp/v3_finalize_ctag.json")

ROOT = "/humanoid_upper_body_v3"
GEOM = f"{ROOT}/Geometry"
PHYS = f"{ROOT}/Physics"

N_BODIES = 59        # 29 base + 30 gripper links
N_MOVABLE = 50       # 26 base + 24 gripper joints (2 mount + 2x2 pad are fixed)
TOTAL_MASS = 120.939  # validate.py: URDF total preserved through the build

R2D = 180.0 / math.pi
D2R = math.pi / 180.0          # multiply a per-radian gain to get per-degree

WHEELS = naming_rules.WHEEL_LINKS
WHEEL_JOINTS = naming_rules.WHEEL_JOINTS
# 用户 2026-09-14 拍板: 轮子走 B 设计 —— 自由滚 (原 A 制动式为
# jointFriction 30 + 限扭 40 Nm, 拖底盘必打滑)。
WHEEL_JOINT_FRICTION = 1.0    # Nm 轴承摩擦
WHEEL_DRIVE_MAX_FORCE = 2.0   # Nm, 钳到地面抓地力矩以下, 推底盘轮子就滚
# URDF joints now carry rule-compliant names (scripts/naming_rules.py); the
# gain/dynamics JSONs are keyed by the internal built names, so translate.
CRUFT = ("/Render", "/OmniverseKit_Persp", "/OmniverseKit_Front",
         "/OmniverseKit_Top", "/OmniverseKit_Right")

# ------------------------------------------------- gripper spec from the URDF
_groot = ET.parse(URDF).getroot()
GRIP_JOINTS = {}   # rule name -> {"type":..., "lower":rad, "upper":rad}
GRIP_MIMICS = {}   # follower -> {"master":..., "multiplier":..., "offset":...}
for _j in _groot.iter("joint"):
    _n, _t = _j.get("name"), _j.get("type")
    if "gripper" not in _n or _t == "fixed":
        continue
    _lim = _j.find("limit")
    _m = _j.find("mimic")
    if _m is not None:
        GRIP_MIMICS[_n] = {"master": _m.get("joint"),
                           "multiplier": float(_m.get("multiplier", 1.0)),
                           "offset": float(_m.get("offset", 0.0))}
    else:
        GRIP_JOINTS[_n] = {"type": _t, "lower": float(_lim.get("lower")),
                           "upper": float(_lim.get("upper"))}
if len(GRIP_JOINTS) != 2 or len(GRIP_MIMICS) != 22:
    raise SystemExit(f"unexpected gripper URDF: {len(GRIP_JOINTS)} drive joints, "
                     f"{len(GRIP_MIMICS)} mimics (expected 2 / 22)")
# masters are drive joints too (right_2 per hand); every gripper movable joint
# gets a drive: the 2 masters get real gains, the 22 followers the same gains
# and are closed from the command side as multiplier x master.  Follower range
# = master range x multiplier, and a NEGATIVE multiplier flips the interval
# (found 2026-09-15: authoring lower>upper made PhysX clamp every negative
# command back to 0 - the joints never moved).
GRIP_ALL = {**GRIP_JOINTS}
for _f, _m in GRIP_MIMICS.items():
    _mt = next(j for j in _groot.iter("joint") if j.get("name") == _m["master"])
    _lim = _mt.find("limit")
    _lo = float(_lim.get("lower")) * _m["multiplier"]
    _hi = float(_lim.get("upper")) * _m["multiplier"]
    GRIP_ALL[_f] = {"type": _mt.get("type"), "lower": min(_lo, _hi),
                    "upper": max(_lo, _hi)}
    if GRIP_ALL[_f]["lower"] >= GRIP_ALL[_f]["upper"]:
        raise SystemExit(f"degenerate limit range for {_f}: [{_lo}, {_hi}]")
if len(GRIP_ALL) != N_MOVABLE - 26:
    raise SystemExit(f"expected 24 gripper movable joints, got {len(GRIP_ALL)}")

# 夹爪驱动初值 (上机按实测收): k=30 Nm/rad 在 maxForce 2 Nm 下满载稳态误差
# ~0.067 rad(3.8 度), 对两指小夹爪足够; c=0.2 配 240 Hz 隐式求解稳定。
GRIP_K = 30.0     # N*m/rad
GRIP_C = 0.2      # N*m*s/rad
GRIP_EFFORT = 2.0   # N*m   (= URDF effort)
GRIP_VEL = 4.0      # rad/s (= URDF clamped velocity)

rep = {"error": None, "src": str(SRC), "dst": str(DST),
       "grip_mimics": GRIP_MIMICS}


def load(name):
    """gain/dynamics JSONs are keyed by the original hyphenated joint names."""
    raw = json.loads((PKG / "scripts" / name).read_text())
    return {k.replace("-", "_"): v for k, v in raw.items()}


def body_ancestor(prim):
    p = prim.GetParent()
    while p and p.GetPath() != Sdf.Path.absoluteRootPath:
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            return p
        p = p.GetParent()
    return None


try:
    gains = load("drive_gains.json")
    dyn = load("joint_dynamics.json")

    st = Usd.Stage.Open(str(SRC))
    if st is None:
        raise SystemExit(f"cannot open {SRC}")
    # The 5.1 API importer marks every link's visuals/collisions instanceable
    # (internal references to masters under /visuals + /colliders).  Flatten
    # keeps instances as proxies, which drops real Mesh prims from the export,
    # breaks material bindings (gray model) and makes PhysX/RTX work against
    # proxies.  Clear the flags so Flatten inlines the composed geometry as
    # real prims, then drop the masters.
    n_inst = 0
    for p in st.Traverse():
        if p.IsInstanceable():
            p.SetInstanceable(False)
            n_inst += 1
    rep["instances_cleared"] = n_inst
    # The importer authors links behind Payload references; resolve everything
    # into one in-memory layer so CopySpec can see the specs it needs to move.
    flat = st.Flatten()
    st = Usd.Stage.Open(flat)
    layer = st.GetRootLayer()
    for masters in ("/visuals", "/colliders"):
        mp = st.GetPrimAtPath(masters)
        if mp and mp.IsValid():
            st.RemovePrim(mp.GetPath())
    rep["root_children"] = [c.GetName() for c in
                            st.GetPseudoRoot().GetChildren()]
    pseudo = st.GetRootLayer().GetPrimAtPath("/")
    rep["metersPerUnit"] = pseudo.GetInfo("metersPerUnit") if pseudo else None

    layer = st.GetRootLayer()

    # ------------------------------------------------ 1. recenter the origin
    # Sample while the kinematic nesting is still intact, then translate the
    # root body - its subtree (everything) moves with it.
    bodies = [p for p in st.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(bodies) != N_BODIES:
        raise SystemExit(f"expected {N_BODIES} rigid bodies, found {len(bodies)}")
    axles = []
    for w in WHEELS:
        prims = [p for p in bodies if p.GetName() == w]
        if len(prims) != 1:
            raise SystemExit(f"wheel body {w}: found {len(prims)} matches")
        m = UsdGeom.Xformable(prims[0]).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        axles.append(m.ExtractTranslation())
    ax = sum(a[0] for a in axles) / 4
    ay = sum(a[1] for a in axles) / 4

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              ["default", "render", "proxy", "guide"])
    bottom = min(cache.ComputeWorldBound(b).ComputeAlignedRange().GetMin()[2]
                 for b in bodies)
    off = Gf.Vec3d(-ax, -ay, -bottom)
    rep["axle_centroid"] = [ax, ay]
    rep["bottom_z"] = bottom
    rep["offset"] = list(off)

    root_body = next(p for p in bodies if p.GetName() == "base_link"
                     and p.HasAPI(UsdPhysics.ArticulationRootAPI))

    # Sample each body's world transform while the kinematic nesting is still
    # intact and the scope still at identity.  These are what the flatten step
    # bakes as Geometry-local transforms.
    orig = {p.GetPath(): UsdGeom.Xformable(p).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()) for p in bodies}

    # The offset goes on the asset ROOT SCOPE, not on the root body: the
    # importer nests sibling links under a payload wrapper, so moving the root
    # body would leave every other link behind.
    scope_xf = UsdGeom.Xformable(st.GetPrimAtPath(ROOT))
    sops = {op.GetOpName(): op for op in scope_xf.GetOrderedXformOps()}
    t = sops.get("xformOp:translate")
    if t is not None:
        cur = Gf.Vec3d(t.Get())
        t.Set(Gf.Vec3f(cur + off) if isinstance(t.Get(), Gf.Vec3f) else cur + off)
    else:
        scope_xf.AddTranslateOp().Set(off)

    expect = {p.GetPath(): UsdGeom.Xformable(p).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()) for p in bodies}

    # ----------------------------------- 1b. rescue the materials
    # The importer puts materials under the payload wrapper (which the flatten
    # below deletes), leaving every binding dangling -> gray model.  Relocate
    # them to ROOT/Materials (the GUI importer's convention from v2) and remap
    # their bindings along with the body-path rewrite.
    doomed = [p.GetPath() for p in bodies] + [Sdf.Path(f"{ROOT}/base_link")]
    mat_remap = {}
    mat_scope = Sdf.Path(f"{ROOT}/Materials")
    for m in [p for p in st.Traverse() if p.IsA(UsdShade.Material)]:
        mp = m.GetPath()
        if any(mp.HasPrefix(d) for d in doomed):
            if not st.GetPrimAtPath(mat_scope):
                st.DefinePrim(mat_scope, "Scope")
            new = mat_scope.AppendChild(mp.name)
            if not Sdf.CopySpec(layer, mp, layer, new):
                raise SystemExit(f"CopySpec material failed: {mp}")
            mat_remap[mp] = new
    rep["materials_rescued"] = len(mat_remap)

    # ------------------------------------- 2. move joints scope to Physics/
    # Two importer layouts: API import nests everything under a payload
    # wrapper -> {ROOT}/base_link/joints; GUI import puts bodies AND the
    # joints scope directly under ROOT -> {ROOT}/joints.  Support both.
    if not st.GetPrimAtPath(PHYS):
        st.DefinePrim(PHYS, "Scope")
    jscope = None
    for cand in (f"{ROOT}/joints", f"{ROOT}/base_link/joints"):
        p = st.GetPrimAtPath(cand)
        if p.IsValid():
            jscope = p
            break
    if jscope is None:
        raise SystemExit("joints scope not found at {ROOT}/joints or "
                         "{ROOT}/base_link/joints")
    if not Sdf.CopySpec(layer, jscope.GetPath(), layer, Sdf.Path(f"{PHYS}/joints")):
        raise SystemExit("CopySpec joints scope failed")
    st.RemovePrim(jscope.GetPath())   # drop the original (stale-body rels)

    # ------------------------------------------- 3. flatten the body tree
    geom = Sdf.Path(GEOM)
    if not st.GetPrimAtPath(geom):
        st.DefinePrim(geom, "Scope")
    nested = [p for p in bodies if body_ancestor(p) is not None]
    remap = {}
    body_paths = [p.GetPath() for p in bodies]
    for prim in bodies:                      # ALL bodies become siblings
        old = prim.GetPath()
        new = geom.AppendChild(old.name)
        if st.GetPrimAtPath(new):
            raise SystemExit(f"cannot flatten: {new} already exists")
        if not Sdf.CopySpec(layer, old, layer, new):
            raise SystemExit(f"CopySpec failed: {old} -> {new}")
        for other in body_paths:
            if other != old and other.HasPrefix(old):
                st.RemovePrim(new.AppendPath(other.MakeRelativePath(old)))
        remap[old] = new
        # Bake the sampled world transform as the new local transform so the
        # move out of the kinematic nesting preserves every pose exactly
        # (locals were parent-relative; Geometry is an identity scope).
        xf2 = UsdGeom.Xformable(st.GetPrimAtPath(new))
        for op in xf2.GetOrderedXformOps():
            xf2.GetPrim().RemoveProperty(op.GetOpName())
        xf2.ClearXformOpOrder()
        xf2.AddTransformOp().Set(orig[old])
    for old in sorted(remap, key=lambda p: -p.pathElementCount):
        st.RemovePrim(old)                   # deepest first: wrapper last

    # the old wrapper base_link is now empty; drop it
    wrapper = st.GetPrimAtPath(f"{ROOT}/base_link")
    if wrapper.IsValid():
        st.RemovePrim(wrapper.GetPath())

    # repoint every relationship that named an old body or material path
    rewritten = 0
    ordered = sorted(list(remap.items()) + list(mat_remap.items()),
                     key=lambda kv: -kv[0].pathElementCount)
    for prim in st.Traverse():
        for rel in prim.GetRelationships():
            targets = rel.GetTargets()
            moved = []
            for t in targets:
                for old, new in ordered:
                    if t == old or t.HasPrefix(old):
                        t = t.ReplacePrefix(old, new)
                        break
                moved.append(t)
            if moved != targets:
                rel.SetTargets(moved)
                rewritten += 1
    rep["nested_bodies_flattened"] = len(remap)
    rep["relationships_repointed"] = rewritten

    expect = {remap.get(k, k): v for k, v in expect.items()}

    # -------------------------------------------------- 4. drives + friction
    # Mimic stripping happens in the same pass: 5.1 turns <mimic> into
    # PhysxMimicJointAPI, but the cross-finger chains misbehave (measured
    # 2026-09-15), so followers become plain limited joints driven from the
    # command side.  Gains: base joints from drive_gains.json (via the
    # REVERSE_JOINTS translation), gripper joints from the constants above.
    WHEEL_JOINTS_set = WHEEL_JOINTS
    applied, mimic_stripped, grip_limits_fixed = [], [], []
    for prim in st.Traverse():
        if not prim.IsA(UsdPhysics.Joint) or prim.IsA(UsdPhysics.FixedJoint):
            continue
        name = prim.GetName()
        # --- strip any PhysxMimicJointAPI (+ its attributes/relationships)
        apis = [a for a in prim.GetAppliedSchemas()
                if a.startswith("PhysxMimicJointAPI")]
        for a in apis:
            inst = a.split(":", 1)[1] if ":" in a else ""
            prim.RemoveAPI(PhysxSchema.PhysxMimicJointAPI, inst)
        for pn in [n for n in prim.GetPropertyNames()
                   if n.startswith("physxMimicJoint")]:
            prim.RemoveProperty(pn)
        if apis:
            mimic_stripped.append(name)

        is_grip = name in GRIP_ALL
        built = naming_rules.REVERSE_JOINTS.get(name, name)
        g, d = gains.get(built), dyn.get(built)
        if g is None or d is None:
            if not is_grip:
                rep.setdefault("no_gains", []).append(name)
                continue
            g, d = {"stiffness": GRIP_K, "damping": GRIP_C}, \
                   {"effort": GRIP_EFFORT, "velocity": GRIP_VEL}
        is_rev = prim.IsA(UsdPhysics.RevoluteJoint)
        token = "angular" if is_rev else "linear"
        drive = UsdPhysics.DriveAPI.Apply(prim, token)
        conv = D2R if is_rev else 1.0
        k, c = float(g["stiffness"]), float(g["damping"])
        if name in WHEEL_JOINTS_set and k != 0.0:
            raise SystemExit(f"{name}: wheel must have zero stiffness")
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(k * conv)
        drive.CreateDampingAttr().Set(c * conv)
        drive.CreateMaxForceAttr().Set(float(d["effort"]))
        if name in WHEEL_JOINTS_set:
            drive.CreateTargetVelocityAttr().Set(0.0)
            # B 自由滚: 限扭压到抓地以下, 0 速目标只是 2 Nm 的轻阻力
            drive.CreateMaxForceAttr().Set(WHEEL_DRIVE_MAX_FORCE)
        else:
            # 0 = 闭合位 (v3 基座零位; 夹爪行程语义同为 0=闭合)
            drive.CreateTargetPositionAttr().Set(0.0)
        vel = float(d["velocity"])
        prim.CreateAttribute("physxJoint:maxJointVelocity",
                             Sdf.ValueTypeNames.Float, False).Set(
            vel * R2D if is_rev else vel)
        if name in WHEEL_JOINTS_set:
            PhysxSchema.PhysxJointAPI.Apply(prim)
            prim.CreateAttribute("physxJoint:jointFriction",
                                 Sdf.ValueTypeNames.Float).Set(
                WHEEL_JOINT_FRICTION)
        if is_grip:
            # limits re-authored from the URDF (rad -> deg for revolute; a
            # LIMIT is an angle so R2D here - conv above is the GAIN
            # per-rad->per-degree factor and goes the other way):
            # protects against an import that did NOT check Ignore Mimic,
            # where 5.1 drops left-hand follower limits to +/-inf.  Follower
            # URDF limits are already multiplied by the mimic multiplier.
            spec = GRIP_ALL[name]
            lo, hi = spec["lower"], spec["upper"]
            lim_conv = R2D if is_rev else 1.0
            prim.GetAttribute("physics:lowerLimit").Set(lo * lim_conv)
            prim.GetAttribute("physics:upperLimit").Set(hi * lim_conv)
            grip_limits_fixed.append(name)
        applied.append(name)
    rep["drives_applied"] = sorted(applied)
    rep["mimic_stripped"] = sorted(mimic_stripped)
    rep["grip_limits_fixed"] = sorted(grip_limits_fixed)
    if len(applied) != N_MOVABLE:
        raise SystemExit(f"applied {len(applied)} drives, expected {N_MOVABLE}")

    # ------------------------------------------- 5. articulation headroom
    art = next(p for p in st.Traverse()
               if p.HasAPI(UsdPhysics.ArticulationRootAPI))
    PhysxSchema.PhysxArticulationAPI.Apply(art)
    art.CreateAttribute("physxArticulation:solverPositionIterationCount",
                        Sdf.ValueTypeNames.Int).Set(32)
    art.CreateAttribute("physxArticulation:solverVelocityIterationCount",
                        Sdf.ValueTypeNames.Int).Set(4)
    rep["articulation"] = art.GetPath().pathString

    # --------------------------------- 6b. normalize collision prims (v2)
    # The 5.1 API importer authors CollisionAPI on the per-hull *container*
    # Xforms (physics:approximation=convexHull on an Xform makes PhysX cook
    # the whole subtree as one blob - or worse, the children as dynamic
    # triangle meshes) and leaves every collision prim at default purpose, so
    # all colliders get RENDERED as gray shells on top of the visual model.
    # v2's flat has it right: CollisionAPI+convexHull on each hull Mesh and
    # purpose=guide on everything collision.  Normalize to that.
    def mesh_descendants(prim):
        for c in prim.GetChildren():
            if c.IsA(UsdGeom.Mesh):
                yield c
            else:
                yield from mesh_descendants(c)

    coll_fixed = 0
    for prim in list(st.Traverse()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if prim.IsA(UsdGeom.Gprim):
            UsdGeom.Imageable(prim).CreatePurposeAttr().Set(
                UsdGeom.Tokens.guide)
            continue
        # container Xform: push collision down onto each hull mesh
        for mesh in mesh_descendants(prim):
            UsdPhysics.CollisionAPI.Apply(mesh)
            mc = UsdPhysics.MeshCollisionAPI.Apply(mesh)
            mc.CreateApproximationAttr().Set("convexHull")
            UsdGeom.Imageable(mesh).CreatePurposeAttr().Set(
                UsdGeom.Tokens.guide)
            coll_fixed += 1
        prim.RemoveAPI(UsdPhysics.CollisionAPI)
        prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
        if prim.GetAttribute("physics:approximation"):
            prim.RemoveProperty("physics:approximation")
    rep["collision_meshes_normalized"] = coll_fixed

    # --------------------------------------------------- 6. physics scene
    scenes = [p for p in st.Traverse() if p.IsA(UsdPhysics.Scene)]
    if scenes:
        scene = scenes[0]
    else:
        scene = st.DefinePrim("/physicsScene", "PhysicsScene")
        UsdPhysics.Scene(scene)
    PhysxSchema.PhysxSceneAPI.Apply(scene)
    scene.CreateAttribute("physxScene:timeStepsPerSecond",
                          Sdf.ValueTypeNames.Int).Set(240)
    mpu = rep["metersPerUnit"] or 1.0
    ga = scene.CreateAttribute("physics:gravityAcceleration",
                               Sdf.ValueTypeNames.Float)
    ga.Set(-9.81 / mpu)
    rep["scene"] = scene.GetPath().pathString

    for c in CRUFT:
        p = st.GetPrimAtPath(c)
        if p and p.IsValid():
            st.RemovePrim(p.GetPath())

    # ------------------------- 6c. weld mesh points + drop authored normals
    # The 5.1 API importer writes STL meshes UNWELDED - every facet keeps its
    # own 3 points (weld ratio 1.0) plus a per-point normal: ~84 B/tri.  Weld
    # at 1e-9 m (lossless for shared STL corners; convex-hull cooking ignores
    # duplicate points) and drop authored normals (Hydra recomputes them).
    w_meshes = w_before = w_after = 0
    for p in st.Traverse():
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        pts = m.GetPointsAttr().Get()
        idx = m.GetFaceVertexIndicesAttr().Get()
        if pts is None or idx is None or not len(pts):
            continue
        w_meshes += 1
        w_before += len(pts)
        q = np.array(pts, dtype=np.float64).reshape(-1, 3)
        np.round(q, 9, out=q)          # 1e-9 m weld tolerance
        view = q.view([("x", np.float64), ("y", np.float64),
                       ("z", np.float64)]).ravel()
        uniq, inv = np.unique(view, return_inverse=True)
        w_after += len(uniq)
        new_pts = np.stack([uniq["x"], uniq["y"], uniq["z"]],
                           axis=1).astype(np.float32)
        m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(new_pts))
        m.GetFaceVertexIndicesAttr().Set(
            Vt.IntArray.FromNumpy(inv[np.asarray(idx)].astype(np.int32)))
        if m.GetNormalsAttr().HasAuthoredValue():
            m.GetNormalsAttr().Clear()
        ext = m.GetExtentAttr()
        if ext.HasAuthoredValue():
            ext.Set([Gf.Vec3f(*[float(v) for v in new_pts.min(axis=0)]),
                     Gf.Vec3f(*[float(v) for v in new_pts.max(axis=0)])])
    rep["mesh_welding"] = {"meshes": w_meshes, "points_before": w_before,
                           "points_after": w_after}
    if w_before and w_after / w_before > 0.9:
        raise SystemExit(f"welding ineffective: {w_after}/{w_before} points")

    root = st.GetPrimAtPath(ROOT)
    st.SetDefaultPrim(root)

    st.GetRootLayer().Export(str(DST))
    rep["exported"] = str(DST)

    # ------------------------------------------------------- 7. verify
    chk = Usd.Stage.Open(str(DST))
    fails = []

    inst_left = [p.GetPath().pathString for p in chk.Traverse()
                 if p.IsInstanceable()]
    rep["verify_instanceable"] = len(inst_left)
    if inst_left:
        fails.append(f"instanceable prims remain: {inst_left[:3]}")
    n_mesh = sum(1 for p in chk.Traverse() if p.IsA(UsdGeom.Mesh))
    rep["verify_mesh_prims"] = n_mesh
    if n_mesh == 0:
        fails.append("no real Mesh prims in export")
    n_norm = sum(1 for p in chk.Traverse() if p.IsA(UsdGeom.Mesh)
                 and UsdGeom.Mesh(p).GetNormalsAttr().HasAuthoredValue())
    rep["verify_meshes_with_normals"] = n_norm
    if n_norm:
        fails.append(f"{n_norm} meshes still carry authored normals")
    mats2 = [p for p in chk.Traverse() if p.IsA(UsdShade.Material)]
    rep["verify_materials"] = [p.GetPath().pathString for p in mats2]
    if not mats2:
        fails.append("no materials in export")
    dangling = []
    for p in chk.Traverse():
        rel = p.GetRelationship("material:binding")
        if not rel:
            continue
        for t in rel.GetTargets():
            tp = chk.GetPrimAtPath(t)
            if not tp or not tp.IsValid():
                dangling.append(f"{p.GetPath()} -> {t}")
    rep["verify_bindings_dangling"] = dangling[:5]
    if dangling:
        fails.append(f"{len(dangling)} dangling material bindings")

    # collision structure must match v2: CollisionAPI only on Gprims, all
    # guide purpose, every mesh collider convexHull-approximated
    bad_coll = []
    n_coll = 0
    for p in chk.Traverse():
        if not p.HasAPI(UsdPhysics.CollisionAPI):
            continue
        n_coll += 1
        if not p.IsA(UsdGeom.Gprim):
            bad_coll.append(f"container {p.GetPath()}")
            continue
        pur = UsdGeom.Imageable(p).GetPurposeAttr().Get()
        if pur != UsdGeom.Tokens.guide:
            bad_coll.append(f"{p.GetPath()} purpose={pur}")
        if p.IsA(UsdGeom.Mesh):
            ap = p.GetAttribute("physics:approximation")
            if not ap or ap.Get() != "convexHull":
                bad_coll.append(f"{p.GetPath()} approx={ap.Get() if ap else None}")
    rep["verify_colliders"] = n_coll
    rep["verify_collision_bad"] = bad_coll[:5]
    if bad_coll:
        fails.append(f"{len(bad_coll)} bad collision prims")

    bodies2 = [p for p in chk.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    rep["verify_bodies"] = len(bodies2)
    if len(bodies2) != N_BODIES:
        fails.append(f"bodies {len(bodies2)} != {N_BODIES}")
    bad_nesting = [f"{p.GetName()} under {body_ancestor(p).GetName()}"
                   for p in bodies2 if body_ancestor(p) is not None]
    if bad_nesting:
        fails.append(f"nested bodies: {bad_nesting[:5]}")

    drift = []
    for path, want in expect.items():
        p = chk.GetPrimAtPath(path)
        if not p or not p.IsValid():
            drift.append(f"{path} MISSING")
            continue
        got = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        dd = (got.ExtractTranslation() - want.ExtractTranslation()).GetLength()
        if dd > 1e-6:
            drift.append(f"{p.GetName()} moved {dd * 1e3:.3f} mm")
    if drift:
        fails.append(f"pose drift: {drift[:5]}")

    c2 = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           ["default", "render", "proxy", "guide"])
    r = c2.ComputeWorldBound(chk.GetPrimAtPath(ROOT)).ComputeAlignedRange()
    rep["flat_bottom_z"] = r.GetMin()[2]
    rep["flat_extent"] = [list(r.GetMin())[:2], list(r.GetMax())[:2]]
    if abs(r.GetMin()[2]) > 1e-4:
        fails.append(f"bottom z {r.GetMin()[2]:.6f} != 0")

    joints2 = [p for p in chk.Traverse()
               if p.IsA(UsdPhysics.Joint) and not p.IsA(UsdPhysics.FixedJoint)]
    bad_drive, unwired, bad_lim, mim_left = [], [], [], []
    for j in joints2:
        tok = "angular" if j.IsA(UsdPhysics.RevoluteJoint) else "linear"
        dr = UsdPhysics.DriveAPI.Get(j, tok)
        if not dr or not dr.GetStiffnessAttr().HasAuthoredValue():
            bad_drive.append(j.GetName())
        ends = [t for rel in (UsdPhysics.Joint(j).GetBody0Rel(),
                              UsdPhysics.Joint(j).GetBody1Rel())
                for t in rel.GetTargets()]
        live = [t for t in ends
                if chk.GetPrimAtPath(t)
                and chk.GetPrimAtPath(t).HasAPI(UsdPhysics.RigidBodyAPI)]
        if not live:
            unwired.append(j.GetName())
        if j.GetName() in GRIP_ALL:
            lo = j.GetAttribute("physics:lowerLimit").Get()
            hi = j.GetAttribute("physics:upperLimit").Get()
            conv = R2D if j.IsA(UsdPhysics.RevoluteJoint) else 1.0
            want = (GRIP_ALL[j.GetName()]["lower"] * conv,
                    GRIP_ALL[j.GetName()]["upper"] * conv)
            if lo is None or hi is None or abs(lo - want[0]) > 1e-3 \
                    or abs(hi - want[1]) > 1e-3:
                bad_lim.append(f"{j.GetName()} [{lo},{hi}] != [{want[0]},{want[1]}]")
            if any(a.startswith("PhysxMimicJointAPI")
                   for a in j.GetAppliedSchemas()):
                mim_left.append(j.GetName())
    rep["movable_joints"] = len(joints2)
    rep["verify_grip_limits_bad"] = bad_lim[:5]
    rep["verify_mimic_left"] = mim_left
    if len(joints2) != N_MOVABLE:
        fails.append(f"movable joints {len(joints2)} != {N_MOVABLE}")
    if bad_drive:
        fails.append(f"joints without drive: {bad_drive}")
    if unwired:
        fails.append(f"joints with no live body: {unwired}")
    if bad_lim:
        fails.append(f"gripper limits wrong: {bad_lim}")
    if mim_left:
        fails.append(f"mimic APIs remain: {mim_left}")

    for j in joints2:
        if j.GetName() in WHEEL_JOINTS_set:
            v = j.GetAttribute("physxJoint:jointFriction").Get()
            if v is None or abs(v - WHEEL_JOINT_FRICTION) > 1e-6:
                fails.append(f"{j.GetName()} jointFriction={v}")
            mf = UsdPhysics.DriveAPI.Get(j, "angular").GetMaxForceAttr().Get()
            if mf is None or abs(mf - WHEEL_DRIVE_MAX_FORCE) > 1e-6:
                fails.append(f"{j.GetName()} drive maxForce={mf}")
            tv = UsdPhysics.DriveAPI.Get(j, "angular").GetTargetVelocityAttr().Get()
            if tv is None or abs(tv) > 1e-9:
                fails.append(f"{j.GetName()} targetVelocity={tv}")
            kk = UsdPhysics.DriveAPI.Get(j, "angular").GetStiffnessAttr().Get()
            if kk != 0.0:
                fails.append(f"{j.GetName()} stiffness={kk}")
    art2 = next(p for p in chk.Traverse()
                if p.HasAPI(UsdPhysics.ArticulationRootAPI))
    spi = art2.GetAttribute("physxArticulation:solverPositionIterationCount").Get()
    svi = art2.GetAttribute("physxArticulation:solverVelocityIterationCount").Get()
    rep["verify_solver"] = [spi, svi]
    if (spi, svi) != (32, 4):
        fails.append(f"solver {spi}/{svi} != 32/4")
    sc2 = next(p for p in chk.Traverse() if p.IsA(UsdPhysics.Scene))
    tps = sc2.GetAttribute("physxScene:timeStepsPerSecond").Get()
    rep["verify_timestep"] = tps
    if tps != 240:
        fails.append(f"timestep {tps} != 240")

    mass = sum(UsdPhysics.MassAPI(p).GetMassAttr().Get() or 0.0
               for p in chk.Traverse() if p.HasAPI(UsdPhysics.MassAPI))
    rep["total_mass"] = round(mass, 3)
    if abs(mass - TOTAL_MASS) > 0.5:
        fails.append(f"mass {mass:.3f} != {TOTAL_MASS}")

    rep["verify_fails"] = fails
    rep["ok"] = not fails
except Exception as e:  # noqa: BLE001
    import traceback
    rep["error"] = f"{type(e).__name__}: {e}"
    rep["traceback"] = traceback.format_exc().splitlines()[-6:]

with open(REPORT, "w") as f:
    json.dump(rep, f, indent=1, ensure_ascii=False)

app.close()
