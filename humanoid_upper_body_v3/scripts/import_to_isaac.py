#!/usr/bin/env python3
"""Import the v3 URDF into USD with the isaacsim5.1 Kit importer, headless.

Run with:  /home/jy/micromamba/envs/isaacsim5.1/bin/python3.11 scripts/import_to_isaac.py

Produces exactly what the GUI importer produces (the .usda plus a Payload/
directory with the mesh sublayers) at  usd/humanoid_upper_body_v3.usda.
Drives / PhysX tuning / rigid-body flattening are NOT done here - Kit swallows
prints and the pure-pxr pass in finalize_usd.py (proven on v2) does that with
real verification.  Kit still swallows prints in this mode, so status lands in
/tmp/v3_import_result.json.

Config choices (defaults in parens):
  fix_base=False (True)        free base - the URDF has no world attachment
  merge_fixed_joints=False (True)  keep Z1/Y1 etc. so ROS bridge sees the URDF tree
  import_inertia_tensor=True (True)  use the URDF inertials, the CAD-computed ones
  self_collision=False (False) adjacent-filtering only, as the collision spec assumes
  make_default_prim=True (False) so opening the file selects the robot
"""

import json
from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

PKG = Path(__file__).resolve().parent.parent
URDF = PKG / "urdf" / "humanoid_upper_body_v3.urdf"
OUT = PKG / "usd" / "humanoid_upper_body_v3.usda"

res = {"urdf": str(URDF), "out": str(OUT), "error": None}
try:
    import omni.kit.commands
    from isaacsim.asset.importer.urdf import _urdf
    from pxr import Usd

    cfg = _urdf.ImportConfig()
    cfg.set_fix_base(False)
    cfg.set_merge_fixed_joints(False)
    cfg.set_import_inertia_tensor(True)
    cfg.set_self_collision(False)
    cfg.set_make_default_prim(True)
    cfg.set_default_drive_type(
        int(_urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION))

    OUT.parent.mkdir(parents=True, exist_ok=True)
    root_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(URDF),
        import_config=cfg,
        dest_path=str(OUT),
        get_articulation_root=True,
    )
    res["root_path"] = str(root_path)

    # import_robot authors into the layer at dest_path; make sure it is on disk
    # whatever stage handle it used internally.
    from pxr import Sdf

    Sdf.Layer.FindOrOpen(str(OUT)).Save()

    # reopen from disk and count what actually landed
    chk = Usd.Stage.Open(str(OUT))
    bodies = [p.GetPath().pathString for p in chk.Traverse()
              if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    joints = [p.GetName() for p in chk.Traverse() if p.IsA(UsdPhysics.Joint)]
    res["bodies"] = len(bodies)
    res["joints"] = len(joints)
    res["art_root"] = [p.GetPath().pathString for p in chk.Traverse()
                       if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    res["has_root_joint"] = bool(chk.GetPrimAtPath("/humanoid_upper_body_v3/Physics/root_joint"))
except Exception as e:  # noqa: BLE001
    res["error"] = f"{type(e).__name__}: {e}"

with open("/tmp/v3_import_result.json", "w") as f:
    json.dump(res, f, indent=1)

app.close()
