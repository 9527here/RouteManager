"""全新场景一键恢复: 7相机添加+文档落位+对象默认态固化 + 三色球跟踪实验。"""
import json
import traceback

import numpy as np
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom
from PIL import Image

stage = omni.usd.get_context().get_stage()
xc = UsdGeom.XformCache(Usd.TimeCode.Default())
out = {}


def gf_to_np(gf_m):
    M = np.array([[gf_m[i][j] for j in range(4)] for i in range(4)])
    return M.T


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p),
                              np.sin(p), np.cos(y), np.sin(y))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def T(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def mat2quat(m):
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    return Gf.Quatd(w, x, y, z)


R_opt = rpy_to_R(np.deg2rad(-90), 0, np.deg2rad(-90))
R_cam = rpy_to_R(np.deg2rad(180), 0, 0)

CAMS = {
    "head_zed_x": ("head_zed_x_link", "head_zed_x_cam_v2", "head_pitch_link",
                   [0.015296, -0.006469, 0.131663], [0.0, 0.004008, 0.000029],
                   [1920, 1200], 1144.08, [0.10, 8.0]),
    "head_surround_left": ("head_surround_left_link",
                           "head_surround_left_cam_v2", "head_pitch_link",
                           [-0.035643, 0.023177, 0.128458],
                           [0.004008, 0.0, 1.570825],
                           [1920, 1200], 672.20, [0.30, 15.0]),
    "head_surround_right": ("head_surround_right_link",
                            "head_surround_right_cam_v2", "head_pitch_link",
                            [-0.033642, -0.036098, 0.128450],
                            [-0.004008, 0.0, -1.570768],
                            [1920, 1200], 672.20, [0.30, 15.0]),
    "head_surround_rear": ("head_surround_rear_link",
                           "head_surround_rear_cam_v2", "head_pitch_link",
                           [-0.057780, -0.006461, 0.128546],
                           [0.0, -0.004008, -3.141564],
                           [1920, 1200], 672.20, [0.30, 15.0]),
    "chest_gemini_335lg": ("chest_gemini_335lg_link", "chest_gemini_cam_v2",
                           "torso_pitch_link",
                           [0.096566, -0.001075, 0.219600],
                           [0.0, 0.318750, 0.0],
                           [1280, 800], 640.00, [0.15, 10.0]),
    "left_wrist_gemini_335lg": ("left_wrist_gemini_335lg_link",
                                "left_wrist_gemini_cam_v2", "left_wrist_link",
                                [-0.047500, -0.031557, 0.118409],
                                [0.0, 0.415400, 1.449500],
                                [1280, 800], 640.00, [0.15, 10.0]),
    "right_wrist_gemini_335lg": ("right_wrist_gemini_335lg_link",
                                 "right_wrist_gemini_cam_v2",
                                 "right_wrist_link",
                                 [0.047500, 0.031557, 0.118409],
                                 [0.0, 1.327400, -1.453800],
                                 [1280, 800], 640.00, [0.15, 10.0]),
}

# ---- 1) 添加相机(带内参/裁剪) ----
out["add"] = {}
for nm, (link, leaf, parent, xyz, rpy, res, fx, clip) in CAMS.items():
    ol_path = f"/humanoid/{link}/{nm}_optical_link"
    cam_path = f"{ol_path}/{leaf}"
    try:
        executor.add_camera(prim_path=cam_path, attach_to=ol_path,
                            resolution=res, intrinsics={"fx": fx},
                            clipping_range=clip)
        out["add"][nm] = "OK"
    except Exception as e:
        out["add"][nm] = repr(e)[:200]

# ---- 2) 文档落位(直接写 USD ops) ----
out["placed"] = {}
LOCALS = {}
for nm, (link, leaf, parent, xyz, rpy, res, fx, clip) in CAMS.items():
    ol_path = f"/humanoid/{link}/{nm}_optical_link"
    cam_path = f"{ol_path}/{leaf}"
    p_cam = stage.GetPrimAtPath(cam_path)
    p_P = stage.GetPrimAtPath(f"/humanoid/{parent}")
    p_ol = stage.GetPrimAtPath(ol_path)
    if not (p_cam.IsValid() and p_P.IsValid() and p_ol.IsValid()):
        out["placed"][nm] = "PRIM_MISSING"
        continue
    M_P = gf_to_np(xc.GetLocalToWorldTransform(p_P))
    M_cam_w = M_P @ T(rpy_to_R(*rpy), xyz) @ T(R_opt, [0, 0, 0]) \
        @ T(R_cam, [0, 0, 0])
    M_ol = gf_to_np(xc.GetLocalToWorldTransform(p_ol))
    M_local = np.linalg.inv(M_ol) @ M_cam_w
    LOCALS[nm] = M_local
    q = mat2quat(M_local[:3, :3])
    UsdGeom.Xformable(p_cam).ClearXformOpOrder()
    p_cam.CreateAttribute("xformOp:translate",
                          Sdf.ValueTypeNames.Double3).Set(
        tuple(float(v) for v in M_local[:3, 3]))
    p_cam.CreateAttribute("xformOp:orient",
                          Sdf.ValueTypeNames.Quatd).Set(q)
    p_cam.CreateAttribute("xformOpOrder",
                          Sdf.ValueTypeNames.StringArray).Set(
        ["xformOp:translate", "xformOp:orient"])
    w = gf_to_np(xc.GetLocalToWorldTransform(p_cam))
    out["placed"][nm] = [round(v, 3) for v in w[:3, 3]]

# ---- 3) 同步 Camera 对象内部状态 + 固化默认态(防 stop/play 复位) ----
out["sync"] = {}
for nm, (link, leaf, parent, xyz, rpy, res, fx, clip) in CAMS.items():
    ol_path = f"/humanoid/{link}/{nm}_optical_link"
    cam_path = f"{ol_path}/{leaf}"
    o = executor._cameras.get(cam_path)
    if o is None or nm not in LOCALS:
        out["sync"][nm] = "NO_OBJ"
        continue
    Ml = LOCALS[nm]
    q = mat2quat(Ml[:3, :3])
    try:
        o.set_local_pose(
            translation=[float(v) for v in Ml[:3, 3]],
            orientation=[q.real, q.imaginary[0], q.imaginary[1],
                         q.imaginary[2]],
            camera_axes="usd")
        msg = "pose_ok"
    except Exception as e:
        msg = "pose_err:" + repr(e)[:120]
    try:
        o.set_default_state()
        msg += "+default_ok"
    except Exception as e:
        try:
            o.save_state()
            msg += "+save_ok"
        except Exception as e2:
            msg += "+default_err:" + repr(e2)[:80]
    out["sync"][nm] = msg

# ---- 4) 三色参考球(相对机器人当前位姿前方 2/3/4 米) ----
M_base = gf_to_np(xc.GetLocalToWorldTransform(
    stage.GetPrimAtPath("/humanoid/base_link")))
yaw = np.arctan2(M_base[1, 0], M_base[0, 0])
fwd = np.array([np.cos(yaw), np.sin(yaw)])
rgt = np.array([-fwd[1], fwd[0]])
bxy = M_base[:2, 3]


def sph_xy(d, lat):
    return bxy + d * fwd + lat * rgt


def put_sphere(path, color, xy, z=0.2, r=0.2):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        s = UsdGeom.Sphere.Define(stage, path)
        prim = s.GetPrim()
    u = UsdGeom.Sphere(prim)
    u.CreateRadiusAttr().Set(r)
    u.CreateDisplayColorAttr().Set([Gf.Vec3f(*color)])
    UsdGeom.Xformable(prim).ClearXformOpOrder()
    prim.CreateAttribute("xformOp:translate",
                         Sdf.ValueTypeNames.Double3).Set(
        (float(xy[0]), float(xy[1]), float(z)))
    prim.CreateAttribute("xformOpOrder",
                         Sdf.ValueTypeNames.StringArray).Set(
        ["xformOp:translate"])
    return str(prim.GetPath())


SPHERES = {
    "/World/ref_red": ((1.0, 0.05, 0.05), sph_xy(2.0, 0.4)),
    "/World/ref_green": ((0.05, 1.0, 0.05), sph_xy(3.0, -0.6)),
    "/World/ref_blue": ((0.10, 0.15, 1.0), sph_xy(4.0, 0.2)),
}
out["spheres"] = {p: put_sphere(p, c, xy) for p, (c, xy) in SPHERES.items()}
out["base_yaw_deg"] = round(float(np.degrees(yaw)), 1)

# ---- 5) 渲染全部相机 ----
for _ in range(12):
    executor.world.step(render=True)
out["render"] = {}
for nm, (link, leaf, parent, xyz, rpy, res, fx, clip) in CAMS.items():
    cam_path = f"/humanoid/{link}/{nm}_optical_link/{leaf}"
    try:
        cam_obj = executor._cameras.get(cam_path)
        if cam_obj is None:
            out["render"][nm] = "NO_OBJECT"
            continue
        rgb = None
        for _ in range(20):
            rgb = cam_obj.get_rgba()
            if rgb is not None and rgb.size:
                break
            executor.world.step(render=True)
        if rgb is None or not rgb.size:
            out["render"][nm] = "EMPTY"
            continue
        img = rgb[:, :, :3] if rgb.ndim == 3 and rgb.shape[2] >= 3 else rgb
        Image.fromarray(img.astype("uint8")).save(f"/tmp/s4_{nm}.jpg")
        out["render"][nm] = list(img.shape[:2])
    except Exception as e:
        out["render"][nm] = repr(e)[:200]

# ---- 6) 渲染后 prim 世界位姿(供本地投影比对) ----
out["cam_world"] = {}
for nm, (link, leaf, parent, xyz, rpy, res, fx, clip) in CAMS.items():
    cam_path = f"/humanoid/{link}/{nm}_optical_link/{leaf}"
    p_cam = stage.GetPrimAtPath(cam_path)
    if not p_cam.IsValid():
        continue
    w = gf_to_np(xc.GetLocalToWorldTransform(p_cam))
    out["cam_world"][nm] = {
        "R": [[round(float(v), 4) for v in row] for row in w[:3, :3]],
        "C": [round(float(v), 3) for v in w[:3, 3]],
    }

with open("/tmp/cam_setup_s3.json", "w") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("OK")
