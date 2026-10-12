"""桌前场景:把 VLA 资产里的桌子(剥掉 piper)放到机器人前方,桌面上摆可抓物体。

要点(2026-09-27 侦察结论):
  · 桌子只引用 piper_table_clean_v2.usd 的 /World/table 子 prim(piper 在
    /World/piperv2,不引用即天然排除);直接引 SubUSDs/table.usd 会得到半高桌。
  · 桌顶面 z=0.7366m、尺寸 1.83(X)×1.22(Y),自身贴地、以原点为中心。
  · 物体取 SubUSDs/model_*.usd(瓶/充电器),挂到桌子 prim 下并加刚体/碰撞。
必须在"抬臂之后、加相机之前后皆可"(prim 编辑会让物理视图失效,抬臂须先用)。
"""
import json

import numpy as np
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

stage = omni.usd.get_context().get_stage()
xc = UsdGeom.XformCache(Usd.TimeCode.Default())
out = {"table": {}, "objects": {}}

VLA = "/home/jy/VLA/Collected_piper_table_clean_v2"
TABLE_USD = f"{VLA}/piper_table_clean_v2.usd"
SUBS = f"{VLA}/piper_table/SubUSDs"
TABLE_PRIM = "/World/vla_table"
DIST = 1.05          # 桌子中心到机器人基座的水平距离(m)
TOP_Z = 0.7366       # 桌面高度(资产实测值)


def gf_to_np(gf_m):
    return np.array([[gf_m[i][j] for j in range(4)] for i in range(4)]).T


# ---- 机器人基座位姿 → 桌子世界位姿 ----
M = gf_to_np(xc.GetLocalToWorldTransform(
    stage.GetPrimAtPath("/humanoid/base_link")))
yaw = float(np.arctan2(M[1, 0], M[0, 0]))
fwd = np.array([np.cos(yaw), np.sin(yaw)])
center = M[:2, 3] + DIST * fwd
out["table"] = {"yaw_deg": round(np.degrees(yaw), 2),
                "center": [round(float(v), 3) for v in center]}

# ---- 引用桌子(只取 /World/table) ----
prim = stage.GetPrimAtPath(TABLE_PRIM)
if not prim.IsValid():
    prim = stage.DefinePrim(TABLE_PRIM)
try:
    prim.GetReferences().AddReference(TABLE_USD, "/World/table")
    out["table"]["ref"] = "ok"
except Exception as e:
    out["table"]["ref"] = "ERR:" + repr(e)[:200]

q = Gf.Quatd(float(np.cos(yaw / 2)), 0.0, 0.0, float(np.sin(yaw / 2)))


def set_op(p, name, value, dbl_type=Gf.Vec3d, flt_type=Gf.Vec3f):
    """写 xformOp: 资产里已有的 op 可能是 Float 精度, 类型必须匹配。"""
    a = p.GetAttribute(name)
    if a.IsValid() and a.GetTypeName() == Sdf.ValueTypeNames.Float3:
        a.Set(flt_type(*value))
    elif a.IsValid():
        a.Set(dbl_type(*value))
    else:
        p.CreateAttribute(name, Sdf.ValueTypeNames.Double3).Set(
            tuple(float(v) for v in value))


def set_orient(p, q):
    a = p.GetAttribute("xformOp:orient")
    if a.IsValid() and a.GetTypeName() == Sdf.ValueTypeNames.Quatf:
        a.Set(Gf.Quatf(float(q.real), float(q.imaginary[0]),
                       float(q.imaginary[1]), float(q.imaginary[2])))
    elif a.IsValid():
        a.Set(q)
    else:
        p.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatd).Set(q)


set_op(prim, "xformOp:translate", (float(center[0]), float(center[1]), 0.0))
set_orient(prim, q)
prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.StringArray).Set(
    ["xformOp:translate", "xformOp:orient"])

# ---- 桌面上的可抓物体(桌子局部系;面向机器人的一侧是 -X) ----
# (文件, 局部x, 局部y, 说明)  —— 资产原点一般在底面中心
OBJECTS = [
    ("cube_a", ("cube", 0.035), -0.45, -0.35, (0.90, 0.35, 0.15)),
    ("cyl_b", ("cyl", 0.030, 0.12), -0.35, 0.38, (0.20, 0.60, 0.90)),
    ("cube_c", ("cube", 0.030), -0.20, 0.20, (0.95, 0.80, 0.20)),
]
from pxr import UsdShade
for name, spec, lx, ly, rgb in OBJECTS:
    path = f"{TABLE_PRIM}/{name}"
    try:
        if stage.GetPrimAtPath(path).IsValid():
            stage.RemovePrim(path)   # 幂等
        p = stage.DefinePrim(path)
        shape = spec[0]
        if shape == "cube":
            g = UsdGeom.Cube.Define(stage, path)
            g.GetSizeAttr().Set(1.0)
            g.GetExtentAttr().Set([Gf.Vec3f(-0.5, -0.5, -0.5),
                                   Gf.Vec3f(0.5, 0.5, 0.5)])
            half = spec[1]
            scale = (half * 2, half * 2, half * 2)
            z_off = half
        else:  # cylinder
            r, h = spec[1], spec[2]
            g = UsdGeom.Cylinder.Define(stage, path)
            g.GetRadiusAttr().Set(1.0)
            g.GetHeightAttr().Set(1.0)
            g.GetAxisAttr().Set("Z")
            scale = (r, r, h)
            z_off = h / 2.0
        g.GetDisplayColorAttr().Set([Gf.Vec3f(*rgb)])
        if p.GetAttribute("xformOp:scale").IsValid():
            p.GetAttribute("xformOp:scale").Set(Gf.Vec3f(*scale))
        else:
            p.CreateAttribute("xformOp:scale",
                              Sdf.ValueTypeNames.Float3).Set(
                Gf.Vec3f(*scale))
        set_op(p, "xformOp:translate",
               (float(lx), float(ly), TOP_Z + float(z_off)))
        p.CreateAttribute("xformOpOrder",
                          Sdf.ValueTypeNames.StringArray).Set(
            ["xformOp:translate", "xformOp:scale"])
        # 物理: 刚体 + 碰撞 + 摩擦材质(抓取靠摩擦)
        UsdPhysics.RigidBodyAPI.Apply(p)
        UsdPhysics.CollisionAPI.Apply(p)
        mass = UsdPhysics.MassAPI.Apply(p)
        mass.CreateMassAttr(0.05)
        mat_path = f"{path}/physmat"
        mat = UsdShade.Material.Define(stage, mat_path)
        pmat = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        pmat.CreateDynamicFrictionAttr(1.0)
        pmat.CreateStaticFrictionAttr(1.0)
        pmat.CreateRestitutionAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            mat, UsdShade.Tokens.weakerThanDescendant, "physics")
        out["objects"][name] = {"spec": str(spec), "pos": [lx, ly]}
    except Exception as e:
        out["objects"][name] = "ERR:" + repr(e)[:150]

# ---- 第三人称遥操作相机(挂 base_link 下随机器人移动, 头显流送这一路) ----
# 头显流 head_zed_x 时看不到桌面; 这一路从机器人后上方俯视桌面 + 双臂,
# 是抓取作业的标准视角。挂载用 add_camera(需要 HIDE_MOUNTS=False 以免隐藏
# 基座本体视觉)。
try:
    executor.HIDE_MOUNTS = False
    tv_ol = "/humanoid/base_link/tv_ol"
    tv_cam = f"{tv_ol}/tv_cam"
    # 自建相机对象(不经 add_camera 的 scene 管理: 重跑时对象/prims 不会相互
    # 失效, 名字带时间戳避免 scene 重名)
    import time as _time
    executor._cameras.pop(tv_cam, None)
    if stage.GetPrimAtPath(tv_ol).IsValid():
        stage.RemovePrim(tv_ol)
    stage.DefinePrim(tv_ol)
    UsdGeom.Camera.Define(stage, tv_cam)
    o = None
    # 相对基座的位姿: 世界系构造(基座后 1.6m、上 1.9m, 看桌前 1.05m / 高 0.75),
    # 再反算回 base_link 局部系——不依赖 base 轴的约定假设。
    M_base = gf_to_np(xc.GetLocalToWorldTransform(
        stage.GetPrimAtPath("/humanoid/base_link")))
    fwd_w = M_base[:3, 0]                    # 基座朝向 = 其 X 轴
    fwd_w[2] = 0.0
    fwd_w /= max(np.linalg.norm(fwd_w), 1e-9)
    base_w = M_base[:3, 3]
    eye_w = base_w - 1.6 * fwd_w + np.array([0.0, 0.0, 1.9])
    tgt_w = base_w + 1.05 * fwd_w + np.array([0.0, 0.0, 0.75])
    d = tgt_w - eye_w
    d /= np.linalg.norm(d)

    def _rpy_to_R(r, p, y):
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy_ = np.cos(y), np.sin(y)
        return (np.array([[cy, -sy_, 0], [sy_, cy, 0], [0, 0, 1]])
                @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
                @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))

    pitch = float(-np.arcsin(np.clip(d[2], -1, 1)))
    yawc = float(np.arctan2(d[1], d[0]))
    R_opt = _rpy_to_R(np.deg2rad(-90), 0, np.deg2rad(-90))
    R_cam = _rpy_to_R(np.deg2rad(180), 0, 0)
    R_w = _rpy_to_R(0.0, pitch, yawc) @ R_opt @ R_cam
    M_cam_w = np.eye(4)
    M_cam_w[:3, :3] = R_w
    M_cam_w[:3, 3] = eye_w
    M_loc = np.linalg.inv(M_base) @ M_cam_w
    eye_l = M_loc[:3, 3]
    R = M_loc[:3, :3]
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1) * 2
        q = Gf.Quatd(0.25 * s, (R[2, 1] - R[1, 2]) / s,
                     (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            q = Gf.Quatd((R[2, 1] - R[1, 2]) / s, 0.25 * s,
                         (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
        elif i == 1:
            s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            q = Gf.Quatd((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                         0.25 * s, (R[1, 2] + R[2, 1]) / s)
        else:
            s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            q = Gf.Quatd((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                         (R[1, 2] + R[2, 1]) / s, 0.25 * s)
    p_cam = stage.GetPrimAtPath(tv_cam)
    p_cam.CreateAttribute("xformOp:translate",
                          Sdf.ValueTypeNames.Double3).Set(
        tuple(float(v) for v in eye_l))
    p_cam.CreateAttribute("xformOp:orient",
                          Sdf.ValueTypeNames.Quatd).Set(q)
    p_cam.CreateAttribute("xformOpOrder",
                          Sdf.ValueTypeNames.StringArray).Set(
        ["xformOp:translate", "xformOp:orient"])
    # 建 isaacsim 相机对象(唯一名字, 直写内参)
    from isaacsim.sensors.camera import Camera as _IsaCamera
    o = _IsaCamera(prim_path=tv_cam,
                   name=f"tv_cam_{int(_time.time())}",
                   resolution=(1280, 800))
    cp = o.prim
    cp.GetAttribute("focalLength").Set(10.477)   # fx=640 @1280 (mm 直写)
    cp.GetAttribute("horizontalAperture").Set(20.955)
    cp.GetAttribute("clippingRange").Set(Gf.Vec2f(0.05, 100.0))
    o.initialize()
    executor._cameras[tv_cam] = o
    try:
        o.set_local_pose(translation=[float(v) for v in eye_l],
                         orientation=[q.real, q.imaginary[0],
                                      q.imaginary[1], q.imaginary[2]],
                         camera_axes="usd")
        o.set_default_state()
    except Exception:
        pass
    out["teleop_view_cam"] = "ok"
except Exception as e:
    out["teleop_view_cam"] = "ERR:" + repr(e)[:200]

with open("/tmp/add_table.json", "w") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("OK")