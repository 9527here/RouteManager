"""为静态场景中已存在的相机 prim 注册 isaacsim Camera 对象(渲染用)。

静态场景里的 Camera prim 是"死的"USD 数据; get_rgba 需要 Camera 对象。
这里只建对象(prim 已存在, 不 define 新 prim)+ initialize, 然后**验证物理
视图是否仍然健康**——若 jacobian 可读, 说明"机器人加载后零 prim 编辑"的目标
达成, 全部控制 API(关节写/底盘/IK)可用。
"""
import json
import time

out = {"cams": {}}
# 只让这些相机参与渲染/流送; 其余**保留对象但暂停**(省 GPU/CPU)
RENDER_CAMS = {"head_zed_x"}
CAMS = [
    ("head_zed_x", 1920, 1200),
    ("head_surround_left", 1920, 1200),
    ("head_surround_right", 1920, 1200),
    ("head_surround_rear", 1920, 1200),
    ("chest_gemini_335lg", 1280, 800),
    ("left_wrist_gemini_335lg", 1280, 800),
    ("right_wrist_gemini_335lg", 1280, 800),
    ("tv_cam", 1280, 800),
]

from isaacsim.sensors.camera import Camera as IsaCamera  # noqa: E402

import omni.usd  # noqa: E402

stage = omni.usd.get_context().get_stage()

for nm, w, h in CAMS:
    path = f"/World/cams/{nm}"
    if executor._cameras.get(path) is not None:      # noqa: F821
        out["cams"][nm] = "already"
        continue
    if not stage.GetPrimAtPath(path).IsValid():
        out["cams"][nm] = "PRIM_MISSING"
        continue
    try:
        o = IsaCamera(prim_path=path, name=f"{nm}_{int(time.time())}",
                      resolution=(w, h))
        if nm in RENDER_CAMS:
            o.initialize()               # 只有流送相机建渲染管线
            out["cams"][nm] = "ok(rendering)"
        else:
            # 保留对象但**不初始化**: 无渲染管线/无 buffer(get_rgba 不可用,
            # 需要时调 o.initialize() 即可启用)
            out["cams"][nm] = "ok(reserved)"
        executor._cameras[path] = o      # noqa: F821
    except Exception as e:
        out["cams"][nm] = "ERR:" + repr(e)[:130]

# 同时把新路径登记为流送相机(isac_teleop 按 'tv_cam' 匹配 ✓ 路径含 tv_cam)
# 关键验证: 物理视图是否仍健康
try:
    J = executor.articulation.get_jacobians()          # noqa: F821
    out["jacobian"] = f"ALIVE {J.shape}"
except Exception as e:
    out["jacobian"] = "DEAD:" + repr(e)[:100]
try:
    q = executor.articulation.get_joint_positions()    # noqa: F821
    out["dof_read"] = "ALIVE"
except Exception as e:
    out["dof_read"] = "DEAD:" + repr(e)[:100]

# ---- 修正相机静止位姿 + 生成跟随偏移 ----
# 背景: static_scene.usda 烘焙的头部相机位置是旧文档链的错误值(脑后 -0.115),
# isaac_teleop 旧跟随偏移用 XformCache/aim 链计算也不可靠(fabric 不同步 →
# 相机落后 8cm、朝向偏 90°, 用户实测投诉)。这里在机器人静止、物理视图健康时
# 一次性实测: 位置 = 挂载连杆 mesh bbox 中心(固定关节连杆 USD 姿态恒为静止位,
# 可信), 朝向 = col0=光轴/col1=图像上(cam_setup_s7fix3 渲染验证的约定),
# 写回相机 prim(编辑已有 prim 的 xformOp 安全), 并生成
# scripts/scene/cam_follow_offsets.json 供 _init_cam_follow 使用。
import numpy as np                      # noqa: E402
from pxr import Gf, Usd, UsdGeom        # noqa: E402


def _q2Rm(q):
    w, x, y, z = [float(v) for v in q]
    n = max((w * w + x * x + y * y + z * z) ** 0.5, 1e-9)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _R2qm(R):
    tr = float(np.trace(R))
    if tr > 0:
        s = np.sqrt(tr + 1) * 2
        return [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s]
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s,
                (R[0, 2] + R[2, 0]) / s]
    if i == 1:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s,
                (R[1, 2] + R[2, 1]) / s]
    s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
            (R[1, 2] + R[2, 1]) / s, 0.25 * s]


try:
    art = executor.articulation                      # noqa: F821
    lnames = list(art.body_names)
    lt = np.asarray(art._physics_view.get_link_transforms()).reshape(-1, 7)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    # (相机, 父连杆, 挂载连杆, 静止世界系光轴; None=沿用烘焙位姿[腕部 s11 已验证])
    FIX = [
        ("head_zed_x", "head_pitch_link", "head_zed_x_link", "table"),
        ("head_surround_left", "head_pitch_link",
         "head_surround_left_link", [0, 1, 0]),
        ("head_surround_right", "head_pitch_link",
         "head_surround_right_link", [0, -1, 0]),
        ("head_surround_rear", "head_pitch_link",
         "head_surround_rear_link", [-1, 0, 0]),
        ("chest_gemini_335lg", "torso_pitch_link",
         "chest_gemini_335lg_link", [1, 0, 0]),
        ("left_wrist_gemini_335lg", "left_wrist_link",
         "left_wrist_gemini_335lg_link", None),
        ("right_wrist_gemini_335lg", "right_wrist_link",
         "right_wrist_gemini_335lg_link", None),
    ]
    offs = {}
    for nm, parent, mount, aim in FIX:
        cprim = stage.GetPrimAtPath(f"/World/cams/{nm}")
        if not cprim.IsValid() or parent not in lnames:
            continue
        v = lt[lnames.index(parent)]
        Mp = np.eye(4)
        Mp[:3, :3] = _q2Rm(v[3:])
        Mp[:3, 3] = np.asarray(v[:3], float)
        # ---- 位置: 挂载连杆 mesh bbox 中心(固定关节连杆 USD 姿态恒为静止位) ----
        pw = None
        for cand in (f"/humanoid/{mount}/visuals", f"/humanoid/{mount}"):
            try:
                mp = stage.GetPrimAtPath(cand)
                if not mp.IsValid():
                    continue
                bb = bc.ComputeWorldBound(mp).ComputeAlignedRange()
                rng = bb.GetMax() - bb.GetMin()
                if max(rng[0], rng[1], rng[2]) < 1e-4:
                    continue
                c = (bb.GetMin() + bb.GetMax()) / 2.0
                pw = np.array([c[0], c[1], c[2]], float)
                break
            except Exception:
                continue
        # ---- 朝向 ----
        if aim is None:
            # 沿用烘焙的世界位姿(腕部 s11 验证过"朝前看不到自己"), 只算跟随偏移
            if pw is None:
                t = cprim.GetAttribute("xformOp:translate").Get()
                pw = np.array([t[0], t[1], t[2]], float)
            o = cprim.GetAttribute("xformOp:orient").Get()
            Rw = _q2Rm([o.GetReal(), *o.GetImaginary()])
        else:
            if aim == "table":
                # 第一视角工作相机指向桌面(用户实测: 水平朝向只看到天空+桌沿)
                tgt = None
                for cand in ("/World/vla_table", "/World/table"):
                    tpr = stage.GetPrimAtPath(cand)
                    if not tpr.IsValid():
                        continue
                    bb = bc.ComputeWorldBound(tpr).ComputeAlignedRange()
                    c = (bb.GetMin() + bb.GetMax()) / 2.0
                    tgt = np.array([c[0], c[1], float(bb.GetMax()[2])], float)
                    break
                if tgt is None or pw is None:
                    continue
                a = tgt - pw
            else:
                a = np.asarray(aim, float)
            a = a / max(float(np.linalg.norm(a)), 1e-9)
            z = np.array([0.0, 0.0, 1.0])
            u = z - a * float(z @ a)
            nu = float(np.linalg.norm(u))
            u = u / nu if nu > 1e-6 else np.array([0.0, 1.0, 0.0])
            # USD 相机约定(2026-09-28 两次渲染实测): 视线=-col2, 图像上=+col1。
            # 列序 [right, up, -view]; 早先按 col0=视线 构造会把画面转到 +Y
            # (斜地平线+单臂入画, 用户实测投诉)。
            Rw = np.column_stack([np.cross(a, u), u, -a])
            if pw is None:
                pw = Mp[:3, 3].copy()
        Mc = np.eye(4)
        Mc[:3, :3] = Rw
        Mc[:3, 3] = pw
        cprim.GetAttribute("xformOp:translate").Set(
            Gf.Vec3d(float(pw[0]), float(pw[1]), float(pw[2])))
        q = _R2qm(Rw)
        cprim.GetAttribute("xformOp:orient").Set(
            Gf.Quatd(float(q[0]), float(q[1]), float(q[2]), float(q[3])))
        offs[nm] = {"parent": parent,
                    "off": (np.linalg.inv(Mp) @ Mc).tolist(),
                    "rest_pos": [round(float(x), 4) for x in pw]}
    with open("/home/jy/RouteManager/scripts/scene/cam_follow_offsets.json",
              "w") as f:
        json.dump(offs, f, indent=1)
    out["cam_fix"] = {k: v["rest_pos"] for k, v in offs.items()}
except Exception as e:
    out["cam_fix"] = "ERR:" + repr(e)[:200]

with open("/tmp/register_cams.json", "w") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("OK")