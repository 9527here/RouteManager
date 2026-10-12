"""s11: 两腕相机重标定 —— 光轴=夹爪接近轴(现场实测 base->pad 中点),
位置推到 pad 前方 7cm(越过手指, 画面纯前方看不到自己), 世界Z投影up 防滚转。
渲染 /tmp/s11_*.jpg。"""
import json
import time

import numpy as np
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom

stage = omni.usd.get_context().get_stage()
xc = UsdGeom.XformCache(Usd.TimeCode.Default())
out = {"aim": {}, "render": {}}


def gf_to_np(gf_m):
    return np.array([[gf_m[i][j] for j in range(4)] for i in range(4)]).T


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


def link_M(nm):
    return gf_to_np(xc.GetLocalToWorldTransform(
        stage.GetPrimAtPath(f"/humanoid/{nm}")))


AHEAD = 0.07  # 相机越过 pad 的距离(m)

for side in ("left", "right"):
    nm = f"{side}_wrist_gemini_335lg"
    try:
        M_wrist = link_M(f"{side}_wrist_link")
        invR = np.linalg.inv(M_wrist)[:3, :3]
        b = link_M(f"{side}_gripper_base_link")[:3, 3]
        p1 = link_M(f"{side}_gripper_left_pad_link")[:3, 3]
        p2 = link_M(f"{side}_gripper_right_pad_link")[:3, 3]
        mid = 0.5 * (p1 + p2)
        d_w = mid - b
        d_w = d_w / np.linalg.norm(d_w)
        # 光轴 = 接近轴(腕系)
        d_l = invR @ d_w
        # up = 世界 Z 投影(腕系表达)
        z_w = np.array([0.0, 0.0, 1.0])
        up_w = z_w - d_w * (z_w @ d_w)
        up_w /= np.linalg.norm(up_w)
        up_l = invR @ up_w
        r_l = np.cross(d_l, up_l)
        R_local = np.column_stack([d_l, up_l, r_l])
        # 位置: pad 中点前方 7cm(世界系算好转腕系)
        pos_w = mid + AHEAD * d_w
        pos_l = (np.linalg.inv(M_wrist) @ np.append(pos_w, 1.0))[:3]
        M_cam_new_w = M_wrist @ T(R_local, pos_l)
        ol_path = f"/humanoid/{nm}_link/{nm}_optical_link"
        p_ol = stage.GetPrimAtPath(ol_path)
        p_cam = stage.GetPrimAtPath(
            f"{ol_path}/{side}_wrist_gemini_cam_v2")
        if not (p_ol.IsValid() and p_cam.IsValid()):
            out["aim"][side] = "PRIM_MISSING"
            continue
        M_ol_w = gf_to_np(xc.GetLocalToWorldTransform(p_ol))
        M_loc_new = np.linalg.inv(M_ol_w) @ M_cam_new_w
        q = mat2quat(M_loc_new[:3, :3])
        p_cam.CreateAttribute("xformOp:translate",
                              Sdf.ValueTypeNames.Double3).Set(
            tuple(float(v) for v in M_loc_new[:3, 3]))
        p_cam.CreateAttribute("xformOp:orient",
                              Sdf.ValueTypeNames.Quatd).Set(q)
        p_cam.CreateAttribute("xformOpOrder",
                              Sdf.ValueTypeNames.StringArray).Set(
            ["xformOp:translate", "xformOp:orient"])
        o = executor._cameras.get(
            f"{ol_path}/{side}_wrist_gemini_cam_v2")
        if o is not None:
            o.set_local_pose(
                translation=[float(v) for v in M_loc_new[:3, 3]],
                orientation=[q.real, q.imaginary[0], q.imaginary[1],
                             q.imaginary[2]], camera_axes="usd")
            o.set_default_state()
        out["aim"][side] = {
            "cam_world": [round(float(v), 3) for v in pos_w],
            "axis_world": [round(float(v), 3) for v in d_w]}
    except Exception as e:
        out["aim"][side] = "ERR:" + repr(e)[:200]

time.sleep(0.5)
for side in ("left", "right"):
    nm = f"{side}_wrist_gemini_335lg"
    try:
        o = None
        for k, v in executor._cameras.items():
            if nm in k:
                o = v
                break
        if o is None:
            out["render"][side] = "NO_OBJ"
            continue
        executor.world.step(render=True)
        img = o.get_rgba()[:, :, :3]
        from PIL import Image
        Image.fromarray(img).save(f"/tmp/s11_{side}_wrist.jpg", quality=90)
        out["render"][side] = "saved"
    except Exception as e:
        out["render"][side] = "ERR:" + repr(e)[:150]

with open("/tmp/cam_setup_s11.json", "w") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("OK")
