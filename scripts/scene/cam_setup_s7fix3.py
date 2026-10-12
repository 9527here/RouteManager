"""s7fix3: 环视 3 路用世界Z投影up构造旋转(上午验证过的数学, 防滚转), 腕部不动。"""
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


R_opt = None  # 不再需要: ol->cam 的既有链不变, 直接在 ol 系写绝对目标
XYZ_DOC = {
    "head_surround_left": [-0.035643, 0.023177, 0.128458],
    "head_surround_right": [-0.033642, -0.036098, 0.128450],
    "head_surround_rear": [-0.057780, -0.006461, 0.128546],
}
# 目标光轴(基座系水平方向)
DIR_B = {"head_surround_left": [0, 1, 0],
         "head_surround_right": [0, -1, 0],
         "head_surround_rear": [-1, 0, 0]}

for nm, leaf in (("head_surround_left", "head_surround_left_cam_v2"),
                 ("head_surround_right", "head_surround_right_cam_v2"),
                 ("head_surround_rear", "head_surround_rear_cam_v2")):
    try:
        # 上午验证的数学: 光轴 d_local, up = 世界Z投影, 右 = cross(d, up) —— 全在 ref 系内
        # ref = head_pitch_link; 其世界系需读 prim(与 s6 相同途径)
        M_ref = gf_to_np(xc.GetLocalToWorldTransform(
            stage.GetPrimAtPath("/humanoid/head_pitch_link")))
        M_base = gf_to_np(xc.GetLocalToWorldTransform(
            stage.GetPrimAtPath("/humanoid/base_link")))
        inv_ref = np.linalg.inv(M_ref)[:3, :3]
        d_w = M_base[:3, :3] @ np.array(DIR_B[nm], float)
        d_w = d_w / np.linalg.norm(d_w)
        d_l = inv_ref @ d_w
        # up: 世界 Z 投影(在 ref 系内表达)
        z_w = np.array([0.0, 0.0, 1.0])
        up_w = z_w - d_w * (z_w @ d_w)
        up_w /= np.linalg.norm(up_w)
        up_l = inv_ref @ up_w
        r_l = np.cross(d_l, up_l)
        R_local = np.column_stack([d_l, up_l, r_l])  # 列: 前/上/右(相机惯例)
        M_cam_new_w = M_ref @ T(R_local, XYZ_DOC[nm])
        ol_path = f"/humanoid/{nm}_link/{nm}_optical_link"
        p_ol = stage.GetPrimAtPath(ol_path)
        p_cam = stage.GetPrimAtPath(f"{ol_path}/{leaf}")
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
        o = executor._cameras.get(f"{ol_path}/{leaf}")
        if o is not None:
            o.set_local_pose(
                translation=[float(v) for v in M_loc_new[:3, 3]],
                orientation=[q.real, q.imaginary[0], q.imaginary[1],
                             q.imaginary[2]], camera_axes="usd")
            o.set_default_state()
        out["aim"][nm] = "OK"
    except Exception as e:
        out["aim"][nm] = "ERR:" + repr(e)[:200]

time.sleep(0.5)
for nm in XYZ_DOC:
    try:
        o = None
        for k, v in executor._cameras.items():
            if nm in k:
                o = v
                break
        if o is None:
            out["render"][nm] = "NO_OBJ"
            continue
        executor.world.step(render=True)
        img = o.get_rgba()[:, :, :3]
        from PIL import Image
        Image.fromarray(img).save(f"/tmp/s10_{nm}.jpg", quality=90)
        out["render"][nm] = "saved"
    except Exception as e:
        out["render"][nm] = "ERR:" + repr(e)[:150]

with open("/tmp/cam_setup_s7fix3.json", "w") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("OK")
