"""把 3x2 cubemap 六格图转成等距柱状(equirect)全景图, 供 DomeLight 环境贴图用。

- 输入: 六格拼图(3列x2行, 每格正方形), 顺序默认 [px, nx, py / ny, pz, nz]
- 输出: scripts/scene/background_faces/env_equirect.png  (4096x2048)
- 坐标约定: +X 前, +Y 左, +Z 上; equirect 经度自 +X 逆时针, 纬度自水平面
  px 面: 视线 +X, 图上=+Z, 图右=-Y   (与 head 相机一致)
用法: python3 scripts/scene/make_env_panorama.py [源图] [--order px,nx,py,ny,pz,nz]
"""
import os
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "background_faces")
ORDER = ["px", "nx", "py", "ny", "pz", "nz"]
W_OUT, H_OUT = 4096, 2048


def face_dirs(tag, u, v):
    """u,v ∈ [-1,1] (u 右为正, v 上为正) → 该面上的单位方向(世界系)。"""
    if tag == "px":   return np.stack([np.ones_like(u), -u, v], -1)
    if tag == "nx":   return np.stack([-np.ones_like(u), u, v], -1)
    if tag == "py":   return np.stack([u, v, np.ones_like(u)], -1)
    if tag == "ny":   return np.stack([u, -v, -np.ones_like(u)], -1)
    if tag == "pz":   return np.stack([u, np.ones_like(u), -v], -1)
    return np.stack([-u, -np.ones_like(u), -v], -1)   # nz


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else (
        "/tmp/claude-1000/-home-jy-RouteManager/8138b6c7-db40-4548-8971-"
        "66d93425fd0f/images/12.png")
    order = ORDER
    if "--order" in sys.argv:
        order = sys.argv[sys.argv.index("--order") + 1].split(",")
    os.makedirs(OUT_DIR, exist_ok=True)

    im = Image.open(src).convert("RGB")
    W, H = im.size
    fw, fh = W / 3.0, H / 2.0
    faces = {}
    for i, tag in enumerate(order):
        c, r = i % 3, i // 3
        faces[tag] = np.asarray(im.crop((int(round(c * fw)), int(round(r * fh)),
                                         int(round((c + 1) * fw)),
                                         int(round((r + 1) * fh)))).convert("RGB"))
        print(f"  {tag}: {faces[tag].shape}")

    # 输出网格: 经度 λ 自 +X 逆时针(-π..π), 纬度 φ (-π/2..π/2)
    lam = (np.arange(W_OUT) / W_OUT * 2 - 1) * np.pi
    phi = (0.5 - np.arange(H_OUT) / H_OUT) * np.pi
    L, P = np.meshgrid(lam, phi)
    d = np.stack([np.cos(P) * np.cos(L), np.cos(P) * np.sin(L), np.sin(P)], -1)

    ax = np.abs(d)
    face_idx = np.argmax(ax, axis=-1)
    tags = ["px", "nx", "py", "ny", "pz", "nz"]
    out = np.zeros((H_OUT, W_OUT, 3), dtype=np.uint8)
    for fi, tag in enumerate(tags):
        m = face_idx == fi
        if not m.any():
            continue
        dd = d[m]
        # 由方向反解 u,v(与 face_dirs 互逆)
        if tag in ("px", "nx"):
            u = (-dd[:, 1] / ax[m][:, 0]) if tag == "px" else (dd[:, 1] / ax[m][:, 0])
            v = dd[:, 2] / ax[m][:, 0]
        elif tag in ("py", "ny"):
            u = dd[:, 0] / ax[m][:, 2]
            v = (dd[:, 1] / ax[m][:, 2]) if tag == "py" else (-dd[:, 1] / ax[m][:, 2])
        else:
            u = dd[:, 0] / ax[m][:, 1]
            v = (-dd[:, 2] / ax[m][:, 1]) if tag == "pz" else (-dd[:, 2] / ax[m][:, 1])
        fi_img = faces[tag]
        h, w, _ = fi_img.shape
        col = np.clip(((u + 1) * 0.5 * (w - 1)).round().astype(int), 0, w - 1)
        row = np.clip(((1 - (v + 1) * 0.5) * (h - 1)).round().astype(int), 0, h - 1)
        out[m] = fi_img[row, col]

    p = os.path.join(OUT_DIR, "env_equirect.png")
    Image.fromarray(out).save(p)
    print(f"OK: {p} ({W_OUT}x{H_OUT})")


if __name__ == "__main__":
    main()
