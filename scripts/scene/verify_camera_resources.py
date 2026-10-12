"""验证相机 render product 数量与 GPU 显存占用。

通过 /python POST 到 Isaac Sim 执行:
  curl -X POST http://localhost:8226 -H "Content-Type: text/plain" \
       --data-binary @scripts/scene/verify_camera_resources.py

输出写到 /tmp/camera_resource_report.json
"""
import json
import subprocess

import omni.usd
from pxr import Usd, UsdGeom, UsdRender

stage = omni.usd.get_context().get_stage()

report = {
    "camera_prims_on_stage": [],
    "render_products_on_stage": [],
    "registered_cameras": {},
    "gpu_memory_mb": {},
}

# 1. 列出 stage 上所有 Camera prim（GUI 看到的就是这些）
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Camera):
        report["camera_prims_on_stage"].append(str(prim.GetPath()))

# 2. 列出 stage 上所有 RenderProduct prim（真正占 GPU 显存的）
for prim in stage.Traverse():
    if prim.IsA(UsdRender.Product):
        rp_path = str(prim.GetPath())
        res_attr = prim.GetAttribute("resolution")
        res = res_attr.Get() if res_attr else None
        report["render_products_on_stage"].append({
            "path": rp_path,
            "resolution": tuple(res) if res else None,
        })

# 3. 检查 executor._cameras 中每个 Camera 对象的 _render_product 状态
try:
    cams = executor._cameras  # noqa: F821
    for path, cam_obj in cams.items():
        info = {
            "prim_path": cam_obj.prim_path,
            "has_render_product": cam_obj._render_product is not None,
            "render_product_path": cam_obj._render_product_path,
            "is_paused": cam_obj.is_paused() if cam_obj._render_product is not None else "N/A(no render product)",
            "resolution": cam_obj.get_resolution(),
            "custom_annotators": list(cam_obj._custom_annotators.keys()),
        }
        report["registered_cameras"][path] = info
except Exception as e:
    report["registered_cameras"] = {"error": repr(e)[:200]}

# 4. GPU 显存（通过 nvidia-smi，不依赖 torch.cuda）
try:
    out = subprocess.check_output(
        ["nvidia-smi", "--query-gpu=index,memory.used,memory.total,memory.free",
         "--format=csv,noheader,nounits"],
        timeout=5
    ).decode().strip()
    for line in out.splitlines():
        parts = [p.strip() for p in line.split(",")]
        idx = parts[0]
        report["gpu_memory_mb"][f"gpu{idx}"] = {
            "used": int(parts[1]),
            "total": int(parts[2]),
            "free": int(parts[3]),
        }
except Exception as e:
    report["gpu_memory_mb"] = {"error": repr(e)[:200]}

# 5. 总结
report["summary"] = {
    "total_camera_prims": len(report["camera_prims_on_stage"]),
    "total_render_products": len(report["render_products_on_stage"]),
    "cameras_with_render_product": sum(
        1 for v in report["registered_cameras"].values()
        if isinstance(v, dict) and v.get("has_render_product")
    ),
    "cameras_reserved_no_render": sum(
        1 for v in report["registered_cameras"].values()
        if isinstance(v, dict) and not v.get("has_render_product")
    ),
}

with open("/tmp/camera_resource_report.json", "w") as f:
    json.dump(report, f, indent=2, ensure_ascii=False, default=str)

print(json.dumps(report["summary"], indent=2))
print("Full report: /tmp/camera_resource_report.json")
