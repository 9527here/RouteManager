"""对比测试：reserved 相机 initialize 前后的显存差异。

通过 /python POST 到 Isaac Sim 执行:
  curl -X POST http://localhost:8226 -H "Content-Type: text/plain" \
       --data-binary @scripts/scene/test_camera_gpu_impact.py

用法:
  1. 先跑一次，记录 "before" 显存
  2. 等 2-3 秒让 GPU 稳定
  3. 脚本会自动对 reserved 相机调 initialize()，然后记录 "after" 显存
  4. 最后自动 destroy() 恢复原状

输出写到 /tmp/camera_gpu_delta.json
"""
import json
import subprocess
import time

import omni.usd

def get_gpu_mem():
    """通过 nvidia-smi 读 GPU 0 的 used memory (MB)"""
    try:
        out = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
            timeout=5
        ).decode().strip()
        return int(out.splitlines()[0].strip())
    except Exception:
        return None

result = {}

# ── Phase 1: 读当前状态 ──
try:
    cams = executor._cameras  # noqa: F821
except Exception as e:
    result = {"error": f"no executor._cameras: {e}"}
    print(json.dumps(result))
    raise

before_mem = get_gpu_mem()
reserved = {}
rendering = {}
for path, cam in cams.items():
    nm = path.rsplit("/", 1)[-1]
    if cam._render_product is None:
        reserved[nm] = cam
    else:
        rendering[nm] = cam

result["phase1_before"] = {
    "gpu_used_mb": before_mem,
    "rendering_cams": list(rendering.keys()),
    "reserved_cams": list(reserved.keys()),
    "reserved_count": len(reserved),
}

# ── Phase 2: 对所有 reserved 相机调 initialize() ──
time.sleep(2)  # 等 GPU 稳定
for nm, cam in reserved.items():
    try:
        cam.initialize()
    except Exception as e:
        result[f"init_err_{nm}"] = repr(e)[:150]

time.sleep(3)  # 等 RTX 管线建立 + GPU 分配

after_mem = get_gpu_mem()
result["phase2_after_init"] = {
    "gpu_used_mb": after_mem,
    "delta_mb": (after_mem - before_mem) if (before_mem and after_mem) else "N/A",
}

# ── Phase 3: destroy 所有刚初始化的，恢复原状 ──
for nm, cam in reserved.items():
    try:
        cam.destroy()
    except Exception as e:
        result[f"destroy_err_{nm}"] = repr(e)[:150]

time.sleep(2)  # 等 GPU 释放
restored_mem = get_gpu_mem()
result["phase3_after_destroy"] = {
    "gpu_used_mb": restored_mem,
    "delta_from_before": (restored_mem - before_mem) if (before_mem and restored_mem) else "N/A",
}

# ── 注意：destroy 后 Camera 对象的 _render_product 为 None，
#    但对象本身仍在 executor._cameras 中（与初始 reserved 状态一致）───

result["conclusion"] = (
    f"reserved 相机 initialize 前 GPU: {before_mem}MB, "
    f"全部 initialize 后: {after_mem}MB (delta: {result['phase2_after_init']['delta_mb']}MB), "
    f"destroy 后: {restored_mem}MB"
)

with open("/tmp/camera_gpu_delta.json", "w") as f:
    json.dump(result, f, indent=2, ensure_ascii=False)
print(result["conclusion"])
print("Full report: /tmp/camera_gpu_delta.json")
