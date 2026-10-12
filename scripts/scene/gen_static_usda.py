"""由 static_vals.json 生成 static_scene.usda(本地运行, 不经服务器)。

静态场景要在**机器人之前**加载: 机器人加载后任何 prim 编辑都会让物理后端
失效, 届时所有关节读写(set_joint_*/get_jacobians/底盘)全部不可用。
"""
import json
import math
import os

VALS = "/tmp/static_vals.json"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "static_scene.usda")
TABLE_USD = ("/home/jy/VLA/Collected_piper_table_clean_v2/"
             "piper_table_clean_v2.usd")

v = json.load(open(VALS))
L = []
L.append("#usda 1.0")
L.append("(")
L.append('    defaultPrim = "World"')
L.append("    metersPerUnit = 1")
L.append('    upAxis = "Z"')
L.append(")")
L.append("")
L.append('def Xform "World"')
L.append("{")
L.append('    def Xform "cams"')
L.append("    {")

for nm, c in v["cams"].items():
    if not isinstance(c, dict):
        continue
    p, q = c["pos"], c["quat"]
    L.append(f'        def Camera "{nm}"')
    L.append("        {")
    L.append(f"            double3 xformOp:translate = ({p[0]}, {p[1]}, {p[2]})")
    L.append(f"            quatd xformOp:orient = ({q[0]}, {q[1]}, {q[2]}, {q[3]})")
    L.append('            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]')
    L.append(f"            float focalLength = {c['focal']}")
    L.append(f"            float horizontalAperture = {c['hap']}")
    L.append(f"            float verticalAperture = {c['vap']}")
    L.append(f"            float2 clippingRange = ({c['clip'][0]}, {c['clip'][1]})")
    L.append("        }")
L.append("    }")

# 桌子: **长边横置**(垂直于机器人视线) + 近边留出间隙——按用户的碰撞要求,
# 直接由几何算出位姿(不再用旧实测值, 旧值长边沿视线伸出 0.13m 顶翻了机器人)。
BASE_YAW = 0.0          # 机器人初始朝向(静态场景里机器人 yaw≈0)
DIST = 1.00             # 桌心到机器人中心(m)
TABLE_LONG = 1.834 / 2  # 长边半宽(局部 X)
TABLE_SHORT = 1.227 / 2 # 短边半宽(局部 Y)
Rz = lambda a: (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))  # yaw → quat(wxyz)
# 长边横置 = 桌子局部 X 指向世界 Y → 桌子 yaw = 机器人 yaw + 90°
table_yaw = BASE_YAW + math.pi / 2
tq = Rz(table_yaw)
tpos = (DIST * math.cos(BASE_YAW), DIST * math.sin(BASE_YAW), 0.0)
gap = DIST - TABLE_SHORT           # 近边到机器人中心的距离(应 0.35-0.5m)
L.append('    def Xform "vla_table" (')
L.append(f"        prepend references = @{TABLE_USD}@</World/table>")
L.append("    )")
L.append("    {")
L.append(f"        double3 xformOp:translate = ({tpos[0]}, {tpos[1]}, 0.0)")
L.append(f"        quatd xformOp:orient = ({tq[0]}, {tq[1]}, {tq[2]}, {tq[3]})")
L.append('        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]')
L.append("    }")
print(f"桌子: 横置 yaw={math.degrees(table_yaw):.1f}°, 近边距机器人中心 {gap:.3f}m")

# 可抓物体: 桌子**局部**坐标 → 世界(桌面高 0.7366, 靠机器人一侧是 -局部Y)
OBJS = [
    ("cube_a", "Cube", (-0.35, -0.45), 0.035, (0.90, 0.35, 0.15)),
    ("cyl_b", "Cylinder", (0.38, -0.35), 0.030, (0.20, 0.60, 0.90)),
    ("cube_c", "Cube", (0.20, -0.20), 0.030, (0.95, 0.80, 0.20)),
]
ct, st = math.cos(table_yaw), math.sin(table_yaw)
for nm, kind, (lx, ly), half, rgb in OBJS:
    wx = tpos[0] + ct * lx - st * ly
    wy = tpos[1] + st * lx + ct * ly
    wz = 0.7366 + (half if kind == "Cube" else 0.06)
    if kind == "Cube":
        scale = (half * 2, half * 2, half * 2)
        L.append(f'    def Cube "{nm}" (')
        L.append('        apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"]')
        L.append("    )")
        L.append("    {")
        L.append("        double size = 1")
        L.append(f"        double3 xformOp:translate = ({wx:.4f}, {wy:.4f}, {wz:.4f})")
        L.append(f"        float3 xformOp:scale = ({scale[0]}, {scale[1]}, {scale[2]})")
    else:
        L.append(f'    def Cylinder "{nm}" (')
        L.append('        apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"]')
        L.append("    )")
        L.append("    {")
        L.append("        double radius = 1")
        L.append("        double height = 1")
        L.append('        uniform token axis = "Z"')
        L.append(f"        double3 xformOp:translate = ({wx:.4f}, {wy:.4f}, {wz:.4f})")
        L.append(f"        float3 xformOp:scale = ({half}, {half}, 0.12)")
    L.append('        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]')
    L.append(f"        color3f[] primvars:displayColor = [({rgb[0]}, {rgb[1]}, {rgb[2]})]")
    L.append("        float physics:mass = 0.05")
    L.append("    }")

# 参考球
for nm, key in (("ref_red", "/World/ref_red"), ("ref_green", "/World/ref_green"),
                ("ref_blue", "/World/ref_blue")):
    o = v["others"].get(key)
    if not isinstance(o, dict):
        continue
    p = o["pos"]
    L.append(f'    def Sphere "{nm}"')
    L.append("    {")
    L.append("        double radius = 0.2")
    L.append(f"        double3 xformOp:translate = ({p[0]}, {p[1]}, {p[2]})")
    L.append('        uniform token[] xformOpOrder = ["xformOp:translate"]')
    L.append("    }")

L.append("}")
L.append("")

open(OUT, "w").write("\n".join(L))
print("wrote", OUT, len(L), "lines")