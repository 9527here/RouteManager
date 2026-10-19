#!/usr/bin/env python3
"""把 humanoid_upper_body_v3 整理成一个可直接发出去的 ROS URDF 包。

- URDF: 绝对路径 -> package://<pkg>/...  （另出一份 relative 版）
- meshes/visual  : 二次误差精简到 <= CAP 面（外观件，不影响物理）
- meshes/collision: 原样拷贝（碰撞体不动，物理保真）
- 附 package.xml / CMakeLists.txt / launch / config，不含 README
"""
import os, re, shutil, sys, json
import numpy as np

SRC = "/home/jy/datasets/vla_data/L1_制作资产/载体设备/humanbody_urdf/humanoid_upper_body_v3"
SRC_URDF = os.path.join(SRC, "urdf/humanoid_upper_body_v3_ctag2f120_rm65_isaac.urdf")
OUT = "/home/jy/RouteManager/assets/deliver/humanoid_upper_body_v3"
PKG = "humanoid_upper_body_v3"
# visual 网格面数预算：按原面数 1/25 给，但夹在 [MIN_TRI, MAX_TRI] 之间。
# 小件（<=MIN_TRI）原样不动；机身这类大件拿到 MAX_TRI，细节保留得多。
MIN_TRI, MAX_TRI = 25000, 80000


def budget(n):
    return min(n, max(MIN_TRI, min(MAX_TRI, n // 25)))

MESH_PREFIX = SRC + "/meshes/"


def log(*a):
    print(*a, flush=True)


# ---------------------------------------------------------------- 1. URDF
raw = open(SRC_URDF, encoding="utf-8").read()

refs = sorted(set(re.findall(r'filename="([^"]+)"', raw)))
bad = [r for r in refs if not r.startswith(MESH_PREFIX)]
assert not bad, f"有 mesh 不在 meshes/ 下: {bad}"
log(f"URDF mesh 引用: {len(refs)} 个，全部在 meshes/ 下")

# 空 name 的 material 在 ROS 里不合法，补个名字（颜色原样不动）
n_empty = raw.count('<material name="">')
raw = raw.replace('<material name="">', '<material name="white">')
log(f"修正空 material 名: {n_empty} 处 -> \"white\"")

# 去掉开头那几行生成器注释（指向内部脚本，发出去没意义）
raw = re.sub(r"^<!--.*?-->\s*", "", raw, count=1, flags=re.S)

urdf_pkg = raw.replace(f'filename="{MESH_PREFIX}', f'filename="package://{PKG}/meshes/')
urdf_rel = raw.replace(f'filename="{MESH_PREFIX}', 'filename="meshes/')
assert "package://" in urdf_pkg and "/home/jy" not in urdf_pkg and "/home/jy" not in urdf_rel

# ---------------------------------------------------------------- 2. 目录
if os.path.exists(OUT):
    shutil.rmtree(OUT)
for d in ("urdf", "meshes/visual", "meshes/collision", "launch", "config"):
    os.makedirs(os.path.join(OUT, d), exist_ok=True)

open(os.path.join(OUT, f"urdf/{PKG}.urdf"), "w", encoding="utf-8").write(urdf_pkg)
open(os.path.join(OUT, f"urdf/{PKG}_relative.urdf"), "w", encoding="utf-8").write(urdf_rel)
log(f"写入 urdf/{PKG}.urdf 和 urdf/{PKG}_relative.urdf")

# ---------------------------------------------------------------- 3. 网格
import open3d as o3d
o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)

vis_refs = sorted(r for r in refs if "/meshes/visual/" in r)
col_refs = sorted(r for r in refs if "/meshes/collision/" in r)

report = []
for r in col_refs:                                   # 碰撞体原样拷
    dst = os.path.join(OUT, os.path.relpath(r, SRC))
    shutil.copy2(r, dst)

for r in vis_refs:
    dst = os.path.join(OUT, os.path.relpath(r, SRC))
    n_src = os.path.getsize(r)
    m = o3d.io.read_triangle_mesh(r)
    nt = len(m.triangles)
    tgt = budget(nt)
    if tgt >= nt:
        shutil.copy2(r, dst)
        report.append((os.path.basename(r), nt, nt, n_src, os.path.getsize(dst), False))
        continue
    m2 = m.simplify_quadric_decimation(target_number_of_triangles=tgt)
    m2.remove_duplicated_vertices()
    m2.remove_degenerate_triangles()
    m2.remove_unreferenced_vertices()
    m2.compute_triangle_normals()      # open3d 的 STL writer 要求有面法线
    m2.compute_vertex_normals()
    # 几何自检：包围盒必须对得上
    b0 = np.asarray(m.get_axis_aligned_bounding_box().get_extent())
    b1 = np.asarray(m2.get_axis_aligned_bounding_box().get_extent())
    err = float(np.max(np.abs(b0 - b1)) * 1000)
    assert err < 2.0, f"{r} 精简后包围盒差 {err:.2f}mm"
    assert o3d.io.write_triangle_mesh(dst, m2, write_ascii=False)
    report.append((os.path.basename(r), nt, len(m2.triangles), n_src,
                   os.path.getsize(dst), True))

tot_before = sum(x[3] for x in report)
tot_after = sum(x[4] for x in report)
log(f"\nvisual 网格 {len(vis_refs)} 个，{tot_before/1e6:.1f}MB -> {tot_after/1e6:.1f}MB")
log(f"collision 网格 {len(col_refs)} 个，原样拷贝")
log("\n精简明细 (>CAP 的)：")
log(f"  {'name':32s} {'tris':>9s} -> {'tris':>7s}   {'MB':>7s} -> {'MB':>6s}")
for n, a, b, sa, sb, dec in sorted(report, key=lambda x: -x[3]):
    if dec:
        log(f"  {n:32s} {a:9d} -> {b:7d}   {sa/1e6:7.1f} -> {sb/1e6:6.1f}")

# ---------------------------------------------------------------- 4. 包元数据
open(os.path.join(OUT, "package.xml"), "w", encoding="utf-8").write(f'''<?xml version="1.0"?>
<package format="2">
  <name>{PKG}</name>
  <version>1.0.0</version>
  <description>
    轮式双臂人形上装 v3。升降柱 + 俯仰躯干 + 2 自由度头部 + 双臂睿尔曼
    RM65-6FB-V 六轴 + 长广溪 CT ctag2f120 二指夹爪（12 关节四连杆，软件 mimic）
    + 四脚轮底盘，共 61 个 link / 60 个 joint。
    urdf/{PKG}.urdf          网格用 package:// 引用
    urdf/{PKG}_relative.urdf 网格用包内相对路径（工具不认 package:// 时用）
  </description>
  <maintainer email="noreply@example.com">RouteManager</maintainer>
  <license>Proprietary</license>

  <buildtool_depend>catkin</buildtool_depend>
  <depend>roslaunch</depend>
  <depend>robot_state_publisher</depend>
  <depend>rviz</depend>
  <depend>joint_state_publisher_gui</depend>
</package>
''')

open(os.path.join(OUT, "CMakeLists.txt"), "w", encoding="utf-8").write('''cmake_minimum_required(VERSION 3.0.2)

project(humanoid_upper_body_v3)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir config launch meshes urdf)
\tinstall(DIRECTORY ${dir}/
\t\tDESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
''')

open(os.path.join(OUT, "launch/display.launch"), "w", encoding="utf-8").write(f'''<launch>
  <arg name="gui" default="true"/>

  <param name="robot_description"
         textfile="$(find {PKG})/urdf/{PKG}.urdf"/>

  <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui"
        type="joint_state_publisher_gui" if="$(arg gui)"/>
  <node name="joint_state_publisher" pkg="joint_state_publisher"
        type="joint_state_publisher" unless="$(arg gui)"/>
  <node name="robot_state_publisher" pkg="robot_state_publisher"
        type="robot_state_publisher"/>
  <node name="rviz" pkg="rviz" type="rviz" required="true"/>
</launch>
''')

# 可控关节清单（mimic 从动关节不列）
import xml.etree.ElementTree as ET
root = ET.fromstring(urdf_pkg)
act = [j.get("name") for j in root.findall("joint")
       if j.get("type") != "fixed" and j.find("mimic") is None]
lines = [f"# {PKG} — 可控关节（{len(act)} 个）。夹爪的从动关节由 <mimic> 跟随，不在此列。",
         "controller_joint_names:"]
lines += [f"  - {n}" for n in act]
open(os.path.join(OUT, "config/joint_names.yaml"), "w", encoding="utf-8").write("\n".join(lines) + "\n")

log(f"\n可控关节 {len(act)} 个 -> config/joint_names.yaml")

# ---------------------------------------------------------------- 5. 自检
n_links = len(root.findall("link")); n_joints = len(root.findall("joint"))
log(f"\nURDF 自检: robot={root.get('name')}  links={n_links}  joints={n_joints}")

missing = [r for r in refs
           if not os.path.exists(os.path.join(OUT, os.path.relpath(r, SRC)))]
assert not missing, f"缺文件: {missing}"

for base in (f"{PKG}.urdf", f"{PKG}_relative.urdf"):
    p = os.path.join(OUT, "urdf", base)
    ET.parse(p)                                     # XML 合法性
log("两份 URDF XML 解析通过，所有 mesh 文件到位")

du = sum(os.path.getsize(os.path.join(dp, f))
         for dp, _, fs in os.walk(OUT) for f in fs)
log(f"\n包总计: {du/1e6:.1f} MB")
for d in ("urdf", "meshes/visual", "meshes/collision", "launch", "config"):
    s = sum(os.path.getsize(os.path.join(dp, f))
            for dp, _, fs in os.walk(os.path.join(OUT, d)) for f in fs)
    log(f"  {d:20s} {s/1e6:8.2f} MB")
