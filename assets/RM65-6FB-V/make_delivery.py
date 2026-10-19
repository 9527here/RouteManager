"""make_delivery.py — 打一个**可以整包发人**的 RM65 + CT 夹爪 URDF 包。

为什么要单独打：仓库里那三份 URDF 都发不了 ——
  · `*_isaac.urdf`       mesh 是写死的绝对路径（/home/jy/...），换机器全失效
  · `RM65-6FB-V_ctag2f120.urdf`  夹爪 mesh 引的是 `package://crt_ctag2f120_...`，
                         而那份 STL 根本不在本包里，收件人没有
  · `RM65-6FB-V.urdf`   只有臂、没夹爪

产出（`assets/deliver/RM65-6FB-V_ctag2f120/`，并压成同名 .zip）：

    RM65-6FB-V_ctag2f120/
    ├── package.xml / CMakeLists.txt      ROS 包（package:// 能解析）
    ├── README.md                         改了什么、主动关节是哪个
    ├── urdf/
    │   ├── RM65-6FB-V_ctag2f120.urdf           package:// 版（ROS/rviz/gazebo）
    │   └── RM65-6FB-V_ctag2f120_relative.urdf  相对路径版（Isaac/PyBullet/MuJoCo）
    ├── meshes/rm65/*.STL   9 个（臂）
    ├── meshes/ctag/*.STL   15 个（夹爪）
    ├── launch/display.launch
    └── config/joint_names_RM65-6FB-V.yaml

⚠️ 臂和夹爪都有 `base_link.STL`，**必须分目录**，不能平铺。
"""
import os
import shutil
import xml.etree.ElementTree as ET
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC_URDF = os.path.join(HERE, "urdf", "RM65-6FB-V_ctag2f120.urdf")
ARM_MESH = os.path.join(HERE, "meshes")
CT_PKG = ("/home/jy/datasets/vla_data/L1_制作资产/载体设备/changingtek/"
          "crt_ctag2f120_gripper_visualization")
CT_MESH = os.path.join(CT_PKG, "meshes")

OUT_ROOT = os.path.join(os.path.dirname(HERE), "deliver")
#: ROS 包名。**不能有 `-`**（rospack 只认 [a-zA-Z][a-zA-Z0-9_]*），
#: 厂家原包叫 `RM65-6FB-V` 其实是不合规的，这里改成合法名。
PKG_NAME = "rm65_6fb_v_ctag2f120"
#: 目录名保留可读的写法
DIR_NAME = "RM65-6FB-V_ctag2f120"
OUT = os.path.join(OUT_ROOT, DIR_NAME)


def _mesh_subdir(link_name):
    """夹爪的 link 统一带 ctag_ 前缀，其余是臂。"""
    return "ctag" if link_name.startswith("ctag_") else "rm65"


def _rewrite(root, absolute):
    """按 link 归类重写 mesh 路径；返回 (包内路径, 磁盘源文件) 列表。"""
    need = []
    for link in root.findall("link"):
        sub = _mesh_subdir(link.get("name"))
        for tag in ("visual", "collision"):
            for e in link.findall(tag):
                g = e.find("geometry")
                if g is None or g.find("mesh") is None:
                    continue
                base = g.find("mesh").get("filename").split("/")[-1]
                rel = f"meshes/{sub}/{base}"
                g.find("mesh").set(
                    "filename", os.path.join(OUT, rel) if absolute
                    else f"package://{PKG_NAME}/{rel}")
                need.append((sub, base))
    return need


def build():
    if os.path.exists(OUT):
        shutil.rmtree(OUT)
    for d in ("urdf", "meshes/rm65", "meshes/ctag", "launch", "config", "scripts"):
        os.makedirs(os.path.join(OUT, d), exist_ok=True)

    tree = ET.parse(SRC_URDF)
    root = tree.getroot()

    # --- 先收集需要的 mesh，再按子目录拷贝 ---
    need = _rewrite(root, absolute=False)
    copied = set()
    for sub, base in need:
        if (sub, base) in copied:
            continue
        src = os.path.join(CT_MESH if sub == "ctag" else ARM_MESH, base)
        if not os.path.exists(src):
            raise FileNotFoundError(f"缺 mesh: {src}")
        shutil.copy2(src, os.path.join(OUT, "meshes", sub, base))
        copied.add((sub, base))
    print(f"  mesh: rm65 {len([c for c in copied if c[0]=='rm65'])} 个, "
          f"ctag {len([c for c in copied if c[0]=='ctag'])} 个")

    # --- package:// 版 ---
    tree.write(os.path.join(OUT, "urdf", f"{DIR_NAME}.urdf"),
               encoding="utf-8", xml_declaration=True)

    # --- 相对路径版（Isaac / PyBullet 之类不认 package:// 的）---
    tree2 = ET.parse(SRC_URDF)
    root2 = tree2.getroot()
    for link in root2.findall("link"):
        sub = _mesh_subdir(link.get("name"))
        for tag in ("visual", "collision"):
            for e in link.findall(tag):
                g = e.find("geometry")
                if g is None or g.find("mesh") is None:
                    continue
                base = g.find("mesh").get("filename").split("/")[-1]
                g.find("mesh").set("filename", f"meshes/{sub}/{base}")
    tree2.write(os.path.join(OUT, "urdf", f"{DIR_NAME}_relative.urdf"),
                encoding="utf-8", xml_declaration=True)

    # --- ROS 包的配套文件 ---
    open(os.path.join(OUT, "package.xml"), "w").write(f"""<?xml version="1.0"?>
<package format="2">
  <name>{PKG_NAME}</name>
  <version>1.0.0</version>
  <description>睿尔曼 RM65-6FB-V 六轴臂 + 长广溪 CT ctag2f120 夹爪（合并，自包含）</description>
  <maintainer email="noreply@example.com">RouteManager</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>catkin</buildtool_depend>
</package>
""")
    open(os.path.join(OUT, "CMakeLists.txt"), "w").write(
        "cmake_minimum_required(VERSION 3.0.2)\n"
        f"project({PKG_NAME})\nfind_package(catkin REQUIRED)\ncatkin_package()\n"
        "install(DIRECTORY urdf meshes launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})\n")

    shutil.copy2(os.path.join(HERE, "config/joint_names_RM65-6FB-V.yaml"),
                 os.path.join(OUT, "config/joint_names_RM65-6FB-V.yaml"))
    # launch 自己写：厂家那份指向的是臂单件、还引了不存在的 rviz 配置
    open(os.path.join(OUT, "launch", "display.launch"), "w").write(
        f"""<launch>
  <param name="robot_description"
         textfile="$(find {PKG_NAME})/urdf/{DIR_NAME}.urdf" />
  <!-- joint_state_publisher_gui 会给 ctag_R2 出滑块，其余 11 个从动关节
       由 robot_state_publisher 按 <mimic> 自动跟随 -->
  <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui"
        type="joint_state_publisher_gui" />
  <node name="robot_state_publisher" pkg="robot_state_publisher"
        type="robot_state_publisher" />
  <node name="rviz" pkg="rviz" type="rviz" />
</launch>
""")

    open(os.path.join(OUT, "README.md"), "w").write(README)
    open(os.path.join(OUT, "scripts", "ctag_gripper.py"), "w").write(GRIPPER_PY)
    print(f"  写出 {OUT}")

    zpath = shutil.make_archive(os.path.join(OUT_ROOT, DIR_NAME), "zip", OUT_ROOT, DIR_NAME)
    print(f"  打包 {zpath} ({os.path.getsize(zpath)/1e6:.2f} MB)")
    return zpath


GRIPPER_PY = '''"""ctag_gripper.py — 由**一个**开合量算出 CT ctag2f120 夹爪全部 12 个关节角。

给**不支持 URDF `<mimic>`** 的加载器用（PyBullet 就完全不支持）。支持 mimic 的
工具（rviz 的 robot_state_publisher、MuJoCo、Isaac 开 parse_mimic）不用这个文件，
只驱动 `ctag_R2` 即可。

用法：
    from ctag_gripper import joint_values, opening_mm
    q = joint_values(0.7)          # {关节名: 弧度}，12 个
    print(opening_mm(0.7))         # 86.2

θ 的含义：0 = 全闭，0.7 = 全开。它同时也是 `ctag_R2` 和 `ctag_L2` 的角度。
"""

#: 全开角(rad)
OPEN_RAD = 0.7

#: θ(rad) → 指垫**中心距**(mm)，实测标定。开口 = 中心距 − 垫厚
TABLE = ((0.000, 2.53), (0.100, 15.00), (0.200, 27.96), (0.300, 40.83),
         (0.400, 53.41), (0.500, 65.64), (0.600, 77.31), (0.700, 88.23))

#: 每片指垫厚度(mm)
PAD_THICKNESS_MM = 2.0

#: 同向跟随（含两个主动根关节）
SAME = ("ctag_R2", "ctag_L2",
        "ctag_Right_in", "ctag_R3", "ctag_Right_Main", "ctag_Right_Slip",
        "ctag_Left_in", "ctag_Left_Slip")
#: 左指对称连杆，反向
NEG = ("ctag_L3", "ctag_Left_Main")
#: 小角度连杆
SMALL = ("ctag_R1", "ctag_L1")
#: 小角度连杆的倍率
SMALL_GAIN = -0.15


def joint_values(theta):
    """θ(rad) → {关节名: 角度(rad)}，12 个关节全给。"""
    t = min(max(float(theta), 0.0), OPEN_RAD)
    out = {n: t for n in SAME}
    out.update({n: -t for n in NEG})
    out.update({n: SMALL_GAIN * t for n in SMALL})
    return out


def center_sep_mm(theta):
    """θ(rad) → 两片指垫中心距(mm)，线性插值实测表。"""
    t = min(max(float(theta), 0.0), OPEN_RAD)
    for (q0, s0), (q1, s1) in zip(TABLE, TABLE[1:]):
        if t <= q1:
            return s0 + (s1 - s0) * (t - q0) / (q1 - q0)
    return TABLE[-1][1]


def opening_mm(theta):
    """θ(rad) → 两片指垫**内表面**间距(mm)，即实际开口。"""
    return center_sep_mm(theta) - PAD_THICKNESS_MM


if __name__ == "__main__":
    for t in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7):
        print(f"theta={t:.2f} rad  开口={opening_mm(t):6.2f} mm")
    print()
    for k, v in joint_values(0.35).items():
        print(f"  {k:22s} {v:+.5f}")
'''


README = """# RM65-6FB-V + CT ctag2f120 夹爪

睿尔曼 RM65-6FB-V 六轴视觉臂，末端挂长广溪 CT ctag2f120 平行夹爪。
**自包含**：24 个 link / 23 个 joint，臂和夹爪的 STL 全在 `meshes/` 里。

## 用哪份 URDF

| 文件 | 给谁用 |
|---|---|
| `urdf/RM65-6FB-V_ctag2f120.urdf` | **ROS**（rviz / gazebo / robot_state_publisher）。mesh 走 `package://RM65-6FB-V_ctag2f120/meshes/...`，把整个目录放进 catkin 工作空间的 `src/` 再 `catkin_make` 即可 |
| `urdf/RM65-6FB-V_ctag2f120_relative.urdf` | **非 ROS**（Isaac Sim / PyBullet / MuJoCo / trimesh）。mesh 是相对路径，以 URDF 所在目录为基准解析 |

两份除了 mesh 路径写法完全一样。

```bash
# ROS 下看一眼
roslaunch RM65-6FB-V_ctag2f120 display.launch
```

## 夹爪怎么控制

夹爪是四连杆，**12 个可动件**。结构上分两类：

```
ctag_base_link（夹爪本体）
├─ ctag_R2 ── ctag_Right_in ── [Right_Pad]  ← 右指，R2 是这根手指的根关节
└─ ctag_L2 ── ctag_Left_in  ── [Left_Pad]   ← 左指，L2 是这根手指的根关节
   （R3 / Right_Main / R1 / Right_Slip / L3 / Left_Main / L1 / Left_Slip 是两根手指内部的连杆）
```

**`ctag_R2` 和 `ctag_L2` 是两个主动根关节** —— 厂家 URDF 里只有这两个写
`effort="2"`，其余 10 个全是 `effort="0"`。**只驱动 `ctag_R2` 是打不开夹爪的**：
左指的根是 `ctag_L2`，不给它值左指不动。

URDF 里 `ctag_L2` 带 `<mimic joint="ctag_R2" multiplier="1"/>`，所以：

| 你的工具 | 怎么做 |
|---|---|
| **支持 `<mimic>`**（rviz 的 robot_state_publisher、MuJoCo、Isaac 需开 `parse_mimic`） | 只给 `ctag_R2` 一个值就行，其余 11 个自动跟随。**这时不要再去驱动 `ctag_L2`**，会和 mimic 打架 |
| **不支持 `<mimic>`**（PyBullet 完全不支持、不少自研加载器也不支持） | **12 个关节全部显式给值**，用下面的表 |

⚠️ 常见坑：在 PyBullet 之类不支持 mimic 的加载器里**只驱动 `ctag_R2`**，
只有右指会动、左指不动，看起来像夹爪坏了。这时必须把 `ctag_L2` 也一起驱动。

### 12 个关节的取值（θ = 开合量，0 = 全闭，0.7 = 全开）

| 关节 | 值 | |
|---|---|---|
| `ctag_R2` `ctag_L2` | **θ** | 两个主动根关节 |
| `ctag_Right_in` `ctag_R3` `ctag_Right_Main` `ctag_Right_Slip` `ctag_Left_in` `ctag_Left_Slip` | θ | 连杆，同向 |
| `ctag_L3` `ctag_Left_Main` | **−θ** | 左指对称连杆，反向 |
| `ctag_R1` `ctag_L1` | **−0.15 θ** | 小角度连杆 |

`scripts/ctag_gripper.py` 里就是这个函数，直接调 `joint_values(θ)` 拿 12 个值。

### θ ↔ 开口（实测，指垫中心距 − 2 mm 垫厚）

| θ (rad) | 0.00 | 0.10 | 0.20 | 0.30 | 0.40 | 0.50 | 0.60 | 0.70 |
|---|---|---|---|---|---|---|---|---|
| 开口 (mm) | 0.53 | 13.0 | 26.0 | 38.8 | 51.4 | 63.6 | 75.3 | 86.2 |

（这个 86.2 mm 是**这只夹爪**的行程。真机上的睿尔曼 rm 夹爪是 70 mm。）

臂是普通六轴：`joint_1` ~ `joint_6`，`joint_6` 是法兰自转（±360°）。
`camera_rojoint` / `camera_link` 是腕上相机的支架，不接相机时可以忽略。

## 相对厂家原件改了什么

原始件：睿尔曼 `RM65-6FB-V.urdf` + 长广溪 `crt_ctag2f120.urdf`。改动：

1. **补全夹爪从动关节的限位**。厂家导出的 11 个从动关节 `limit` 全是 `[0, 0]`
   （SolidWorks 一次性导出的占位值），直接导进任何物理引擎都会被锁死在 0 位，
   主动关节一动连杆就散架。已按机构实际行程填好（见上表倍率对应的区间）。
2. **夹爪挂装角**：`link_6_to_ctag_joint` 的 rpy = `0 0 1.5708`，即夹爪相对法兰
   绕工具轴自转 90°。法兰上 4 个螺丝孔本来就是 90° 一档，改这个值就是换一档。
   想让相机正对两指张开方向的话需要这个角度。
3. **剥掉夹爪连杆件的 collision**，只留两片指垫（`ctag_Right_Pad_Link` /
   `ctag_Left_Pad_Link`）。导入器会把每个 collision mesh 做成**凸包**，合拢时
   手指连杆要折进本体、两侧连杆互相穿插，凸包一撞上就把关节顶开、手指合不拢。
   抓取只需要指垫参与碰撞。
4. mesh 路径改写成包内路径（原文件是 SolidWorks 的绝对路径 / 跨包引用）。

惯性参数、几何、关节轴、零位构型**都是厂家原值，没动过**。

## 已知参数

- 工作半径 627 mm，额定负载 5 kg，本体 7.3 kg，重复定位 ±0.05 mm
- 零位：手臂竖直向上。J1 轴 z=240.5 mm，J2/J3 轴 z=496.5 mm，
  J4/J5 轴 z=706.5 mm，J6 法兰面 z=891 mm
- 臂的关节限位（rad）：J1 ±3.106、J2 ±2.2689、J3 ±2.356、J4 ±3.106、J5 ±2.234、J6 ±6.28
- 厂家 URDF **没有 `<dynamics>` 标签**（无 damping/friction）。纯运动学用没问题；
  跑物理仿真要自己补驱动增益，否则关节会一直晃
"""


if __name__ == "__main__":
    build()
