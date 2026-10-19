"""make_rm65_ctag.py — 官方 RM65-6FB-V + 末端 CT 夹爪。

把 changingtek `crt_ctag2f120` 的 14 个关节/15 个 link 原样并到官方 RM65 的
`link_6` 法兰上，产出一份自洽的 URDF。

- 挂装：`link_6 → ctag_base_link` 的 fixed 关节，origin 全零。
  官方 link_6 的 mesh 沿 -z 延伸 69.5mm（法兰面 = link_6 原点），
  而 link_5→link_6 的方向在 link_6 系里正是 +z ⇒ 夹爪朝 +z 就是朝外。
- 命名：CT 的 link/joint 一律加 `ctag_` 前缀（官方已有 `base_link`/`camera_link`），
  mimic 的引用名同步改写。
- mesh：官方走 `package://RM65-6FB-V/...`（包目录就在 assets/RM65-6FB-V），
  CT 走 `package://crt_ctag2f120_gripper_visualization/...`，两者都改写成绝对路径。

产出 `RM65-6FB-V_ctag2f120.urdf`（相对路径版，包内可用）与
`RM65-6FB-V_ctag2f120_isaac.urdf`（绝对路径版，Isaac 导入用）。
"""
import os
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
RM65_URDF = os.path.join(HERE, "urdf", "RM65-6FB-V.urdf")
RM65_MESH = os.path.join(HERE, "meshes")
CT_PKG = ("/home/jy/datasets/vla_data/L1_制作资产/载体设备/changingtek/"
          "crt_ctag2f120_gripper_visualization")
CT_URDF = os.path.join(CT_PKG, "urdf", "crt_ctag2f120.urdf")
CT_MESH = os.path.join(CT_PKG, "meshes")

DST = os.path.join(HERE, "urdf", "RM65-6FB-V_ctag2f120.urdf")
DST_ISAAC = os.path.join(HERE, "urdf", "RM65-6FB-V_ctag2f120_isaac.urdf")
PREFIX = "ctag_"

#: 夹爪相对 link_6 的挂装姿态 rpy（弧度）。**绕 Z 就是绕工具轴自转**：
#:   "0 0 0"      → 两指沿 link_6 的 X 张开（厂家默认）
#:   "0 0 1.5708" → 自转 90°，两指改沿 Y 张开
#: 腕上相机（camera_rojoint→camera_link）也挂在 link_6 上，跟夹爪一起转，
#: 所以想「让相机看到两指」必须在这里转，转 joint_6 是没用的（两个一起转，
#: 相对视角不变）。2026-10-12 用户要求：转 90°。
MOUNT_RPY = "0 0 1.5708"

#: **必须覆盖的关节限位**。CT 官方 URDF 的 11 个从动关节 limit 全是 `[0, 0]`
#: （SolidWorks 一次性导出的占位值）—— 直接导入 Isaac 时它们被锁死在 0 位，
#: 主动关节一动就把连杆撕开、整只夹爪散架（用户 2026-10-12 实测）。
#: 下面是 2026-09-15 合并进人形时填好的真实值，从
#: `humanoid_upper_body_v3_ctag2f120_rm65_isaac.urdf` 逐条抄过来。
LIMITS = {
    "R2": (0.0, 0.7), "Right_in": (0.0, 0.7), "R3": (0.0, 0.7),
    "Right_Main": (0.0, 0.7), "R1": (-0.105, 0.0), "Right_Slip": (0.0, 0.7),
    "L2": (0.0, 0.7), "Left_in": (0.0, 0.7), "L3": (-0.7, 0.0),
    "Left_Main": (-0.7, 0.0), "L1": (-0.105, 0.0), "Left_Slip": (0.0, 0.7),
}


def _fix_limits(root):
    n = 0
    for j in root.findall("joint"):
        spec = LIMITS.get(j.get("name"))
        if spec is None or j.get("type") == "fixed":
            continue
        lim = j.find("limit")
        if lim is None:
            lim = ET.SubElement(j, "limit")
        lim.set("lower", str(spec[0]))
        lim.set("upper", str(spec[1]))
        if lim.get("effort") is None:
            lim.set("effort", "60")
        if lim.get("velocity") is None:
            lim.set("velocity", "3.14")
        n += 1
    return n


#: 只有这两片指垫保留碰撞体，其余 CT 件（本体 + 全部连杆）一律剥掉。
#: **为什么**：Isaac 把每个 collision mesh 做成**凸包**。合拢时手指的连杆要折进
#: 本体里、两侧连杆互相穿插，凸包一撞上就把关节顶开 —— 实测命令合拢(0)时
#: `ctag_Right_in/Left_in` 被顶到 +0.076 rad、`*_Main` 顶到 ±0.035 rad，
#: 手指合不拢、外观上就是「穿模」（2026-10-12 实测，刚度从 5730 提到 100000
#: 误差才降到 0.003）。抓取只需要指垫参与碰撞，连杆不参与，剥掉零损失。
KEEP_COLLISION = ("Right_Pad_Link", "Left_Pad_Link")


def _strip_linkage_collision(ct_root):
    """剥掉 CT 夹爪上除指垫外所有 link 的 <collision>，返回剥掉的个数。"""
    n = 0
    for link in ct_root.findall("link"):
        if link.get("name") in KEEP_COLLISION:
            continue
        for c in link.findall("collision"):
            link.remove(c)
            n += 1
    return n


def _rewrite_mesh(root, pkg_name, mesh_dir, absolute):
    n = 0
    for link in root.findall("link"):
        for tag in ("visual", "collision"):
            for e in link.findall(tag):
                g = e.find("geometry")
                if g is None:
                    continue
                m = g.find("mesh")
                if m is None:
                    continue
                fn = m.get("filename")
                base = fn.split("/")[-1]
                m.set("filename", os.path.join(mesh_dir, base) if absolute
                      else f"package://{pkg_name}/meshes/{base}")
                n += 1
    return n


def build(absolute: bool):
    tree = ET.parse(RM65_URDF)
    root = tree.getroot()
    _rewrite_mesh(root, "RM65-6FB-V", RM65_MESH, absolute)

    ct = ET.parse(CT_URDF).getroot()
    _rewrite_mesh(ct, "crt_ctag2f120_gripper_visualization", CT_MESH, absolute)
    n_lim = _fix_limits(ct)          # 覆盖掉源文件里 [0,0] 的废限位
    print(f"  已填真实限位: {n_lim} 个关节")
    # 必须在改名之前剥碰撞（下面按原始 link 名匹配）
    n_col = _strip_linkage_collision(ct)
    print(f"  已剥掉非指垫碰撞体: {n_col} 个")

    rename = {}          # link 名映射
    jrename = {}         # joint 名映射（mimic 引用的是关节名，不是 link 名）
    for link in ct.findall("link"):
        old = link.get("name")
        link.set("name", PREFIX + old)
        rename[old] = PREFIX + old
    for j in ct.findall("joint"):
        old = j.get("name")
        jrename[old] = PREFIX + old
        j.set("name", PREFIX + old)
        j.find("parent").set("link", rename[j.find("parent").get("link")])
        j.find("child").set("link", rename[j.find("child").get("link")])
        mim = j.find("mimic")
        if mim is not None:
            mim.set("joint", jrename.get(mim.get("joint"), PREFIX + mim.get("joint")))
        root.append(j)
    for link in ct.findall("link"):
        root.append(link)

    # 挂装: link_6 -> ctag_base_link。位置零偏，姿态按 MOUNT_RPY。
    # 绕 **Z** 转就是绕工具轴自转（link_6 的 +z 即夹爪朝外方向）——法兰上
    # 4 个螺丝孔本来就是 90° 一档，真机也这么调。
    mount = ET.SubElement(root, "joint")
    mount.set("name", "link_6_to_ctag_joint")
    mount.set("type", "fixed")
    ET.SubElement(mount, "parent").set("link", "link_6")
    ET.SubElement(mount, "child").set("link", PREFIX + "base_link")
    ET.SubElement(mount, "origin").set("xyz", "0 0 0")
    mount.find("origin").set("rpy", MOUNT_RPY)

    out = DST_ISAAC if absolute else DST
    tree.write(out, encoding="utf-8", xml_declaration=True)
    print(f"wrote {out}")
    return out


if __name__ == "__main__":
    for ab in (False, True):
        p = build(ab)
        r = ET.parse(p).getroot()
        print(f"   links={len(r.findall('link'))}  joints={len(r.findall('joint'))}")
