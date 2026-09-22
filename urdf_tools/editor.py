"""
urdf_tools.editor — URDF 文件修改工具（纯 XML，导入 Isaac 之前用）
==================================================================

用途：在人形机器人 URDF 导入 Isaac Sim 之前，修改关节限位/动力学参数、
查看关节结构。导入后的修改请在仿真里直接操作 USD prim。

注意：Isaac Sim 不直接编辑 URDF，标准流程是
    修改 URDF（本模块）→ import_urdf 导入为 USD → 在仿真中控制
"""

import xml.etree.ElementTree as ET


class UrdfEditor:
    def __init__(self, urdf_path):
        self.urdf_path = urdf_path
        self.tree = ET.parse(urdf_path)
        self.root = self.tree.getroot()

    # ==================== 查看 ====================

    def list_joints(self):
        """列出所有关节：名字、类型、父/子 link、限位。"""
        joints = []
        for j in self.root.findall("joint"):
            limit = j.find("limit")
            joints.append({
                "name": j.get("name"),
                "type": j.get("type"),
                "parent": j.find("parent").get("link") if j.find("parent") is not None else None,
                "child": j.find("child").get("link") if j.find("child") is not None else None,
                "lower": float(limit.get("lower", 0)) if limit is not None else None,
                "upper": float(limit.get("upper", 0)) if limit is not None else None,
            })
        return joints

    def list_links(self):
        return [l.get("name") for l in self.root.findall("link")]

    # ==================== 修改 ====================

    def set_joint_limit(self, joint_name, lower=None, upper=None,
                        effort=None, velocity=None):
        """修改关节限位。关节没有 <limit> 标签时会自动创建。"""
        joint = self._find_joint(joint_name)
        if joint.get("type") == "fixed":
            raise ValueError(f"关节 '{joint_name}' 是 fixed 类型，没有限位")
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")
        for attr, value in [("lower", lower), ("upper", upper),
                            ("effort", effort), ("velocity", velocity)]:
            if value is not None:
                limit.set(attr, str(float(value)))

    def set_joint_dynamics(self, joint_name, damping=None, friction=None):
        """修改关节动力学参数（阻尼/摩擦），不存在时自动创建。"""
        joint = self._find_joint(joint_name)
        dyn = joint.find("dynamics")
        if dyn is None:
            dyn = ET.SubElement(joint, "dynamics")
        for attr, value in [("damping", damping), ("friction", friction)]:
            if value is not None:
                dyn.set(attr, str(float(value)))

    def set_joint_origin(self, joint_name, xyz=None, rpy=None):
        """修改关节相对父 link 的安装位姿。"""
        joint = self._find_joint(joint_name)
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin")
        if xyz is not None:
            origin.set("xyz", " ".join(str(float(v)) for v in xyz))
        if rpy is not None:
            origin.set("rpy", " ".join(str(float(v)) for v in rpy))

    # ==================== 保存 ====================

    def save(self, output_path=None):
        """保存。默认覆盖原文件；传 output_path 另存。"""
        path = output_path or self.urdf_path
        ET.indent(self.tree, space="  ")
        self.tree.write(path, encoding="utf-8", xml_declaration=True)
        return path

    # ==================== 内部 ====================

    def _find_joint(self, joint_name):
        joint = self.root.find(f"joint[@name='{joint_name}']")
        if joint is None:
            raise ValueError(f"URDF 中不存在关节 '{joint_name}'，"
                             f"现有关节: {[j.get('name') for j in self.root.findall('joint')]}")
        return joint
