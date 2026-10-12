"""arm_kin.py — 手臂正运动学 + 数值雅可比（纯 numpy/XML，不依赖 Isaac）。

**为什么不用 Isaac 的实时雅可比**：PhysX 给的是**绕连杆质心**的雅可比，
而我们要控制的是 TCP(末端工具点)，两者差一个 ω×(p_tcp − p_com) 项；实测
角速度行完全一致、线速度行差 ~5cm×ω。用 FK 数值微分则对任意参考点都精确。

**为什么不用预计算雅可比表**：表只在 home 附近采样，实测当前位形与最近样本
差 q4 1.02rad / q6 1.26rad，线性外推完全失效 —— 这是"一按离合机械臂就甩"的
根因。FK 数值微分在任何位形都精确，且没有额外状态。

FK 精度已与仿真实测交叉验证：TCP 位置偏差 0.05mm，雅可比角速度行完全一致。
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

#: 本项目机器人 URDF（关节链的权威来源）。默认=7 轴旧臂模型，与
#: `v3_ctag2f120_cameras` 预设（usd_path，无 URDF 可查）配套。
URDF = ("/home/jy/datasets/vla_data/L1_制作资产/载体设备/humanbody_urdf/"
        "humanoid_upper_body_v3/urdf/humanoid_upper_body_v3_ctag2f120.urdf")


def set_urdf(path):
    """切到另一个机器人模型时同步 FK 链来源。

    **必须跟着实际加载的机器人走**：链里的可动关节数要和仿真 dof 数一致。
    官方 RM65 重建版是 6 轴（无 `*_wrist_joint`），若 FK 仍按 7 轴旧 URDF 建链，
    `plan()` 的 q/limits 形状对不上（实测报 "IK 全部起点都失败"），
    `isaac_teleop._ensure_kin` 的关节顺序安全闸也会直接判定不一致而停用 IK
    ——表现为"手柄推了机械臂不动"。由 `RobotManager.load_urdf` 在导入后调用。
    """
    global URDF
    URDF = path


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def _aa(axis, q):
    a = np.asarray(axis, float)
    n = np.linalg.norm(a)
    if n < 1e-12:
        return np.eye(3)
    a = a / n
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)


def _T(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def load_chain(urdf_path, root_link, tip_link):
    """从 URDF 取 root_link → tip_link 的关节链（按父子顺序）。"""
    root = ET.parse(urdf_path).getroot()
    by_child = {}
    for j in root.findall("joint"):
        o, a = j.find("origin"), j.find("axis")
        lim = j.find("limit")
        by_child[j.find("child").get("link")] = {
            "name": j.get("name"), "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "xyz": [float(v) for v in (o.get("xyz") or "0 0 0").split()],
            "rpy": [float(v) for v in (o.get("rpy") or "0 0 0").split()],
            "axis": [float(v) for v in
                     ((a.get("xyz") if a is not None else None)
                      or "0 0 1").split()],
            "limit": ([float(lim.get("lower", -3.14)),
                       float(lim.get("upper", 3.14))]
                      if lim is not None else [-3.14, 3.14]),
        }
    chain, link = [], tip_link
    while link != root_link:
        if link not in by_child:
            raise RuntimeError(
                f"URDF 里 {root_link} 到 {tip_link} 不连通（断在 {link}）")
        chain.append(by_child[link])
        link = by_child[link]["parent"]
    chain.reverse()
    return chain


def joint_names(chain):
    return [j["name"] for j in chain if j["type"] != "fixed"]


def joint_limits(chain):
    return np.asarray([j["limit"] for j in chain if j["type"] != "fixed"],
                      dtype=float)


def fk(chain, q):
    """正运动学：返回 (R, t)，tip 在 root 系里的位姿（列向量约定）。"""
    M = np.eye(4)
    qi = 0
    for j in chain:
        M = M @ _T(_rpy(*j["rpy"]), np.asarray(j["xyz"], float))
        if j["type"] != "fixed":
            M = M @ _T(_aa(j["axis"], q[qi]), np.zeros(3))
            qi += 1
    return M[:3, :3], M[:3, 3]


def ee_world(chain, q, R_ab, t_ab):
    """末端在世界系的 (位置, 旋转)。R_ab/t_ab = 手臂基座的世界位姿。"""
    R, t = fk(chain, q)
    Rw = R_ab @ R
    return Rw @ np.zeros(3) + (R_ab @ t + t_ab), Rw


def jacobian(chain, q, R_ab, t_ab, eps=2e-4):
    """数值雅可比 (n, 6)，列 = [vx vy vz wx wy wz]，对**末端原点**精确。"""
    q = np.asarray(q, float)
    n = len(joint_names(chain))
    J = np.zeros((n, 6))
    for i in range(n):
        qp, qm = q.copy(), q.copy()
        qp[i] += eps
        qm[i] -= eps
        pp, Rp = ee_world(chain, qp, R_ab, t_ab)
        pm, Rm = ee_world(chain, qm, R_ab, t_ab)
        J[i, :3] = (pp - pm) / (2 * eps)
        dR = Rp @ Rm.T
        J[i, 3:] = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0],
                             dR[1, 0] - dR[0, 1]]) / (2 * 2 * eps)
    return J
