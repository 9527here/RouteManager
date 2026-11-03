"""阀门操作的纯离线规划：独立 FK、升降摆位、指垫 TCP、绕真实轴的连续 IK。

本模块没有 HTTP、Isaac 或运动下发入口。``plan`` 的闭爪段只是无载模型边界；
接触后的执行必须重新量 TCP，并调用 ``plan_arc`` 重规划，不能把模型当接触反馈。
"""
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
import arm_kin
import gripper_model
from scripts.valve_ik import rotation_angle, rotation_error, solve_ik
from scripts.valve_feedback import projected_wheel_width_mm, wheel_support_interval


class PlanningError(ValueError):
    """输入、FK、限位或连续 IK 不满足规划条件。"""


def _vector(value, size, label):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise PlanningError(f"{label} 必须是 {size} 维有限向量")
    return result


def _rotation(value, label):
    result = np.asarray(value, dtype=float)
    if (result.shape != (3, 3) or not np.isfinite(result).all() or
            not np.allclose(result.T @ result, np.eye(3), atol=1e-5, rtol=0.) or
            abs(np.linalg.det(result) - 1.) > 1e-5):
        raise PlanningError(label + " 必须是正交且右手的旋转矩阵")
    return result


def pose_matrix(pose):
    """量测位姿采用 position(米)、orientation(wxyz)，不接受歧义四元数标签。"""
    p = _vector(pose["position"], 3, "位置")
    q = _vector(pose["orientation"], 4, "wxyz 四元数")
    if np.linalg.norm(q) < 1e-9:
        raise PlanningError("四元数不能为零")
    return p, Rotation.from_quat(q / np.linalg.norm(q), scalar_first=True).as_matrix()


def axis_rotation(axis, angle_deg):
    axis = _vector(axis, 3, "转轴方向")
    norm = np.linalg.norm(axis)
    if norm < 1e-9 or not math.isfinite(angle_deg):
        raise PlanningError("转轴方向不能为零，角度必须有限")
    return Rotation.from_rotvec(axis / norm * math.radians(angle_deg)).as_matrix()


def grasp_rotation(axis, roll_deg=0.0):
    """工具 +Z 沿阀轴反向接近；自由滚转绕阀轴，不硬套世界欧拉角。"""
    axis = _vector(axis, 3, "转轴方向")
    if np.linalg.norm(axis) < 1e-9:
        raise PlanningError("转轴方向不能为零")
    z = -axis / np.linalg.norm(axis)
    reference = np.array([1., 0., 0.]) if abs(z[0]) < .9 else np.array([0., 1., 0.])
    x = reference - np.dot(reference, z) * z
    x /= np.linalg.norm(x)
    return axis_rotation(axis, roll_deg) @ np.column_stack([x, np.cross(z, x), z])


class ValvePlanner:
    """单臂规划器；所有方法只读模型并计算，返回的计划都不能代替避碰验收。"""
    def __init__(self, urdf_path, side="right", position_tol=.005, rotation_tol_deg=3.):
        if side not in ("left", "right"):
            raise PlanningError("side 必须是 left/right")
        self.urdf_path = str(Path(urdf_path).resolve())
        self.side = side
        self.root = ET.parse(self.urdf_path).getroot()
        self.joints = {j.get("name"): j for j in self.root.findall("joint")}
        self.chain = arm_kin.load_chain(self.urdf_path, f"{side}_arm_base_link",
                                       f"{side}_gripper_base_link")
        self.names = arm_kin.joint_names(self.chain)
        self.limits = arm_kin.joint_limits(self.chain)
        margin = np.minimum(.02, .1 * (self.limits[:, 1] - self.limits[:, 0]))
        self.lo, self.hi = self.limits[:, 0] + margin, self.limits[:, 1] - margin
        self.position_tol, self.rotation_tol_deg = float(position_tol), float(rotation_tol_deg)
        if (not math.isfinite(self.position_tol) or self.position_tol <= 0 or
                not math.isfinite(self.rotation_tol_deg) or self.rotation_tol_deg <= 0):
            raise PlanningError("残差阈值必须有限且为正")
        self.leader_name = f"{side}_gripper_right_2_joint"
        self.pad_chains = [arm_kin.load_chain(
            self.urdf_path, f"{side}_gripper_base_link", f"{side}_gripper_{finger}_pad_link")
            for finger in ("left", "right")]

    def _pad_kinematics(self, *, position=None, leader=None, joint_positions=None):
        if sum(x is not None for x in (position, leader, joint_positions)) != 1:
            raise PlanningError("position、leader、joint_positions 必须且只能提供一种")
        if position is not None:
            if not math.isfinite(position) or not 1 <= position <= 1000:
                raise PlanningError("夹爪 position 必须在 1..1000")
            known = {self.leader_name: gripper_model.leader_for_fraction((position - 1.) / 999.)}
        elif leader is not None:
            if not math.isfinite(leader):
                raise PlanningError("夹爪 leader 必须有限")
            known = {self.leader_name: float(leader)}
        else:
            known = {name: float(value) for name, value in joint_positions.items()}
        if not all(math.isfinite(v) for v in known.values()):
            raise PlanningError("夹爪关节角含非有限值")

        def resolve(name, seen=()):
            if name in known:
                return known[name]
            if name in seen:
                raise PlanningError("夹爪 mimic 有环: " + name)
            mimic = self.joints[name].find("mimic")
            if mimic is None:
                raise PlanningError("缺少夹爪关节反馈/主动关节: " + name)
            return (float(mimic.get("multiplier", "1")) *
                    resolve(mimic.get("joint"), seen + (name,)) +
                    float(mimic.get("offset", "0")))

        return [arm_kin.fk(chain, [resolve(n) for n in arm_kin.joint_names(chain)]) for chain in self.pad_chains]

    def _pad_positions(self, **kwargs):
        return np.asarray([p for _, p in self._pad_kinematics(**kwargs)])

    def tcp_offset(self, *, position=None, leader=None, joint_positions=None):
        """URDF+mimic 无载 TCP；实测全指关节优先于 mimic，单 leader 则显式按模型估计。"""
        return self._pad_positions(position=position, leader=leader, joint_positions=joint_positions).mean(axis=0)

    def grasp_support(self, snapshot, R_grasp=None, jaw_axis_world=None, *, allow_legacy_fallback=False):
        if jaw_axis_world is None:
            if R_grasp is None:
                jaw_axis_world = snapshot['state']['sides'][self.side]['jaw_axis_world']
            else:
                pads = self._pad_positions(position=1000)
                axis = pads[1] - pads[0]
                jaw_axis_world = np.asarray(R_grasp) @ (axis / np.linalg.norm(axis))
        try:
            return wheel_support_interval(snapshot['geometry'], jaw_axis_world,
                                           allow_legacy_fallback=allow_legacy_fallback)
        except (ValueError, KeyError) as error:
            raise PlanningError(str(error)) from error

    def grasp_width_mm(self, snapshot, R_grasp=None, jaw_axis_world=None, **kwargs):
        return self.grasp_support(snapshot, R_grasp, jaw_axis_world, **kwargs)['width_mm']

    def _model_inner_face_offset(self, snapshot, *, allow_legacy_fallback=False):
        sources = snapshot['state']['sides'][self.side].get('pad_support_sources')
        if not sources:
            if allow_legacy_fallback:
                return 0., 'explicit_offline_legacy_geometric_TCP_reference_only'
            raise PlanningError('缺少实际pad碰撞支持点，实际规划不能猜inner-face中点')
        shapes = {row['finger']: row for row in sources}
        frames = self._pad_kinematics(position=1000)
        middle = .5 * (frames[0][1] + frames[1][1])
        axis = frames[1][1] - frames[0][1]
        axis /= np.linalg.norm(axis)
        values = []
        for finger, (R, p) in zip(('left', 'right'), frames):
            data = shapes[finger]
            if data.get('units') != 'm' or data.get('coordinate_frame') != 'pad rigid body local':
                raise PlanningError('pad支撑顶点单位/参考系无效')
            points = np.asarray(data['support_vertices_local_m'], float)
            projected = points @ (R.T @ axis) + float(axis @ p)
            values.append(projected.max() if finger == 'left' else projected.min())
        return float(.5 * sum(values) - axis @ middle), 'URDF pad frames with measured same collision supports'

    def independent_anchor(self, snapshot):
        """从同批实测臂座预测法兰；不从法兰反推臂座后做恒等零残差自检。"""
        state = snapshot["state"]
        if state.get("ref_link") not in (None, "", "world"):
            raise PlanningError("锚点必须在世界系")
        names, values = state["joint_names"], state["positions"]
        if len(names) != len(values) or len(names) != len(set(names)):
            raise PlanningError("关节反馈长度不符或名字重复")
        mapping = dict(zip(names, values))
        if any(n not in mapping for n in self.names):
            raise PlanningError("机器人与 URDF 的手臂关节不匹配")
        q = _vector([mapping[n] for n in self.names], len(self.names), "手臂关节")
        p_ab, R_ab = pose_matrix(snapshot["arm_base"])
        side = state["sides"][self.side]
        p_f, R_f = pose_matrix(side["flange"])
        p_tcp = _vector(side["tcp"]["position"], 3, "实测 TCP")
        predicted, predicted_R = arm_kin.ee_world(self.chain, q, R_ab, p_ab)
        pe, ae = float(np.linalg.norm(predicted - p_f)), rotation_angle(predicted_R, R_f)
        if pe >= .008 or ae >= 3.:
            raise PlanningError(f"独立 FK 锚点失败: {pe*1000:.3f}mm / {ae:.3f}°")
        return {"q": q, "R_ab": R_ab, "t_ab": p_ab, "tcp_off": R_f.T @ (p_tcp - p_f),
                "tcp": p_tcp, "R_flange": R_f, "joint_positions": mapping,
                "metrics": {"position_mm": pe * 1000., "rotation_deg": ae}}

    def _body_rotation(self, snapshot, anchor):
        base_pose = snapshot.get("base_pose", snapshot.get("base"))
        if base_pose is not None:
            return pose_matrix(base_pose)[1], "measured_base_pose"
        # 旧快照未保存基座；完整 URDF 身体链的旋转与升降无关，可从实测臂座求方向。
        body = arm_kin.load_chain(self.urdf_path, "base_link", f"{self.side}_arm_base_link")
        R = np.eye(3)
        for joint in body:
            R = R @ arm_kin._rpy(*joint["rpy"])
            if joint["type"] in ("revolute", "continuous"):
                if joint["name"] not in anchor["joint_positions"]:
                    raise PlanningError("缺少身体链关节: " + joint["name"])
                R = R @ arm_kin._aa(joint["axis"], anchor["joint_positions"][joint["name"]])
            elif joint["type"] not in ("fixed", "prismatic"):
                raise PlanningError("不支持的身体链关节类型")
        return anchor["R_ab"] @ R.T, "derived_from_measured_arm_base_and_urdf_body_rotation"

    def placement(self, snapshot, body_xy=(0., 0.), lift_delta=0.):
        anchor = self.independent_anchor(snapshot)
        xy = _vector(body_xy, 2, "假设世界水平位移")
        if not math.isfinite(lift_delta):
            raise PlanningError("升降变化必须有限")
        lift = self.joints.get("lift_joint")
        if lift is None or lift.get("type") != "prismatic":
            raise PlanningError("URDF 缺少 lift_joint 移动关节")
        values = anchor["joint_positions"]
        if "lift_joint" not in values:
            raise PlanningError("缺少升降实测位置")
        limit = lift.find("limit")
        lower, upper = (float(limit.get(k)) for k in ("lower", "upper"))
        lift_target = float(values["lift_joint"]) + lift_delta
        if not math.isfinite(lift_target) or not lower <= lift_target <= upper:
            raise PlanningError(f"升降目标 {lift_target} 超出 [{lower},{upper}]")
        R_base, source = self._body_rotation(snapshot, anchor)
        parent = lift.find("parent").get("link")
        R_parent = np.eye(3)
        if parent != "base_link":
            parent_chain = arm_kin.load_chain(self.urdf_path, "base_link", parent)
            for joint in parent_chain:
                R_parent = R_parent @ arm_kin._rpy(*joint["rpy"])
                if joint["type"] in ("revolute", "continuous"):
                    R_parent = R_parent @ arm_kin._aa(joint["axis"], values[joint["name"]])
        origin = lift.find("origin")
        rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
        local_axis = np.array([float(v) for v in lift.find("axis").get("xyz").split()])
        if np.linalg.norm(local_axis) < 1e-9:
            raise PlanningError("升降轴向不能为零")
        axis_world = R_base @ R_parent @ arm_kin._rpy(*rpy) @ (local_axis / np.linalg.norm(local_axis))
        shift = np.r_[xy, 0.] + lift_delta * axis_world
        return {**anchor, "t_ab": anchor["t_ab"] + shift,
                "placement": {"body_xy_world_m": xy.tolist(), "lift_delta_m": float(lift_delta),
                              "lift_target_m": lift_target, "lift_limits_m": [lower, upper],
                              "lift_axis_world": axis_world.tolist(), "base_rotation_source": source,
                              "arm_base_delta_world_m": shift.tolist(),
                              "hypothetical": bool(np.linalg.norm(shift) > 1e-12)}}

    def _row(self, name, q, tcp, R, off, R_ab, t_ab, **extra):
        pf, Rf = arm_kin.ee_world(self.chain, q, R_ab, t_ab)
        flange_target = tcp - R @ off
        pe = float(np.linalg.norm(pf - flange_target))
        tcp_pe = float(np.linalg.norm(pf + Rf @ off - tcp))
        ae = rotation_angle(Rf, R)
        if pe >= self.position_tol or tcp_pe >= self.position_tol or ae >= self.rotation_tol_deg:
            raise PlanningError(f"{name} IK 失败: flange={pe*1000:.3f}mm, TCP={tcp_pe*1000:.3f}mm, rot={ae:.3f}°")
        margin = np.minimum(q - self.limits[:, 0], self.limits[:, 1] - q)
        if np.any(margin <= 0):
            raise PlanningError(name + " 超出关节限位")
        return {"name": name, "q": np.asarray(q).tolist(), "tcp": np.asarray(tcp).tolist(),
                "R": np.asarray(R).tolist(), "tcp_off": np.asarray(off).tolist(),
                "flange_error_mm": pe * 1000., "tcp_error_mm": tcp_pe * 1000.,
                "rotation_error_deg": ae, "min_joint_margin_deg": float(np.degrees(margin.min())), **extra}

    def _solve(self, q, tcp, R, off, R_ab, t_ab, multistart=False, position_precision_m=None):
        target = tcp - R @ off
        precision = self.position_tol if position_precision_m is None else min(self.position_tol, float(position_precision_m))
        if not math.isfinite(precision) or precision <= 0:
            raise PlanningError('位置求解精度必须为正')
        if not multistart:
            # 把TCP预算分配给平移和旋转杠杆误差；否则DLS过早停止后每次都落入昂贵LS。
            result = solve_ik(self.chain, q, target, R, R_ab, t_ab, self.limits,
                              position_tol=min(5e-4,precision*.4),
                              rotation_tol=min(3e-3,precision*.4/max(float(np.linalg.norm(off)),1e-9)))
            pf, Rf = arm_kin.ee_world(self.chain, result[0], R_ab, t_ab)
            if (np.linalg.norm(pf + Rf @ off - tcp) < precision and
                    result[1] < precision and result[2] < self.rotation_tol_deg):
                return result[0]
        starts = [np.clip(q, self.lo, self.hi)]
        if multistart:
            starts += list(np.random.default_rng(8).uniform(self.lo, self.hi, (20, len(q))))
        best = None
        for seed in starts:
            def residual(x):
                p, r = arm_kin.ee_world(self.chain, x, R_ab, t_ab)
                return np.r_[p - target, .5 * rotation_error(r, R)]
            fit = least_squares(residual, np.clip(seed, self.lo + 1e-9, self.hi - 1e-9),
                                bounds=(self.lo, self.hi), max_nfev=180,
                                ftol=1e-10, xtol=1e-10, gtol=1e-10)
            score = float(np.linalg.norm(residual(fit.x)))
            if best is None or score < best[0]:
                best = (score, fit.x)
            p, r = arm_kin.ee_world(self.chain, fit.x, R_ab, t_ab)
            if (np.linalg.norm(p + r @ off - tcp) < min(5e-4, precision) and
                    np.linalg.norm(p - target) < min(5e-4, precision) and rotation_angle(r, R) < .17):
                # 合格解立即返回，不能被此前仅按加权score更小的未合格解覆盖。
                return fit.x
        if position_precision_m is not None:
            p, r = arm_kin.ee_world(self.chain, best[1], R_ab, t_ab)
            tcp_error = float(np.linalg.norm(p + r @ off - tcp))
            flange_error = float(np.linalg.norm(p - target))
            angle_error = rotation_angle(r, R)
            if (tcp_error >= precision or flange_error >= precision or
                    angle_error >= self.rotation_tol_deg):
                raise PlanningError(f'指定位置精度未满足: TCP={tcp_error*1000:.6f}mm, '
                                    f'flange={flange_error*1000:.6f}mm, rotation={angle_error:.6f}deg, '
                                    f'要求位置<{precision*1000:.6f}mm')
        return best[1]

    def plan_line(self, q_current, start_tcp, end_tcp, R, tcp_off, R_ab, t_ab,
                  *, step_m=.01, name="line", include_start=True, max_joint_step_deg=8.):
        if not math.isfinite(step_m) or not 0 < step_m <= .01:
            raise PlanningError("直线步长必须在 (0,0.01] 米")
        start, end = _vector(start_tcp, 3, "直线起点"), _vector(end_tcp, 3, "直线终点")
        off = _vector(tcp_off, 3, "TCP 偏移")
        R, R_ab, t_ab = _rotation(R, "目标朝向"), _rotation(R_ab, "臂座朝向"), _vector(t_ab, 3, "臂座位置")
        count = max(1, math.ceil(np.linalg.norm(end - start) / step_m))
        q = _vector(q_current, len(self.names), "起始关节角")
        rows = []
        previous_tcp = start.copy()
        def append_point(tcp, depth=0):
            nonlocal q, previous_tcp
            qn = self._solve(q,tcp,R,off,R_ab,t_ab)
            if float(abs(qn-q).max()) > math.radians(max_joint_step_deg):
                if depth>=8 or np.linalg.norm(tcp-previous_tcp)<1e-5:
                    self._check_step(q,qn,max_joint_step_deg,name)
                midpoint=(previous_tcp+tcp)*.5
                append_point(midpoint,depth+1);append_point(tcp,depth+1)
                return
            rows.append(self._row(f'{name}_{len(rows)}',qn,tcp,R,off,R_ab,t_ab))
            q,previous_tcp=qn,tcp.copy()
        for i in range(0 if include_start else 1, count + 1):
            tcp = start + (end - start) * (i / count)
            append_point(tcp)
        return {"waypoints": rows, "metrics": self.metrics(rows)}

    def plan_arc(self, q_current, tcp_current, R_current, off_current, axis_point,
                 axis_dir, angle, R_ab, t_ab, *, angular_step_deg=1.,
                 include_start=True, max_joint_step_deg=8.):
        """按接触后实测 TCP 重规划：角度单位度，axis_dir 世界系，保留完整偏心圆弧。"""
        if not math.isfinite(angular_step_deg) or not 0 < angular_step_deg <= 1.:
            raise PlanningError("旋转步长必须在 (0,1] 度")
        if not math.isfinite(angle):
            raise PlanningError("转角必须有限")
        tcp0, off = _vector(tcp_current, 3, "实测 TCP"), _vector(off_current, 3, "实测 TCP 偏移")
        axis_point = _vector(axis_point, 3, "轴上一点")
        R0 = _rotation(R_current, "实测朝向")
        R_ab, t_ab = _rotation(R_ab, "臂座朝向"), _vector(t_ab, 3, "臂座位置")
        q = _vector(q_current, len(self.names), "实测关节角")
        # 起点也必须自洽，不能拿旧 TCP 或旧基座把当前位置硬拉回历史规划。
        self._row("arc_anchor", q, tcp0, R0, off, R_ab, t_ab)
        count = max(1, math.ceil(abs(angle) / angular_step_deg))
        rows = []
        for i in range(0 if include_start else 1, count + 1):
            theta = angle * i / count
            rot = axis_rotation(axis_dir, theta)
            tcp, R = axis_point + rot @ (tcp0 - axis_point), rot @ R0
            qn = self._solve(q, tcp, R, off, R_ab, t_ab)
            self._check_step(q, qn, max_joint_step_deg, "轮轴轨迹")
            rows.append(self._row(f"turn_{i}", qn, tcp, R, off, R_ab, t_ab,
                                  angle_deg=float(theta)))
            q = qn
        return {"waypoints": rows, "metrics": self.metrics(rows),
                "axis_point": axis_point.tolist(),
                "axis_world": (_vector(axis_dir, 3, "转轴") / np.linalg.norm(axis_dir)).tolist(),
                "angle_deg": float(angle), "tcp_source": "caller_measured_at_contact"}

    @staticmethod
    def _check_step(q, qn, max_joint_step_deg, label):
        if not math.isfinite(max_joint_step_deg) or max_joint_step_deg <= 0:
            raise PlanningError("最大关节步幅必须有限且为正")
        delta = float(np.degrees(np.max(np.abs(qn - q))))
        if delta > max_joint_step_deg:
            raise PlanningError(f"{label} 关节跳变 {delta:.3f}°，拒绝切换 IK 分支")

    @staticmethod
    def metrics(rows):
        if not rows:
            return {"waypoint_count": 0}
        qs = np.asarray([r["q"] for r in rows])
        delta = np.diff(qs, axis=0)
        return {"waypoint_count": len(rows),
                "max_tcp_error_mm": max(r["tcp_error_mm"] for r in rows),
                "max_flange_error_mm": max(r["flange_error_mm"] for r in rows),
                "max_rotation_error_deg": max(r["rotation_error_deg"] for r in rows),
                "min_joint_limit_margin_deg": min(r["min_joint_margin_deg"] for r in rows),
                "max_adjacent_joint_delta_deg": float(np.degrees(np.abs(delta).max())) if len(delta) else 0.}

    def plan(self, snapshot, *, body_xy=(0., 0.), lift_delta=0., angle=90., clearance=.12,
             roll_deg=None, close_position=None, linear_step_m=.01, angular_step_deg=1.,
             grasp_axial_offset_m=0.0, allow_legacy_grasp_reference=False,
             approach_open_margin_mm=None):
        """完整预览计划；移动量可为假设，执行方须另过全身碰撞和当前反馈门槛。"""
        if (not math.isfinite(clearance) or clearance <= 0 or not math.isfinite(angle) or
                not math.isfinite(grasp_axial_offset_m)):
            raise PlanningError("接近距离必须为正，转角和抓取轴向偏移必须有限")
        if approach_open_margin_mm is not None and (not math.isfinite(approach_open_margin_mm)
                                                   or approach_open_margin_mm <= 0):
            raise PlanningError('接近开口余量必须为有限正毫米数')
        placed = self.placement(snapshot, body_xy, lift_delta)
        geometry = snapshot["geometry"]
        measured_center = _vector(geometry["center"], 3, "手轮中心")
        axis_point = _vector(geometry.get("axis_point", geometry.get("axis_point_world",
                              geometry.get("link_world"))), 3, "轮轴原点")
        axis = _vector(geometry.get("axis_world", [0., 0., 1.]), 3, "轮轴方向")
        if np.linalg.norm(axis) < 1e-9:
            raise PlanningError("轮轴方向不能为零")
        axis = axis / np.linalg.norm(axis)
        # 抓取高度属于待避碰/接触验证的规划变量，绝不修改量测轮盘中心或原输入。
        grasp_center = measured_center
        if 'grasp_point_local_m' in geometry:
            p_w,R_w=pose_matrix(geometry['wheel_pose'])
            grasp_center=p_w+R_w@_vector(geometry['grasp_point_local_m'],3,'局部夹持点')
        proposed_center = grasp_center + float(grasp_axial_offset_m) * axis
        radius_mm = float(geometry["radius_mm"])
        if not math.isfinite(radius_mm) or radius_mm <= 0:
            raise PlanningError("手轮半径必须为正")
        off_release = self.tcp_offset(position=1000)
        R_ab, t_ab, q0 = placed["R_ab"], placed["t_ab"], placed["q"]
        rolls = list(range(0, 360, 30)) if roll_deg is None else [float(roll_deg)]
        failures = []
        for roll in rolls:
            try:
                R0 = grasp_rotation(axis, roll)
                support = self.grasp_support(snapshot, R_grasp=R0,
                                             allow_legacy_fallback=allow_legacy_grasp_reference)
                grasp_width = support['width_mm']
                inner_offset, inner_source = self._model_inner_face_offset(snapshot, allow_legacy_fallback=allow_legacy_grasp_reference)
                jaw = np.asarray(support['jaw_axis_world'])
                midpoint_delta = support['midpoint_world_projection_m'] - float(jaw @ proposed_center) - inner_offset
                # 支撑中点本身就是夹持目标；半预算只解释不对中为何会丢单侧接触。
                alignment = midpoint_delta
                center = proposed_center + jaw * alignment
                pre = center + clearance * axis
                if grasp_width >= gripper_model.STROKE_MM:
                    raise PlanningError('当前夹持方向支撑宽超过标定净开口')
                # 改预张开量必须连同TCP漂移、mimic与整段避碰一起规划，不能只改执行刻度。
                # 已有接触的入口仍先全开，避免把恢复松爪误变成继续收紧。
                contacts = snapshot.get('contacts', {})
                occupied = contacts.get('left_pad_touching') or contacts.get('right_pad_touching')
                position_approach = (1000. if approach_open_margin_mm is None or occupied else
                                     float(np.clip(1. + 999. * (grasp_width + approach_open_margin_mm)
                                                   / gripper_model.STROKE_MM, 1., 1000.)))
                off_open = self.tcp_offset(position=position_approach)
                position_close = (float(np.clip(1. + 999. * grasp_width / gripper_model.STROKE_MM, 1., 1000.))
                                  if close_position is None else close_position)
                off_close = self.tcp_offset(position=position_close)
                q_pre = self._solve(q0, pre, R0, off_open, R_ab, t_ab, multistart=True)
                pre_row = self._row("pre", q_pre, pre, R0, off_open, R_ab, t_ab)
                descent = self.plan_line(q_pre, pre, center, R0, off_open, R_ab, t_ab,
                                         step_m=linear_step_m, name="descent", include_start=False)["waypoints"]
                q = np.asarray(descent[-1]["q"])
                close_rows = []
                # 逐开口解模型端点，不把 TCP 的插值误当成夹爪真实运动。
                for position in np.linspace(position_approach, position_close, 9)[1:]:
                    off = self.tcp_offset(position=float(position))
                    qn = self._solve(q, center, R0, off, R_ab, t_ab)
                    self._check_step(q, qn, 8., "闭爪模型补偿")
                    close_rows.append(self._row("close_model", qn, center, R0, off, R_ab, t_ab,
                                                gripper_position_model=float(position), model_only=True))
                    q = qn
                turn = self.plan_arc(q, center, R0, off_close, axis_point, axis, angle, R_ab, t_ab,
                                     angular_step_deg=angular_step_deg, include_start=False)["waypoints"]
                q, tcp_end, R_end = (np.asarray(turn[-1][key]) for key in ("q", "tcp", "R"))
                open_rows = []
                for position in np.linspace(position_close, 1000., 9)[1:]:
                    off = self.tcp_offset(position=float(position))
                    qn = self._solve(q, tcp_end, R_end, off, R_ab, t_ab)
                    self._check_step(q, qn, 8., "开爪模型补偿")
                    open_rows.append(self._row("open_model", qn, tcp_end, R_end, off, R_ab, t_ab,
                                               gripper_position_model=float(position), model_only=True))
                    q = qn
                ascent = self.plan_line(q, tcp_end, tcp_end + clearance * axis, R_end, off_release,
                                        R_ab, t_ab, step_m=linear_step_m, name="ascent",
                                        include_start=False)["waypoints"]
                stages = {"pre": [pre_row], "descent": descent, "close_model": close_rows,
                          "turn_model": turn, "open_model": open_rows, "ascent_model": ascent}
                rows = [row for stage in stages.values() for row in stage]
                return {"schema_version": 1, "kind": "offline_kinematic_preview",
                        "side": self.side, "urdf": self.urdf_path, "joint_names": self.names,
                        "anchor_metrics": placed["metrics"], "placement": placed["placement"],
                        "R_ab": R_ab.tolist(), "t_ab": t_ab.tolist(), "roll_deg": roll,
                        "angle_deg": float(angle), "axis_point": axis_point.tolist(),
                        "axis_world": axis.tolist(), "center": measured_center.tolist(),
                        "measured_center": measured_center.tolist(), "grasp_tcp": center.tolist(),
                        "grasp_axial_offset_m": float(grasp_axial_offset_m),
                        "close_position_model": float(position_close), "grasp_support_width_mm": grasp_width,
                        "approach_position_model": position_approach,
                        "approach_open_margin_mm": approach_open_margin_mm,
                        "radius_based_diameter_mm_not_used_for_parallel_grasp": 2 * radius_mm,
                        "grasp_support": support,
                        "grasp_midpoint_alignment": {'model_inner_face_source': inner_source,
                            'model_inner_face_TCP_projection_offset_m': inner_offset,
                            'support_mid_minus_model_inner_mid_m': midpoint_delta,
                            'half_total_squeeze_reference_m': .00025,
                            'misalignment_exceeds_half_squeeze_budget': abs(midpoint_delta) > .00025,
                            'applied_jaw_axis_correction_m': alignment},
                        "legacy_grasp_reference_only": support['legacy_reference_only'],
                        "tcp_open_model": off_open.tolist(), "tcp_close_model": off_close.tolist(),
                        "tcp_release_model": off_release.tolist(),
                        "open_model_vs_measured_tcp_mm": float(np.linalg.norm(off_open - placed["tcp_off"]) * 1000.),
                        "stages": stages, "metrics": self.metrics(rows),
                        "initial_transit_collision_planned": False, "collision_validated": False,
                        "contact_validated": False, "requires_contact_replan": True,
                        "warning": "开合与转动只是无载模型预览；接触后必须实测 TCP 并重规划，未验证移动/手臂避碰或实际夹持"}
            except PlanningError as error:
                failures.append({"roll_deg": roll, "error": str(error)})
        raise PlanningError("没有完整连续路径通过: " + str(failures))
