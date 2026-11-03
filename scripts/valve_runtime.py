"""已有仿真主线程内的阀门反馈状态机；本文件没有 HTTP 或启动 Isaac 的入口。

先用 ``preflight`` 做只读全身扫掠，再由 ``run`` 单臂流送。所有阈值是仿真参考
控制阈值，写入结果，不代表硬件规格。真实接触、当前反馈或碰撞检查失败时拒绝继续。
"""
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[1]
for directory in (str(REPO), str(REPO / 'scripts')):
    if directory not in sys.path:
        sys.path.insert(0, directory)
from valve_feedback import capture
from valve_plan import PlanningError, ValvePlanner, axis_rotation, pose_matrix
from valve_safety import ValveSafety
from valve_transit import plan_transit
from valve_ik import rotation_angle, solve_ik
from valve_grip_stability import ContactHoldWindow
import gripper_model


REFERENCE_POLICY = {
    'environment_margin_m': .005, 'self_margin_m': .008,
    'tool_backing_margin_m': .001,
    'contact_penetration_m': .001, 'base_drift_m': .001,
    'base_rotation_deg': .10, 'mount_drift_m': .001, 'mount_rotation_deg': .5,
    'joint_anchor_gap_mm': 1., 'joint_axis_alignment_deg': .5, 'joint_swing_deg': .5,
    'arm_tracking_rad': .12, 'settle_position_rad': .02,
    'settle_velocity_rad_s': .03, 'settle_dwell_s': .20, 'settle_timeout_s': 6.,
    'settle_tcp_position_m': .002, 'settle_tcp_rotation_deg': 1.,
    'joint_path_speed_rad_s': 1.20, 'phase_min_duration_s': .25,
    'turn_joint_speed_rad_s': 2.0,
    'control_period_s': 1./60.,
    'grip_speed_position_s': 180., 'grip_open_speed_position_s': 400., 'grip_timeout_s': 8.,
    'grip_fast_speed_position_s': 1000., 'grip_approach_margin_mm': 8., 'grip_slow_margin_mm': 3.,
    'grip_open_min_position': 995., 'grip_position_tracking_units': 3., 'grip_bound_wait_s': 2.,
    'grip_reference_squeeze_mm': .5, 'contact_dwell_s': .15,
    'contact_loss_s': .15, 'slip_position_m': .006, 'slip_rotation_deg': 5.,
    'wheel_wrong_direction_deg': 2., 'wheel_lag_deg': 8.,
    'wheel_final_error_deg': 3., 'wheel_jump_per_step_deg': 15.,
    'wall_timeout_s': 600., 'progress_period_s': .20,
    'collision_check_period_s': .04, 'initial_stability_s': .30,
    'initial_base_stability_m': .001, 'initial_base_stability_deg': .10,
}


class OperationRejected(RuntimeError):
    pass


class OperationStopped(RuntimeError):
    pass


def _write(path, data):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False))
    temporary.replace(path)


def _policy(overrides):
    result = dict(REFERENCE_POLICY)
    for key, value in (overrides or {}).items():
        if key not in result or not math.isfinite(value) or value <= 0:
            raise ValueError('未知或无效的仿真参考控制阈值: ' + str(key))
        result[key] = float(value)
    if 'joint_path_speed_rad_s' in (overrides or {}) and 'turn_joint_speed_rad_s' not in (overrides or {}):
        result['turn_joint_speed_rad_s'] = result['joint_path_speed_rad_s']
    if result['grip_approach_margin_mm'] < result['grip_slow_margin_mm']:
        raise ValueError('预张开余量不能小于接触前慢速余量')
    return result


def _urdf(path):
    if path:
        return str(path)
    config = json.loads((REPO / 'config.json').read_text())
    return config['robot']['presets'][config['robot']['name']]['urdf_path']


def _arm(snapshot, planner):
    state = snapshot['state']
    names = state['joint_names']
    q = np.asarray([state['positions'][names.index(n)] for n in planner.names], float)
    velocity = np.asarray(state['velocities'], float)
    if velocity.shape != (len(names),) or not np.isfinite(velocity).all() or not np.isfinite(q).all():
        raise OperationRejected('关节位置/速度反馈无效')
    return q, velocity[[names.index(n) for n in planner.names]]


def _passive(snapshot, policy):
    parking = snapshot.get('robot_parking_anchor')
    if (snapshot.get('robot_native_fixed_base') is not True or not parking
            or parking.get('type') not in ('world', 'kinematic_body')):
        raise OperationRejected('底盘尚未驻车固定，拒绝启动阀门动作')
    joint = snapshot['wheel_joint']
    if not snapshot.get('base_fixed_to_world'):
        raise OperationRejected('阀体没有到世界/运动学固定工位的有效固定证明')
    if joint.get('drive_schemas') or any(k.startswith('drive:') for k in joint.get('attributes', {})):
        raise OperationRejected('轮轴仍有 DriveAPI/驱动属性，拒绝把轮轴驱动当作人形拧动')
    if (joint['anchor_gap_mm'] > policy['joint_anchor_gap_mm'] or
            joint['axis_alignment_deg'] > policy['joint_axis_alignment_deg'] or
            joint['off_axis_swing_deg'] > policy['joint_swing_deg']):
        raise OperationRejected('轮轴锚点/轴向/约束摆动超过参考阈值')
    if not snapshot['state'].get('playing', False):
        raise OperationRejected('仿真未在运行')
    if not snapshot.get('contacts', {}).get('available', False):
        raise OperationRejected('双指垫接触报告不可用，不能执行真实夹持')
    _load_bearing_pads(snapshot['contacts'])


def _load_bearing_pads(contacts):
    """承载来自实际有限正法向冲量，不回退到旧touching/count/总冲量猜测。"""
    if not contacts.get('available') or contacts.get('read_succeeded') is not True:
        raise OperationRejected('承载接触读取失败: ' + str(contacts.get('read_error', '报告未验证')))
    fields = ('left_pad_normal_impulse_Ns', 'right_pad_normal_impulse_Ns',
              'unexpected_robot_normal_impulse_Ns')
    try:
        values = [float(contacts[name]) for name in fields]
    except (KeyError, TypeError, ValueError) as error:
        raise OperationRejected('承载接触缺少明确法向冲量字段') from error
    if any(not math.isfinite(value) or value < 0. for value in values):
        raise OperationRejected('承载接触法向冲量非有限/负幅值')
    verified = contacts.get('load_verifiable') is True
    if not verified:
        if any(value > 0. for value in values):
            raise OperationRejected('不可验证的接触报告声称存在载荷')
        return False, False
    if values[2] > 0.:
        raise OperationRejected('手轮与非指垫机器人部件存在实际承载接触')
    return tuple(contacts.get(side + '_pad_load_bearing') is True and value > 0.
                 for side, value in zip(('left', 'right'), values[:2]))


def _contact_pairs(safety, wheel):
    pairs = []
    for leaf in ('right_gripper_left_pad_link', 'right_gripper_right_pad_link'):
        paths = [p for p in safety.model.poses0 if p.rsplit('/', 1)[-1] == leaf]
        if len(paths) != 1:
            raise OperationRejected('无法唯一定位实际指垫刚体: ' + leaf)
        pairs.append((paths[0], wheel))
    return pairs


def _tool_backing_clearances(safety, wheel, policy):
    """只为两片指垫背板指定正净空；它们仍是环境碰撞对，不能穿模或获得接触许可。"""
    overrides = []
    for leaf in ('right_gripper_left_in_link', 'right_gripper_right_in_link'):
        paths = [p for p in safety.model.poses0 if p.rsplit('/', 1)[-1] == leaf]
        if len(paths) != 1:
            raise OperationRejected('无法唯一定位实际夹爪背板刚体: ' + leaf)
        overrides.append({
            'bodies': [paths[0], wheel], 'margin_m': policy['tool_backing_margin_m'],
            'reason': ('正常夹持背板间隙小于统一5mm参考缓冲；本明确身体对采用正'
                       f'{policy["tool_backing_margin_m"]*1000:.3f}mm工具净空，仍禁止穿模，不是接触对'),
        })
    return overrides


def _grip_map(planner, safety, position):
    leader = gripper_model.leader_for_fraction((float(position) - 1.) / 999.)
    known = {planner.leader_name: leader}

    def resolve(name, seen=()):
        if name in known:
            return known[name]
        if name in seen:
            raise OperationRejected('夹爪 mimic 有环')
        mimic = planner.joints[name].find('mimic')
        if mimic is None:
            raise OperationRejected('夹爪关节缺少主动角/mimic: ' + name)
        value = (float(mimic.get('multiplier', 1.)) * resolve(mimic.get('joint'), seen + (name,)) +
                 float(mimic.get('offset', 0.)))
        known[name] = value
        return value

    return {name: resolve(name) for name in safety.model.names
            if name.startswith(planner.side + '_gripper_')}


def _competing_controllers(executor):
    if getattr(executor, '_motion', None) is not None or getattr(executor, '_auto_pick', None) is not None:
        raise OperationRejected('机器人已有运动/自动抓取任务，不能竞争控制')
    if getattr(executor, '_base_cmd', None) is not None:
        raise OperationRejected('底盘速度/停车控制仍激活，禁止固定底盘操作')
    parking = getattr(executor, '_base_parking', None)
    if parking is not None and not callable(parking) and bool(parking):
        raise OperationRejected('底盘停车状态机仍在运行')
    if getattr(executor, '_udp_targets', None) is not None:
        raise OperationRejected('UDP 关节目标仍存在，不能竞争控制')
    if getattr(executor, '_ros2_contract', None) is not None:
        raise OperationRejected('ROS2 契约节点仍激活，先明确停止控制输入')
    ros = getattr(executor, '_ros_controller', None)
    if ros is not None and ros.is_initialized():
        raise OperationRejected('进程内 ROS 关节控制器仍激活')
    vr = getattr(executor, '_vr_teleop', None)
    if vr is not None and getattr(vr, '_running', False):
        raise OperationRejected('VR 遥操作仍运行，不能竞争控制')
    for attribute in ('_l3_station', '_l3station'):
        station = getattr(executor, attribute, None)
        if station is None:
            continue
        if (getattr(station, '_robot_busy', False) or getattr(station, '_pending_action', None) is not None or
                getattr(station, 'queue', ()) or getattr(station, '_action_job', None) is not None or
                getattr(station, '_legacy_job', None) is not None):
            raise OperationRejected('L3 人形/机构/待办动作仍激活')
        approach = getattr(station, '_approach', None)
        if approach is not None and approach.busy:
            raise OperationRejected('L3 底盘接近任务仍激活')


def _wheel_transform(snapshot, angle_deg):
    p, R = pose_matrix(snapshot['geometry']['wheel_pose'])
    center = np.asarray(snapshot['geometry']['axis_point_world'], float)
    rot = axis_rotation(snapshot['geometry']['axis_world'], angle_deg)
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = rot @ R, center + rot @ (p - center)
    return result


def _closing_step(command_position, bound, width_mm, measured_gap_mm, contacts, dt, policy,
                  slow_latched=False):
    """快段只走无接触空行程；目标/实测任一到慢区或出现接触后，本次闭爪不再加速。"""
    if not all(math.isfinite(float(v)) for v in (command_position, bound, width_mm, measured_gap_mm, dt)) or dt <= 0:
        raise OperationRejected('闭爪速度分段缺少有限实际开口反馈')
    switch = float(np.clip(1. + 999. * (width_mm + policy['grip_slow_margin_mm'])
                           / gripper_model.STROKE_MM, 1., 1000.))
    touched = any(contacts.get(k) for k in ('left_pad_touching', 'right_pad_touching',
                                           'left_pad_load_bearing', 'right_pad_load_bearing'))
    slow = bool(slow_latched or touched or command_position <= switch+1e-9
                or measured_gap_mm <= width_mm + policy['grip_slow_margin_mm'])
    speed = policy['grip_speed_position_s'] if slow else policy['grip_fast_speed_position_s']
    floor = bound if slow else max(bound, switch)
    target = max(floor, command_position - speed * dt)
    # 入口已比参考边界更紧时，不借闭爪阶段偷偷向外跳目标。
    target = min(command_position, target)
    return {'position': target, 'speed_position_s': speed, 'slow_latched': slow,
            'mode': 'contact_slow' if slow else 'free_fast', 'switch_position': switch}


def _prepare(executor, valve_root, output, body_xy, lift_delta, roll_deg, angle, urdf_path, policy,
             allow_invariant_assembly=False, grasp_axial_offset_m=0.0):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    snapshot = capture(executor, valve_root)
    _write(output / 'snapshot.json', snapshot)
    _passive(snapshot, policy)
    _competing_controllers(executor)
    planner = ValvePlanner(_urdf(urdf_path))
    planner.independent_anchor(snapshot)
    _arm(snapshot, planner)
    if any(abs(float(x)) > 1e-10 for x in body_xy) or abs(lift_delta) > 1e-10:
        return ({'safe': False, 'requires_placement': True,
                 'reason': '此执行器尚未接入经过避碰的底盘/升降摆位；拒绝把假设臂座当实测执行',
                 'requested_body_xy': list(body_xy), 'requested_lift_delta': lift_delta,
                 'reference_policy': policy, 'stage_modified': False}, None)
    plan = planner.plan(snapshot, body_xy=(0., 0.), lift_delta=0., roll_deg=roll_deg, angle=angle,
                        grasp_axial_offset_m=grasp_axial_offset_m,
                        approach_open_margin_mm=policy['grip_approach_margin_mm'])
    _write(output / 'plan.json', plan)
    # 每次都重新采当前几何，不能借用旧阀门/旧电柜快照。
    safety = ValveSafety(executor, directory=output / 'geometry')
    contacts = _contact_pairs(safety, snapshot['wheel_joint']['body1'])
    kwargs = {k: policy[k] for k in ('environment_margin_m', 'self_margin_m', 'contact_penetration_m')}
    kwargs['contact_pairs'] = contacts
    assembly_reference = None
    if allow_invariant_assembly:
        controlled = planner.names + [n for n in safety.model.names if n.startswith('right_gripper_')]
        assembly_reference = safety.establish_invariant_assembly_reference(controlled)
        kwargs['invariant_assembly_reference'] = assembly_reference
    # 原始统一5mm参考诊断完整保留；工具专用正净空不改写实际几何距离。
    raw_actual = safety.check_configuration(refresh=True, **kwargs)
    tool_clearances = _tool_backing_clearances(safety, snapshot['wheel_joint']['body1'], policy)
    kwargs['clearance_overrides'] = tool_clearances
    actual = safety.check_configuration(refresh=False, **kwargs)
    if not actual['safe']:
        return ({'safe': False, 'requires_placement': False, 'reason': '当前全身真实净空不满足',
                 'actual': actual, 'raw_actual_uniform_environment_margin': raw_actual,
                 'tool_backing_clearance_overrides': tool_clearances,
                 'reference_policy': policy, 'stage_modified': False}, None)
    initial = safety.q.copy()
    approach_position = plan.get('approach_position_model', 1000.)
    open_q = safety.full_q(_grip_map(planner, safety, approach_position), q=initial)
    opening = safety.check_path([initial, open_q], **kwargs)
    paths = {'initial_open': opening}
    if not opening['safe']:
        return ({'safe': False, 'requires_placement': False, 'reason': '初始渐开夹爪扫掠未通过',
                 'actual': actual, 'raw_actual_uniform_environment_margin': raw_actual,
                 'tool_backing_clearance_overrides': tool_clearances,
                 'paths': paths, 'reference_policy': policy, 'stage_modified': False}, None)
    previous = open_q
    wheel_body = snapshot['wheel_joint']['body1']
    wheel_angle = 0.
    transit_diagnostics = None
    for phase, rows in plan['stages'].items():
        configurations = [previous]
        environment = [{wheel_body: _wheel_transform(snapshot, wheel_angle)}]
        for row in rows:
            partial = dict(zip(planner.names, row['q']))
            default_position = (approach_position if phase in ('pre', 'descent') else
                                plan['close_position_model'] if phase == 'turn_model' else 1000.)
            position = row.get('gripper_position_model', default_position)
            partial.update(_grip_map(planner, safety, position))
            configurations.append(safety.full_q(partial, q=initial))
            if phase == 'turn_model':
                wheel_angle = row['angle_deg']
            environment.append({wheel_body: _wheel_transform(snapshot, wheel_angle)})
        paths[phase] = safety.check_path(configurations, environment_poses=environment, **kwargs)
        if phase == 'pre' and not paths[phase]['safe']:
            direct_failure = paths[phase]
            route = plan_transit(safety, planner, configurations[0], configurations[-1], kwargs)
            transit_diagnostics = {'direct_path_failure': direct_failure, 'search': route}
            _write(output / 'transit_search.json', transit_diagnostics)
            if route['safe']:
                configurations = [np.asarray(q, float) for q in route['configurations']]
                arm_indices = [safety.model.names.index(n) for n in planner.names]
                replacement = []
                for index, full_q in enumerate(configurations[1:], 1):
                    q = full_q[arm_indices]
                    if index == len(configurations) - 1:
                        # 最后一点仍是原先的任务目标，不把 IK 残差改写成 FK 恒等零。
                        replacement.append(dict(rows[-1]))
                        continue
                    import arm_kin
                    pf, Rf = arm_kin.ee_world(planner.chain, q,
                                              np.asarray(plan['R_ab']), np.asarray(plan['t_ab']))
                    off = np.asarray(plan['tcp_open_model'])
                    replacement.append(planner._row(
                        f'transit_{index}', q, pf + Rf @ off, Rf, off,
                        np.asarray(plan['R_ab']), np.asarray(plan['t_ab']), transit_only=True))
                plan['stages']['pre'] = replacement
                plan['initial_transit_collision_planned'] = True
                plan['transit_metrics'] = route['metrics']
                plan['metrics'] = planner.metrics([r for stage in plan['stages'].values() for r in stage])
                _write(output / 'plan.json', plan)
                environment = [{wheel_body: _wheel_transform(snapshot, 0.)} for _ in configurations]
                paths[phase] = safety.check_path(configurations, environment_poses=environment, **kwargs)
        previous = configurations[-1]
        if not paths[phase]['safe']:
            break
    safe = all(p['safe'] for p in paths.values()) and len(paths) == len(plan['stages']) + 1
    public = {'safe': safe, 'requires_placement': False, 'actual': actual, 'paths': paths,
              'raw_actual_uniform_environment_margin': raw_actual,
              'tool_backing_clearance_overrides': tool_clearances,
              'transit_diagnostics': transit_diagnostics,
              'plan_metrics': plan['metrics'], 'contact_pairs': contacts,
              'measured_center': plan.get('measured_center', plan.get('center')),
              'grasp_tcp': plan.get('grasp_tcp', plan.get('center')),
              'grasp_axial_offset_m': float(grasp_axial_offset_m),
              'reference_policy': policy, 'thresholds_calibrated_to_hardware': False,
              'invariant_assembly_audit_explicitly_enabled': bool(allow_invariant_assembly),
              'invariant_assembly_reference': assembly_reference,
              'globally_collision_free': actual.get('globally_collision_free', False),
              'stage_modified': False, 'reason': '只读全身扫掠通过' if safe else '全路径扫掠未通过'}
    internal = {'snapshot': snapshot, 'planner': planner, 'safety': safety,
                'plan': plan, 'contact_pairs': contacts, 'collision_kwargs': kwargs, 'feedback_cache': True}
    return public, internal


def preflight(executor, valve_root='/World/ValveStation/Valve', output=None,
              body_xy=(0., 0.), lift_delta=0., roll_deg=0., angle=90., urdf_path=None, policy=None,
              allow_invariant_assembly=False, grasp_axial_offset_m=0.0):
    """主线程只读采样/规划/全身扫掠，不推进物理，也不写目标。"""
    output = Path(output or REPO / 'outputs/valve_operation_20261102')
    output.mkdir(parents=True, exist_ok=True)
    reference = _policy(policy)
    try:
        result, _ = _prepare(executor, valve_root, output, body_xy, lift_delta,
                             roll_deg, angle, urdf_path, reference, allow_invariant_assembly, grasp_axial_offset_m)
    except Exception as error:
        result = {'safe': False, 'requires_placement': False, 'error': str(error),
                  'error_type': type(error).__name__, 'reference_policy': reference, 'stage_modified': False}
    _write(output / 'preflight.json', result)
    return result


class _Runtime:
    def __init__(self, executor, valve_root, output, internal, policy):
        self.executor, self.valve_root, self.output = executor, valve_root, Path(output)
        self.planner, self.safety, self.plan = (internal[k] for k in ('planner', 'safety', 'plan'))
        self.collision_kwargs, self.policy = internal['collision_kwargs'], policy
        self.initial = internal['snapshot']
        self.current = self.initial
        self.phase = 'preflight'
        self.started = False
        self.wall_start, self.last_progress = time.monotonic(), -float('inf')
        self.last_angle = float(self.initial['wheel_angle_deg_wrapped'])
        self.last_collision_check = -float('inf')
        self.accumulated = 0.
        self.turn_origin = None
        self.relative_contact = None
        self.contact_lost_at = None
        self.command = None
        self.trace = []
        self.metrics = {'max_arm_tracking_rad': 0., 'max_slip_position_m': 0.,
                        'max_slip_rotation_deg': 0., 'physics_steps': 0}
        self._grip_diagnostic = None
        self.feedback_reader = None
        self.gripper_velocity_caps = None
        if internal.get('feedback_cache'):
            caps = np.asarray(executor.articulation._physics_view.get_dof_max_velocities()).reshape(-1)
            names = list(executor.articulation.dof_names)
            arm_caps = caps[[names.index(n) for n in self.planner.names]]
            self.gripper_velocity_caps = {n: float(caps[i]) for i, n in enumerate(names)
                                           if n.startswith('right_gripper_')}
            if (not self.gripper_velocity_caps or
                    any(not math.isfinite(c) or c <= 0 for c in self.gripper_velocity_caps.values())):
                raise OperationRejected('原生夹爪关节速度上限缺失/无效')
            if (not np.isfinite(arm_caps).all() or np.any(arm_caps <= 0)
                    or max(self.policy['joint_path_speed_rad_s'],self.policy['turn_joint_speed_rad_s']) > float(arm_caps.min())):
                raise OperationRejected('参考动作速度超过原生机械臂速度上限')
            from valve_feedback_cache import FeedbackReader
            reader = FeedbackReader(executor, valve_root, self.initial)
            try:
                self.current = reader()
                self.safety.feedback_reader = reader
                self.feedback_reader = reader
            except BaseException:
                reader.close();raise

    def _read_snapshot(self):
        return self.feedback_reader() if self.feedback_reader is not None else capture(self.executor, self.valve_root)

    def close_feedback(self):
        if self.feedback_reader is not None:
            self.feedback_reader.close()
            self.safety.feedback_reader = None

    def _time(self):
        return float(self.executor.world.current_time)

    def _guard(self, snapshot, *, contact_required=False, desired_angle=None):
        if (self.output / 'STOP').exists():
            raise OperationStopped('收到合作停止文件，保持停止，不自动恢复')
        if time.monotonic() - self.wall_start > self.policy['wall_timeout_s']:
            raise OperationRejected('执行墙钟超时')
        _passive(snapshot, self.policy)
        _competing_controllers(self.executor)
        q, velocities = _arm(snapshot, self.planner)
        p0, R0 = pose_matrix(self.initial['base_pose'])
        p, R = pose_matrix(snapshot['base_pose'])
        if np.linalg.norm(p - p0) > self.policy['base_drift_m'] or rotation_angle(R, R0) > self.policy['base_rotation_deg']:
            raise OperationRejected('操作中底盘发生漂移，停止')
        mount0 = self.initial.get('fixed_anchor', {}).get('pose')
        mount = snapshot.get('fixed_anchor', {}).get('pose')
        if mount0 is not None:
            if mount is None:
                raise OperationRejected('固定工位证明/反馈消失')
            pm0, Rm0 = pose_matrix(mount0)
            pm, Rm = pose_matrix(mount)
            if (np.linalg.norm(pm - pm0) > self.policy['mount_drift_m'] or
                    rotation_angle(Rm, Rm0) > self.policy['mount_rotation_deg']):
                raise OperationRejected('阀体固定工位发生漂移')
        if self.command is not None:
            error = float(np.max(np.abs(q - self.command)))
            self.metrics['max_arm_tracking_rad'] = max(self.metrics['max_arm_tracking_rad'], error)
            if error > self.policy['arm_tracking_rad']:
                raise OperationRejected(f'手臂跟踪误差过大: {error:.4f}rad')
        if contact_required or self._time() - self.last_collision_check >= self.policy['collision_check_period_s']:
            actual = self.safety.check_configuration(refresh=True, **self.collision_kwargs)
            self.last_collision_check = self._time()
            if not actual['safe']:
                _write(self.output / 'collision_stop.json', actual)
                raise OperationRejected('实时全身碰撞/净空检查未通过')
        angle = float(snapshot['wheel_angle_deg_wrapped'])
        delta = (angle - self.last_angle + 180.) % 360. - 180.
        if abs(delta) > self.policy['wheel_jump_per_step_deg']:
            raise OperationRejected('轮轴单步角度跳变，反馈不可用')
        self.accumulated += delta
        self.last_angle = angle
        if contact_required:
            contacts = snapshot['contacts']
            both = all(_load_bearing_pads(contacts))
            if both:
                self.contact_lost_at = None
            elif self.contact_lost_at is None:
                self.contact_lost_at = self._time()
            elif self._time() - self.contact_lost_at > self.policy['contact_loss_s']:
                raise OperationRejected('转动中失去双指垫接触（无可验证承载）')
            if self.relative_contact is not None:
                wp, wR = pose_matrix(snapshot['geometry']['wheel_pose'])
                tcp = np.asarray(snapshot['state']['sides']['right']['tcp']['position'])
                _, fR = pose_matrix(snapshot['state']['sides']['right']['flange'])
                rp, rR = self.relative_contact
                slip_p = float(np.linalg.norm(wR.T @ (tcp - wp) - rp))
                slip_R = rotation_angle(wR.T @ fR, rR)
                self.metrics['max_slip_position_m'] = max(self.metrics['max_slip_position_m'], slip_p)
                self.metrics['max_slip_rotation_deg'] = max(self.metrics['max_slip_rotation_deg'], slip_R)
                if slip_p > self.policy['slip_position_m'] or slip_R > self.policy['slip_rotation_deg']:
                    raise OperationRejected(f'夹爪相对手轮滑移: {slip_p*1000:.2f}mm/{slip_R:.2f}°')
        if desired_angle is not None and self.turn_origin is not None:
            actual_turn = self.accumulated - self.turn_origin
            direction = 1. if self.plan['angle_deg'] >= 0 else -1.
            if direction * actual_turn < -self.policy['wheel_wrong_direction_deg']:
                raise OperationRejected('手轮实际转动方向相反')
            if abs(desired_angle - actual_turn) > self.policy['wheel_lag_deg']:
                raise OperationRejected(f'手轮转角没有跟随: desired={desired_angle:.2f}, actual={actual_turn:.2f}')
        self.current = snapshot
        self._progress()
        return q, velocities

    def _progress(self, force=False):
        now = self._time()
        if force or now - self.last_progress >= self.policy['progress_period_s']:
            row = {'phase': self.phase, 'sim_time': now, 'wheel_accumulated_deg': self.accumulated,
                   'turn_deg': None if self.turn_origin is None else self.accumulated - self.turn_origin,
                   'metrics': self.metrics.copy(), 'wall_elapsed_s': time.monotonic() - self.wall_start}
            _write(self.output / 'progress.json', row)
            self.trace.append(row)
            self.last_progress = now

    def _step(self, **guard):
        before = self._time()
        self.executor.step()
        if self._time() <= before:
            raise OperationRejected('仿真物理时钟没有推进')
        self.metrics['physics_steps'] += 1
        snapshot = self._read_snapshot()
        self._record_grip_sample(snapshot, 'physics_step')
        return self._guard(snapshot, **guard)

    def _record_grip_sample(self, snapshot, sample_kind):
        """只存已有读数；不把SDK速度与位姿差分混同，也不改变任何放行条件。"""
        diagnostic = self._grip_diagnostic
        if diagnostic is None:
            return
        try:
            state, contacts = snapshot['state'], snapshot['contacts']
            names = state['joint_names']
            idx = names.index(self.planner.leader_name)
            def number(value):
                value = float(value)
                return value if math.isfinite(value) else None
            row = {'sample_kind': sample_kind, 'sim_time_s': number(snapshot['sim_time']),
                   'wall_elapsed_s': number(time.monotonic() - self.wall_start),
                   'world_time_s': number(self._time()), 'physics_dt_s': number(snapshot['physics_dt']),
                   'leader_name': self.planner.leader_name,
                   'raw_leader_q_rad': number(state['positions'][idx]),
                   'raw_leader_velocity_rad_s': number(state['velocities'][idx]),
                   'last_dispatched_command_position': diagnostic['last_command_position'],
                   'last_dispatched_calibrated_angle_rad': diagnostic['last_command_angle_rad'],
                   'grip_speed_mode': diagnostic.get('speed_mode'),
                   'contact_available': contacts.get('available'),
                   'contact_read_succeeded': contacts.get('read_succeeded'),
                   'contact_load_verifiable': contacts.get('load_verifiable'),
                   'contact_read_error': contacts.get('read_error'),
                   'control_evaluated': False, 'control_stable_since_s': None,
                   'control_dwell_elapsed_s': None}
            for side in ('left', 'right'):
                for suffix in ('_pad_touching', '_pad_load_bearing'):
                    row[side + suffix] = contacts.get(side + suffix)
                row[side + '_pad_normal_impulse_Ns'] = number(contacts[side + '_pad_normal_impulse_Ns'])
            try:
                row['both_load_bearing'] = all(_load_bearing_pads(contacts))
                row['load_predicate_error'] = None
            except Exception as error:
                row['both_load_bearing'] = False
                row['load_predicate_error'] = str(error)
            raw_v = row['raw_leader_velocity_rad_s']
            row['stopped'] = raw_v is not None and abs(raw_v) < self.policy['settle_velocity_rad_s']
            try:
                # cmd_positions可回退到实测q；这里直接读native目标并明确失败，不冒充可靠target。
                targets = np.asarray(self.executor.articulation._physics_view.get_dof_position_targets()).reshape(-1)
                if targets.size != len(names) or not math.isfinite(float(targets[idx])):
                    raise ValueError('native position target维度/数值无效')
                row['native_leader_target_rad'] = float(targets[idx])
                row['native_target_read_error'] = None
            except Exception as error:
                row['native_leader_target_rad'] = None
                row['native_target_read_error'] = str(error)
            diagnostic['samples'].append(row)
        except Exception as error:
            # 观测失败只记录，原有控制门仍负责拒绝；日志不得制造新的控制路径。
            diagnostic['record_errors'].append(type(error).__name__ + ': ' + str(error))

    def _record_grip_decision(self, stable):
        diagnostic = self._grip_diagnostic
        if diagnostic is not None and diagnostic['samples']:
            row = diagnostic['samples'][-1]
            row['control_evaluated'] = True
            row['control_stable_since_s'] = stable
            row['control_dwell_elapsed_s'] = 0. if stable is None else self._time() - stable

    def _arm_velocity(self, velocity):
        setter = getattr(self.executor.articulation, 'set_joint_velocity_targets', None)
        if setter is not None:
            if hasattr(self.executor,'_require_robot'):self.executor._require_robot()
            names = list(self.executor.articulation.dof_names)
            setter(np.asarray(velocity,dtype=np.float32).reshape(1,-1),
                   joint_indices=[names.index(n) for n in self.planner.names])

    def _speed_limit(self):
        return self.policy['turn_joint_speed_rad_s'] if self.phase == 'turn' else self.policy['joint_path_speed_rad_s']

    def _arm_command(self, q, velocity=None):
        if (self.output / 'STOP').exists():
            raise OperationStopped('收到停止文件，禁止发送下一条运动目标')
        q = np.asarray(q, float)
        if q.shape != (len(self.planner.names),) or not np.isfinite(q).all():
            raise OperationRejected('拒绝非有限或维度错误的单臂指令')
        if velocity is not None:
            velocity = np.asarray(velocity,float)
            if (velocity.shape != q.shape or not np.isfinite(velocity).all()
                    or abs(velocity).max() > self._speed_limit()*(1.+1e-6)):
                raise OperationRejected('拒绝超过参考速度或非有限的单臂速度前馈')
        self.executor.send_joint_command(q.tolist(), duration=0., joint_names=self.planner.names)
        self.command = q.copy()
        self.started = True
        if velocity is not None:self._arm_velocity(velocity)

    def _sweep(self, configurations, environment_poses=None):
        report = self.safety.check_path(configurations, environment_poses=environment_poses,
                                        **self.collision_kwargs)
        if not report['safe']:
            _write(self.output / ('replan_collision_' + self.phase + '.json'), report)
            raise OperationRejected('新轨迹全身扫掠检查未通过: ' + self.phase)
        return report

    def _rows_sweep(self, rows, moving_wheel=False):
        self.safety.refresh()
        configurations = [self.safety.q.copy()]
        configurations += [self.safety.full_q(dict(zip(self.planner.names, row['q']))) for row in rows]
        environment = None
        if moving_wheel:
            body = self.current['wheel_joint']['body1']
            environment = [{body: _wheel_transform(self.current, 0.)}]
            environment += [{body: _wheel_transform(self.current, row['angle_deg'])} for row in rows]
        return self._sweep(configurations, environment)

    def _coupled_wheel_poses(self, angles):
        """抓住后预测被操作轮盘的同步运动；实际跟随、滑移与净空仍逐物理步检查。"""
        actual = self.accumulated-self.turn_origin
        body = self.current['wheel_joint']['body1']
        return [{body:_wheel_transform(self.current,float(angle)-actual)}for angle in angles]

    def _settle(self, target, contact=False, desired_angle=None, tcp_target=None, R_target=None):
        start, stable = self._time(), None
        target = np.asarray(target, float).copy()
        while True:
            q, velocity = self._step(contact_required=contact, desired_angle=desired_angle)
            ready = (np.max(np.abs(q - target)) < self.policy['settle_position_rad'] and
                     np.max(np.abs(velocity)) < self.policy['settle_velocity_rad_s'])
            if tcp_target is not None:
                actual_tcp, actual_R = pose_matrix(self.current['state']['sides']['right']['tcp'])
                tcp_error = float(np.linalg.norm(actual_tcp - tcp_target))
                tcp_ready = (tcp_error < self.policy['settle_tcp_position_m']
                             and rotation_angle(actual_R, R_target) < self.policy['settle_tcp_rotation_deg'])
                ready = ready and tcp_ready
            stable = (self._time() if stable is None else stable) if ready else None
            if stable is not None and self._time() - stable >= self.policy['settle_dwell_s']:
                return
            if self._time() - start > self.policy['settle_timeout_s']:
                raise OperationRejected('等待实际关节位置/速度收敛超时，拒绝放行')
            if tcp_target is not None and not tcp_ready:
                # 自由底盘上伸臂会改变升降/躯干实际姿态；不能只等冻结臂座下的q目标。
                anchor = self.planner.independent_anchor(self.current)
                fit = self.planner._solve(q, tcp_target, R_target, anchor['tcp_off'],
                                          anchor['R_ab'], anchor['t_ab'])
                base = self.command if self.command is not None else q
                delta = fit - base
                budget = self._speed_limit() * float(self.current['physics_dt'])
                command = base + delta * min(1., budget / max(float(abs(delta).max()), 1e-12))
                self.safety.refresh()
                partial = dict(zip(self.planner.names, command))
                environment = (self._coupled_wheel_poses([self.accumulated-self.turn_origin,desired_angle])
                               if contact and desired_angle is not None else None)
                self._sweep([self.safety.q.copy(), self.safety.full_q(partial)],environment)
                self._arm_command(command)
                target = command
                self.metrics['tcp_feedback_corrections'] = self.metrics.get('tcp_feedback_corrections', 0) + 1
                self.metrics['max_settle_tcp_error_m'] = max(self.metrics.get('max_settle_tcp_error_m', 0.), tcp_error)

    def stream(self, rows, phase, contact=False):
        self.phase = phase
        self._guard(self._read_snapshot(), contact_required=contact)
        self._rows_sweep(rows, moving_wheel=contact)
        current_q, _ = _arm(self.current, self.planner)
        points = np.vstack([current_q, [r['q'] for r in rows]])
        start_tcp, start_R = pose_matrix(self.current['state']['sides']['right']['tcp'])
        tcp_points = np.vstack([start_tcp, [r['tcp'] for r in rows]])
        rotation_points = [start_R] + [np.asarray(r['R'], float) for r in rows]
        lengths = np.max(np.abs(np.diff(points, axis=0)), axis=1)
        cumulative = np.r_[0., np.cumsum(lengths)]
        total = float(cumulative[-1])
        duration = max(self.policy['phase_min_duration_s'], 1.5 * total / self._speed_limit())
        t0 = self._time()
        angles = np.r_[0., [r.get('angle_deg', 0.) for r in rows]]
        def sample(u):
            distance = total * (u * u * (3 - 2*u))
            if total < 1e-12:
                target, desired = points[-1], float(angles[-1])
                tcp_goal, rotation_goal = tcp_points[-1], rotation_points[-1]
            else:
                i = min(len(points) - 2, max(0, int(np.searchsorted(cumulative, distance, side='right') - 1)))
                blend = (distance - cumulative[i]) / max(lengths[i], 1e-12)
                target = points[i] * (1 - blend) + points[i+1] * blend
                desired = float(angles[i] * (1 - blend) + angles[i+1] * blend)
                tcp_goal = tcp_points[i] * (1 - blend) + tcp_points[i+1] * blend
                delta_R = Rotation.from_matrix(rotation_points[i].T @ rotation_points[i+1]).as_rotvec()
                rotation_goal = rotation_points[i] @ Rotation.from_rotvec(delta_R * blend).as_matrix()
            return target, desired, tcp_goal, rotation_goal
        dt = float(self.current['physics_dt'])
        control_steps = max(1, int(round(self.policy['control_period_s'] / dt)))
        while True:
            elapsed = self._time() - t0
            count = min(control_steps, max(1, int(math.ceil((duration - elapsed) / dt))))
            u = min(1., (elapsed + count*dt) / duration)
            target, desired, tcp_goal, rotation_goal = sample(u)
            _, start_angle, _, _ = sample(min(1., elapsed / duration))
            # 实时臂座反馈贯穿整段；否则自由底盘的微小转动会在下降前积累厘米级侧偏。
            anchor = self.planner.independent_anchor(self.current)
            fit = self.planner._solve(target, tcp_goal, rotation_goal, anchor['tcp_off'],
                                      anchor['R_ab'], anchor['t_ab'])
            base = self.command if self.command is not None else anchor['q']
            delta = fit - base
            budget = self._speed_limit() * dt * count
            target = base + delta * min(1., budget / max(float(abs(delta).max()), 1e-12))
            self.safety.refresh()
            # 先完整扫掠本周期的插值段，再逐物理步发送插值目标；接触/驻车反馈不降频。
            environment = (self._coupled_wheel_poses([self.accumulated-self.turn_origin,start_angle,desired])
                           if contact else None)
            self._sweep([self.safety.q.copy(), self.safety.full_q(dict(zip(self.planner.names, base))),
                         self.safety.full_q(dict(zip(self.planner.names, target)))],environment)
            for substep in range(count):
                fraction = (substep + 1) / count
                # 位置轨迹自带速度前馈，避免零速度目标的阻尼造成D/K比例的跟随滞后。
                self._arm_command(base + (target-base)*fraction, velocity=(target-base)/(count*dt))
                angle = start_angle + (desired-start_angle)*fraction
                self._step(contact_required=contact, desired_angle=angle if contact else None)
            if u >= 1.:
                break
        self._arm_velocity(np.zeros(len(self.planner.names)))
        self._settle(points[-1], contact=contact,
                     desired_angle=float(angles[-1]) if contact else None,
                     tcp_target=np.asarray(rows[-1]['tcp']), R_target=np.asarray(rows[-1]['R']))

    def grip(self, closing, *, maintain_tcp=True):
        phase = 'contact_close' if closing else ('release' if maintain_tcp else 'initial_open')
        diagnostic = {'schema': 'valve_grip_diagnostic_v1', 'phase': phase,
            'time_clock': 'snapshot.sim_time and executor.world.current_time; simulation seconds',
            'records_only_this_invocation': True,
            'does_not_reconstruct_previous_bench_contact_history': True,
            'control_thresholds_unchanged': True,
            'settle_velocity_limit_rad_s': self.policy['settle_velocity_rad_s'],
            'contact_dwell_required_s': self.policy['contact_dwell_s'],
            'last_command_position': None, 'last_command_angle_rad': None,
            'samples': [], 'record_errors': [], 'exit_reason': 'interrupted',
            'wall_started': time.monotonic()}
        previous = self._grip_diagnostic
        self._grip_diagnostic = diagnostic
        try:
            self._grip(closing, maintain_tcp=maintain_tcp)
            diagnostic['exit_reason'] = 'completed'
        except BaseException as error:
            diagnostic['exit_reason'] = str(error)
            diagnostic['exit_error_type'] = type(error).__name__
            raise
        finally:
            self._grip_diagnostic = previous
            diagnostic['wall_duration_s'] = time.monotonic() - diagnostic['wall_started']
            diagnostic['sample_count'] = len(diagnostic['samples'])
            diagnostic['physics_step_sample_count'] = sum(
                row['sample_kind'] == 'physics_step' for row in diagnostic['samples'])
            try:
                # 约4000步只占少量内存；结束统一落盘，避免每帧磁盘IO改变时序。
                _write(self.output / (phase + '_grip_diagnostics.json'), diagnostic)
            except Exception as error:
                self.metrics.setdefault('grip_diagnostic_write_errors', []).append(str(error))

    def _grip(self, closing, *, maintain_tcp=True):
        self.phase = 'contact_close' if closing else ('release' if maintain_tcp else 'initial_open')
        snapshot = self._read_snapshot()
        self._record_grip_sample(snapshot, 'entry_snapshot')
        self._guard(snapshot)
        q, _ = _arm(snapshot, self.planner)
        hold_arm = q.copy()
        side = snapshot['state']['sides']['right']
        tcp_target, R_target = pose_matrix(side['tcp'])
        current_position = float(side['gripper']['position'])
        support = self.planner.grasp_support(snapshot)
        grasp_width = support['width_mm']
        jaw = np.asarray(support['jaw_axis_world'])
        if 'inner_face_midpoint_world' not in side:
            raise OperationRejected('实际夹持缺少pad inner-face中点，禁止猜TCP对齐')
        inner_mid = np.asarray(side['inner_face_midpoint_world'], float)
        midpoint_delta = support['midpoint_world_projection_m'] - float(jaw @ inner_mid)
        half_budget_m = self.policy['grip_reference_squeeze_mm'] / 2000.
        correction = midpoint_delta if closing else 0.
        tcp_target += jaw * correction
        _write(self.output / (self.phase + '_support_reference.json'), {
            'support': support, 'actual_inner_face_midpoint_world': inner_mid.tolist(),
            'actual_geometric_tcp_world': side['tcp']['position'],
            'support_mid_minus_inner_mid_m': midpoint_delta,
            'half_total_squeeze_budget_m': half_budget_m, 'applied_tcp_jaw_correction_m': correction,
            'misalignment_exceeds_half_squeeze_budget': abs(midpoint_delta) > half_budget_m,
            'corrected_tcp_target_world': tcp_target.tolist(),
            'radius_diameter_mm_not_used': 2 * snapshot['geometry']['radius_mm'],
            'total_squeeze_budget_mm_unchanged': self.policy['grip_reference_squeeze_mm']})
        opening = max(0., grasp_width - self.policy['grip_reference_squeeze_mm'])
        bound = (1. + 999. * opening / gripper_model.STROKE_MM if closing else
                 self.plan.get('approach_position_model', 1000.) if not maintain_tcp else 1000.)
        bound = float(np.clip(bound, 1., 1000.))
        preload_to_bound = bool(closing and snapshot['geometry'].get('grasp_region',{}).get('preload_to_bound',False))
        started, stable = self._time(), None
        contact_hold = ContactHoldWindow(self.policy)
        command_position = current_position
        slow_latched = False
        last_grip_sent = None
        dt = float(snapshot['physics_dt'])
        speed = (self.policy['grip_speed_position_s'] if closing else
                 self.policy['grip_fast_speed_position_s'] if not maintain_tcp and bound < current_position else
                 self.policy['grip_open_speed_position_s'])
        _write(self.output / (self.phase + '_speed_reference.json'), {
            'target_position': bound, 'entry_position': current_position,
            'approach_position_model': self.plan.get('approach_position_model', 1000.),
            'fast_speed_position_s': self.policy['grip_fast_speed_position_s'],
            'slow_speed_position_s': self.policy['grip_speed_position_s'],
            'slow_margin_mm': self.policy['grip_slow_margin_mm'],
            'total_squeeze_budget_mm': self.policy['grip_reference_squeeze_mm']})
        timeout = max(self.policy['grip_timeout_s'], abs(bound - current_position) / speed + self.policy['settle_timeout_s'])
        while True:
            if self._time() - started > timeout:
                raise OperationRejected('渐进夹爪阶段超时，未建立可验证接触/开口')
            contacts = self.current['contacts']
            both = all(_load_bearing_pads(contacts))
            names = self.current['state']['joint_names']
            leader_idx = names.index(self.planner.leader_name)
            leader_velocity = abs(float(self.current['state']['velocities'][leader_idx]))
            stopped = leader_velocity < self.policy['settle_velocity_rad_s']
            hold = None
            if closing:
                try:
                    preload_complete = not preload_to_bound or command_position <= bound+1e-9
                    hold = contact_hold.update(self.current, now=self._time(), both_loaded=both and preload_complete)
                    if not preload_complete:hold['reason']='loading_reference_preload'
                except (KeyError, ValueError) as error:
                    raise OperationRejected('夹持停稳反馈无效: ' + str(error)) from error
                if self._grip_diagnostic is not None and self._grip_diagnostic['samples']:
                    self._grip_diagnostic['samples'][-1]['mechanical_hold'] = hold
            if closing and hold['stopped']:
                stable = self._time() - hold['dwell_s']
                self._record_grip_decision(stable)
                break
            else:
                stable = (self._time() - hold['dwell_s'] if closing and hold['reason'] == 'waiting_continuous_dwell'
                          else None)
                self._record_grip_decision(stable)
            if not closing or not both or (preload_to_bound and command_position>bound):
                if closing:
                    next_step = _closing_step(command_position, bound, grasp_width,
                        self.current['state']['sides']['right']['inner_face_gap_mm'], contacts, dt,
                        self.policy, slow_latched)
                    slow_latched = next_step['slow_latched']
                    command_position = next_step['position']
                    if self._grip_diagnostic is not None:self._grip_diagnostic['speed_mode'] = next_step['mode']
                else:
                    delta = bound - command_position
                    command_position += math.copysign(min(abs(delta), speed*dt), delta)
                    if self._grip_diagnostic is not None:
                        self._grip_diagnostic['speed_mode'] = 'preposition' if not maintain_tcp else 'release'
            q, _ = _arm(self.current, self.planner)
            if maintain_tcp:
                anchor = self.planner.independent_anchor(self.current)
                # 支撑中点预算仅0.25mm，旧0.5mm DLS提前终止会吞掉必要的对中修正。
                if (self.command is not None and np.linalg.norm(anchor['tcp']-tcp_target)<.0001
                        and rotation_angle(anchor['R_flange'],R_target)<self.planner.rotation_tol_deg):
                    qn = self.command.copy()
                else:
                    qn = self.planner._solve(q, tcp_target, R_target, anchor['tcp_off'],
                                             anchor['R_ab'], anchor['t_ab'], position_precision_m=.0001)
                self.planner._check_step(q, qn, 8., '实测TCP夹持补偿')
                import arm_kin
                pf, Rf = arm_kin.ee_world(self.planner.chain, qn, anchor['R_ab'], anchor['t_ab'])
                pe = float(np.linalg.norm(pf + Rf @ anchor['tcp_off'] - tcp_target))
                ae = rotation_angle(Rf, R_target)
                if pe >= .005 or ae >= 3.:
                    raise OperationRejected('按实测开合 TCP 补偿 IK 失败')
            else:
                qn = hold_arm
            arm_changed = self.command is None or float(abs(qn-self.command).max())>1e-9
            grip_changed = last_grip_sent is None or abs(command_position-last_grip_sent)>1e-9
            if arm_changed or grip_changed:
                self.safety.refresh()
                partial = dict(zip(self.planner.names, qn))
                partial.update(_grip_map(self.planner, self.safety, command_position))
                self._sweep([self.safety.q.copy(), self.safety.full_q(partial)])
                if arm_changed:self._arm_command(qn)
                if grip_changed:
                    # 全部mimic目标也受原生关节速度上限约束；不修改原生cap来迁就快段。
                    caps = getattr(self, 'gripper_velocity_caps', None)
                    if caps is not None:
                        prior = _grip_map(self.planner, self.safety,
                                          current_position if last_grip_sent is None else last_grip_sent)
                        following = _grip_map(self.planner, self.safety, command_position)
                        if any(abs(following[n]-prior[n])/dt > cap*(1.+1e-6) for n,cap in caps.items()):
                            raise OperationRejected('夹爪指令速度超过原生关节上限')
                    self.executor.set_gripper_position(side='right', position=command_position)
                    last_grip_sent = command_position
            if grip_changed and self._grip_diagnostic is not None:
                try:
                    self._grip_diagnostic['last_command_position'] = command_position
                    self._grip_diagnostic['last_command_angle_rad'] = gripper_model.leader_for_fraction(
                        (command_position - 1.) / 999.)
                except Exception as error:
                    self._grip_diagnostic['record_errors'].append('command observation: ' + str(error))
            self.started = True
            self._step()
            actual_position = float(self.current['state']['sides']['right']['gripper']['position'])
            if (not closing and abs(command_position-bound) < 1e-9 and
                    abs(actual_position-bound) <= 1000.-self.policy['grip_open_min_position'] and stopped):
                break
            if (closing and command_position <= bound and not both and
                    self._time() - started > self.policy['grip_bound_wait_s']):
                # 等实际驱动追上参考下界后仍无双垫接触，不能继续深压赌摩擦。
                if (abs(actual_position - command_position) < self.policy['grip_position_tracking_units'] and stopped):
                    raise OperationRejected('参考最大收紧范围内没有双指垫承载接触')
        _write(self.output / (self.phase + '_feedback.json'), self.current)

    def stop(self):
        errors = []
        try:
            self._arm_velocity(np.zeros(len(self.planner.names)))
        except Exception as error:
            errors.append('清除单臂速度前馈: ' + str(error))
        try:
            self.executor.set_base_velocity(vx=0., wz=0., vy=0.)
        except Exception as error:
            errors.append('停轮: ' + str(error))
        try:
            names = list(self.executor.articulation.dof_names)
            measured = np.asarray(self.executor.articulation.get_joint_positions()).reshape(-1)
            partial_names = self.planner.names + [self.planner.leader_name]
            hold = np.array([measured[names.index(name)] for name in partial_names], float)
            if not np.isfinite(hold).all():
                raise RuntimeError('实测关节非有限，拒绝写 NaN hold')
            self.executor._motion = None
            self.executor.send_joint_command(hold.tolist(), duration=0., joint_names=partial_names)
        except Exception as error:
            errors.append('单臂/夹爪有限反馈保持: ' + str(error))
        return errors

    def execute(self):
        if (self.output / 'STOP').exists():
            raise OperationStopped('停止标志仍存在，拒绝自行恢复')
        # 只推进已有仿真并读回根位置；此窗口内不发送任何手臂/夹爪/底盘目标。
        self.phase = 'initial_stability'
        start = self._time()
        base_p, base_R = pose_matrix(self.current['base_pose'])
        while self._time() - start < self.policy['initial_stability_s']:
            _, velocity = self._step()
            p, R = pose_matrix(self.current['base_pose'])
            if (np.linalg.norm(p - base_p) > self.policy['initial_base_stability_m'] or
                    rotation_angle(R, base_R) > self.policy['initial_base_stability_deg'] or
                    np.max(np.abs(velocity)) > self.policy['settle_velocity_rad_s']):
                raise OperationRejected('初始无目标稳定窗口未通过，拒绝启动电机目标')
        self.grip(False, maintain_tcp=False)
        self.stream(self.plan['stages']['pre'], 'pre')
        self.stream(self.plan['stages']['descent'], 'descent')
        self.grip(True)
        self.phase = 'contact_replan'
        contact = self._read_snapshot()
        self._guard(contact, contact_required=True)
        anchor = self.planner.independent_anchor(contact)
        wp, wR = pose_matrix(contact['geometry']['wheel_pose'])
        self.relative_contact = (wR.T @ (anchor['tcp'] - wp), wR.T @ anchor['R_flange'])
        self.turn_origin = self.accumulated
        arc = self.planner.plan_arc(anchor['q'], anchor['tcp'], anchor['R_flange'], anchor['tcp_off'],
                                    contact['geometry']['axis_point_world'], contact['geometry']['axis_world'],
                                    self.plan['angle_deg'], anchor['R_ab'], anchor['t_ab'])
        _write(self.output / 'contact_replanned_arc.json', arc)
        self.stream(arc['waypoints'], 'turn', contact=True)
        turned = self.accumulated - self.turn_origin
        if abs(turned - self.plan['angle_deg']) > self.policy['wheel_final_error_deg']:
            raise OperationRejected('真实轮轴转角没有到达目标')
        self.grip(False)
        self.phase = 'retreat_replan'
        snapshot = self._read_snapshot()
        self._guard(snapshot)
        anchor = self.planner.independent_anchor(snapshot)
        axis = np.asarray(snapshot['geometry']['axis_world'])
        retreat = self.planner.plan_line(anchor['q'], anchor['tcp'], anchor['tcp'] + .12 * axis,
                                         anchor['R_flange'], anchor['tcp_off'], anchor['R_ab'], anchor['t_ab'],
                                         name='retreat')
        _write(self.output / 'retreat_replanned.json', retreat)
        self.stream(retreat['waypoints'], 'retreat')
        self.phase = 'complete'
        final = self._read_snapshot()
        self._guard(final)
        actual = self.accumulated - self.turn_origin
        if abs(actual - self.plan['angle_deg']) > self.policy['wheel_final_error_deg']:
            raise OperationRejected('松爪/退出后轮轴实际角度偏离目标')
        self._progress(force=True)
        return {'success': True, 'phase': 'complete', 'target_angle_deg': self.plan['angle_deg'],
                'actual_angle_deg': actual, 'metrics': self.metrics,
                'verified_by': 'passive wheel physical angle, actual contacts, relative slip, all-body collision checks'}


def _run(executor, valve_root='/World/ValveStation/Valve', output=None,
        body_xy=(0., 0.), lift_delta=0., roll_deg=0., angle=90., urdf_path=None, policy=None,
        allow_invariant_assembly=False, grasp_axial_offset_m=0.0):
    """必须由仿真主线程调用；通过 executor.step 推进，实际完成才返回 success=true。"""
    output = Path(output or REPO / 'outputs/valve_operation_20261102')
    output.mkdir(parents=True, exist_ok=True)
    reference, runtime = _policy(policy), None
    check = None
    try:
        if (output / 'STOP').exists():
            raise OperationStopped('停止文件存在；需新的明确运行请求清理后才可执行')
        if hasattr(executor.world, '_scene'):
            # 官方Fabric仅改变位姿传输路径，避免240Hz写回USD/属性面板，不改变物理步长。
            import carb
            from isaacsim.core.simulation_manager import SimulationManager
            if (not SimulationManager.is_fabric_enabled()
                    or carb.settings.get_settings().get('/physics/updateToUsd')):
                SimulationManager.enable_fabric(True)
            if not SimulationManager.get_physics_sim_view().is_valid:
                executor.world.pause()
                raise OperationRejected('快速位姿通路切换后Tensor视图无效，拒绝下发')
        check, internal = _prepare(executor, valve_root, output, body_xy, lift_delta,
                                    roll_deg, angle, urdf_path, reference, allow_invariant_assembly, grasp_axial_offset_m)
        _write(output / 'preflight.json', check)
        if not check['safe']:
            result = {'success': False, 'phase': 'preflight', 'motion_started': False, 'preflight': check}
        else:
            runtime = _Runtime(executor, valve_root, output, internal, reference)
            result = runtime.execute()
            result['motion_started'] = runtime.started
    except Exception as error:
        stopped = isinstance(error, OperationStopped)
        result = {'success': False, 'phase': 'stopped' if stopped else (runtime.phase if runtime else 'preflight'),
                  'error': str(error), 'error_type': type(error).__name__,
                  'motion_started': bool(runtime and runtime.started)}
        if runtime and runtime.started:
            result['stop_errors'] = runtime.stop()
            result['metrics'] = runtime.metrics
            result['actual_angle_deg'] = None if runtime.turn_origin is None else runtime.accumulated - runtime.turn_origin
    result['reference_policy'] = reference
    result['thresholds_calibrated_to_hardware'] = False
    result['invariant_assembly_audit_explicitly_enabled'] = bool(allow_invariant_assembly)
    result['globally_collision_free'] = bool(check and check.get('globally_collision_free', False))
    result['grasp_axial_offset_m'] = float(grasp_axial_offset_m)
    result['tool_backing_clearance_overrides'] = check.get('tool_backing_clearance_overrides', []) if check else []
    if runtime is not None:
        runtime.close_feedback()
        _write(output / 'trace.json', runtime.trace)
    _write(output / 'result.json', result)
    _write(output / 'progress.json', {k: v for k, v in result.items() if k != 'preflight'})
    return result


def run(executor, valve_root='/World/ValveStation/Valve', output=None,
        body_xy=(0., 0.), lift_delta=0., roll_deg=0., angle=90., urdf_path=None, policy=None,
        allow_invariant_assembly=False, grasp_axial_offset_m=0.0):
    """GUI执行时临时取消选择，避免属性面板为每个物理步重建；结束恢复原选择。"""
    context = selection = stage = None
    previous = []
    if getattr(executor, 'simulation_app', None) is not None:
        import omni.usd
        context = omni.usd.get_context();stage = context.get_stage();selection = context.get_selection()
        previous = selection.get_selected_prim_paths()
        if previous:selection.set_selected_prim_paths([],False)
    try:
        return _run(executor, valve_root, output, body_xy, lift_delta, roll_deg, angle, urdf_path,
                    policy, allow_invariant_assembly, grasp_axial_offset_m)
    finally:
        try:
            if previous and context.get_stage() == stage and not selection.get_selected_prim_paths():
                selection.set_selected_prim_paths(previous,False)
        except Exception:
            import logging
            logging.getLogger(__name__).warning('界面选择恢复失败，动作结果仍保留',exc_info=True)
