"""独立位姿包络小动作诊断，不修改 strict verifier 或覆盖其失败结论。

仅供已启动、已暂停的仿真主线程显式调用。原始 SDK 速度全量保存，非头 .03
速度门只作独立诊断；真实关节上限、61刚体位姿和96形状包络仍是硬拒绝门。
"""
import json
import math
from pathlib import Path
import threading

import numpy as np
from scipy.spatial.transform import Rotation

from verify_transport_stability import _read_fixed, _preserved, _subscribe


def _matrix(row):
    row = np.asarray(row, float)
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(row[3:]).as_matrix()
    matrix[:3, 3] = row[:3]
    return matrix


def _wheel_axes(safety, paths):
    """轮轴在各轮刚体系的方向来自同次capture的实际joint T1，不猜轴向。"""
    names = {'front_left_wheel_joint', 'rear_left_wheel_joint',
             'front_right_wheel_joint', 'rear_right_wheel_joint'}
    result = {}
    for joint in safety.model.data['joints']:
        if joint['name'] not in names:
            continue
        token = joint['axis']
        frame = np.asarray(joint['T1'], float)
        if token not in ('X', 'Y', 'Z') or frame.shape != (4, 4) or not np.isfinite(frame).all():
            raise RuntimeError('实际轮关节轴/T1反馈无效')
        axis = frame[:3, :3] @ np.eye(3)['XYZ'.index(token)]
        norm = np.linalg.norm(axis)
        if norm < 1e-10 or joint['body1'] not in paths:
            raise RuntimeError('轮轴不能映射到实际刚体')
        result[paths.index(joint['body1'])] = axis / norm
    if len(result) != 4:
        raise RuntimeError('四个真实轮刚体轴覆盖不完整')
    return result


def _angle_metrics(actual_R, reference_R, wheel_axes):
    """轮绕自由滚轴的spin不算车体朝向摆动；原完整SO3角仍完整返回。"""
    raw = np.degrees(Rotation.from_matrix(actual_R @ reference_R.transpose(0, 2, 1)).magnitude())
    task = raw.copy()
    swings = []
    for index, axis in wheel_axes.items():
        observed, expected = actual_R[index] @ axis, reference_R[index] @ axis
        cross = np.linalg.norm(np.cross(observed, expected))
        angle = math.degrees(math.atan2(float(cross), float(np.clip(observed @ expected, -1., 1.))))
        task[index] = angle
        swings.append(angle)
    return task, raw, np.asarray(swings)


def _cylinder_rules(shapes, paths, wheel_axes):
    """只验证真实圆截面且与轮关节轴平行的Cylinder；失败者保留raw角点门。"""
    wheel_by_body = {paths[index]: axis for index, axis in wheel_axes.items()}
    rules, raw_corner_mask, records = {}, [], []
    for index, shape in enumerate(shapes):
        count = len(shape['corners'])
        eligible = False
        if shape['body'] in wheel_by_body and shape.get('type') == 'Cylinder':
            try:
                matrix = np.asarray(shape['matrix'], float)
                token = shape['attributes']['axis']
                radius, height = float(shape['attributes']['radius']), float(shape['attributes']['height'])
                if matrix.shape != (4, 4) or not np.isfinite(matrix).all() or token not in ('X', 'Y', 'Z'):
                    raise ValueError('Cylinder矩阵/轴无效')
                columns = matrix[:3, :3]
                scales = np.linalg.norm(columns, axis=0)
                if np.any(scales <= 0.) or not math.isfinite(radius) or not math.isfinite(height) or radius <= 0. or height <= 0.:
                    raise ValueError('Cylinder尺寸无效')
                unit_columns = columns / scales
                radial = [i for i in range(3) if i != 'XYZ'.index(token)]
                if (not np.allclose(unit_columns.T @ unit_columns, np.eye(3), atol=1e-6, rtol=0.)
                        or not np.isclose(scales[radial[0]], scales[radial[1]], atol=1e-8, rtol=1e-6)):
                    raise ValueError('非正交或径向非圆截面')
                axis = unit_columns[:, 'XYZ'.index(token)]
                if np.linalg.norm(np.cross(axis, wheel_by_body[shape['body']])) > 1e-5:
                    raise ValueError('Cylinder轴与真实轮关节轴不平行')
                center = np.asarray(shape['corners'], float).mean(axis=0)
                if np.linalg.norm(center - matrix[:3, 3]) > 1e-6:
                    raise ValueError('Cylinder包络中心与局部原点不符')
                length = float(np.linalg.norm(shape['corners'] - center, axis=1).max())
                if not math.isfinite(length) or length < max(radius * scales[radial[0]], height * scales['XYZ'.index(token)] / 2):
                    raise ValueError('Cylinder角点未保守包围实际尺寸')
                rules[index] = {'axis': axis, 'center': center, 'L': length}
                records.append({'shape': shape.get('path', str(index)), 'body': shape['body'],
                                'axis_body': axis.tolist(), 'center_body': center.tolist(),
                                'L_m': length, 'method': 'center_delta + L*norm(axis_actual-axis_reference)',
                                'radial_circle_and_joint_axis_verified': True})
                eligible = True
            except (KeyError, ValueError, TypeError):
                pass
        raw_corner_mask.extend([not eligible] * count)
    return rules, np.asarray(raw_corner_mask, bool), records


def _shape_metrics(actual, reference, shapes, cylinder_rules):
    raw, task, actual_corners, centers, axes = [], [], [], [], []
    for index, shape in enumerate(shapes):
        a, b = actual[shape['body']], reference[shape['body']]
        ac = shape['corners'] @ a[:3, :3].T + a[:3, 3]
        bc = shape['corners'] @ b[:3, :3].T + b[:3, 3]
        value = float(np.linalg.norm(ac - bc, axis=1).max())
        actual_corners.append(ac)
        raw.append(value)
        if index in cylinder_rules:
            rule = cylinder_rules[index]
            ca, cb = a[:3, :3] @ rule['center'] + a[:3, 3], b[:3, :3] @ rule['center'] + b[:3, 3]
            aa, ab = a[:3, :3] @ rule['axis'], b[:3, :3] @ rule['axis']
            value = float(np.linalg.norm(ca - cb) + rule['L'] * np.linalg.norm(aa - ab))
            centers.append(ca)
            axes.append(aa)
        task.append(value)
    return (np.asarray(task), np.asarray(raw), np.concatenate(actual_corners),
            np.asarray(centers).reshape(-1, 3), np.asarray(axes).reshape(-1, 3))


def _prepare(executor, directory, original_target, head):
    from valve_safety import ValveSafety
    safety = ValveSafety(executor, directory=directory, expected_robot_shapes=96)
    qbase, _ = safety.refresh()
    path = [qbase.copy()]
    for target in (original_target + .01, original_target - .01, original_target):
        start = path[-1][head]
        count = max(1, math.ceil(abs(target - start) / .002))
        for angle in np.linspace(start, target, count + 1)[1:]:
            q = qbase.copy()
            q[head] = angle
            path.append(q)
    whole = safety.check_path(path, environment_margin_m=.012, self_margin_m=.008,
                              surface_step_m=.002, joint_step_rad=.002)
    arms = [safety.check(min_clearance_m=.05, refresh=False, poses=safety.predict(q)) for q in path]
    result = {'safe': bool(whole['safe'] and all(r['safe'] for r in arms)),
              'whole_body': whole, 'arms': arms, 'assembly_exemptions_used': False}
    Path(directory).mkdir(parents=True, exist_ok=True)
    (Path(directory) / 'preflight.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    if not result['safe']:
        raise RuntimeError('96形状扫掠12mm/自碰撞8mm/双臂50mm未通过')
    paths = executor.articulation._physics_view.link_paths
    paths = list(paths[0]) if paths and not isinstance(paths[0], str) else list(paths)
    if len(paths) != 61 or len(set(paths)) != 61:
        raise RuntimeError('61个真实机器人刚体路径覆盖不完整')
    if safety.model.names != list(executor.articulation.dof_names):
        raise RuntimeError('几何模型与关节反馈顺序不一致')
    root = np.asarray(safety.poses[safety.model.root], float).copy()
    shapes = [s for s in safety.model.shapes if s['robot']]
    if len(shapes) != 96 or any(s['body'] not in paths for s in shapes):
        raise RuntimeError('96碰撞形状未全部绑定实际61刚体')
    expected = safety.model.fk(qbase, root)
    if any(path not in expected for path in paths):
        raise RuntimeError('FK没有覆盖全部61刚体')
    return safety, np.asarray(qbase).copy(), root, paths, shapes, result


def diagnose_pose_bounded_head_motion(executor, output):
    """完成独立诊断并返回分开的轨迹/位姿包络/raw速度结果；最终暂停。"""
    from physics_wake import wake_if_sleeping
    from valve_runtime import _competing_controllers
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    art, world = executor.articulation, executor.world
    names = list(art.dof_names)
    report = {'diagnostic_completed': False, 'actual_trajectory_completed': False,
              'pose_envelope_passed': False, 'strict_velocity_gate_passed': None,
              'raw_velocity_caps_gate_passed': None,
              'raw_q_caps_gate_passed': None, 'motion_phase_caps_passed': None,
              'motion_pose_passed': None, 'motion_strict_velocity_gate_passed': None,
              'plant_settling_excursions': {'actuator_commands_added': False,
                  'q_cap_violation_samples': 0, 'positive_velocity_cap_violation_samples': 0,
                  'internal_pose_excursion_samples': 0, 'joint_q_excursions': {},
                  'joint_velocity_excursions': {}, 'max_internal_body_position_m': 0.,
                  'max_internal_body_angle_deg': 0., 'max_internal_shape_geometry_m': 0.},
              'configured_zero_cap_joint_names': [],
              'configured_zero_cap_velocity_exceeded': False,
              'configured_zero_cap_velocity_violations': {},
              'strict_verifier_changed': False, 'prior_strict_failures_reclassified': False,
              'reference_thresholds': {'body_position_m': .001, 'body_angle_deg': .1,
                  'shape_corner_m': .002, 'root_position_m': .001, 'root_full_angle_deg': .1,
                  'raw_nonhead_velocity_diagnostic': .03, 'head_tracking_rad': .02,
                  'head_arrival_rad': .002, 'head_arrival_velocity_rad_s': .02,
                  'arrival_dwell_s': .25, 'wake_pose_dwell_s': 2., 'wake_timeout_s': 6.,
                  'geometry_period_s': .04}, 'command_log': [],
              'notes': ['独立位姿包络诊断不是原全50关节瞬时速度验收的通过结果',
                        'SDK原始速度、离散dq/dt均保存；不将二者当作相同物理量',
                        '所有阈值为仿真参考，非硬件标定']}
    keys = ('t', 'q', 'v', 'links', 'root_v', 'expected_links', 'head_reference', 'phase',
            'max_body_position_error_m', 'max_body_angle_error_deg', 'max_shape_corner_error_m',
            'body_raw_full_angle_error_deg', 'wheel_axis_swing_error_deg',
            'max_raw_shape_corner_error_m', 'shape_geometry_error_bound_m', 'shape_raw_corner_error_m',
            'world_expected_links', 'world_body_position_error_m', 'world_body_raw_full_angle_error_deg',
            'world_body_orientation_metric_error_deg', 'world_shape_geometry_error_bound_m',
            'world_shape_raw_corner_error_m', 'root_position_error_m', 'root_full_angle_error_deg')
    samples = {k: [] for k in keys}
    owner = threading.get_ident()
    phase, elapsed, desired = 'initial_paused', 0., None
    baseline = None
    original_target, head = None, None
    unsubscribe = None
    callback_error = None
    last = None
    safety, qbase, root_ref, paths, shapes = None, None, None, None, None
    wheel_axes = None
    cylinder_rules, raw_corner_mask = None, None
    limits, caps = None, None
    wake_window, wake_since = [], None
    wake_ready = False
    head_written = False
    pose_failure = False
    last_geometry = -float('inf')
    initially_paused = not world.is_playing()

    def collect(dt, context=None):
        nonlocal elapsed, callback_error, last, wake_since, wake_ready, pose_failure
        try:
            if threading.get_ident() != owner or not math.isfinite(dt) or dt <= 0:
                raise RuntimeError('物理回调线程/步长无效')
            elapsed += float(dt)
            q = np.asarray(art.get_joint_positions()).reshape(-1).copy()
            v = np.asarray(art.get_joint_velocities()).reshape(-1).copy()
            links = np.asarray(art._physics_view.get_link_transforms()).reshape(-1, 7).copy()
            root_v = np.asarray(art.get_velocities()).reshape(-1).copy()
            if q.shape != (50,) or v.shape != (50,) or links.shape != (61, 7):
                raise RuntimeError('物理状态维度不匹配')
            if not all(np.isfinite(a).all() for a in (q, v, links, root_v)):
                raise RuntimeError('实际q/v/刚体反馈非有限')
            for k, value in (('t', elapsed), ('q', q), ('v', v), ('links', links),
                             ('root_v', root_v), ('head_reference', desired), ('phase', phase)):
                samples[k].append(value)
            warm, motion = phase == 'wake_settle', phase.startswith('head_')
            other_v = np.delete(v, head)
            if report['strict_velocity_gate_passed'] is None:
                report['strict_velocity_gate_passed'] = True
            if np.max(abs(other_v)) > .03:
                report['strict_velocity_gate_passed'] = False
            if motion:
                if report['motion_strict_velocity_gate_passed'] is None:
                    report['motion_strict_velocity_gate_passed'] = True
                if np.max(abs(other_v)) > .03:
                    report['motion_strict_velocity_gate_passed'] = False
            if not all(_preserved(baseline, _read_fixed(art), head).values()):
                raise RuntimeError('其余49目标/全部速度目标/增益/力矩上限发生改变')
            # 只用实际runtime限位，含已授权caster±90；容差仅为float32数值舍入量级。
            tolerance = 8 * np.finfo(np.float32).eps * np.maximum(1., np.max(abs(limits), axis=1))
            q_bad = (q < limits[:, 0] - tolerance) | (q > limits[:, 1] + tolerance)
            if report['raw_q_caps_gate_passed'] is None:
                report['raw_q_caps_gate_passed'] = True
            if q_bad.any():
                report['raw_q_caps_gate_passed'] = False
                if warm:
                    plant = report['plant_settling_excursions']
                    plant['q_cap_violation_samples'] += 1
                    for index in np.flatnonzero(q_bad):
                        item = plant['joint_q_excursions'].setdefault(names[index], {'sample_count': 0, 'maximum_outside_rad_or_m': 0.})
                        item['sample_count'] += 1
                        item['maximum_outside_rad_or_m'] = max(item['maximum_outside_rad_or_m'],
                            float(max(limits[index, 0] - q[index], q[index] - limits[index, 1], 0.)))
                else:
                    report['motion_phase_caps_passed'] = False
                    raise RuntimeError('实际q超过runtime硬限（仅允许float32舍入量级）')
            if report['raw_velocity_caps_gate_passed'] is None:
                report['raw_velocity_caps_gate_passed'] = True
            for index in np.flatnonzero(caps == 0.):
                value = abs(float(v[index]))
                if value > 0.:
                    report['raw_velocity_caps_gate_passed'] = False
                    report['configured_zero_cap_velocity_exceeded'] = True
                    item = report['configured_zero_cap_velocity_violations'].setdefault(
                        names[index], {'configured_cap': 0., 'sample_count': 0,
                                       'max_abs_sdk_velocity_rad_s': 0.,
                                       'body_pose_envelope_still_required': True})
                    item['sample_count'] += 1
                    item['max_abs_sdk_velocity_rad_s'] = max(item['max_abs_sdk_velocity_rad_s'], value)
            # 未受控相机源配置为0时不伪造positive cap；保持原值并单列raw cap失败。
            # 头部和其余实际positive有限上限仍严格检查，所有刚体/形状包络不豁免。
            finite_caps = np.isfinite(caps) & (caps > 0.)
            velocity_bad = np.zeros(50, dtype=bool)
            velocity_bad[finite_caps] = abs(v[finite_caps]) > caps[finite_caps] + 8 * np.finfo(np.float32).eps * np.maximum(1., caps[finite_caps])
            if velocity_bad.any():
                report['raw_velocity_caps_gate_passed'] = False
                if warm:
                    plant = report['plant_settling_excursions']
                    plant['positive_velocity_cap_violation_samples'] += 1
                    for index in np.flatnonzero(velocity_bad):
                        item = plant['joint_velocity_excursions'].setdefault(names[index],
                            {'configured_positive_cap': float(caps[index]), 'sample_count': 0, 'maximum_abs_sdk_velocity': 0.})
                        item['sample_count'] += 1
                        item['maximum_abs_sdk_velocity'] = max(item['maximum_abs_sdk_velocity'], abs(float(v[index])))
                else:
                    report['motion_phase_caps_passed'] = False
                    raise RuntimeError('实际速度超过实际finite runtime上限')
            if motion and report['motion_phase_caps_passed'] is None:
                report['motion_phase_caps_passed'] = True
            qref = qbase.copy()
            qref[head] = desired
            actual = {p: _matrix(row) for p, row in zip(paths, links)}
            root = actual[safety.model.root]
            # 关节命令是相对角：内部跟踪误差须在同帧实测根位姿下比较。
            # 根对固定capture的漂移另有独立硬门，不通过长杆效应重复计入内部误差。
            predicted = safety.model.fk(qref, root)
            world_predicted = safety.model.fk(qref, root_ref)
            expected_m = np.stack([predicted[path] for path in paths])
            world_expected_m = np.stack([world_predicted[path] for path in paths])
            actual_R = Rotation.from_quat(links[:, 3:]).as_matrix()
            position_errors = np.linalg.norm(links[:, :3] - expected_m[:, :3, 3], axis=1)
            angle_errors, raw_full_angles, wheel_swings = _angle_metrics(actual_R, expected_m[:, :3, :3], wheel_axes)
            shape_errors, raw_corner_errors, actual_corners, cylinder_centers, cylinder_axes = _shape_metrics(
                actual, predicted, shapes, cylinder_rules)
            world_shape_errors, world_raw_corners, *_ = _shape_metrics(actual, world_predicted, shapes, cylinder_rules)
            world_position_errors = np.linalg.norm(links[:, :3] - world_expected_m[:, :3, 3], axis=1)
            world_angles, world_raw_angles, _ = _angle_metrics(actual_R, world_expected_m[:, :3, :3], wheel_axes)
            surface_error = float(shape_errors.max())
            expected_links = np.c_[expected_m[:, :3, 3], Rotation.from_matrix(expected_m[:, :3, :3]).as_quat()]
            world_expected_links = np.c_[world_expected_m[:, :3, 3], Rotation.from_matrix(world_expected_m[:, :3, :3]).as_quat()]
            root_distance = float(np.linalg.norm(root[:3, 3] - root_ref[:3, 3]))
            root_angle = float(Rotation.from_matrix(root[:3, :3] @ root_ref[:3, :3].T).magnitude())
            for key, value in (('expected_links', expected_links), ('max_body_position_error_m', float(position_errors.max())),
                               ('max_body_angle_error_deg', float(angle_errors.max())), ('max_shape_corner_error_m', surface_error),
                               ('body_raw_full_angle_error_deg', raw_full_angles), ('wheel_axis_swing_error_deg', wheel_swings),
                               ('max_raw_shape_corner_error_m', float(raw_corner_errors.max())),
                               ('shape_geometry_error_bound_m', shape_errors), ('shape_raw_corner_error_m', raw_corner_errors),
                               ('world_expected_links', world_expected_links), ('world_body_position_error_m', world_position_errors),
                               ('world_body_raw_full_angle_error_deg', world_raw_angles),
                               ('world_body_orientation_metric_error_deg', world_angles),
                               ('world_shape_geometry_error_bound_m', world_shape_errors), ('world_shape_raw_corner_error_m', world_raw_corners),
                               ('root_position_error_m', root_distance), ('root_full_angle_error_deg', math.degrees(root_angle))):
                samples[key].append(value)
            last = {'q': q, 'v': v, 'links': links, 'root': root,
                    'corners': actual_corners, 'root_distance': root_distance}
            internal_bad = position_errors.max() > .001 or angle_errors.max() > .1 or surface_error > .002
            if root_distance > .001 or abs(root_angle) > math.radians(.1):
                pose_failure = True
                raise RuntimeError(f'实际位姿包络超限: root={root_distance*1000:.4f}mm/{math.degrees(root_angle):.5f}deg')
            if warm:
                plant = report['plant_settling_excursions']
                plant['max_internal_body_position_m'] = max(plant['max_internal_body_position_m'], float(position_errors.max()))
                plant['max_internal_body_angle_deg'] = max(plant['max_internal_body_angle_deg'], float(angle_errors.max()))
                plant['max_internal_shape_geometry_m'] = max(plant['max_internal_shape_geometry_m'], surface_error)
                if internal_bad:
                    plant['internal_pose_excursion_samples'] += 1
            elif internal_bad:
                pose_failure = True
                report['motion_pose_passed'] = False
                raise RuntimeError(f'实际位姿包络超限: body={position_errors.max()*1000:.4f}mm/'
                                   f'{angle_errors.max():.5f}deg/corners={surface_error*1000:.4f}mm/'
                                   f'root={root_distance*1000:.4f}mm/{math.degrees(root_angle):.5f}deg')
            elif motion and report['motion_pose_passed'] is None:
                report['motion_pose_passed'] = True
            if phase.startswith('head_') and abs(float(q[head]) - desired) > .02:
                raise RuntimeError('实际头关节跟踪误差超过.02rad')
            if phase == 'wake_settle':
                wake_window.append((elapsed, links[:, :3].copy(), actual_R.copy(), actual_corners.copy(),
                                    cylinder_centers, cylinder_axes))
                while len(wake_window) > 1 and wake_window[1][0] <= elapsed - .25:
                    wake_window.pop(0)
                position_span = np.linalg.norm(np.ptp(np.stack([a[1] for a in wake_window]), axis=0), axis=1).max()
                angle_span = max(float(_angle_metrics(a[2], wake_window[0][2], wheel_axes)[0].max()) for a in wake_window)
                corners_window = np.stack([a[3][raw_corner_mask] for a in wake_window])
                surface_span = float(np.linalg.norm(np.ptp(corners_window, axis=0), axis=1).max()) if raw_corner_mask.any() else 0.
                if cylinder_rules:
                    center_span = np.linalg.norm(np.ptp(np.stack([a[4] for a in wake_window]), axis=0), axis=1)
                    axis_span = np.linalg.norm(np.ptp(np.stack([a[5] for a in wake_window]), axis=0), axis=1)
                    L = np.asarray([rule['L'] for rule in cylinder_rules.values()])
                    surface_span = max(surface_span, float((center_span + L * axis_span).max()))
                head_ready = abs(float(q[head]) - original_target) < .002 and abs(float(v[head])) < .02
                stable = (elapsed - wake_window[0][0] >= .25 - 1e-9 and position_span <= .001
                          and angle_span <= .1 and surface_span <= .002
                          and not q_bad.any() and not velocity_bad.any() and head_ready)
                wake_since = (elapsed if wake_since is None else wake_since) if stable else None
                wake_ready = wake_since is not None and elapsed - wake_since >= 2.
        except Exception as error:
            # 任一硬拒都不能把此前已完成的少量FK帧误报成整段包络通过。
            pose_failure = True
            if phase.startswith('head_'):
                report['motion_pose_passed'] = False
            if callback_error is None:
                callback_error = error
                report['failed_frame'] = {'recorded_sample_count': len(samples['t']),
                                          'physics_time_s': elapsed, 'phase': phase,
                                          'error': str(error)}
            world.pause()

    def geometry():
        nonlocal last_geometry, pose_failure
        if elapsed - last_geometry < .04:
            return
        actual = safety.check_configuration(refresh=True, environment_margin_m=.012, self_margin_m=.008)
        hands = safety.check(min_clearance_m=.05, refresh=False)
        last_geometry = elapsed
        if not actual['safe'] or not hands['safe']:
            pose_failure = True
            if phase.startswith('head_'):
                report['motion_pose_passed'] = False
            (output / 'geometry_stop.json').write_text(json.dumps({'actual': actual, 'arms': hands}, ensure_ascii=False, indent=2))
            raise RuntimeError('实时96形状12mm/自碰撞8mm/双臂50mm检查未通过')

    def step():
        before = elapsed
        executor.step(render=(len(samples['t']) % 8 == 0))
        if callback_error is not None:
            raise callback_error
        if elapsed <= before or last is None:
            raise RuntimeError('物理回调没有推进')
        geometry()

    def command(value):
        nonlocal desired, head_written
        desired = float(value)
        art.set_joint_position_targets(np.array([[desired]], dtype=np.float32), joint_indices=[head])
        head_written = True

    try:
        if not initially_paused or (hasattr(world, 'is_stopped') and world.is_stopped()):
            raise RuntimeError('必须从物理有效paused状态开始')
        _competing_controllers(executor)
        if len(names) != 50 or 'head_yaw_joint' not in names:
            raise RuntimeError('仅支持当前50DOF整机')
        head = names.index('head_yaw_joint')
        baseline = _read_fixed(art)
        if not all(a.shape == (50,) and np.isfinite(a).all() for a in baseline.values()):
            raise RuntimeError('目标/增益备份无效')
        original_target = float(baseline['position_targets'][head])
        desired = original_target
        limits = np.asarray(art.get_dof_limits(), float).reshape(-1, 2)
        # 本机legacy Articulation的名称是get_joint_max_velocities；直接读已验证的tensor接口。
        caps = np.asarray(art._physics_view.get_dof_max_velocities(), float).reshape(-1)
        if (limits.shape != (50, 2) or np.isnan(limits).any() or caps.shape != (50,)
                or np.isnan(caps).any() or np.any(caps < 0) or np.any(limits[:, 0] >= limits[:, 1])):
            raise RuntimeError('实际runtime限位/速度上限不可用')
        if not math.isfinite(float(caps[head])) or caps[head] <= 0.:
            raise RuntimeError('受控头部速度上限必须是实际finite positive值')
        zero_names = [names[i] for i in np.flatnonzero(caps == 0.)]
        if any(name not in ('left_camera_rojoint', 'right_camera_rojoint') for name in zero_names):
            raise RuntimeError('非相机零速度上限未纳入本独立诊断范围')
        report['configured_zero_cap_joint_names'] = zero_names
        report['commanded_head_runtime_velocity_cap_rad_s'] = float(caps[head])
        if not limits[head, 0] <= original_target - .01 < original_target + .01 <= limits[head, 1]:
            raise RuntimeError('头部往返目标超出实际硬限')
        report['runtime_velocity_caps'] = [float(v) if np.isfinite(v) else 'unbounded' for v in caps]
        report['runtime_q_limits'] = [[float(v) if np.isfinite(v) else ('-unbounded' if v < 0 else 'unbounded') for v in row] for row in limits]
        report['original_head_target_rad'] = original_target
        safety, qbase, root_ref, paths, shapes, proof = _prepare(executor, output / 'paused_geometry', original_target, head)
        wheel_axes = _wheel_axes(safety, paths)
        cylinder_rules, raw_corner_mask, circle_records = _cylinder_rules(shapes, paths, wheel_axes)
        report['wheel_body_axis_vectors'] = {paths[index]: axis.tolist() for index, axis in wheel_axes.items()}
        report['orientation_metric'] = '四轮body使用实际轴向swing；非wheel完整SO3角；raw完整角另存，center与全96形状物理几何2mm门不变'
        report['shape_geometry_metric'] = {'verified_wheel_cylinders': circle_records,
            'other_shape_count_using_raw_corners': len(shapes) - len(cylinder_rules),
            'threshold_m': .002, 'raw_corner_errors_still_recorded': True,
            'method': '仅验证的圆截面轮Cylinder采用SO2对称集的Hausdorff保守上界；其余形状仍raw角点'}
        report['initial_preflight'] = proof
        report['internal_frame'] = 'current_measured_base_link'
        report['world_diagnostic_frame'] = 'fixed_capture_root_ref; full prior-style residuals recorded, not reclassified'
        report['wake_stage_contract'] = '无新actuator目标；finite/root/fullgeometry始终硬门，初始内部ref/q/cap整定越界归档；actual world pose窗+q/caps+head到位连续2s后fresh capture才动作'
        report['motion_caps_scope'] = '实际q硬限与finite positive SDK cap；零相机cap另列raw失败，绝不伪造正上限'
        unsubscribe = _subscribe(collect)
        phase = 'wake_settle'
        world.play()
        report['wake'] = wake_if_sleeping(executor)
        if not report['wake'].get('supported') or report['wake'].get('sleep_after') is not False:
            raise RuntimeError('原生唤醒未确认awake，无state-setter fallback')
        while not wake_ready:
            step()
            if elapsed > 6.:
                raise RuntimeError('6s内位姿窗/q硬限/positive SDK caps/头部到位未连续稳定2s')
        report['wake_settle_duration_s'] = elapsed
        safety, qbase, root_ref, paths, shapes, proof = _prepare(executor, output / 'settled_geometry', original_target, head)
        wheel_axes = _wheel_axes(safety, paths)
        cylinder_rules, raw_corner_mask, circle_records = _cylinder_rules(shapes, paths, wheel_axes)
        report['post_wake_preflight'] = proof
        start_reference = original_target
        for label, target in (('head_positive', original_target + .01), ('head_negative', original_target - .01), ('head_return', original_target)):
            phase, start = label, elapsed
            report['command_log'].append({'phase': phase, 'start_time': start, 'target_rad': target})
            while elapsed - start < 2.:
                u = min(1., (elapsed - start) / 2.)
                command(start_reference + (target - start_reference) * u * u * (3 - 2 * u))
                step()
            command(target)
            arrival_start, dwell = elapsed, None
            while True:
                step()
                ready = abs(float(last['q'][head]) - target) < .002 and abs(float(last['v'][head])) < .02
                dwell = (elapsed if dwell is None else dwell) if ready else None
                if dwell is not None and elapsed - dwell >= .25:
                    break
                if elapsed - arrival_start > 3.:
                    raise RuntimeError('头部实际位置/速度未在3s内到位')
            start_reference = target
        action_q = np.asarray(samples['q'])[np.char.startswith(np.asarray(samples['phase']), 'head_'), head]
        if (not len(action_q) or np.ptp(action_q) < .016
                or abs(float(last['q'][head]) - original_target) >= .002):
            raise RuntimeError('实际幅度/回位没有证明完整轨迹')
        report['actual_trajectory_completed'] = True
        report['diagnostic_completed'] = True
        phase = 'complete'
    except Exception as error:
        report.update(error=str(error), error_type=type(error).__name__)
    finally:
        if unsubscribe is not None:
            unsubscribe()
        if initially_paused:
            world.pause()
        report['head_safe_hold_target'] = None
        report['head_safe_hold_written'] = False
        report['originaltarget_pending_return'] = False
        final_head_actual = None
        if baseline is not None:
            try:
                final = _read_fixed(art)
                preserved = _preserved(baseline, final, head)
                completed = (report['diagnostic_completed'] and report['actual_trajectory_completed']
                             and all(preserved.values())
                             and final['position_targets'][head] == original_target)
                measured = np.asarray(art.get_joint_positions()).reshape(-1)
                if measured.shape == (50,) and math.isfinite(float(measured[head])):
                    final_head_actual = float(measured[head])
                if head_written and not completed:
                    report['originaltarget_pending_return'] = True
                    # 失败后仅在暂停状态保持可信当前角；不把原目标阶跃回去制造新激励。
                    if (final_head_actual is None or not limits[head, 0] <= final_head_actual <= limits[head, 1]):
                        report['head_safe_hold_error'] = '暂停后的实际头角非有限/超硬限，未写任何目标'
                    else:
                        hold = float(np.float32(final_head_actual))
                        report['head_safe_hold_target'] = hold
                        if final['position_targets'][head] != hold:
                            art.set_joint_position_targets(np.array([[hold]], dtype=np.float32), joint_indices=[head])
                            report['head_safe_hold_written'] = True
                        report['originaltarget_pending_return'] = hold != original_target
                        report['pending_return_requires'] = '另一次明确、有界、平滑的单头关节返回；本诊断不自动恢复或扳物理状态'
                final = _read_fixed(art)
                report['preserved'] = _preserved(baseline, final, head)
                report['original_head_target_restored'] = bool(final['position_targets'][head] == original_target)
                if completed and not report['original_head_target_restored']:
                    report['diagnostic_completed'] = False
                if not all(report['preserved'].values()):
                    report['diagnostic_completed'] = False
            except Exception as error:
                report['diagnostic_completed'] = False
                report['stop_error'] = str(error)
                report['originaltarget_pending_return'] = bool(head_written)
        report['head_actual_position_after_pause_rad'] = final_head_actual
        report['phase'], report['final_paused'] = phase, not world.is_playing()
        report['all_recorded_frames_have_fk_coverage'] = bool(samples['t'] and len(samples['expected_links']) == len(samples['t']))
        report['pose_envelope_passed'] = bool(report['motion_pose_passed'] is True and
            report['all_recorded_frames_have_fk_coverage'] and not pose_failure)
        if samples['q']:
            q, v, t = np.asarray(samples['q']), np.asarray(samples['v']), np.asarray(samples['t'])
            mask = np.char.startswith(np.asarray(samples['phase']), 'head_')
            span = float(np.ptp(q[mask, head])) if mask.any() else 0.
            return_error = None if final_head_actual is None else abs(final_head_actual - original_target)
            report.update(actual_head_motion_span_rad=span, actual_head_return_error_rad=return_error,
                          raw_sdk_velocity_absolute_peak=np.max(abs(v), axis=0).tolist())
            report['residual_peaks'] = {key: float(np.max(np.asarray(samples[key]))) for key in
                ('max_body_position_error_m', 'max_body_angle_error_deg', 'max_shape_corner_error_m',
                 'world_body_position_error_m', 'world_body_raw_full_angle_error_deg',
                 'world_shape_geometry_error_bound_m', 'world_shape_raw_corner_error_m',
                 'root_position_error_m', 'root_full_angle_error_deg') if samples[key]}
            if report['all_recorded_frames_have_fk_coverage'] and mask.any():
                report['motion_residual_peaks'] = {key: float(np.max(np.asarray(samples[key])[mask])) for key in
                    ('max_body_position_error_m', 'max_body_angle_error_deg', 'max_shape_corner_error_m',
                     'root_position_error_m', 'root_full_angle_error_deg')}
            if report['actual_trajectory_completed'] and (span < .016 or return_error is None or return_error >= .002):
                report.update(diagnostic_completed=False, actual_trajectory_completed=False,
                              error='实际幅度/回位没有证明完整轨迹')
            arrays = {key: np.asarray(value) for key, value in samples.items()}
            arrays['discrete_dqdt'] = np.diff(q, axis=0) / np.diff(t)[:, None]
            arrays['joint_names'] = np.asarray(names)
            arrays['body_paths'] = np.asarray(paths)
            report['discrete_dqdt_absolute_peak'] = np.max(abs(arrays['discrete_dqdt']), axis=0).tolist() if len(t) > 1 else None
            np.savez_compressed(output / 'samples.npz', **arrays)
        report['sample_count'], report['duration_physics_s'] = len(samples['t']), elapsed
        (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return report
