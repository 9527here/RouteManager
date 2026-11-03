"""运输姿态恢复后的单头关节实际动作复测；仅由已有仿真主线程显式调用。

初始必须暂停。只写 head_yaw 一个位置目标；不修改物理状态、增益、速度目标或 sleep。
阈值是本次仿真验收参考，不是硬件规格。失败时暂停，恢复原头部目标但不强行回位。
"""
import json
import math
from pathlib import Path
import threading

import numpy as np
from scipy.spatial.transform import Rotation


def _read_fixed(art):
    view = art._physics_view
    k, d = art.get_gains()
    return {'position_targets': np.asarray(view.get_dof_position_targets()).reshape(-1).copy(),
            'velocity_targets': np.asarray(view.get_dof_velocity_targets()).reshape(-1).copy(),
            'kp': np.asarray(k).reshape(-1).copy(), 'kd': np.asarray(d).reshape(-1).copy(),
            'max_force': np.asarray(view.get_dof_max_forces()).reshape(-1).copy()}


def _preserved(before, after, head_index):
    other = [i for i in range(len(before['position_targets'])) if i != head_index]
    return {'other_49_position_targets': bool(np.array_equal(before['position_targets'][other], after['position_targets'][other])),
            **{name: bool(np.array_equal(before[name], after[name]))
               for name in ('velocity_targets', 'kp', 'kd', 'max_force')}}


def _preflight(executor, output, original_target, head_index):
    from valve_safety import ValveSafety
    safety = ValveSafety(executor, directory=output, expected_robot_shapes=96)
    current, _ = safety.refresh()
    q_path = [current.copy()]
    for target in (original_target + .01, original_target - .01, original_target):
        start = q_path[-1][head_index]
        count = max(1, int(math.ceil(abs(target - start) / .002)))
        for value in np.linspace(start, target, count + 1)[1:]:
            q = current.copy()
            q[head_index] = value
            q_path.append(q)
    path = safety.check_path(q_path, environment_margin_m=.012, self_margin_m=.008,
                             surface_step_m=.002, joint_step_rad=.002)
    arms = [safety.check(min_clearance_m=.05, refresh=False, poses=safety.predict(q)) for q in q_path]
    report = {'safe': bool(path['safe'] and all(row['safe'] for row in arms)),
              'whole_body': path, 'arms': arms, 'joint_name': 'head_yaw_joint',
              'amplitude_rad': .01, 'assembly_exemptions_used': False}
    (Path(output) / 'head_motion_preflight.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    if not report['safe']:
        raise RuntimeError('96形状连续扫掠12mm/自碰撞8mm/双臂50mm未通过')
    return report


def _subscribe(callback):
    from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
    token = SimulationManager.register_callback(callback, SimulationEvent.PHYSICS_POST_STEP)
    return lambda: SimulationManager.deregister_callback(token)


def verify_head_motion(executor, output):
    """显式复测 head_yaw ±.01rad；任何动作期越界立即暂停并返回失败报告。"""
    from physics_wake import wake_if_sleeping
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    art, world = executor.articulation, executor.world
    names, bodies = list(art.dof_names), list(art.body_names)
    report = {'success': False, 'motion_started': False, 'head_joint': 'head_yaw_joint',
              'reference_thresholds': {'root_displacement_m': .001, 'root_yaw_deg': .1,
                  'head_tracking_rad': .02, 'nonhead_velocity': .03, 'head_arrival_rad': .002,
                  'head_arrival_velocity_rad_s': .02, 'wake_stable_dwell_s': 2.0,
                  'arrival_stable_dwell_s': .25,
                  'wake_settle_timeout_s': 6., 'arrival_timeout_s': 3., 'phase_duration_s': 2.},
              'calibrated_to_hardware': False, 'command_log': [], 'phase': 'initial_paused'}
    samples = {key: [] for key in ('t', 'q', 'v', 'links', 'root_v', 'head_reference', 'phase')}
    elapsed, phase, desired = 0., 'initial_paused', None
    callback_error = None
    unsubscribe = None
    baseline, head, original_target = None, None, None
    owner = threading.get_ident()
    root_reference = None
    last_sample = None
    moving = False
    wake_stable_since = None
    wake_ready = False
    wake_window = []
    initially_paused = not world.is_playing()

    def collect(dt, context=None):
        nonlocal elapsed, callback_error, last_sample, wake_stable_since, wake_ready
        try:
            if threading.get_ident() != owner:
                raise RuntimeError('复测物理回调不在原主线程')
            if not math.isfinite(dt) or dt <= 0:
                raise RuntimeError('物理步长无效')
            elapsed += float(dt)
            q = np.asarray(art.get_joint_positions()).reshape(-1).copy()
            v = np.asarray(art.get_joint_velocities()).reshape(-1).copy()
            links = np.asarray(art._physics_view.get_link_transforms()).reshape(-1, 7).copy()
            root_v = np.asarray(art.get_velocities()).reshape(-1).copy()
            if not all(np.isfinite(a).all() for a in (q, v, links, root_v)):
                raise RuntimeError('实测物理状态非有限')
            root = links[bodies.index('base_link')]
            yaw = float(Rotation.from_quat(root[3:]).as_euler('xyz')[2])
            last_sample = {'q': q, 'v': v, 'root': root, 'yaw': yaw}
            for key, value in (('t', elapsed), ('q', q), ('v', v), ('links', links),
                               ('root_v', root_v), ('head_reference', desired), ('phase', phase)):
                samples[key].append(value)
            preserved = _preserved(baseline, _read_fixed(art), head)
            if not all(preserved.values()):
                raise RuntimeError('非头部目标/速度目标/增益/力矩上限被改变: ' + repr(preserved))
            if phase == 'wake_settle':
                # 每一个物理步都重置不稳定窗口，不能只检查一次外层step返回的末帧。
                wake_window.append((elapsed, root[:3].copy(), yaw))
                while len(wake_window) > 1 and wake_window[1][0] <= elapsed - .25:
                    wake_window.pop(0)
                span_ready = (elapsed - wake_window[0][0] >= .25 - 1e-9 and
                              np.ptp(np.array([r[1] for r in wake_window]), axis=0).max() < .001 and
                              np.ptp(np.unwrap([r[2] for r in wake_window])) < math.radians(.1))
                velocity_ready = np.max(abs(np.delete(v, head))) < .03
                wake_stable_since = ((elapsed if wake_stable_since is None else wake_stable_since)
                                     if span_ready and velocity_ready else None)
                # 唤醒暂态中出现过短安静窗；连续2s速度门通过才开始动作，
                # 与实际头关节到位的0.25s dwell分开，速度/根位姿阈值不放宽。
                wake_ready = wake_stable_since is not None and elapsed - wake_stable_since >= 2.0
            if moving:
                p0, yaw0 = root_reference
                drift = float(np.linalg.norm(root[:3] - p0))
                yaw_drift = abs((yaw - yaw0 + math.pi) % (2 * math.pi) - math.pi)
                other = np.delete(v, head)
                if drift > .001 or yaw_drift > math.radians(.1):
                    raise RuntimeError(f'动作期底盘偏移超限: {drift*1000:.3f}mm/{math.degrees(yaw_drift):.4f}deg')
                if abs(float(q[head]) - desired) > .02:
                    raise RuntimeError('动作期实际头关节跟踪误差超过.02rad')
                if np.max(abs(other)) > .03:
                    raise RuntimeError('动作期非头关节实际速度超过.03参考阈值')
        except Exception as error:
            if callback_error is None:
                callback_error = error
            world.pause()

    def step():
        before = elapsed
        executor.step(render=(len(samples['t']) % 8 == 0))
        if callback_error is not None:
            raise callback_error
        if elapsed <= before or last_sample is None:
            raise RuntimeError('物理步回调没有推进，拒绝伪造动作记录')

    def command(value):
        nonlocal desired
        desired = float(value)
        art.set_joint_position_targets(np.array([[desired]], dtype=np.float32), joint_indices=[head])
        report['motion_started'] = True

    try:
        if not initially_paused or (hasattr(world, 'is_stopped') and world.is_stopped()):
            raise RuntimeError('必须从物理有效的paused状态开始，不能抢占运行任务或STOP状态')
        from valve_runtime import _competing_controllers
        _competing_controllers(executor)
        if len(names) != 50 or 'head_yaw_joint' not in names or 'base_link' not in bodies:
            raise RuntimeError('仅支持当前50DOF整机及真实base_link反馈')
        head = names.index('head_yaw_joint')
        baseline = _read_fixed(art)
        if not all(a.shape == (50,) and np.isfinite(a).all() for a in baseline.values()):
            raise RuntimeError('原目标/增益/力矩备份无效')
        original_target = float(baseline['position_targets'][head])
        desired = original_target
        limits = np.asarray(art.get_dof_limits()).reshape(-1, 2)[head]
        if not limits[0] <= original_target - .01 < original_target + .01 <= limits[1]:
            raise RuntimeError('头关节小幅目标超出实际限位')
        report['original_head_target_rad'] = original_target
        report['initial_preflight'] = _preflight(executor, output / 'paused_geometry', original_target, head)
        unsubscribe = _subscribe(collect)
        phase = 'wake_settle'
        report['phase'] = phase
        world.play()
        report['wake'] = wake_if_sleeping(executor)
        if not report['wake'].get('supported') or report['wake'].get('sleep_after') is not False:
            raise RuntimeError('原生唤醒不可用，拒绝速度状态setter fallback')
        while True:
            step()
            if wake_ready:
                break
            if elapsed > 6.:
                raise RuntimeError('原生唤醒后6s内未实测停稳，未开始头部动作')
        report['wake_settle_duration_s'] = elapsed
        report['wake_transient_separately_recorded'] = True
        report['post_wake_preflight'] = _preflight(executor, output / 'settled_geometry', original_target, head)
        root_reference = (last_sample['root'][:3].copy(), last_sample['yaw'])
        moving = True
        # 从已有指令目标起步，不能把有静差的实测角回灌成首帧目标跳变。
        start_reference = original_target
        for label, target in (('head_positive', original_target + .01),
                              ('head_negative', original_target - .01), ('head_return', original_target)):
            phase, report['phase'] = label, label
            begin = elapsed
            report['command_log'].append({'phase': phase, 'start_time': begin, 'target_rad': target})
            while elapsed - begin < 2.:
                u = min(1., (elapsed - begin) / 2.)
                command(start_reference + (target - start_reference) * u * u * (3 - 2 * u))
                step()
            command(target)
            arrival_start, dwell = elapsed, None
            while True:
                step()
                ready = (abs(float(last_sample['q'][head]) - target) < .002 and
                         abs(float(last_sample['v'][head])) < .02)
                dwell = (elapsed if dwell is None else dwell) if ready else None
                if dwell is not None and elapsed - dwell >= .25:
                    break
                if elapsed - arrival_start > 3.:
                    raise RuntimeError('实际头关节未在3s内到位，拒绝进入下一段')
            start_reference = target
        report['success'], report['phase'] = True, 'complete'
    except Exception as error:
        report.update(success=False, error=str(error), error_type=type(error).__name__, phase=phase)
    finally:
        if unsubscribe is not None:
            unsubscribe()
        if initially_paused:
            world.pause()
        if original_target is not None and report['motion_started']:
            # 失败时只恢复一个原目标；保持暂停，绝不强行扳回实际关节角。
            art.set_joint_position_targets(np.array([[original_target]], dtype=np.float32), joint_indices=[head])
        if baseline is not None:
            final = _read_fixed(art)
            report['preserved'] = _preserved(baseline, final, head)
            report['original_head_target_restored'] = bool(final['position_targets'][head] == baseline['position_targets'][head])
            if not all(report['preserved'].values()) or not report['original_head_target_restored']:
                report['success'] = False
        report['final_paused'] = not world.is_playing()
        if samples['q']:
            q = np.asarray(samples['q'])
            phase_array = np.asarray(samples['phase'])
            action_rows = phase_array != 'wake_settle'
            report['actual_head_range_rad'] = [float(q[:, head].min()), float(q[:, head].max())]
            report['actual_head_motion_span_rad'] = (float(np.ptp(q[action_rows, head])) if action_rows.any() else 0.)
            report['actual_head_return_error_rad'] = abs(float(q[-1, head]) - original_target)
            if report['success'] and (report['actual_head_motion_span_rad'] < .016 or report['actual_head_return_error_rad'] >= .002):
                report.update(success=False, error='实测头关节未证明完整往返/回位')
            np.savez_compressed(output / 'samples.npz', **{k: np.asarray(v) for k, v in samples.items()})
        report['sample_count'] = len(samples['t'])
        report['duration_physics_s'] = elapsed
        (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return report
