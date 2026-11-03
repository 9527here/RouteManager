"""只读测量真实 PhysX 抖动；不写目标、关节状态、休眠或求解器参数。"""
import json
from pathlib import Path
import time
import traceback
import threading

import numpy as np
from scipy.spatial.transform import Rotation


def measure(executor, output, steps=1440, render_every=8, physics_step_sampling=False):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    art = executor.articulation
    if getattr(executor, '_base_cmd', None) is not None or getattr(executor, '_motion', None) is not None:
        raise RuntimeError('还有运动指令，不能把运动数据算作待机抖动')
    station = getattr(executor, '_l3_station', None)
    approach = getattr(station, '_approach', None)
    if station and (getattr(station, '_robot_busy', False) or getattr(station, 'queue', ()) or
                    (approach is not None and approach.busy)):
        raise RuntimeError('其他工位任务未停止')
    flags = {'base_cmd': repr(getattr(executor, '_base_cmd', None)),
             'motion': repr(getattr(executor, '_motion', None)),
             'auto_pick': repr(getattr(executor, '_auto_pick', None)),
             'velocity_targets': np.asarray(art._physics_view.get_dof_velocity_targets()).tolist(),
             'position_targets': np.asarray(art._physics_view.get_dof_position_targets()).tolist(),
             'gains': [np.asarray(v).tolist() for v in art.get_gains()],
             'physics_dt': float(executor.world.get_physics_dt()),
             'schema_note': '观测只推进原物理，不修改任何目标、速度状态、schema或sleep'}
    render_step = lambda frame: bool(render_every and frame % render_every == 0)
    originals, calls, traces = {}, {}, []
    for name in ('set_joint_position_targets', 'set_joint_velocity_targets', 'set_joint_velocities', 'apply_action'):
        original = getattr(art, name)
        instance = getattr(art, '__dict__', {})
        originals[name] = (name in instance, instance.get(name))
        calls[name] = 0
        def wrapper(*args, _original=original, _name=name, **kwargs):
            calls[_name] += 1
            if len(traces) < 16:
                traces.append({'method': _name, 'joint_indices': repr(kwargs.get('joint_indices')),
                               'stack': traceback.format_stack(limit=5)[:-1]})
            return _original(*args, **kwargs)
        setattr(art, name, wrapper)
    times, joints, velocities, links, root_v, efforts = [], [], [], [], [], []
    physics_dts, raw_world_times = [], []
    effort_error = None
    started = time.monotonic()
    owner_thread = threading.get_ident()
    callback_error, callback_id = None, None
    simulation_start = float(executor.world.current_time)
    accumulated_dt = 0.

    def collect(step_dt=None, context=None):
        nonlocal effort_error, accumulated_dt, callback_error
        if threading.get_ident() != owner_thread:
            callback_error = RuntimeError('物理回调未在当前主线程，拒绝跨线程访问 Isaac')
            return
        try:
            raw_world_times.append(float(executor.world.current_time))
            if physics_step_sampling:
                if not np.isfinite(step_dt) or step_dt <= 0:
                    raise RuntimeError('物理步长无效')
                accumulated_dt += float(step_dt)
                physics_dts.append(float(step_dt))
                times.append(simulation_start + accumulated_dt)
            else:
                times.append(raw_world_times[-1])
            joints.append(np.asarray(art.get_joint_positions()).reshape(-1).copy())
            velocities.append(np.asarray(art.get_joint_velocities()).reshape(-1).copy())
            links.append(np.asarray(art._physics_view.get_link_transforms()).reshape(-1, 7).copy())
            root_v.append(np.asarray(art.get_velocities()).reshape(-1).copy())
            if effort_error is None:
                try:
                    effort = np.asarray(art.get_measured_joint_efforts()).reshape(-1).copy()
                    if effort.shape != joints[-1].shape or not np.isfinite(effort).all():
                        raise ValueError('求解器关节力矩反馈无效')
                    efforts.append(effort)
                except Exception as error:
                    effort_error = str(error)
        except Exception as error:
            callback_error = error

    try:
        if physics_step_sampling:
            from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
            callback_id = SimulationManager.register_callback(collect, SimulationEvent.PHYSICS_POST_STEP)
            frame = 0
            while len(times) < steps:
                executor.step(render=render_step(frame))
                frame += 1
                if callback_error is not None:
                    raise callback_error
                if frame > steps * 2:
                    raise RuntimeError('逐物理步回调未产生足够样本')
        else:
            for frame in range(steps):
                executor.step(render=render_step(frame))
                if frame % 2 == 0:
                    collect()
                if callback_error is not None:
                    raise callback_error
    finally:
        if callback_id is not None:
            SimulationManager.deregister_callback(callback_id)
        for name, (had_instance_value, instance_value) in originals.items():
            if had_instance_value:
                setattr(art, name, instance_value)
            elif name in getattr(art, '__dict__', {}):
                # 原来来自class descriptor的方法不能留下旧bound-method实例shadow，
                # 否则后续热更新class方法会被这次只读采样遮蔽。
                delattr(art, name)
    q, v, world, t = map(np.asarray, (joints, velocities, links, times))
    if not all(np.isfinite(a).all() for a in (q, v, world, t)):
        raise RuntimeError('抖动观测包含非有限物理状态')
    tc = t - t.mean()
    def stats(values, scale=1.):
        x = np.asarray(values).reshape(len(t), -1)
        centered = x - x.mean(0)
        trend = tc[:, None] * (tc @ centered / (tc @ tc))[None, :]
        return {'span': (np.ptp(x, axis=0) * scale).tolist(),
                'detrended_std': ((centered - trend).std(0) * scale).tolist(),
                'drift': ((x[-1] - x[0]) * scale).tolist()}
    metrics = stats(q, 1000.)
    top = np.argsort(metrics['detrended_std'])[-12:][::-1]
    body_names, joint_names = list(art.body_names), list(art.dof_names)
    types = list(art.get_dof_types())
    units = ['mm' if 'translation' in str(kind).lower() else 'mrad' for kind in types]
    body_rows = []
    for name in ('base_link', 'torso_pitch_link', 'head_yaw_link', 'head_pitch_link',
                 'left_arm_6_link', 'right_arm_6_link', 'right_gripper_base_link'):
        if name not in body_names:
            continue
        rows = world[:, body_names.index(name)]
        angles = np.unwrap(Rotation.from_quat(rows[:, 3:]).as_euler('xyz'), axis=0)
        body_rows.append({'link': name, 'position_mm': stats(rows[:, :3], 1000.),
                          'roll_pitch_yaw_deg': stats(angles, 180 / np.pi)})
    report = {'duration_sim_s': float(t[-1] - t[0]), 'duration_wall_s': time.monotonic() - started,
              'samples': len(t), 'flags': flags, 'command_calls': calls, 'command_traces': traces,
              'top_joints': [{'name': joint_names[i], 'unit': units[i],
                              'span_scaled': metrics['span'][i],
                              'detrended_std_scaled': metrics['detrended_std'][i],
                              ('span_mm' if units[i] == 'mm' else 'span_mrad'): metrics['span'][i],
                              ('peak_velocity_m_s' if units[i] == 'mm' else 'peak_velocity_rad_s'):
                                  float(np.max(np.abs(v[:, i])))} for i in top],
              'links': body_rows, 'all_joint_metrics': metrics,
              'all_joint_metric_units': units,
              'all_joint_names': joint_names, 'all_body_names': body_names,
              'base_root_velocity_peak': np.max(np.abs(np.asarray(root_v)), axis=0).tolist(),
              'method': 'simultaneous physical tensor q/v/link transforms; not authored USD',
              'motor_commands_added_by_measurement': False}
    report['physics_step_sampling'] = bool(physics_step_sampling)
    report['render_every'] = render_every
    report['time_source'] = 'post_physics_callback_accumulated_dt' if physics_step_sampling else 'world_time_at_outer_call'
    report['simulation_start_time'] = simulation_start
    report['simulation_end_time'] = float(executor.world.current_time)
    report['callback_accumulated_dt'] = accumulated_dt if physics_step_sampling else None
    report['callback_main_thread_verified'] = True
    report['measured_joint_efforts_available'] = effort_error is None and len(efforts) == len(t)
    report['measured_joint_efforts_error'] = effort_error
    report['effort_note'] = '求解器投影关节力矩，不能直接等同驱动器电机力矩或用它证明限力'
    arrays = dict(q=q, v=v, links=world, t=t, root_v=np.asarray(root_v))
    if physics_step_sampling:
        arrays.update(physics_dt=np.asarray(physics_dts), raw_world_time=np.asarray(raw_world_times))
    if report['measured_joint_efforts_available']:
        arrays['measured_joint_efforts'] = np.asarray(efforts)
    np.savez_compressed(output / 'samples.npz', **arrays)
    (output / 'measurement.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    return report
