"""仿真状态重置：先只读预览，必须另有用户明确授权才可恢复运输初态。

这不是机器人走位/脱困轨迹，不能把重置当成实际运动成功；不重载机器人或reset整个场景。
"""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def source_state(path, names):
    path = Path(path)
    data = json.loads(path.read_text())
    state = data['state']
    source_names = state['joint_names']
    if len(source_names) != len(set(source_names)) or set(names) != set(source_names):
        raise ValueError('运输初态与当前机器人关节身份不匹配')
    base = data.get('base_pose') or data['base_poses']['base_link']
    p = np.asarray(base['position'], float)
    quaternion = np.asarray(base['orientation'], float)
    if (p.shape != (3,) or quaternion.shape != (4,) or not np.isfinite(p).all()
            or not np.isfinite(quaternion).all() or np.linalg.norm(quaternion) < 1e-12):
        raise ValueError('初态位姿无效')
    root = np.eye(4)
    root[:3, :3] = Rotation.from_quat(quaternion / np.linalg.norm(quaternion), scalar_first=True).as_matrix()
    root[:3, 3] = p
    # 重置到原先已发过的运输目标，不能把带重力偏差的全身反馈当成新目标。
    targets = np.asarray([state['cmd_positions'][source_names.index(n)] for n in names], float)
    if targets.shape != (len(names),) or not np.isfinite(targets).all():
        raise ValueError('初态指令目标无效')
    return {'root': root, 'q': targets, 'targets': targets.copy(),
            'source': str(path.resolve()), 'source_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'pose_policy': '原始同批base实测位姿与已下发运输目标，新的静止初态；非实际走位'}


def preview(safety, source, fixture_delta=(-2.3, -.8, 0.)):
    """仅本地FK/精确距离，保留原柜/板/支腿；把本工具新增工位移远腾通道。"""
    candidate = source_state(source, safety.model.names)
    poses = safety.predict(candidate['q'], candidate['root'])
    for path, matrix in poses.items():
        if path.startswith('/World/ValveStation/') or path.startswith('__static__:/World/ValveStation/'):
            poses[path] = matrix.copy()
            poses[path][:3, 3] += np.asarray(fixture_delta, float)
    actual = safety.check_configuration(q=candidate['q'], poses=poses, environment_margin_m=.012)
    # 父类check()的报告依赖在线world且标记actual_physics_pose；这里明确只做离线候选查询。
    groups = ('left_arm', 'left_hand', 'right_arm', 'right_hand')
    coverage = {group: safety._groups.count(group) for group in groups}
    if any(count == 0 for count in coverage.values()):
        raise RuntimeError('运输候选缺少手臂/夹爪碰撞几何')
    nearest, calls = safety._nearest(poses, safety._external_pairs, groups)
    rows = [row for row in nearest.values() if row is not None]
    hands = {'safe': len(rows) == len(groups) and all(row['distance_m'] >= .05 for row in rows),
             'minimum_distance_m': min((row['distance_m'] for row in rows), default=None),
             'nearest_by_group': nearest, 'min_clearance_m': .05,
             'group_shape_coverage': coverage, 'coal_distance_calls': calls,
             'actual_physics_pose': False, 'offline_candidate': True}
    owned = [i for i, shape in enumerate(safety.model.shapes) if '/World/ValveStation/' in shape['path']]
    original = [i for i, shape in enumerate(safety.model.shapes)
                if i not in owned and not shape['path'].startswith('/World/ground_plane/')]
    fixture_pairs = np.asarray([(i, j) for i in owned for j in original], dtype=int).reshape(-1, 2)
    fixture_nearest, _ = safety._nearest(poses, fixture_pairs, (None,))
    fixture_row = fixture_nearest[None]
    fixture_safe = bool(fixture_row is not None and fixture_row['distance_m'] >= .012)
    return {'safe': bool(actual['safe'] and hands['safe'] and fixture_safe), 'state_write_performed': False,
            'source': candidate['source'], 'source_sha256': candidate['source_sha256'],
            'root': candidate['root'].tolist(), 'q': candidate['q'].tolist(),
            'targets': candidate['targets'].tolist(), 'names': list(safety.model.names),
            'pose_policy': candidate['pose_policy'], 'geometry': actual, 'arms_and_hands': hands,
            'relocated_fixture_vs_original_geometry': {'safe': fixture_safe, 'nearest': fixture_row},
            'global_native_self_collision_free_claimed': False,
            'requires_separate_state_reset_authorization': True,
            'fixture_policy': '仅将本轮新增空载ValveStation移远，原障碍与全部碰撞不改'}


def apply(executor, safety, source, output, *, confirm_state_reset=False, fixture_already_prepared=False):
    """须确认且已暂停物理；只恢复当前机器人，保留其他机构。默认绝不写状态。"""
    if confirm_state_reset is not True:
        raise RuntimeError('没有本轮单独的状态重置授权，禁止把走位许可当重置许可')
    if executor.world.is_playing():
        raise RuntimeError('必须先暂停物理并保存状态，不能在运动中重置')
    if getattr(executor, '_base_cmd', None) is not None or getattr(executor, '_motion', None) is not None:
        raise RuntimeError('仍有控制任务，不能重置')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    check = preview(safety, source, fixture_delta=(0., 0., 0.) if fixture_already_prepared else (-2.3, -.8, 0.))
    if not check['safe']:
        raise RuntimeError('参考运输初态精确避碰预览未通过')
    (output / 'preview.json').write_text(json.dumps(check, ensure_ascii=False, indent=2))
    art = executor.articulation
    before = {'q': np.asarray(art.get_joint_positions()).copy(),
              'v': np.asarray(art.get_joint_velocities()).copy(),
              'root': np.asarray(art._physics_view.get_root_transforms()).copy(),
              'root_v': np.asarray(art.get_velocities()).copy(),
              'targets': np.asarray(art._physics_view.get_dof_position_targets()).copy(),
              'velocity_targets': np.asarray(art._physics_view.get_dof_velocity_targets()).copy(),
              'kp': np.asarray(art.get_gains()[0]).copy(), 'kd': np.asarray(art.get_gains()[1]).copy(),
              'max_force': np.asarray(art._physics_view.get_dof_max_forces()).copy()}
    if (not np.allclose(before['q'].reshape(-1), safety.q, atol=1e-6, rtol=0)
            or not np.allclose(before['velocity_targets'], 0., atol=0., rtol=0)):
        raise RuntimeError('当前状态已变或仍有速度目标，禁止使用旧候选重置')
    np.savez_compressed(output / 'before_reset.npz', **before)
    # 只有用户已确认后才移动本工具的空载夹具；所有原有场景对象保持原位。
    from valve_station import relocate_empty, rebind_existing_views
    if not fixture_already_prepared:
        relocate_empty(executor, [-2., -2., .84], output / 'fixture_relocation',
            '/home/jy/datasets/vla_data/L2_仿真运行/场景/操作台_piper_table_v2/asserts/valve/valve_1/valve_asset.usd')
        # 新prim在首次物理步才实例化；须先完成它，避免重建时从旧USD覆盖tensor恢复值。
        try:
            executor.world.play()
            executor.step(render=False)
        finally:
            executor.world.pause()
        rebind_existing_views(executor)
        from valve_safety import ValveSafety
        safety = ValveSafety(executor, output / 'geometry_after_fixture_bootstrap')
        check = preview(safety, source, fixture_delta=(0., 0., 0.))
        (output / 'preview_after_fixture_bootstrap.json').write_text(json.dumps(check, ensure_ascii=False, indent=2))
        if not check['safe']:
            raise RuntimeError('工位实例化后运输候选不再安全，保持暂停')
    art = executor.articulation
    candidate = source_state(source, list(art.dof_names))
    view = art._physics_view
    root = candidate['root']
    transform = np.r_[root[:3, 3], Rotation.from_matrix(root[:3, :3]).as_quat()].reshape(1, 7).astype(np.float32)
    indices = np.array([0], dtype=np.uint32)
    # 在play已恢复且场景已实例化后写入，不在暂停缓存里伪验证。主线程尚未推进物理。
    executor.world.play()
    try:
        view.set_root_transforms(transform, indices)
        view.set_root_velocities(np.zeros((1, 6), dtype=np.float32), indices)
        view.set_dof_positions(candidate['q'].reshape(1, -1).astype(np.float32), indices)
        view.set_dof_velocities(np.zeros((1, len(candidate['q'])), dtype=np.float32), indices)
        view.set_dof_position_targets(candidate['targets'].reshape(1, -1).astype(np.float32), indices)
        view.set_dof_velocity_targets(before['velocity_targets'].astype(np.float32), indices)
        executor._mimic_targets = None
        executor._swerve_drive_state, executor._swerve_stop_window = None, None
        executor._wheel_pos_target, executor._swerve_last_stop = None, None
        if (not np.allclose(np.asarray(view.get_root_transforms()), transform, atol=1e-6, rtol=0)
                or not np.allclose(np.asarray(view.get_dof_positions()).reshape(-1), candidate['q'], atol=1e-6, rtol=0)
                or not np.allclose(np.asarray(view.get_dof_position_targets()).reshape(-1), candidate['targets'], atol=1e-6, rtol=0)):
            raise RuntimeError('运输初态写入后读回不符，保持暂停，禁止继续')
        executor.step(render=False)
        observed = np.asarray(view.get_root_transforms()).copy()
        actual_root_position_error = float(np.linalg.norm(observed[0, :3] - transform[0, :3]))
        actual_root_angle_error = float((Rotation.from_quat(observed[0, 3:])
                                        * Rotation.from_quat(transform[0, 3:]).inv()).magnitude())
        actual_q = np.asarray(view.get_dof_positions()).copy()
        targets_preserved = bool(np.allclose(np.asarray(view.get_dof_position_targets()).reshape(-1),
                                            candidate['targets'], atol=1e-6, rtol=0))
        (output / 'first_physical_frame.json').write_text(json.dumps({
            'actual_root_transform_xyzw': observed.tolist(), 'requested_root_transform_xyzw': transform.tolist(),
            'actual_position_error_m': actual_root_position_error,
            'actual_orientation_error_rad': actual_root_angle_error,
            'all_position_targets_preserved': targets_preserved,
            'actual_q': actual_q.tolist(),
            'actual_v': np.asarray(view.get_dof_velocities()).tolist()}, ensure_ascii=False, indent=2))
        if (not np.isfinite(actual_q).all() or actual_root_position_error > .001
                or actual_root_angle_error > np.deg2rad(.1) or not targets_preserved):
            raise RuntimeError('首物理帧未保留运输恢复位姿，保持暂停，不能宣称恢复完成')
    finally:
        executor.world.pause()
    if (not np.array_equal(np.asarray(art.get_gains()[0]), before['kp'])
            or not np.array_equal(np.asarray(art.get_gains()[1]), before['kd'])
            or not np.array_equal(np.asarray(view.get_dof_max_forces()), before['max_force'])):
        raise RuntimeError('重置意外改变了驱动增益，保持暂停，需诊断')
    if hasattr(executor, '_arm_transit_safety'):
        executor._arm_transit_safety = None
    owner = getattr(executor, '_l3_station', None)
    if owner is not None:
        owner._approach = None
    result = {'state_reset_performed': True, 'actual_trajectory_executed': False,
              'physics_still_paused': not executor.world.is_playing(), 'scene_reset': False,
              'robot_reloaded': False, 'other_scene_actors_set': False,
              'gain_solver_limit_friction_changes': False,
              'first_actual_physics_frame_verified': True,
              'first_frame_root_position_error_m': actual_root_position_error,
              'first_frame_root_orientation_error_rad': actual_root_angle_error,
              'source': candidate['source'], 'source_sha256': candidate['source_sha256'],
              'acceptance_pending': '必须恢复物理并唤醒/实际动作后复测，不以刚重置的零速度当修复'}
    (output / 'reset_result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    return result
