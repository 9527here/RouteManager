"""RM65 人形四轮速度驱动的显式会话参考阻尼；导入时不启用。

只允许已运行的单台 50DOF RM65 整机，四轮 force 速度驱动 kp=0/maxForce=40。
``apply(executor, output, scale=.1)`` 从首次备份值计算，不累乘；``restore``
精确恢复原运行值和原缓存默认值。仅改四轮正 kd，不写 USD、关节目标、物理状态或 sleep。
这些参数是仿真参考，不是硬件标定。调用方负责碰撞保护及实际动作后的稳定性验收。
"""
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np


WHEELS = ('front_left_wheel_joint', 'rear_left_wheel_joint',
          'front_right_wheel_joint', 'rear_right_wheel_joint')
CASTERS = tuple('caster_' + n.removesuffix('_wheel_joint') + '_joint' for n in WHEELS)
_BACKUP = '_wheel_velocity_stability_reference'


class StabilityReferenceRejected(RuntimeError):
    pass


def _write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))
    temporary.replace(path)


def _hash(names):
    return hashlib.sha256(json.dumps(names, ensure_ascii=False,
                                    separators=(',', ':')).encode()).hexdigest()


def _guard(executor):
    for name in ('_base_cmd', '_motion', '_auto_pick', '_udp_targets', '_ros2_contract'):
        if getattr(executor, name, None) is not None:
            raise StabilityReferenceRejected('运动控制仍激活，不能切换参考阻尼: ' + name)
    parking = getattr(executor, '_base_parking', None)
    if parking is not None and not callable(parking) and bool(parking):
        raise StabilityReferenceRejected('驻车控制仍激活')
    vr = getattr(executor, '_vr_teleop', None)
    if vr is not None and getattr(vr, '_running', False):
        raise StabilityReferenceRejected('VR 控制仍激活')
    ros = getattr(executor, '_ros_controller', None)
    if ros is not None and ros.is_initialized():
        raise StabilityReferenceRejected('ROS 关节控制仍激活')
    for name in ('_l3_station', '_l3station'):
        owner = getattr(executor, name, None)
        if owner is None:
            continue
        approach = getattr(owner, '_approach', None)
        if (getattr(owner, '_robot_busy', False) or getattr(owner, '_pending_action', None) is not None
                or getattr(owner, 'queue', ()) or getattr(owner, '_action_job', None) is not None
                or getattr(owner, '_legacy_job', None) is not None
                or (approach is not None and approach.busy)):
            raise StabilityReferenceRejected('L3/导航控制仍激活')
    if not executor.world.is_playing():
        # set_gains 在 timeline stopped 时可能退回 USD 写入，必须在这里拒绝。
        raise StabilityReferenceRejected('仿真未运行，不能保证 set_gains 只写 runtime')


def _native_drive_types(executor, indices):
    import omni.timeline
    import omni.usd
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import UsdPhysics
    if (omni.timeline.get_timeline_interface().is_stopped() or
            SimulationManager.get_physics_sim_view() is None):
        raise StabilityReferenceRejected('物理后端未运行，禁止落入 USD gain 写入分支')
    art = executor.articulation
    if not art.is_physics_handle_valid():
        raise StabilityReferenceRejected('Articulation 物理句柄无效')
    paths = art._physics_view.dof_paths
    if len(paths) != 1:
        raise StabilityReferenceRejected('只支持单个机器人实例')
    stage = omni.usd.get_context().get_stage()
    result = []
    for index in indices:
        prim = stage.GetPrimAtPath(paths[0][index])
        if not prim or not prim.IsA(UsdPhysics.RevoluteJoint):
            raise StabilityReferenceRejected('轮关节不是有效旋转关节')
        drive = UsdPhysics.DriveAPI(prim, 'angular')
        result.append(str(drive.GetTypeAttr().Get()))
    return result


def _array(value, count, label):
    result = np.asarray(value)
    if result.shape not in ((1, count), (count,)) or not np.isfinite(result).all():
        raise StabilityReferenceRejected(label + ' 维度/数值无效')
    return result.reshape(count).astype(float).copy()


def _read(art):
    names = list(art.dof_names)
    required = [*WHEELS, *CASTERS, 'lift_joint', 'torso_pitch_joint',
                *[f'{side}_arm_{i}_joint' for side in ('left', 'right') for i in range(1, 7)]]
    if (len(names) != 50 or len(set(names)) != 50 or any(n not in names for n in required)
            or any(n in names for n in ('left_wrist_joint', 'right_wrist_joint'))):
        raise StabilityReferenceRejected('仅支持当前 50DOF RM65 六轴双臂四轮八驱整机')
    k, d = art.get_gains()
    view, n = art._physics_view, len(names)
    return {'names': names, 'names_sha256': _hash(names),
            'indices': [names.index(n) for n in WHEELS],
            'kp': _array(k, n, '刚度'), 'kd': _array(d, n, '阻尼'),
            'max_force': _array(view.get_dof_max_forces(), n, '最大力矩'),
            'position_targets': _array(view.get_dof_position_targets(), n, '位置目标'),
            'velocity_targets': _array(view.get_dof_velocity_targets(), n, '速度目标'),
            'cached_kp': _array(art._default_kps, n, '默认刚度缓存'),
            'cached_kd': _array(art._default_kds, n, '默认阻尼缓存')}


def _validate(executor, state):
    indices = state['indices']
    types = _native_drive_types(executor, indices)
    if types != ['force'] * 4:
        raise StabilityReferenceRejected('只支持 force 型轮驱动，实际: ' + repr(types))
    if np.any(state['kp'][indices] != 0.):
        raise StabilityReferenceRejected('轮关节不是 kp=0 的速度驱动')
    if not np.allclose(state['max_force'][indices], 40., rtol=0., atol=1e-6):
        raise StabilityReferenceRejected('轮驱动力矩上限不是已核对的 40 N·m')
    if np.any(state['kd'][indices] <= 0.):
        raise StabilityReferenceRejected('四轮阻尼必须为正')
    if np.any(state['velocity_targets'][indices] != 0.):
        raise StabilityReferenceRejected('仍有非零轮速目标，不能切换参考阻尼')


def _cache_wheels(art, indices, values):
    # public set_gains 会更新选中的 _default_kds。恢复时原缓存可能与原 runtime 不同，
    # 只在这种情况下恢复四项 Python 缓存，不碰其余默认值或物理状态。
    cache = art._default_kds
    if isinstance(cache, np.ndarray):
        cache.reshape(-1)[indices] = np.asarray(values, dtype=cache.dtype)
    else:
        backend = art._backend_utils
        rows = backend.resolve_indices(None, 1, getattr(art, '_device', 'cpu'))
        cols = backend.resolve_indices(indices, len(art.dof_names), getattr(art, '_device', 'cpu'))
        value = backend.convert(np.asarray(values, dtype=np.float32).reshape(1, 4),
                                dtype='float32', device=getattr(art, '_device', 'cpu'))
        art._default_kds = backend.assign(value, cache, [backend.expand_dims(rows, 1), cols])


def _verify(before, after, wheel_kd, cached_wheel_kd):
    if before['names_sha256'] != after['names_sha256'] or before['names'] != after['names']:
        raise StabilityReferenceRejected('操作中机器人关节身份改变')
    indices = before['indices']
    other = [i for i in range(len(before['names'])) if i not in indices]
    for field in ('kp', 'max_force', 'position_targets', 'velocity_targets', 'cached_kp'):
        if not np.array_equal(before[field], after[field]):
            raise StabilityReferenceRejected('公开 gain 调用意外改变 ' + field)
    for field in ('kd', 'cached_kd'):
        if not np.array_equal(before[field][other], after[field][other]):
            raise StabilityReferenceRejected('意外改变非轮关节 ' + field)
    for field, expected in (('kd', wheel_kd), ('cached_kd', cached_wheel_kd)):
        if not np.array_equal(after[field][indices], np.asarray(expected, dtype=float)):
            raise StabilityReferenceRejected('四轮 ' + field + ' 读回与请求不符')


def _set(art, indices, values):
    art.set_gains(kds=np.asarray(values, dtype=np.float32).reshape(1, 4),
                  joint_indices=list(indices), save_to_usd=False)


def _transaction(executor, before, desired, cached_desired, output):
    art, indices = executor.articulation, before['indices']
    try:
        _set(art, indices, desired)
        if cached_desired is not None:
            _cache_wheels(art, indices, cached_desired)
        after = _read(art)
        _verify(before, after, desired, desired if cached_desired is None else cached_desired)
        return after
    except Exception as error:
        rollback_error = None
        try:
            _set(art, indices, before['kd'][indices])
            _cache_wheels(art, indices, before['cached_kd'][indices])
            _verify(before, _read(art), before['kd'][indices], before['cached_kd'][indices])
        except Exception as failure:
            rollback_error = str(failure)
        _write(Path(output) / 'wheel_velocity_stability_failure.json',
               {'error': str(error), 'rollback_verified': rollback_error is None,
                'rollback_error': rollback_error,
                'note': '不写目标来掩盖 gain API 契约异常；若目标读回被改变必须停止并诊断'})
        raise StabilityReferenceRejected('参考阻尼切换失败；' +
                ('已验证回滚: ' if rollback_error is None else '回滚未完整验证: ') + str(error)) from error


def apply(executor, output, scale=.1):
    """显式应用四轮仿真参考 kd；不以当前抖动的非零实测速度拒绝诊断。"""
    is_boolean = isinstance(scale, (bool, np.bool_))
    try:
        scale = float(scale)
    except (TypeError, ValueError) as error:
        raise ValueError('参考比例必须是有限数值') from error
    if is_boolean or not math.isfinite(scale) or not 0. < scale <= 1.:
        raise ValueError('参考比例必须有限且在 (0,1]，不允许零阻尼或隐式增大')
    _guard(executor)
    state = _read(executor.articulation)
    _validate(executor, state)
    backup = getattr(executor, _BACKUP, None)
    if backup is None:
        backup = {'schema': 'wheel_velocity_stability/v1', 'names': state['names'],
                  'names_sha256': state['names_sha256'], 'wheel_names': list(WHEELS),
                  'wheel_indices': state['indices'],
                  'original_wheel_kd': state['kd'][state['indices']].tolist(),
                  'original_cached_wheel_kd': state['cached_kd'][state['indices']].tolist(),
                  'articulation_identity': id(executor.articulation),
                  'output': str(Path(output).resolve()), 'created_wall_time': time.time(),
                  'active': False, 'last_scale': None}
        _write(Path(output) / 'wheel_velocity_stability_backup.json', backup)
        setattr(executor, _BACKUP, backup)
    if (backup['names_sha256'] != state['names_sha256'] or backup['names'] != state['names']
            or backup['articulation_identity'] != id(executor.articulation)):
        raise StabilityReferenceRejected('原备份不属于当前机器人，禁止复用旧会话标定')
    original = np.asarray(backup['original_wheel_kd'], dtype=float)
    desired = np.asarray(original * scale, dtype=np.float32).astype(float)
    if np.any(desired <= 0.) or not np.isfinite(desired).all():
        raise StabilityReferenceRejected('缩放后阻尼不是有效正值')
    after = _transaction(executor, state, desired, None, output)
    backup.update(active=True, last_scale=scale)
    result = {'applied': True, 'reference_scale': scale, 'wheel_names': list(WHEELS),
              'names_sha256': state['names_sha256'], 'source_wheel_kd': original.tolist(),
              'runtime_wheel_kd': after['kd'][state['indices']].tolist(),
              'cached_default_wheel_kd': after['cached_kd'][state['indices']].tolist(),
              'other_damping_preserved': True, 'stiffness_max_force_targets_preserved': True,
              'original_backup_created_once': True, 'saved_to_usd': False,
              'physics_reset_or_state_write': False, 'calibrated_to_hardware': False,
              'note': '仿真参考；未强制休眠/冻结机身，实际运动与再次唤醒稳定性须独立验收'}
    _write(Path(output) / 'wheel_velocity_stability.json', result)
    return result


def restore(executor, output=None):
    """恢复同一 executor 首次保存的四轮 runtime kd 及原默认缓存；不加载旧会话备份。"""
    _guard(executor)
    state = _read(executor.articulation)
    _validate(executor, state)
    backup = getattr(executor, _BACKUP, None)
    if backup is None:
        raise StabilityReferenceRejected('当前 executor 没有原始备份，拒绝盲用磁盘旧值')
    if (backup['names_sha256'] != state['names_sha256'] or backup['names'] != state['names']
            or backup['articulation_identity'] != id(executor.articulation)):
        raise StabilityReferenceRejected('机器人身份与备份不一致')
    output = output or backup['output']
    original = np.asarray(backup['original_wheel_kd'], dtype=float)
    cached = np.asarray(backup['original_cached_wheel_kd'], dtype=float)
    after = _transaction(executor, state, original, cached, output)
    backup.update(active=False, last_scale=None)
    result = {'restored': True, 'wheel_names': list(WHEELS), 'names_sha256': state['names_sha256'],
              'runtime_wheel_kd': after['kd'][state['indices']].tolist(),
              'cached_default_wheel_kd': after['cached_kd'][state['indices']].tolist(),
              'other_damping_preserved': True, 'stiffness_max_force_targets_preserved': True,
              'saved_to_usd': False, 'calibrated_to_hardware': False}
    _write(Path(output) / 'wheel_velocity_stability_restore.json', result)
    return result
