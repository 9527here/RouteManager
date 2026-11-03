"""显式、未默认启用的head_yaw Force型10Hz锁定坐标模型参考。

导入不启用，不接入RobotManager或config。Mii是锁定其他广义坐标的惯量，不是
接触耦合下的实际模态惯量；zeta=1是模型参数，不是已测实物阻尼比。
preview只读；apply(..., reference=True)仅在暂停且无竞争控制时改一个关节KP/KD。
本模块不自行宣布validated，source profile始终记录DEFAULT_UNVALIDATED。
"""
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np


JOINT = 'head_yaw_joint'
FREQUENCY_HZ = 10.
ZETA = 1.
PROFILE = 'HEAD_YAW_FORCE_LOCKED_INERTIA_10HZ_ZETA1_REFERENCE_UNVALIDATED'
_BACKUP = '_head_force_reference_backup'


class HeadForceReferenceRejected(RuntimeError):
    pass


def _write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))
    temporary.replace(path)


def _vector(value, label):
    result = np.asarray(value)
    if result.shape not in ((1, 50), (50,)) or not np.isfinite(result).all():
        raise HeadForceReferenceRejected(label + '维度/数值无效')
    return result.reshape(50).astype(float).copy()


def _state(executor):
    art = executor.articulation
    names = list(art.dof_names)
    if len(names) != 50 or len(set(names)) != 50 or names.count(JOINT) != 1:
        raise HeadForceReferenceRejected('只支持当前50DOF整机的唯一head_yaw_joint')
    kp, kd = art.get_gains()
    view = art._physics_view
    state = {'names': names, 'index': names.index(JOINT),
             'names_sha256': hashlib.sha256(json.dumps(names, ensure_ascii=False,
                                                       separators=(',', ':')).encode()).hexdigest(),
             'kp': _vector(kp, 'KP'), 'kd': _vector(kd, 'KD'),
             'cached_kp': _vector(art._default_kps, '默认KP'),
             'cached_kd': _vector(art._default_kds, '默认KD'),
             'max_force': _vector(view.get_dof_max_forces(), '最大力矩'),
             'position_targets': _vector(view.get_dof_position_targets(), '位置目标'),
             'velocity_targets': _vector(view.get_dof_velocity_targets(), '速度目标'),
             'drive_types': _vector(view.get_drive_types(), '原生DriveType')}
    if state['drive_types'][state['index']] != 1:
        raise HeadForceReferenceRejected('head_yaw不是原生DofDriveType.Force=1')
    return state


def _model(executor, state):
    view, i = executor.articulation._physics_view, state['index']
    matrix = np.asarray(view.get_generalized_mass_matrices(), float)
    gravity = np.asarray(view.get_gravity_compensation_forces(), float)
    if (matrix.shape != (1, 56, 56) or gravity.shape != (1, 56)
            or bool(view.shared_metatype.fixed_base) or not np.isfinite(matrix).all()
            or not np.isfinite(gravity).all()):
        raise HeadForceReferenceRejected('需要finite 56x56浮动基M、56维重力补偿及offset=6')
    if not np.allclose(matrix[0], matrix[0].T, rtol=1e-6, atol=1e-8):
        raise HeadForceReferenceRejected('原生广义质量矩阵不对称')
    eigenvalues = np.linalg.eigvalsh((matrix[0] + matrix[0].T) / 2)
    if eigenvalues[0] <= 0:
        raise HeadForceReferenceRejected('原生广义质量矩阵不是正定')
    inertia, torque = float(matrix[0, i + 6, i + 6]), float(gravity[0, i + 6])
    dt = float(executor.world.get_physics_dt())
    if not math.isfinite(dt) or dt <= 0 or inertia <= 0:
        raise HeadForceReferenceRejected('实际物理步长/锁定坐标惯量无效')
    omega = 2 * math.pi * FREQUENCY_HZ
    k, d = inertia * omega ** 2, 2 * ZETA * inertia * omega
    cap = float(state['max_force'][i])
    error, samples = abs(torque) / k, 1 / (dt * FREQUENCY_HZ)
    reasons = []
    if not all(math.isfinite(x) and x > 0 for x in (inertia, k, d, cap)):
        reasons.append('I/K/D/力矩上限必须positive finite')
    if samples < 20:
        reasons.append('10Hz每周期实际物理采样少于20点')
    if error >= .002:
        reasons.append('预测静态重力误差不小于既有.002rad到位门')
    if abs(torque) >= cap:
        reasons.append('当前模型重力力矩不在原驱动cap内')
    if state['velocity_targets'][i] != 0.:
        reasons.append('head_yaw速度目标非零，纯位置参考的静态误差模型不适用')
    return {'safe': not reasons, 'reasons': reasons, 'profile': PROFILE,
            'profile_status': 'REFERENCE_UNVALIDATED', 'source_force_profile': 'DEFAULT_UNVALIDATED',
            'validated': False, 'hardware_calibrated': False, 'joint': JOINT, 'joint_index': i,
            'native_drive_type': 1, 'native_mass_shape': [1, 56, 56], 'floating_offset': 6,
            'frequency_hz': FREQUENCY_HZ, 'zeta_locked_model': ZETA,
            'inertia_locked_other_coordinates_kg_m2': inertia, 'gravity_torque_Nm': torque,
            'reference_kp': k, 'reference_kd': d, 'max_force_Nm_unchanged': cap,
            'current_kp': float(state['kp'][i]), 'current_kd': float(state['kd'][i]),
            'samples_per_cycle': samples, 'predicted_static_gravity_error_rad': error,
            'minimum_mass_eigenvalue': float(eigenvalues[0]),
            'warning': '锁定坐标线性模型非完整接触耦合动力学；独立测试和现场动作复测前不得标validated'}


def preview(executor):
    """只读模型可行性；safe不代表允许自动apply或已经验证真实运动。"""
    try:
        return _model(executor, _state(executor))
    except Exception as error:
        return {'safe': False, 'reasons': [str(error)], 'error_type': type(error).__name__,
                'profile': PROFILE, 'profile_status': 'REFERENCE_UNVALIDATED',
                'source_force_profile': 'DEFAULT_UNVALIDATED', 'validated': False,
                'hardware_calibrated': False}


def _context(executor):
    import omni.timeline
    from isaacsim.core.simulation_manager import SimulationManager
    from valve_runtime import _competing_controllers
    if executor.world.is_playing() or omni.timeline.get_timeline_interface().is_stopped():
        raise HeadForceReferenceRejected('只允许物理有效的暂停状态，不得运行中或STOP时切换')
    if (SimulationManager.get_physics_simulation_view() is None or
            not executor.articulation.is_physics_handle_valid()):
        raise HeadForceReferenceRejected('物理view无效，禁止public set_gains退到USD分支')
    _competing_controllers(executor)


def _same_identity(executor, state, backup):
    if (backup['names_sha256'] != state['names_sha256'] or backup['names'] != state['names']
            or backup['articulation_identity'] != id(executor.articulation)):
        raise HeadForceReferenceRejected('当前机器人与原参考备份身份不一致')


def _verify(before, after, kp, kd, cached_kp, cached_kd):
    i = before['index']
    other = [n for n in range(50) if n != i]
    if before['names'] != after['names']:
        raise HeadForceReferenceRejected('操作中关节身份改变')
    for field in ('position_targets', 'velocity_targets', 'max_force', 'drive_types'):
        if not np.array_equal(before[field], after[field]):
            raise HeadForceReferenceRejected('意外改变' + field)
    for field, expected in (('kp', kp), ('kd', kd), ('cached_kp', cached_kp), ('cached_kd', cached_kd)):
        if not np.array_equal(before[field][other], after[field][other]) or after[field][i] != expected:
            raise HeadForceReferenceRejected('选定/其余49项' + field + '读回不符')


def _set(art, index, kp, kd):
    art.set_gains(kps=np.array([[kp]], dtype=np.float32), kds=np.array([[kd]], dtype=np.float32),
                  joint_indices=[index], save_to_usd=False)


def _cache(art, index, kp, kd):
    for field, value in (('_default_kps', kp), ('_default_kds', kd)):
        data = getattr(art, field)
        if not isinstance(data, np.ndarray):
            raise HeadForceReferenceRejected('此会话缓存恢复只支持已验证numpy backend')
        data.reshape(-1)[index] = value


def _transaction(executor, before, kp, kd, output, cached=None):
    i, art = before['index'], executor.articulation
    try:
        _set(art, i, kp, kd)
        if cached is not None:
            _cache(art, i, *cached)
        _verify(before, _state(executor), kp, kd, kp if cached is None else cached[0], kd if cached is None else cached[1])
    except Exception as error:
        rollback_error = None
        try:
            _set(art, i, before['kp'][i], before['kd'][i])
            _cache(art, i, before['cached_kp'][i], before['cached_kd'][i])
            _verify(before, _state(executor), before['kp'][i], before['kd'][i],
                    before['cached_kp'][i], before['cached_kd'][i])
        except Exception as failure:
            rollback_error = str(failure)
        _write(Path(output) / 'head_force_reference_failure.json',
               {'error': str(error), 'rollback_verified': rollback_error is None,
                'rollback_error': rollback_error})
        raise HeadForceReferenceRejected('参考切换失败，回滚' +
                ('已验证: ' if rollback_error is None else '未完整验证: ') + str(error)) from error


def _matches(state, backup, active):
    i = state['index']
    prefix = 'applied' if active else 'original'
    for field in ('kp', 'kd', 'cached_kp', 'cached_kd'):
        if state[field][i] != backup[prefix + '_' + field]:
            return False
    return state['max_force'][i] == backup['max_force']


def apply(executor, output, *, reference=False):
    """显式试用模型参考；仅原head KP/KD变更，绝不自动启用或宣布validated。"""
    if reference is not True:
        raise HeadForceReferenceRejected('必须显式reference=True；此profile不是默认或已验证硬件参数')
    _context(executor)
    state = _state(executor)
    model = _model(executor, state)
    if not model['safe']:
        raise HeadForceReferenceRejected('模型预览拒绝: ' + repr(model['reasons']))
    if not isinstance(executor.articulation._default_kps, np.ndarray) or not isinstance(executor.articulation._default_kds, np.ndarray):
        raise HeadForceReferenceRejected('当前实现只支持本机已验证numpy默认缓存')
    backup = getattr(executor, _BACKUP, None)
    if backup is not None:
        _same_identity(executor, state, backup)
        if not _matches(state, backup, backup['active']):
            raise HeadForceReferenceRejected('当前head profile已被其他操作改变，拒绝覆盖')
        if backup['active']:
            result = {'applied': True, 'already_applied': True, 'read_only_confirmation': True,
                      'profile': PROFILE, 'validated': False, 'source_force_profile': 'DEFAULT_UNVALIDATED',
                      'kp': backup['applied_kp'], 'kd': backup['applied_kd']}
            _write(Path(output) / 'head_force_reference.json', result)
            return result
    i = state['index']
    if backup is None:
        backup = {'names': state['names'], 'names_sha256': state['names_sha256'], 'joint': JOINT,
                  'articulation_identity': id(executor.articulation), 'created_wall_time': time.time(),
                  'output': str(Path(output).resolve()), 'active': False, 'max_force': float(state['max_force'][i]),
                  **{'original_' + field: float(state[field][i]) for field in ('kp', 'kd', 'cached_kp', 'cached_kd')},
                  'source_force_profile': 'DEFAULT_UNVALIDATED', 'validated': False}
        _write(Path(output) / 'head_force_reference_backup.json', backup)
        setattr(executor, _BACKUP, backup)
    kp, kd = float(np.float32(model['reference_kp'])), float(np.float32(model['reference_kd']))
    if kp <= 0. or kd <= 0.:
        raise HeadForceReferenceRejected('float32参考K/D不是有效正值')
    _transaction(executor, state, kp, kd, output)
    backup.update(active=True, applied_kp=kp, applied_kd=kd, applied_cached_kp=kp, applied_cached_kd=kd)
    result = {'applied': True, 'already_applied': False, 'profile': PROFILE,
              'profile_status': 'REFERENCE_UNVALIDATED', 'source_force_profile': 'DEFAULT_UNVALIDATED',
              'validated': False, 'hardware_calibrated': False, 'model_preview': model,
              'kp': kp, 'kd': kd, 'other_49_gains_preserved': True,
              'all_targets_max_force_preserved': True, 'cached_defaults_updated': True,
              'saved_to_usd': False, 'physical_state_or_schema_write': False}
    _write(Path(output) / 'head_force_reference.json', result)
    return result


def restore(executor, output=None):
    """只恢复本executor保存的head KP/KD；profile变化时拒绝覆盖别人改动。"""
    _context(executor)
    state = _state(executor)
    backup = getattr(executor, _BACKUP, None)
    if backup is None:
        raise HeadForceReferenceRejected('当前executor没有备份，拒绝盲读旧会话文件')
    _same_identity(executor, state, backup)
    if not _matches(state, backup, backup['active']):
        raise HeadForceReferenceRejected('当前head profile不再匹配已应用/原始profile，拒绝覆盖')
    output = output or backup['output']
    already = not backup['active']
    if not already:
        _transaction(executor, state, backup['original_kp'], backup['original_kd'], output,
                     cached=(backup['original_cached_kp'], backup['original_cached_kd']))
        backup['active'] = False
    result = {'restored': True, 'read_only_confirmation': already, 'source_force_profile': 'DEFAULT_UNVALIDATED',
              'validated': False, 'kp': backup['original_kp'], 'kd': backup['original_kd'],
              'original_cache_restored': True, 'other_49_gains_preserved': True,
              'all_targets_max_force_preserved': True, 'saved_to_usd': False}
    _write(Path(output) / 'head_force_reference_restore.json', result)
    return result
