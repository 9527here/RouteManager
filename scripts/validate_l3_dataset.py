#!/usr/bin/env python3
"""离线核验 L3 HDF5：文件结构、时间/帧对齐、实际机构反馈与任务覆盖。"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys

try:
    import h5py
    import numpy as np
except ImportError as error:
    raise SystemExit('检查器需要 numpy 和 h5py；可使用现有 Isaac Sim 6.2 环境的 Python。\n' + str(error))


def json_safe(value):
    # 失败回合可能确有 NaN；报告用 null 表示，仍保持严格 JSON 可被下游读取。
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def validate_dataset(directory, require_all_tasks=False, allow_failed_episodes=False):
    root = Path(directory)
    failures, warnings, episodes = [], [], []

    def check(condition, message, bucket=failures):
        if not condition:
            bucket.append(message)
        return bool(condition)

    manifest = json.loads((root / 'manifest.json').read_text())
    summary = json.loads((root / 'summary.json').read_text())
    clone_path = root / 'clone_validation.json'
    clone_report = json.loads(clone_path.read_text()) if clone_path.exists() else {}
    check(bool(clone_report), '缺少 clone_validation.json，无法证明并行布局有效')
    check(clone_report.get('pass') is True, '并行固定根布局未通过验证，不能将单体反馈当作有效批量数据')
    clone_errors = clone_report.get('max_root_position_error_m', {})
    check(bool(clone_errors) and all(math.isfinite(float(value)) and float(value) < .002
                                    for value in clone_errors.values()), '并行固定根位置误差缺失或超过2mm')
    declared = {task['task_id']: task for task in manifest['tasks']}
    check(len(declared) == len(manifest['tasks']), '清单存在重复 task_id')
    check(manifest.get('execution_mode') == 'fixture_effort', '清单执行方式不是 fixture_effort')
    check(manifest.get('robot_contact_demonstration') is False, '机构数据缺少非人形示教标记')
    check(manifest.get('calibrated_to_user_hardware') is False, '参考规格标记缺失或被改为已标定')
    cancelled = summary.get('status') == 'cancelled'
    expected_count = len(declared) * manifest['episodes_per_task']
    coverage = Counter()
    all_joint_units = {}
    behavior_failures = []
    with h5py.File(root / 'episodes.h5', 'r') as file:
        check(file.attrs.get('schema_version') == 'l3_fixture_v1', 'HDF5 schema_version 不匹配')
        check(file.attrs.get('execution_mode') == 'fixture_effort', 'HDF5 执行方式不匹配')
        check(not bool(file.attrs.get('robot_contact_demonstration', True)), 'HDF5 缺少非人形示教标记')
        dt = float(file.attrs.get('physics_dt', 0.))
        check(abs(dt - 1/manifest['physics_hz']) < 1e-10, '物理时间步与清单不一致')
        check(abs(float(file.attrs.get('sample_dt', 0.)) - 1/manifest['sample_hz']) < 1e-10,
              '采样时间步与清单不一致')
        check('episodes' in file, '缺少 episodes 组')
        if 'episodes' not in file:
            return {'pass': False, 'data_integrity_pass': False, 'failures': failures}
        for key, group in file['episodes'].items():
            local, behavior = [], []
            try:
                metadata = json.loads(group.attrs['metadata_json'])
                task_id = metadata['task_id']
                check(task_id in declared, '任务不在清单中', local)
                if task_id not in declared:
                    raise ValueError('未知任务')
                task = declared[task_id]
                coverage[task_id] += 1
                for field in ('asset_id', 'kind', 'action', 'source_sha256', 'usd_sha256', 'execution_mode'):
                    check(metadata.get(field) == task.get(field), f'{field} 与清单不一致', local)
                check(not metadata.get('robot_contact_demonstration', True), '错误标为人形接触示教', local)
                check(metadata.get('state_time_alignment') == 'pre_state_action_post_state_same_physics_step',
                      '状态动作时间对齐语义缺失', local)
                joint_names = metadata['joint_names']
                check(len(joint_names) == len(set(joint_names)), '关节名重复', local)
                check(set(joint_names) == set(task['units']), '关节名与单位表不一致', local)
                all_joint_units[task['asset_id']] = dict(task['units'])
                effort_units = {j: 'N' if task['units'][j] == 'm' else 'N*m' for j in joint_names}
                if 'effort_units' in metadata:
                    check(metadata['effort_units'] == effort_units, '广义力单位表错误', local)
                required = ('time', 'q', 'qd', 'effort', 'spring_target', 'next_q', 'next_qd', 'feedback', 'phase')
                missing = [name for name in required if name not in group]
                check(not missing, '缺少数据字段：' + ','.join(missing), local)
                if missing:
                    raise ValueError('回合字段不完整')
                arrays = {name: group[name][:] for name in required}
                n = len(arrays['time'])
                check(n > 1, '回合不足2帧', local)
                check(metadata.get('frame_count') == n, 'frame_count 与实际帧数不一致', local)
                for name, array in arrays.items():
                    check(len(array) == n, name + ' 帧数不一致', local)
                    check(np.isfinite(array).all(), name + ' 存在 NaN/Inf', local)
                for name in ('q', 'qd', 'effort', 'spring_target', 'next_q', 'next_qd'):
                    check(arrays[name].shape == (n, len(joint_names)), name + ' 形状错误', local)
                width = 6 if task['kind'] == 'plate' else 3
                check(arrays['feedback'].shape == (n, width), '反馈列数错误', local)
                time_values = arrays['time']
                check(time_values.ndim == 1 and time_values[0] >= 0., '时间戳格式错误', local)
                check(np.all(np.diff(time_values) > 0), '时间戳未严格递增', local)
                if dt > 0:
                    check(np.max(abs(time_values / dt - np.rint(time_values / dt))) < .002,
                          '时间戳未对齐物理步', local)
                if 'next_time' in group:
                    check(group['next_time'].shape == (n,), 'next_time 帧数错误', local)
                    check(np.allclose(group['next_time'][:] - time_values, dt, atol=1e-7),
                          'next_time 未对应同一物理子步', local)
                q, next_q = arrays['q'], arrays['next_q']
                initial, feedback = np.asarray(metadata['initial_feedback']), arrays['feedback']
                start_expected = np.asarray([task['initial_positions'][name] for name in joint_names])
                check(np.allclose(q[0], start_expected, atol=5e-6), '首帧实际关节状态不符合 reset 初态', local)
                check(0 <= metadata['env_id'] < manifest['num_envs'], '环境编号越界', local)
                check(not metadata.get('timed_out', False), '动作阶段超时', behavior)
                check(not metadata.get('interrupted', False), '回合被中断', behavior)
                check(np.max(abs(next_q[-1] - q[0])) > 1e-5, '初态与终态没有有效变化', behavior)
                if task['kind'] == 'button':
                    index = joint_names.index('button_slide')
                    check(np.allclose(feedback[:, 0], -next_q[:, index], atol=2e-6), '行程反馈与实际位移不一致', local)
                    check(np.all(feedback[:, 1] == feedback[:, 2]), '灯反馈与按钮触发不一致', local)
                    check(np.all(np.isin(feedback[:, 1:], [0., 1.])), '按钮/灯状态不是布尔值', local)
                    if task['action'] == 'press':
                        physical_success = bool(not initial[1] and feedback[-1, 1] and feedback[-1, 0] >= .0026)
                        check(np.all(feedback[-5:, 1] == 1), '按钮按下终态未稳定保持', behavior)
                    else:
                        physical_success = bool(initial[1] and not feedback[-1, 1] and feedback[-1, 0] < .0002)
                        check(np.all(abs(feedback[-5:, 0]) < .0002), '按钮回位终态未稳定保持', behavior)
                    check(physical_success, '按钮终态/触发方向不满足任务', behavior)
                elif task['kind'] == 'selector':
                    index = joint_names.index('selector_turn')
                    check(np.allclose(feedback[:, 0], next_q[:, index], atol=1e-5), '角度反馈与实际关节不一致', local)
                    physical_success = bool(abs(feedback[-1, 0]-math.radians(task['target_deg'])) < math.radians(3)
                                            and feedback[-1, 2])
                    check(np.all(abs(feedback[-5:, 0]-math.radians(task['target_deg'])) < math.radians(3)),
                          '旋钮终态未稳定保持目标档', behavior)
                    check(physical_success, '旋钮未到目标档位并保持', behavior)
                else:
                    check(np.allclose(feedback[:, 0]*2*math.pi, next_q[:, joint_names.index('upper_turn')], atol=1e-4),
                          '上旋钮圈数反馈与实际关节不一致', local)
                    check(np.allclose(feedback[:, 1]*2*math.pi, next_q[:, joint_names.index('lower_turn')], atol=1e-4),
                          '下旋钮圈数反馈与实际关节不一致', local)
                    check(np.allclose(feedback[:, 2], next_q[:, joint_names.index('bridge_swing')], atol=1e-5),
                          '压板摆角反馈与实际关节不一致', local)
                    observed_out = feedback[:, 4].astype(bool)
                    check(np.all(next_q[observed_out, joint_names.index('upper_turn')] >= 4*math.pi-.12)
                          and np.all(next_q[observed_out, joint_names.index('lower_turn')] >= 4*math.pi-.12),
                          '观察到压板投出时螺母仍在锁紧区间', behavior)
                    if task['action'] == 'plate_out':
                        physical_success = bool(feedback[-1, 3] and feedback[-1, 4] and
                                                abs(feedback[-1, 2]-math.pi/2) < math.radians(3))
                    else:
                        physical_success = bool(feedback[-1, 5] and not feedback[-1, 4])
                    check(physical_success, '压板终态不满足投出/投入任务', behavior)
                check(bool(metadata['success']) == bool(physical_success and not metadata.get('timed_out')
                      and not metadata.get('interrupted') and metadata.get('finite')), 'success 标记与实际反馈矛盾', local)
                if manifest.get('rgb'):
                    check('rgb' in group, '声明 RGB 却没有图像', local)
                    if 'rgb' in group:
                        rgb = group['rgb'][:]
                        check(rgb.ndim == 4 and rgb.shape[0] == n and rgb.shape[-1] == 3,
                              'RGB 形状/帧数错误', local)
                        check(rgb.dtype == np.uint8, 'RGB 不是 uint8', local)
                        mean_per_frame = rgb.mean(axis=(1, 2, 3))
                        check(np.all(mean_per_frame > 5), 'RGB 包含黑帧', local)
                        check(np.all(rgb.reshape(n, -1).std(axis=1) > 2), 'RGB 包含纯色帧', local)
                        if 'rgb_time' in group:
                            check(np.allclose(group['rgb_time'][:], time_values+dt, atol=1e-7),
                                  'RGB 时间与 post-state 不同步', local)
                        else:
                            warnings.append(key + ': 无独立 rgb_time，按同步后状态渲染语义解读')
                episodes.append({'episode': key, 'task_id': task_id, 'frames': n,
                                 'success': bool(metadata['success']), 'integrity_pass': not local,
                                 'behavior_pass': not behavior, 'failures': local, 'behavior_failures': behavior,
                                 'initial_feedback': initial.tolist(), 'final_feedback': feedback[-1].tolist(),
                                 'initial_q': q[0].tolist(), 'final_q': next_q[-1].tolist(),
                                 'effort_units': effort_units})
            except Exception as error:
                local.append('回合读取失败：' + str(error))
                episodes.append({'episode': key, 'integrity_pass': False, 'failures': local})
            failures.extend(f'{key}: {message}' for message in local)
            behavior_failures.extend(f'{key}: {message}' for message in behavior)
        if manifest.get('rgb'):
            check('camera/intrinsics' in file, '缺少相机内参')
            if 'camera/intrinsics' in file:
                intrinsics = file['camera/intrinsics'][:]
                check(intrinsics.shape == (manifest['num_envs'], 3, 3), '相机内参数量/形状错误')
                check(np.isfinite(intrinsics).all() and np.all(intrinsics[:, 0, 0] > 0), '相机内参非法')
            if 'camera/position_world' in file:
                position = file['camera/position_world'][:]
                check(position.shape == (manifest['num_envs'], 3) and np.isfinite(position).all(), '相机世界位置数量/形状或数值错误')
                check('camera/quaternion_world_xyzw' in file, '缺少相机世界姿态四元数')
                if 'camera/quaternion_world_xyzw' in file:
                    rotation = file['camera/quaternion_world_xyzw'][:]
                    check(rotation.shape == (manifest['num_envs'], 4) and np.isfinite(rotation).all(), '相机姿态数量/形状或数值错误')
                    check(np.allclose(np.linalg.norm(rotation, axis=1), 1., atol=1e-4), '相机姿态四元数未归一化')
            elif 'camera/position' not in file and 'camera/world_pose' not in file:
                warnings.append('未保存相机外参，不能仅凭图像恢复世界坐标投影')
    check(summary.get('completed_episodes') == len(episodes), '汇总回合数与 HDF5 不一致')
    check(summary.get('successful_episodes') == sum(row.get('success', False) for row in episodes), '汇总成功数与实际标记不一致')
    if not cancelled:
        check(len(episodes) == expected_count, '完成回合数不符合任务数×重复数')
        for task_id in declared:
            check(coverage[task_id] == manifest['episodes_per_task'], task_id + ': 重复回合数量错误')
    if require_all_tasks:
        check(len(declared) == 33 and len(coverage) == 33, '未覆盖全部33个 L3 任务')
        check(Counter(task['kind'] for task in declared.values()) == {'button': 8, 'selector': 15, 'plate': 10},
              '全部任务类型覆盖数错误')
    integrity_pass = not failures
    behavior_pass = not behavior_failures and all(row.get('success', False) for row in episodes) and not cancelled
    return json_safe({'pass': integrity_pass and (allow_failed_episodes or behavior_pass),
            'data_integrity_pass': integrity_pass, 'behavior_pass': behavior_pass,
            'cancelled': cancelled, 'recorded_episodes': len(episodes), 'expected_episodes': expected_count,
            'clone_validation': clone_report,
            'covered_tasks': len(coverage), 'task_repetitions': dict(coverage), 'joint_units': all_joint_units,
            'failures': failures, 'behavior_failures': behavior_failures,
            'warnings': sorted(set(warnings)), 'episodes': episodes,
            'time_alignment': '每行是按采样频率抽取的一个物理子步；q/effort/next_q对应同一步，行间并非连续子步'})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--require-all-tasks', action='store_true')
    parser.add_argument('--allow-failed-episodes', action='store_true', help='只要求数据完整，仍报告行为失败')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:
        result = validate_dataset(args.directory, args.require_all_tasks, args.allow_failed_episodes)
    except Exception as error:
        result = {'pass': False, 'data_integrity_pass': False, 'failures': [str(error)]}
    output = args.output or args.directory / 'validation.json'
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    print(json.dumps({key: value for key, value in result.items() if key != 'episodes'}, ensure_ascii=False, indent=2))
    return 0 if result['pass'] else 2


if __name__ == '__main__':
    sys.exit(main())
