#!/usr/bin/env python3
"""四轮会话参考阻尼的纯 mock 回归；没有 Kit、HTTP 或机器人入口。"""

from _bootstrap import ROOT as _TEST_ROOT  # 统一离线测试的项目/工具导入路径
import copy
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import wheel_velocity_stability as reference  # noqa: E402


NAMES = ['lift_joint', *reference.CASTERS, 'torso_pitch_joint', *reference.WHEELS,
         'head_yaw_joint', 'head_pitch_joint',
         *[f'{s}_arm_{i}_joint' for s in ('left', 'right') for i in range(1, 7)],
         'left_camera_rojoint', 'right_camera_rojoint',
         *[f'{s}_gripper_{part}_joint' for s in ('left', 'right') for part in
           ('right_2', 'right_3', 'right_main', 'left_2', 'left_3', 'left_main',
            'right_in', 'right_1', 'right_slip', 'left_in', 'left_1', 'left_slip')]]
IDS = [NAMES.index(n) for n in reference.WHEELS]


class FakeArticulation:
    def __init__(self):
        self.dof_names = list(NAMES)
        self.kp = np.arange(50, dtype=np.float32).reshape(1, 50) + 100.
        self.kp[0, IDS] = 0.
        self.kd = (np.arange(50, dtype=np.float32) + 30.).reshape(1, 50)
        self.kd[0, IDS] = np.array([5713.869, 5714., 5715., 5716.], dtype=np.float32)
        self.maxforce = np.full((1, 50), 60., dtype=np.float32)
        self.maxforce[0, IDS] = 40.
        self.pt = np.arange(50, dtype=np.float32).reshape(1, 50) / 20.
        self.vt = np.full((1, 50), .02, dtype=np.float32)
        self.vt[0, IDS] = 0.
        self._default_kps, self._default_kds = self.kp.copy(), self.kd.copy()
        self.calls, self.fault = [], None
        self._physics_view = SimpleNamespace(
            get_dof_max_forces=lambda: self.maxforce.copy(),
            get_dof_position_targets=lambda: self.pt.copy(),
            get_dof_velocity_targets=lambda: self.vt.copy())

    def get_gains(self):
        return self.kp.copy(), self.kd.copy()

    def set_gains(self, *, kds, joint_indices, save_to_usd, kps=None):
        self.calls.append({'kds': kds.copy(), 'joint_indices': list(joint_indices),
                           'save_to_usd': save_to_usd, 'kps': kps})
        if self.fault == 'partial_raise':
            self.fault = None
            self.kd[0, joint_indices[0]] = kds[0, 0]
            self._default_kds[0, joint_indices[0]] = kds[0, 0]
            raise RuntimeError('injected partial public API failure')
        self.kd[0, joint_indices] = kds[0]
        self._default_kds[0, joint_indices] = kds[0]
        if self.fault == 'wrong_readback':
            self.fault = None
            self.kd[0, joint_indices[0]] += 1.
        if self.fault == 'missing_cache_update':
            self.fault = None
            self._default_kds[0, joint_indices[0]] += 1.


class FakeExecutor:
    def __init__(self):
        self.articulation = FakeArticulation()
        self.world = SimpleNamespace(is_playing=lambda: True)
        self._base_cmd = self._motion = self._auto_pick = self._udp_targets = self._ros2_contract = None

    def step(self, *args, **kwargs):
        raise AssertionError('参考模块不得推进物理')

    def send_joint_command(self, *args, **kwargs):
        raise AssertionError('参考模块不得写关节目标')

    def set_base_velocity(self, *args, **kwargs):
        raise AssertionError('参考模块不得发底盘指令')


class WheelVelocityReferenceRegression(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='synthetic_wheel_reference_')
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name)
        self.executor = FakeExecutor()
        self.art = self.executor.articulation
        self.original = reference._read(self.art)
        self.addCleanup(mock.patch.stopall)
        self.types = mock.patch.object(reference, '_native_drive_types', return_value=['force'] * 4).start()
        mock.patch('urllib.request.urlopen', side_effect=AssertionError('禁止 HTTP')).start()
        mock.patch('socket.create_connection', side_effect=AssertionError('禁止 socket')).start()

    def test_apply_only_four_kd_and_updates_default_cache(self):
        result = reference.apply(self.executor, self.output)
        expected = (self.original['kd'][IDS] * .1).astype(np.float32)
        after = reference._read(self.art)
        reference._verify(self.original, after, expected, expected)
        self.assertFalse(result['calibrated_to_hardware'])
        self.assertFalse(result['saved_to_usd'])
        self.assertEqual(len(self.art.calls), 1)
        call = self.art.calls[0]
        self.assertEqual(call['joint_indices'], IDS)
        self.assertEqual(call['kds'].shape, (1, 4))
        self.assertIsNone(call['kps'])
        self.assertFalse(call['save_to_usd'])

    def test_repeated_apply_uses_original_not_current(self):
        reference.apply(self.executor, self.output, .1)
        baseline = (self.output / 'wheel_velocity_stability_backup.json').read_bytes()
        reference.apply(self.executor, self.output, .1)
        np.testing.assert_array_equal(self.art.kd[0, IDS], (self.original['kd'][IDS] * .1).astype(np.float32))
        reference.apply(self.executor, self.output, .25)
        np.testing.assert_array_equal(self.art.kd[0, IDS], (self.original['kd'][IDS] * .25).astype(np.float32))
        self.assertEqual((self.output / 'wheel_velocity_stability_backup.json').read_bytes(), baseline)

    def test_cache_survives_simulated_reset_without_resetting_here(self):
        reference.apply(self.executor, self.output, .1)
        # 仅 fake 模拟 World.reset 的缓存回读，不调用真实 reset。
        self.art.kd = self.art._default_kds.copy()
        np.testing.assert_array_equal(self.art.kd[0, IDS], (self.original['kd'][IDS] * .1).astype(np.float32))

    def test_restore_exact_original_runtime_and_distinct_original_cache(self):
        self.art._default_kds[0, IDS] += np.array([1., 2., 3., 4.], dtype=np.float32)
        before = reference._read(self.art)
        reference.apply(self.executor, self.output, .1)
        result = reference.restore(self.executor)
        self.assertTrue(result['restored'])
        after = reference._read(self.art)
        reference._verify(before, after, before['kd'][IDS], before['cached_kd'][IDS])

    def test_restore_preserves_later_nonwheel_changes(self):
        reference.apply(self.executor, self.output, .1)
        i = NAMES.index('head_pitch_joint')
        self.art.kd[0, i] += 17.
        self.art._default_kds[0, i] += 19.
        before = reference._read(self.art)
        reference.restore(self.executor, self.output)
        self.assertEqual(self.art.kd[0, i], before['kd'][i])
        self.assertEqual(self.art._default_kds[0, i], before['cached_kd'][i])

    def test_partial_api_failure_rolls_back_runtime_cache_and_targets(self):
        for fault in ('partial_raise', 'wrong_readback', 'missing_cache_update'):
            with self.subTest(fault=fault):
                self.art.fault = fault
                before = reference._read(self.art)
                with self.assertRaisesRegex(reference.StabilityReferenceRejected, '已验证回滚'):
                    reference.apply(self.executor, self.output, .1)
                after = reference._read(self.art)
                for field in ('kp', 'kd', 'cached_kp', 'cached_kd', 'max_force', 'position_targets', 'velocity_targets'):
                    np.testing.assert_array_equal(before[field], after[field])
                report = json.loads((self.output / 'wheel_velocity_stability_failure.json').read_text())
                self.assertTrue(report['rollback_verified'])

    def test_invalid_scale_rejects_before_gain_write(self):
        for scale in (0., -1., 1.01, float('nan'), float('inf'), True, None, 'bad'):
            with self.subTest(scale=scale), self.assertRaises(ValueError):
                reference.apply(self.executor, self.output, scale)
        self.assertEqual(self.art.calls, [])
        self.assertFalse(hasattr(self.executor, reference._BACKUP))

    def test_nonforce_drive_is_rejected(self):
        self.types.return_value = ['force', 'force', 'acceleration', 'force']
        with self.assertRaises(reference.StabilityReferenceRejected):
            reference.apply(self.executor, self.output)
        self.assertEqual(self.art.calls, [])

    def test_non_velocity_mode_or_changed_force_cap_rejects(self):
        for field, value in (('kp', 1.), ('maxforce', 41.), ('vt', .01), ('kd', 0.)):
            with self.subTest(field=field):
                original = getattr(self.art, field).copy()
                getattr(self.art, field)[0, IDS[0]] = value
                with self.assertRaises(reference.StabilityReferenceRejected):
                    reference.apply(self.executor, self.output)
                setattr(self.art, field, original)
        self.assertEqual(self.art.calls, [])

    def test_active_controller_or_paused_world_rejects(self):
        for name in ('_base_cmd', '_motion', '_auto_pick', '_udp_targets', '_ros2_contract'):
            with self.subTest(name=name):
                setattr(self.executor, name, {})
                with self.assertRaises(reference.StabilityReferenceRejected):
                    reference.apply(self.executor, self.output)
                setattr(self.executor, name, None)
        self.executor.world.is_playing = lambda: False
        with self.assertRaises(reference.StabilityReferenceRejected):
            reference.apply(self.executor, self.output)
        self.assertEqual(self.art.calls, [])

    def test_nonzero_actual_jitter_velocity_does_not_block_diagnostic(self):
        self.art.get_joint_velocities = lambda: np.full((1, 50), .7, dtype=np.float32)
        self.assertTrue(reference.apply(self.executor, self.output)['applied'])

    def test_names_hash_or_robot_object_change_rejects(self):
        reference.apply(self.executor, self.output)
        calls = len(self.art.calls)
        self.art.dof_names = list(reversed(NAMES))
        with self.assertRaises(reference.StabilityReferenceRejected):
            reference.apply(self.executor, self.output)
        self.assertEqual(len(self.art.calls), calls)
        self.executor.articulation = FakeArticulation()
        with self.assertRaises(reference.StabilityReferenceRejected):
            reference.restore(self.executor, self.output)
        self.assertEqual(self.executor.articulation.calls, [])

    def test_no_backup_restore_never_reads_old_disk_as_authority(self):
        (self.output / 'wheel_velocity_stability_backup.json').write_text('{}')
        with self.assertRaises(reference.StabilityReferenceRejected):
            reference.restore(self.executor, self.output)
        self.assertEqual(self.art.calls, [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
