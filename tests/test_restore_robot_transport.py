#!/usr/bin/env python3
"""运输初态恢复的离线故障注入回归；不启动Isaac、不连接HTTP。

假后端独立执行SDK的shape/dtype契约，并在“物理首帧”主动回滚状态，检查恢复
逻辑是否把暂停时的echo错误当作成功。本测试不替代真实物理步、碰撞与awake验收。
"""

from _bootstrap import ROOT as _TEST_ROOT  # 统一离线测试的项目/工具导入路径
import copy
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
import restore_robot_transport as restore

NAMES = ['lift_joint', 'right_arm_1_joint', 'caster_front_left_joint']
COMMAND = np.array([-.09, .82, 0.], np.float32)


class TensorBackend:
    """SDK低层setter的独立契约假对象，无场景/仿真依赖。"""
    def __init__(self):
        self.q = np.zeros((1, 3), np.float32)
        self.v = np.zeros_like(self.q)
        self.targets = np.zeros_like(self.q)
        self.velocity_targets = np.zeros_like(self.q)
        self.root = np.array([[.37, -.35, .33, 0., 0., 0., 1.]], np.float32)
        self.initial_root = self.root.copy()
        self.root_v = np.zeros((1, 6), np.float32)
        self.kp = np.ones_like(self.q) * 100
        self.kd = np.ones_like(self.q) * 10
        self.forces = np.ones_like(self.q) * 20
        self.writes = []
        self.fail_on = None

    def _write(self, name, field, data, indices, shape):
        if name == self.fail_on:
            raise RuntimeError('injected backend write failure')
        if data.shape != shape or data.dtype != np.float32:
            raise AssertionError('SDK requires contiguous float32/count-major tensor')
        if indices.dtype != np.uint32 or indices.tolist() != [0]:
            raise AssertionError('SDK requires uint32 articulation index, not joint indices')
        self.writes.append(name)
        setattr(self, field, np.array(data, copy=True))

    def set_root_transforms(self, d, i): self._write('root', 'root', d, i, (1, 7))
    def set_root_velocities(self, d, i): self._write('root_v', 'root_v', d, i, (1, 6))
    def set_dof_positions(self, d, i): self._write('q', 'q', d, i, (1, 3))
    def set_dof_velocities(self, d, i): self._write('v', 'v', d, i, (1, 3))
    def set_dof_position_targets(self, d, i): self._write('targets', 'targets', d, i, (1, 3))
    def set_dof_velocity_targets(self, d, i): self._write('velocity_targets', 'velocity_targets', d, i, (1, 3))
    def get_root_transforms(self): return self.root.copy()
    def get_root_velocities(self): return self.root_v.copy()
    def get_dof_positions(self): return self.q.copy()
    def get_dof_velocities(self): return self.v.copy()
    def get_dof_position_targets(self): return self.targets.copy()
    def get_dof_velocity_targets(self): return self.velocity_targets.copy()
    def get_dof_max_forces(self): return self.forces.copy()


class PhysicsLoop:
    def __init__(self, backend, effect='settle'):
        self.backend, self.effect = backend, effect
        self.playing = False
        self.steps = 0

    def is_playing(self): return self.playing
    def play(self): self.playing = True
    def pause(self): self.playing = False

    def step(self, render=False):
        if not self.playing:
            raise AssertionError('physical step must occur while playing')
        self.steps += 1
        self.backend.root[0, 2] += 2e-5  # 正常首帧微小重力settle；禁止只认立即echo。
        if self.effect == 'root_rebound': self.backend.root = self.backend.initial_root.copy()
        if self.effect == 'yaw_rebound': self.backend.root[0, 3:] = [0, 0, 0, 1]
        if self.effect == 'target_rebound': self.backend.targets[:] = 0
        if self.effect == 'invalid_q': self.backend.q[0, 1] = np.nan

    def reset(self): raise AssertionError('whole-scene reset forbidden')


class RestoreContract(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='restore_transport_mock_')
        self.addCleanup(self.tmp.cleanup)
        self.directory = Path(self.tmp.name)
        self.source = self.directory / 'source.json'
        self.data = {'state': {'joint_names': NAMES,
                              'positions': [-.095, .80, .01],
                              'cmd_positions': COMMAND.tolist()},
                     'base_pose': {'position': [-.02, -.76, .33],
                                   'orientation': Rotation.from_euler('z', .6).as_quat(scalar_first=True).tolist()}}
        self.write_source()
        self.backend = TensorBackend()
        self.world = PhysicsLoop(self.backend)
        art = types.SimpleNamespace(dof_names=NAMES, _physics_view=self.backend,
            get_joint_positions=self.backend.get_dof_positions,
            get_joint_velocities=self.backend.get_dof_velocities,
            get_velocities=self.backend.get_root_velocities,
            get_gains=lambda: (self.backend.kp.copy(), self.backend.kd.copy()))
        self.executor = types.SimpleNamespace(world=self.world, step=self.world.step,
            articulation=art, _base_cmd=None, _motion=None)
        self.safety = types.SimpleNamespace(q=self.backend.q.reshape(-1).copy(),
                                             model=types.SimpleNamespace(names=NAMES))
        self.output = self.directory / 'result'
        self.relocate = mock.Mock(side_effect=AssertionError('prepared fixture must not be deleted or relocated'))
        station = types.ModuleType('valve_station')
        station.relocate_empty, station.rebind_existing_views = self.relocate, mock.Mock()
        self.addCleanup(mock.patch.stopall)
        mock.patch.dict(sys.modules, {'valve_station': station}).start()
        self.preview = mock.patch.object(restore, 'preview', return_value={'safe': True}).start()

    def write_source(self):
        self.source.write_text(json.dumps(self.data))

    def apply(self, **kwargs):
        return restore.apply(self.executor, self.safety, self.source, self.output, **kwargs)

    def test_default_no_authorization_is_zero_write(self):
        with self.assertRaisesRegex(RuntimeError, '授权'):
            self.apply()
        self.assertEqual(self.backend.writes, [])
        self.assertFalse(self.output.exists())
        self.preview.assert_not_called()
        self.relocate.assert_not_called()

    def test_playing_or_busy_is_zero_write(self):
        for field in ('playing', '_base_cmd', '_motion'):
            with self.subTest(field=field):
                self.world.playing = field == 'playing'
                self.executor._base_cmd = object() if field == '_base_cmd' else None
                self.executor._motion = object() if field == '_motion' else None
                with self.assertRaises(RuntimeError): self.apply(confirm_state_reset=True)
                self.assertEqual(self.backend.writes, [])
                self.relocate.assert_not_called()

    def test_source_identity_and_quaternion_invalid(self):
        original = copy.deepcopy(self.data)
        variants = [('duplicate', [NAMES[0]]*3, [1,0,0,0]),
                    ('missing', ['other', *NAMES[1:]], [1,0,0,0]),
                    ('zero_quat', NAMES, [0,0,0,0]),
                    ('nan_quat', NAMES, [float('nan'),0,0,1])]
        for tag,names,quat in variants:
            with self.subTest(tag=tag):
                self.data = copy.deepcopy(original)
                self.data['state']['joint_names'] = names
                self.data['base_pose']['orientation'] = quat
                self.write_source()
                with self.assertRaises(ValueError): restore.source_state(self.source, NAMES)
                self.assertEqual(self.backend.writes, [])

    def test_source_uses_named_command_target_not_gravity_feedback(self):
        source = restore.source_state(self.source, list(reversed(NAMES)))
        np.testing.assert_allclose(source['q'], COMMAND[::-1])
        np.testing.assert_array_equal(source['q'], source['targets'])
        self.assertFalse(np.allclose(source['q'], self.data['state']['positions'][::-1]))

    def test_unsafe_preview_or_stale_state_does_not_write(self):
        self.preview.return_value = {'safe': False}
        with self.assertRaises(RuntimeError): self.apply(confirm_state_reset=True, fixture_already_prepared=True)
        self.preview.return_value = {'safe': True}
        self.backend.q[0, 0] = .01
        with self.assertRaises(RuntimeError): self.apply(confirm_state_reset=True, fixture_already_prepared=True)
        self.assertEqual(self.backend.writes, [])
        self.assertEqual(self.world.steps, 0)
        self.relocate.assert_not_called()

    def test_nonzero_velocity_target_is_zero_write(self):
        self.backend.velocity_targets[0, 0] = 1e-6
        with self.assertRaises(RuntimeError): self.apply(confirm_state_reset=True, fixture_already_prepared=True)
        self.assertEqual(self.backend.writes, [])
        self.assertEqual(self.world.steps, 0)

    def test_prepared_success_requires_physical_frame_and_pauses(self):
        result = self.apply(confirm_state_reset=True, fixture_already_prepared=True)
        self.assertEqual(self.preview.call_args.kwargs['fixture_delta'], (0.,0.,0.))
        self.relocate.assert_not_called()
        self.assertEqual(self.world.steps, 1)
        self.assertFalse(self.world.playing)
        self.assertTrue(result['first_actual_physics_frame_verified'])
        self.assertGreater(result['first_frame_root_position_error_m'], 0)
        self.assertFalse(result['actual_trajectory_executed'])
        np.testing.assert_allclose(self.backend.targets[0], COMMAND)
        np.testing.assert_allclose(self.backend.root[0,3:], [0.,0.,np.sin(.3),np.cos(.3)], atol=1e-7)
        self.assertTrue((self.output/'first_physical_frame.json').exists())

    def test_first_physical_root_rebound_cannot_pass(self):
        self.world.effect = 'root_rebound'
        with self.assertRaises(RuntimeError): self.apply(confirm_state_reset=True, fixture_already_prepared=True)
        self.assertEqual(self.world.steps, 1)
        self.assertFalse(self.world.playing)
        self.assertFalse((self.output/'reset_result.json').exists())
        observed = json.loads((self.output/'first_physical_frame.json').read_text())
        self.assertGreater(observed['actual_position_error_m'], .001)

    def test_first_physical_orientation_target_or_q_corruption_rejected(self):
        for effect in ('yaw_rebound', 'target_rebound', 'invalid_q'):
            with self.subTest(effect=effect):
                self.backend = TensorBackend()
                self.world = PhysicsLoop(self.backend, effect)
                self.executor.world, self.executor.step = self.world, self.world.step
                self.executor.articulation._physics_view = self.backend
                self.executor.articulation.get_joint_positions = self.backend.get_dof_positions
                self.executor.articulation.get_joint_velocities = self.backend.get_dof_velocities
                self.executor.articulation.get_velocities = self.backend.get_root_velocities
                self.safety.q = self.backend.q.reshape(-1).copy()
                with self.assertRaises(RuntimeError): self.apply(confirm_state_reset=True, fixture_already_prepared=True)
                self.assertFalse(self.world.playing)
                self.assertEqual(self.world.steps, 1)

    def test_partial_backend_write_failure_always_pauses(self):
        self.backend.fail_on = 'q'
        with self.assertRaisesRegex(RuntimeError, 'injected'):
            self.apply(confirm_state_reset=True, fixture_already_prepared=True)
        self.assertFalse(self.world.playing)
        self.assertEqual(self.world.steps, 0)
        self.assertFalse((self.output/'reset_result.json').exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)
