#!/usr/bin/env python3
"""head Force模型参考的纯mock回归；不导入Kit或调用现场API。"""

from _bootstrap import ROOT as _TEST_ROOT  # 统一离线测试的项目/工具导入路径
import ast
import copy
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock

import numpy as np

sys.path[:0] = [str(Path(__file__).resolve().parent), str(Path(__file__).resolve().parents[1])]
import head_force_reference as reference  # noqa: E402
from test_wheel_velocity_stability import NAMES  # noqa: E402


INDEX = NAMES.index('head_yaw_joint')


class Art:
    def __init__(self):
        self.dof_names = list(NAMES)
        self.k = np.full((1, 50), 100., dtype=np.float32)
        self.d = np.full((1, 50), 20., dtype=np.float32)
        self.k[0, INDEX], self.d[0, INDEX] = 4698.25390625, 205.6314239501953
        self._default_kps, self._default_kds = self.k.copy(), self.d.copy()
        self.cap = np.full((1, 50), 60., dtype=np.float32)
        self.cap[0, INDEX] = 10.
        self.pt = np.arange(50, dtype=np.float32).reshape(1, 50) * .001
        self.vt = np.zeros_like(self.pt)
        self.types = np.ones((1, 50), dtype=np.uint8)
        self.mass = np.eye(56, dtype=np.float32).reshape(1, 56, 56) * .1
        self.mass[0, INDEX + 6, INDEX + 6] = .0020050047896802425
        self.gravity = np.zeros((1, 56), dtype=np.float32)
        self.gravity[0, INDEX + 6] = 1.7463664335082285e-5
        self.gravity[0, INDEX] = 5.  # 故意不同，防止漏掉浮动基offset=6。
        self.meta = SimpleNamespace(fixed_base=False)
        self.calls, self.fault = [], None
        self._physics_view = SimpleNamespace(
            shared_metatype=self.meta,
            get_dof_max_forces=lambda: self.cap.copy(),
            get_dof_position_targets=lambda: self.pt.copy(),
            get_dof_velocity_targets=lambda: self.vt.copy(),
            get_drive_types=lambda: self.types.copy(),
            get_generalized_mass_matrices=lambda: self.mass.copy(),
            get_gravity_compensation_forces=lambda: self.gravity.copy())

    def is_physics_handle_valid(self):
        return True

    def get_gains(self):
        return self.k.copy(), self.d.copy()

    def set_gains(self, *, kps, kds, joint_indices, save_to_usd):
        self.calls.append({'kps': kps.copy(), 'kds': kds.copy(),
                           'indices': list(joint_indices), 'save_to_usd': save_to_usd})
        assert joint_indices == [INDEX] and save_to_usd is False
        self.k[:, joint_indices] = kps
        if self.fault == 'partial':
            self.fault = None
            raise RuntimeError('injected partial API failure')
        self.d[:, joint_indices] = kds
        self._default_kps[:, joint_indices] = kps
        self._default_kds[:, joint_indices] = kds
        if self.fault == 'cache':
            self.fault = None
            self._default_kds[0, INDEX] += 1.


class Executor:
    def __init__(self):
        self.articulation = Art()
        self.playing = False
        self.dt = 1 / 240
        self.world = SimpleNamespace(is_playing=lambda: self.playing, get_physics_dt=lambda: self.dt)

    def step(self, *args, **kwargs):
        raise AssertionError('禁止推进物理')

    def send_joint_command(self, *args, **kwargs):
        raise AssertionError('禁止写目标')


class HeadForceReferenceRegression(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='synthetic_head_reference_')
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name)
        self.executor = Executor()
        self.art = self.executor.articulation
        self.before = reference._state(self.executor)
        self.addCleanup(mock.patch.stopall)
        omni, timeline = ModuleType('omni'), ModuleType('omni.timeline')
        timeline.get_timeline_interface = lambda: SimpleNamespace(is_stopped=lambda: False)
        omni.timeline = timeline
        sim = ModuleType('isaacsim.core.simulation_manager')
        sim.SimulationManager = SimpleNamespace(get_physics_simulation_view=lambda: object())
        mock.patch.dict(sys.modules, {'omni': omni, 'omni.timeline': timeline,
            'isaacsim': ModuleType('isaacsim'), 'isaacsim.core': ModuleType('isaacsim.core'),
            'isaacsim.core.simulation_manager': sim}).start()
        mock.patch('urllib.request.urlopen', side_effect=AssertionError('禁止HTTP')).start()

    def test_preview_is_readonly_and_uses_real_force_enum_floating_offset_and_gravity(self):
        result = reference.preview(self.executor)
        self.assertTrue(result['safe'], result)
        inertia = float(self.art.mass[0, INDEX + 6, INDEX + 6])
        self.assertAlmostEqual(result['reference_kp'], inertia * (2 * np.pi * 10) ** 2)
        self.assertAlmostEqual(result['reference_kd'], 2 * inertia * 2 * np.pi * 10)
        self.assertEqual(result['samples_per_cycle'], 24.)
        self.assertLess(result['predicted_static_gravity_error_rad'], .002)
        self.assertEqual(result['source_force_profile'], 'DEFAULT_UNVALIDATED')
        self.assertFalse(result['validated'])
        self.assertEqual(self.art.calls, [])

    def test_native_getter_names_and_signatures_exist_in_installed_sdk(self):
        base = Path(sys.prefix) / f'lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages/isaacsim'
        sources = list(base.glob('extscache/omni.physics.tensors-*/omni/physics/tensors/api.py'))
        if not sources:
            self.skipTest('此解释器未安装tensor源码')
        tree = ast.parse(sources[0].read_text())
        for name in ('get_drive_types', 'get_generalized_mass_matrices', 'get_gravity_compensation_forces'):
            methods = [node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == name]
            self.assertTrue(methods, name)
            self.assertEqual([a.arg for a in methods[0].args.args], ['self'])

    def test_reference_flag_is_required_before_any_write(self):
        with self.assertRaises(reference.HeadForceReferenceRejected):
            reference.apply(self.executor, self.output)
        self.assertEqual(self.art.calls, [])

    def test_apply_only_one_head_gain_and_default_cache_preserves_all_targets(self):
        result = reference.apply(self.executor, self.output, reference=True)
        after = reference._state(self.executor)
        reference._verify(self.before, after, result['kp'], result['kd'], result['kp'], result['kd'])
        self.assertEqual(len(self.art.calls), 1)
        self.assertEqual(self.art.calls[0]['indices'], [INDEX])
        self.assertEqual(self.art.calls[0]['kps'].shape, (1, 1))
        self.assertEqual(self.art.calls[0]['kds'].shape, (1, 1))
        self.assertFalse(result['validated'])

    def test_repeated_active_apply_is_readonly_and_original_backup_is_immutable(self):
        first = reference.apply(self.executor, self.output, reference=True)
        original = (self.output / 'head_force_reference_backup.json').read_bytes()
        second = reference.apply(self.executor, self.output, reference=True)
        self.assertTrue(second['already_applied'])
        self.assertTrue(second['read_only_confirmation'])
        self.assertEqual(len(self.art.calls), 1)
        self.assertEqual(second['kp'], first['kp'])
        self.assertEqual((self.output / 'head_force_reference_backup.json').read_bytes(), original)

    def test_restore_exact_runtime_and_original_distinct_cache(self):
        self.art._default_kds[0, INDEX] += 7.
        before = reference._state(self.executor)
        reference.apply(self.executor, self.output, reference=True)
        result = reference.restore(self.executor)
        self.assertTrue(result['restored'])
        after = reference._state(self.executor)
        reference._verify(before, after, before['kp'][INDEX], before['kd'][INDEX],
                          before['cached_kp'][INDEX], before['cached_kd'][INDEX])

    def test_restore_refuses_to_overwrite_external_head_profile_change(self):
        reference.apply(self.executor, self.output, reference=True)
        self.art.k[0, INDEX] += 1.
        calls = len(self.art.calls)
        with self.assertRaises(reference.HeadForceReferenceRejected):
            reference.restore(self.executor)
        self.assertEqual(len(self.art.calls), calls)

    def test_restore_preserves_later_other_joint_gain_changes(self):
        reference.apply(self.executor, self.output, reference=True)
        self.art.d[0, INDEX + 1] += 2.
        self.art._default_kds[0, INDEX + 1] += 3.
        before = reference._state(self.executor)
        reference.restore(self.executor)
        self.assertEqual(self.art.d[0, INDEX + 1], before['kd'][INDEX + 1])
        self.assertEqual(self.art._default_kds[0, INDEX + 1], before['cached_kd'][INDEX + 1])

    def test_partial_and_cache_failure_roll_back_without_touching_targets(self):
        for fault in ('partial', 'cache'):
            with self.subTest(fault=fault):
                before = reference._state(self.executor)
                self.art.fault = fault
                with self.assertRaisesRegex(reference.HeadForceReferenceRejected, '回滚已验证'):
                    reference.apply(self.executor, self.output, reference=True)
                after = reference._state(self.executor)
                for key in ('kp', 'kd', 'cached_kp', 'cached_kd', 'position_targets', 'velocity_targets', 'max_force'):
                    np.testing.assert_array_equal(before[key], after[key])

    def test_motion_or_playing_prevents_apply(self):
        self.executor.playing = True
        with self.assertRaises(reference.HeadForceReferenceRejected):
            reference.apply(self.executor, self.output, reference=True)
        self.executor.playing = False
        self.executor._motion = {}
        with self.assertRaises(RuntimeError):
            reference.apply(self.executor, self.output, reference=True)
        self.assertEqual(self.art.calls, [])

    def test_unsafe_native_models_are_rejected_without_gain_writes(self):
        cases = ('nonforce', 'slow_dt', 'large_gravity', 'non_spd', 'wrong_shape', 'fixed_base', 'head_velocity_target')
        for case in cases:
            with self.subTest(case=case):
                executor = Executor()
                art = executor.articulation
                if case == 'nonforce': art.types[0, INDEX] = 2
                elif case == 'slow_dt': executor.dt = .006
                elif case == 'large_gravity': art.gravity[0, INDEX + 6] = 1.
                elif case == 'non_spd': art.mass[0, 0, 0] = -1.
                elif case == 'wrong_shape': art.mass = np.eye(50).reshape(1, 50, 50)
                elif case == 'fixed_base': art.meta.fixed_base = True
                elif case == 'head_velocity_target': art.vt[0, INDEX] = .01
                self.assertFalse(reference.preview(executor)['safe'])
                with self.assertRaises(reference.HeadForceReferenceRejected):
                    reference.apply(executor, self.output, reference=True)
                self.assertEqual(art.calls, [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
