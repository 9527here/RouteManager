#!/usr/bin/env python3
"""AST 抽取真实 set_base_velocity 的纯 mock 合约；不 import robot_manager/Kit。

用法：python tests/test_base_native_wake.py。验证原生唤醒只在首次非零接管时
调用，50维状态/目标保持，失败清理目标，以及非法输入在下发前拒绝。
"""

from _bootstrap import ROOT as _TEST_ROOT  # 统一离线测试的项目/工具导入路径
import ast
import copy
from pathlib import Path
import sys
import types
import unittest
from unittest import mock

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))


def _extract_real_method():
    path = REPO / 'robot_manager.py'
    source = ast.parse(path.read_text(), filename=str(path))
    robot = next(node for node in source.body if isinstance(node, ast.ClassDef) and node.name == 'RobotManager')
    method = next(node for node in robot.body if isinstance(node, ast.FunctionDef) and node.name == 'set_base_velocity')
    code = ast.fix_missing_locations(ast.Module(body=[copy.deepcopy(method)], type_ignores=[]))
    scope = {'np': np, 'time': types.SimpleNamespace(time=lambda: 12345.5)}
    exec(compile(code, str(path), 'exec'), scope)
    return scope['set_base_velocity']


REAL_SET_BASE_VELOCITY = _extract_real_method()


class FakeTensor:
    def __init__(self):
        self.position_targets = np.linspace(-.7, 1.3, 50).reshape(1, -1)
        self.velocity_targets = np.linspace(-.09, .16, 50).reshape(1, -1)
        self.q = np.linspace(-.1, .2, 50).reshape(1, -1)
        self.velocities = np.linspace(-.4, .3, 50).reshape(1, -1)
        self.state_setters = []

    def get_dof_positions(self): return self.q.copy()
    def get_dof_velocities(self): return self.velocities.copy()
    def get_dof_position_targets(self): return self.position_targets.copy()
    def get_dof_velocity_targets(self): return self.velocity_targets.copy()

    def _forbidden(self, name, *args, **kwargs):
        self.state_setters.append(name)
        raise AssertionError('原生wake契约禁止物理状态/目标写入: ' + name)

    def set_dof_positions(self, *a, **k): return self._forbidden('set_dof_positions', *a, **k)
    def set_dof_velocities(self, *a, **k): return self._forbidden('set_dof_velocities', *a, **k)
    def set_root_transforms(self, *a, **k): return self._forbidden('set_root_transforms', *a, **k)
    def set_dof_position_targets(self, *a, **k): return self._forbidden('set_dof_position_targets', *a, **k)
    def set_dof_velocity_targets(self, *a, **k): return self._forbidden('set_dof_velocity_targets', *a, **k)
    def set_dof_stiffnesses(self, *a, **k): return self._forbidden('set_dof_stiffnesses', *a, **k)
    def set_dof_dampings(self, *a, **k): return self._forbidden('set_dof_dampings', *a, **k)


class FakeArticulation:
    def __init__(self):
        self._physics_view = FakeTensor()
        self.dof_names = ([f'caster_{i}' for i in range(4)] + [f'wheel_{i}' for i in range(4)] +
                          [f'other_{i}' for i in range(42)])

    def get_joint_positions(self): return self._physics_view.q.copy()
    def get_joint_velocities(self): return self._physics_view.velocities.copy()
    def set_joint_positions(self, *a, **k): return self._physics_view._forbidden('set_joint_positions', *a, **k)
    def set_joint_velocities(self, *a, **k): return self._physics_view._forbidden('set_joint_velocities', *a, **k)
    def set_joint_position_targets(self, *a, **k): return self._physics_view._forbidden('set_joint_position_targets', *a, **k)
    def set_joint_velocity_targets(self, *a, **k): return self._physics_view._forbidden('set_joint_velocity_targets', *a, **k)
    def set_gains(self, *a, **k): return self._physics_view._forbidden('set_gains', *a, **k)
    def apply_action(self, *a, **k): return self._physics_view._forbidden('apply_action', *a, **k)


class SwerveModelMock:
    def __init__(self):
        self.modules = [types.SimpleNamespace(steering_joint=f'caster_{i}', wheel_joint=f'wheel_{i}')
                        for i in range(4)]

    def cardinal_axis(self, vx, vy, wz):
        if wz != 0 or (vx != 0 and vy != 0): return None
        return 'x' if vx != 0 else ('y' if vy != 0 else None)

    def solve_cardinal(self, vx, vy, actual=None, target=None):
        steering = np.asarray(actual if target is None else target, float)
        if steering.shape != (4,) or not np.isfinite(steering).all() or np.any(np.abs(steering) > np.pi/2 + 1e-8):
            raise ValueError('固定舵角形状/有限性/限位无效')
        return {'executed_twist': [vx, vy, 0.], 'steering': steering,
                'wheel_velocity': np.array([1., -1., 1., -1.]) * (vx + vy)}

    def solve(self, vx, vy, wz, actual):
        return {'executed_twist': [vx, vy, wz], 'steering': np.asarray(actual, float),
                'wheel_velocity': np.full(4, vx + vy + wz)}


class FakeExecutor:
    set_base_velocity = REAL_SET_BASE_VELOCITY

    def __init__(self, model=None):
        self.articulation = FakeArticulation()
        self._base_cfg = {'max_vx': .6, 'max_vy': .6, 'max_wz': 1.6, 'timeout': .5}
        self._base_cmd = None
        self._swerve_drive_state = None
        self._swerve_last_stop = None
        self._swerve_reference = {'reference_only': True}
        self.model, self.engage_calls, self.model_queries = model, 0, 0

    def _require_robot(self): pass
    def _get_swerve_model(self):
        self.model_queries += 1
        return self.model
    def _swerve_base_pose(self): return np.array([.1, -.2, .05])
    def _engage_swerve_drive(self, model):
        # 本测试只隔离真实public method；私有驱动增益切换已由原有底盘测试负责。
        self.engage_calls += 1
        self._swerve_drive_state = {'parking_preposition': None, 'parking_stop_window': None,
                                    'cardinal_axis': None, 'cardinal_preposition': None}


class BaseNativeWakeContract(unittest.TestCase):
    def setUp(self):
        self.addCleanup(mock.patch.stopall)
        self.wake = mock.patch('scripts.physics_wake.wake_if_sleeping', return_value={
            'supported': True, 'sleep_before': False, 'sleep_after': False,
            'wake_called': False, 'state_setters_used': []}).start()
        mock.patch('urllib.request.urlopen', side_effect=AssertionError('禁止HTTP')).start()

    def assert_untouched(self, ex, before):
        view = ex.articulation._physics_view
        self.assertEqual(view.state_setters, [])
        for name, values in before.items():
            np.testing.assert_array_equal(getattr(view, name), values)
            self.assertEqual(getattr(view, name).shape, (1, 50))

    def before(self, ex):
        view = ex.articulation._physics_view
        return {k: getattr(view, k).copy() for k in ('q', 'velocities', 'position_targets', 'velocity_targets')}

    def test_first_nonzero_engage_calls_native_once_and_preserves_all50(self):
        ex = FakeExecutor()
        before = self.before(ex)
        result = ex.set_base_velocity(vx=.10)
        self.assertEqual(result['mode'], 'skid_steer_legacy')
        self.wake.assert_called_once_with(ex)
        self.assertEqual(ex._base_cmd['vx'], .10)
        self.assertEqual(ex._swerve_last_wake['sleep_after'], False)
        self.assert_untouched(ex, before)

    def test_repeated_active_commands_never_repeat_wake(self):
        ex = FakeExecutor()
        before = self.before(ex)
        for vx in (.1, .15, -.1, .2): ex.set_base_velocity(vx=vx)
        self.assertEqual(self.wake.call_count, 1)
        self.assert_untouched(ex, before)

    def test_zero_speed_never_wakes(self):
        for model in (None, SwerveModelMock()):
            with self.subTest(swerve=model is not None):
                ex = FakeExecutor(model)
                before = self.before(ex)
                ex.set_base_velocity(0., 0., 0.)
                self.wake.assert_not_called()
                self.assert_untouched(ex, before)

    def test_verified_swerve_stop_returns_without_engage_or_wake(self):
        ex = FakeExecutor(SwerveModelMock())
        before = self.before(ex)
        q = ex.articulation.get_joint_positions().reshape(-1)
        ex._swerve_last_stop = {'base_pose': ex._swerve_base_pose().tolist(),
                                'wheel_positions': q[4:8].tolist(), 'steering_positions': q[:4].tolist()}
        result = ex.set_base_velocity(0., 0., 0.)
        self.assertTrue(result['stopped'])
        self.assertEqual(ex.engage_calls, 0)
        self.assertIsNone(ex._base_cmd)
        self.wake.assert_not_called()
        self.assert_untouched(ex, before)

    def test_unsupported_or_still_asleep_clears_nonzero_command(self):
        for wake_result in ({'supported': False, 'sleep_after': None},
                            {'supported': True, 'sleep_after': True}):
            with self.subTest(wake_result=wake_result):
                self.wake.return_value = wake_result
                ex = FakeExecutor()
                before = self.before(ex)
                with self.assertRaises(RuntimeError): ex.set_base_velocity(vx=.1)
                self.assertIsNone(ex._base_cmd)
                self.assertEqual(ex._swerve_last_wake, wake_result)
                self.assert_untouched(ex, before)

    def test_swerve_first_and_continuous_lateral_commands(self):
        ex = FakeExecutor(SwerveModelMock())
        before = self.before(ex)
        for vy in (.01, .012, .008):
            result = ex.set_base_velocity(vx=0., vy=vy, wz=0.)
            self.assertEqual(result['mode'], 'four_wheel_independent_steering_drive')
            self.assertEqual(result['vy'], vy)
        self.assertEqual(self.wake.call_count, 1)
        self.assertEqual(ex.engage_calls, 1)
        self.assert_untouched(ex, before)

    def test_fixed_steering_is_validated_before_native_wake(self):
        ex = FakeExecutor(SwerveModelMock())
        before = self.before(ex)
        for target in ([0., 0., 0.], [np.nan] * 4, [2.] * 4):
            with self.subTest(target=target), self.assertRaises(ValueError):
                ex.set_base_velocity(vy=.01, steering_targets=target)
            self.assertIsNone(ex._base_cmd)
            self.assertEqual(ex.engage_calls, 0)
        self.wake.assert_not_called()
        self.assert_untouched(ex, before)

    def test_nonfinite_or_nonnumeric_input_rejected_before_model_query(self):
        for kwargs in ({'vx': float('nan')}, {'vy': float('inf')},
                       {'wz': -float('inf')}, {'vx': 'invalid'}):
            with self.subTest(kwargs=kwargs):
                ex = FakeExecutor()
                before = self.before(ex)
                with self.assertRaises((ValueError, TypeError)): ex.set_base_velocity(**kwargs)
                self.assertIsNone(ex._base_cmd)
                self.assertEqual(ex.model_queries, 0)
                self.assert_untouched(ex, before)
        self.wake.assert_not_called()

    def test_legacy_lateral_and_steering_hint_reject_without_wake(self):
        for kwargs in ({'vy': .01}, {'vx': .1, 'steering_targets': [0.] * 4}):
            with self.subTest(kwargs=kwargs):
                ex = FakeExecutor()
                before = self.before(ex)
                with self.assertRaises(ValueError): ex.set_base_velocity(**kwargs)
                self.assertIsNone(ex._base_cmd)
                self.assert_untouched(ex, before)
        self.wake.assert_not_called()

    def test_clamping_and_wake_report_do_not_touch_targets(self):
        ex = FakeExecutor()
        before = self.before(ex)
        result = ex.set_base_velocity(vx=2., wz=-3.)
        self.assertEqual(result['vx'], .6)
        self.assertEqual(result['wz'], -1.6)
        self.wake.assert_called_once_with(ex)
        self.assert_untouched(ex, before)


if __name__ == '__main__':
    unittest.main(verbosity=2)
