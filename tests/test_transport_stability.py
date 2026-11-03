#!/usr/bin/env python3
"""单头关节复测器的纯 mock 回归；不启动 Kit，不访问 HTTP 或现场对象。"""

from _bootstrap import ROOT as _TEST_ROOT  # 统一离线测试的项目/工具导入路径
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

sys.path[:0] = [str(Path(__file__).resolve().parent), str(Path(__file__).resolve().parents[1])]
import physics_wake  # noqa: E402
import valve_runtime  # noqa: E402
import verify_transport_stability as verify  # noqa: E402


class FakeWorld:
    def __init__(self):
        self.running, self.current_time = False, 0.

    def is_playing(self):
        return self.running

    def is_stopped(self):
        return False

    def play(self):
        self.running = True

    def pause(self):
        self.running = False


class FakeArt:
    def __init__(self, world):
        self.world = world
        self.dof_names = ['head_yaw_joint'] + [f'synthetic_joint_{i}' for i in range(1, 50)]
        self.body_names = ['base_link']
        self.p = np.zeros((1, 50), dtype=np.float32)
        self.p[0, 0] = -9.30143432924524e-5
        self.q = self.p.copy()
        # 主动构造不同于指令目标的静差，才能抓出“首帧反馈回灌”回归。
        self.q[0, 0] = -5.7315e-5
        self.v = np.zeros_like(self.p)
        self.k, self.d, self.force = [np.full_like(self.p, value) for value in (100., 20., 40.)]
        self.vt = np.zeros_like(self.p)
        self.calls = []
        self._physics_view = SimpleNamespace(
            get_dof_position_targets=lambda: self.p.copy(),
            get_dof_velocity_targets=lambda: self.vt.copy(),
            get_dof_max_forces=lambda: self.force.copy(),
            get_link_transforms=lambda: np.array([[[0., 0., 0., 0., 0., 0., 1.]]]))

    def get_gains(self):
        return self.k.copy(), self.d.copy()

    def get_dof_limits(self):
        return np.tile([-1.57, 1.57], (1, 50, 1))

    def get_joint_positions(self):
        return self.q.copy()

    def get_joint_velocities(self):
        return self.v.copy()

    def get_velocities(self):
        return np.zeros((1, 6))

    def set_joint_position_targets(self, value, joint_indices):
        if joint_indices != [0]:
            raise AssertionError('只能写 head_yaw 一个 index')
        self.calls.append({'time': self.world.current_time, 'value': float(value[0, 0])})
        self.p[:, joint_indices] = value


class FakeExecutor:
    def __init__(self, delayed_pulse=False, action_fault=False):
        self.world = FakeWorld()
        self.articulation = FakeArt(self.world)
        self.callback = None
        self.delayed_pulse, self.action_fault = delayed_pulse, action_fault

    def step(self, **kwargs):
        if not self.world.running:
            return
        art = self.articulation
        previous = art.q.copy()
        if art.calls:
            art.q = art.p.copy()
        art.v = (art.q - previous) / .01
        self.world.current_time += .01
        if self.delayed_pulse and .60 < self.world.current_time < 1.68:
            art.v[0, 1] = .031
        if self.action_fault and len(art.calls) > 1:
            art.v[0, 1] = .031
        self.callback(.01)


class HeadMotionRegression(unittest.TestCase):
    def execute(self, executor):
        def subscribe(callback):
            executor.callback = callback
            return lambda: None
        with tempfile.TemporaryDirectory(prefix='synthetic_head_motion_') as out, \
                mock.patch.object(verify, '_subscribe', subscribe), \
                mock.patch.object(verify, '_preflight', return_value={'safe': True}), \
                mock.patch.object(valve_runtime, '_competing_controllers'), \
                mock.patch.object(physics_wake, 'wake_if_sleeping',
                                  return_value={'supported': True, 'sleep_after': False}), \
                mock.patch('urllib.request.urlopen', side_effect=AssertionError('禁止 HTTP')):
            return verify.verify_head_motion(executor, out)

    def test_first_command_is_original_target_and_all_segments_are_continuous(self):
        executor = FakeExecutor()
        original = float(executor.articulation.p[0, 0])
        measured = float(executor.articulation.q[0, 0])
        self.assertNotEqual(original, measured)
        result = self.execute(executor)
        self.assertTrue(result['success'], result)
        calls = executor.articulation.calls
        self.assertEqual(calls[0]['value'], original)
        values = np.array([row['value'] for row in calls])
        # .02rad/2s smoothstep 的最大参考速度=.015rad/s；fake物理步=.01s。
        self.assertLess(float(np.abs(np.diff(values)).max()), .000151)
        starts = result['command_log']
        expected = [original, float(np.float32(original + .01)), float(np.float32(original - .01))]
        for phase, value in zip(starts, expected):
            first = next(row for row in calls if row['time'] >= phase['start_time'] - 1e-9)
            self.assertEqual(first['value'], value)
        self.assertEqual(calls[-1]['value'], original)
        self.assertTrue(all(result['preserved'].values()))
        self.assertTrue(result['final_paused'])

    def test_delayed_wake_transient_requires_two_continuous_seconds(self):
        result = self.execute(FakeExecutor(delayed_pulse=True))
        self.assertTrue(result['success'], result)
        self.assertGreaterEqual(result['wake_settle_duration_s'], 3.67)
        self.assertEqual(result['reference_thresholds']['wake_stable_dwell_s'], 2.)
        self.assertEqual(result['reference_thresholds']['arrival_stable_dwell_s'], .25)

    def test_action_velocity_threshold_is_not_relaxed(self):
        result = self.execute(FakeExecutor(action_fault=True))
        self.assertFalse(result['success'])
        self.assertIn('非头关节', result['error'])
        self.assertTrue(result['original_head_target_restored'])
        self.assertTrue(result['final_paused'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
