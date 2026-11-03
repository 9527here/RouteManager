#!/usr/bin/env python3
"""抖动观测器恢复原方法查找语义的纯mock回归；无Isaac或现场操作。"""

from _bootstrap import ROOT as _TEST_ROOT  # 统一离线测试的项目/工具导入路径
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from measure_robot_jitter import measure


METHODS = ('set_joint_position_targets', 'set_joint_velocity_targets', 'set_joint_velocities', 'apply_action')


class Art:
    def __init__(self):
        self.dof_names, self.body_names = ['synthetic_joint'], ['base_link']
        self._physics_view = SimpleNamespace(
            get_dof_velocity_targets=lambda: np.zeros((1, 1)),
            get_dof_position_targets=lambda: np.zeros((1, 1)),
            get_link_transforms=lambda: np.array([[[0., 0., 0., 0., 0., 0., 1.]]]))

    def get_gains(self):
        return np.ones((1, 1)), np.ones((1, 1))

    def get_joint_positions(self):
        return np.zeros((1, 1))

    def get_joint_velocities(self):
        return np.zeros((1, 1))

    def get_velocities(self):
        return np.zeros((1, 6))

    def get_measured_joint_efforts(self):
        return np.zeros((1, 1))

    def get_dof_types(self):
        return ['Rotation']

    def set_joint_position_targets(self, *args, **kwargs):
        return 'original'

    def set_joint_velocity_targets(self, *args, **kwargs):
        return 'original'

    def set_joint_velocities(self, *args, **kwargs):
        return 'original'

    def apply_action(self, *args, **kwargs):
        return 'original'


class Executor:
    def __init__(self, failure=False):
        self.articulation = Art()
        self.world = SimpleNamespace(current_time=0., get_physics_dt=lambda: .01)
        self.failure = failure

    def step(self, **kwargs):
        if self.failure:
            raise RuntimeError('synthetic measurement exception')
        self.world.current_time += .01


class JitterObserverRegression(unittest.TestCase):
    def run_measure(self, executor):
        with tempfile.TemporaryDirectory(prefix='synthetic_jitter_observer_') as output:
            return measure(executor, output, steps=4, render_every=0)

    def test_class_method_stays_inherited_and_later_replacement_is_visible(self):
        executor = Executor()
        self.assertTrue(all(name not in executor.articulation.__dict__ for name in METHODS))
        self.run_measure(executor)
        self.assertTrue(all(name not in executor.articulation.__dict__ for name in METHODS))
        with mock.patch.object(Art, 'set_joint_position_targets', lambda self: 'new-class-method'):
            self.assertEqual(executor.articulation.set_joint_position_targets(), 'new-class-method')

    def test_existing_instance_override_is_restored_by_exact_identity(self):
        executor = Executor()
        override = lambda: 'existing-instance-override'
        executor.articulation.set_joint_velocity_targets = override
        self.run_measure(executor)
        self.assertIs(executor.articulation.__dict__['set_joint_velocity_targets'], override)
        self.assertEqual(executor.articulation.set_joint_velocity_targets(), 'existing-instance-override')
        self.assertTrue(all(name not in executor.articulation.__dict__ for name in METHODS if name != 'set_joint_velocity_targets'))

    def test_exception_cleanup_restores_descriptor_and_existing_instance_value(self):
        executor = Executor(failure=True)
        override = lambda: 'original-instance'
        executor.articulation.apply_action = override
        with self.assertRaisesRegex(RuntimeError, 'synthetic measurement exception'):
            self.run_measure(executor)
        self.assertIs(executor.articulation.__dict__['apply_action'], override)
        self.assertTrue(all(name not in executor.articulation.__dict__ for name in METHODS if name != 'apply_action'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
