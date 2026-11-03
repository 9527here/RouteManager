#!/usr/bin/env python3
"""独立位姿包络诊断的纯mock回归；所有几何/物理状态均明确synthetic。"""

from _bootstrap import ROOT as _TEST_ROOT  # 统一离线测试的项目/工具导入路径
from pathlib import Path
import ast
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
from scipy.spatial.transform import Rotation

sys.path[:0] = [str(Path(__file__).resolve().parent), str(Path(__file__).resolve().parents[1])]
import diagnose_pose_bounded_head_motion as diagnostic  # noqa: E402
import physics_wake  # noqa: E402
import valve_runtime  # noqa: E402
import valve_safety  # noqa: E402
from test_transport_stability import FakeWorld, FakeArt  # noqa: E402


PATHS = ['/synthetic/base_link', '/synthetic/head_yaw_link'] + [f'/synthetic/body_{i}' for i in range(2, 61)]


def frames(q, root):
    result = {path: root.copy() for path in PATHS}
    result[PATHS[1]][:3, :3] = root[:3, :3] @ Rotation.from_euler('z', q[0]).as_matrix()
    result[PATHS[1]][:3, 3] = root[:3, :3] @ np.array([0., 0., 1.5]) + root[:3, 3]
    return result


class Art(FakeArt):
    def __init__(self, world):
        super().__init__(world)
        self.dof_names[24:26] = ['left_camera_rojoint', 'right_camera_rojoint']
        self.body_names = ['base_link', 'head_yaw_link'] + [f'body_{i}' for i in range(2, 61)]
        self.mode = 'raw_velocity_only'
        self._physics_view.link_paths = [PATHS]
        self._physics_view.get_link_transforms = self.links
        # 与安装版ArticulationView一致：getter在tensor view上，无额外参数。
        self._physics_view.get_dof_max_velocities = self._runtime_caps

    def _runtime_caps(self):
        result = np.full((1, 50), 2.)
        result[0, 2] = float('inf')
        if self.mode in ('zero_camera_projection', 'zero_camera_body_fault'):
            result[0, 24:26] = 0.
        if self.mode == 'zero_head_cap':
            result[0, 0] = 0.
        if self.mode in ('warm_positive_cap_transient', 'never_stable_caps', 'positive_small_motion_cap_fault'):
            result[0, 1] = .1
        return result

    def links(self):
        poses = frames(self.q[0], np.eye(4))
        if self.mode == 'warm_internal_transient' and self.world.current_time < .30:
            poses[PATHS[2]][0, 3] += .0012
        elif self.mode == 'warm_root_fault' and self.world.current_time < .30:
            delta = np.eye(4)
            delta[0, 3] = .0011
            poses = {p: delta @ matrix for p, matrix in poses.items()}
        if len(self.calls) > 1:
            if self.mode == 'body_position_fault':
                poses[PATHS[2]][0, 3] += .0011
            elif self.mode == 'zero_camera_body_fault':
                poses[PATHS[24]][0, 3] += .0011
            elif self.mode == 'surface_fault':
                poses[PATHS[2]][:3, :3] = Rotation.from_euler('z', .001).as_matrix()
            elif self.mode == 'wheel_transverse_fault':
                poses[PATHS[3]][:3, :3] = Rotation.from_euler('x', .11, degrees=True).as_matrix()
            elif self.mode == 'mid_motion_hold_fault' and len(self.calls) > 100:
                poses[PATHS[2]][0, 3] += .0011
            if self.mode in ('legal_root_motion', 'root_translation_fault', 'root_full_rotation_fault', 'legal_root_and_deformation'):
                delta = np.eye(4)
                delta[:3, :3] = Rotation.from_euler('y', .11 if self.mode == 'root_full_rotation_fault' else .05, degrees=True).as_matrix()
                delta[0, 3] = .0011 if self.mode == 'root_translation_fault' else .0005
                if self.mode == 'legal_root_and_deformation':
                    poses[PATHS[2]][1, 3] += .0011
                poses = {p: delta @ matrix for p, matrix in poses.items()}
        return np.array([[np.r_[poses[p][:3, 3], Rotation.from_matrix(poses[p][:3, :3]).as_quat()]
                          for p in PATHS]])


class Executor:
    def __init__(self, mode='raw_velocity_only'):
        self.world = FakeWorld()
        self.articulation = Art(self.world)
        self.articulation.mode = mode
        self.mode, self.callback = mode, None

    def step(self, **kwargs):
        if not self.world.running:
            return
        art, dt = self.articulation, .01
        previous = art.q.copy()
        if art.calls:
            art.q = art.p.copy()
        art.v = (art.q - previous) / dt
        # SDK速度有数值投影信号，实际q/刚体位姿不随之移动。
        art.v[0, 1] = .049
        if self.mode in ('zero_camera_projection', 'zero_camera_body_fault'):
            art.v[0, 24:26] = [.01, -.015]
        if len(art.calls) > 1:
            if self.mode == 'finite_velocity_cap_fault':
                art.v[0, 1] = 2.01
            elif self.mode == 'finite_q_cap_fault':
                art.q[0, 3] = 1.58
            elif self.mode == 'metadata_fault':
                art.vt[0, 3] = .01
            elif self.mode == 'mid_motion_hold_fault':
                art.q[0, 0] -= .001
            elif self.mode == 'mid_motion_nan_head' and len(art.calls) > 100:
                art.q[0, 0] = float('nan')
        self.world.current_time += dt
        if self.mode == 'warm_positive_cap_transient' and self.world.current_time < .30:
            art.v[0, 1] = .1001
        elif self.mode == 'never_stable_caps' and int(self.world.current_time * 100) % 50 < 5:
            art.v[0, 1] = .1001
        elif self.mode == 'warm_q_transient':
            art.q[0, 3] = 1.58 if self.world.current_time < .30 else 0.
        elif self.mode == 'positive_small_motion_cap_fault' and len(art.calls) > 1:
            art.v[0, 1] = .1001
        self.callback(dt)


class Safety:
    def __init__(self, executor, **kwargs):
        self.executor = executor
        self.poses = frames(executor.articulation.q[0], np.eye(4))
        corners = np.array([[-.01, -.01, -.01], [.01, .01, .01]])
        shapes = [{'robot': True, 'body': PATHS[i % 61], 'corners': corners.copy()} for i in range(96)]
        if executor.mode == 'surface_fault':
            shapes[2]['corners'] = np.array([[3., 0., 0.]])
        wheel_names = ('front_left_wheel_joint', 'rear_left_wheel_joint',
                       'front_right_wheel_joint', 'rear_right_wheel_joint')
        joints = [{'name': name, 'body1': PATHS[i + 3], 'axis': 'Z', 'T1': np.eye(4).tolist()}
                  for i, name in enumerate(wheel_names)]
        self.model = SimpleNamespace(names=executor.articulation.dof_names, root=PATHS[0],
                                     shapes=shapes, fk=frames, data={'joints': joints})

    def refresh(self):
        self.poses = frames(self.executor.articulation.q[0], np.eye(4))
        return self.executor.articulation.q[0].copy(), self.poses

    def predict(self, q):
        return frames(q, np.eye(4))

    def check_path(self, *args, **kwargs):
        return {'safe': True, 'coverage': {'robot_shapes': 96}}

    def check(self, **kwargs):
        return {'safe': True}

    def check_configuration(self, **kwargs):
        if self.executor.mode == 'warm_collision' and self.executor.world.is_playing():
            return {'safe': False}
        return {'safe': True}


class PoseBoundedDiagnosticRegression(unittest.TestCase):
    def execute(self, mode):
        executor = Executor(mode)
        def subscribe(callback):
            executor.callback = callback
            return lambda: None
        with tempfile.TemporaryDirectory(prefix='synthetic_pose_bound_') as out, \
                mock.patch.object(diagnostic, '_subscribe', subscribe), \
                mock.patch.object(valve_safety, 'ValveSafety', Safety), \
                mock.patch.object(valve_runtime, '_competing_controllers'), \
                mock.patch.object(physics_wake, 'wake_if_sleeping', return_value={'supported': True, 'sleep_after': False}), \
                mock.patch('urllib.request.urlopen', side_effect=AssertionError('禁止 HTTP')):
            report = diagnostic.diagnose_pose_bounded_head_motion(executor, out)
        return report, executor

    def test_mock_velocity_cap_entry_matches_installed_sdk_signature(self):
        package = Path(sys.prefix) / f'lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages/isaacsim'
        sources = list(package.glob('extscache/omni.physics.tensors-*/omni/physics/tensors/api.py'))
        if not sources:
            self.skipTest('本解释器未安装Isaac tensor源码，保留其余纯mock回归')
        tree = ast.parse(sources[0].read_text())
        methods = [node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
                   and node.name == 'get_dof_max_velocities']
        self.assertTrue(methods, '安装SDK没有预期的native tensor getter')
        self.assertEqual([arg.arg for arg in methods[0].args.args], ['self'])
        art = Art(FakeWorld())
        self.assertFalse(hasattr(art, 'get_max_joint_velocities'))
        self.assertEqual(art._physics_view.get_dof_max_velocities().shape, (1, 50))

    def test_pose_and_trajectory_can_complete_while_raw_velocity_gate_stays_failed(self):
        report, executor = self.execute('raw_velocity_only')
        self.assertTrue(report['diagnostic_completed'], report)
        self.assertTrue(report['actual_trajectory_completed'])
        self.assertTrue(report['pose_envelope_passed'])
        self.assertTrue(report['all_recorded_frames_have_fk_coverage'])
        self.assertIs(report['strict_velocity_gate_passed'], False)
        self.assertIn('unbounded', report['runtime_velocity_caps'])
        self.assertGreater(report['actual_head_motion_span_rad'], .019)
        self.assertTrue(all(report['preserved'].values()))
        self.assertTrue(report['final_paused'])
        self.assertFalse(report['strict_verifier_changed'])
        self.assertFalse(report['prior_strict_failures_reclassified'])
        self.assertTrue(all(c['value'] == c['value'] for c in executor.articulation.calls))

    def test_body_position_envelope_is_hard_rejection(self):
        report, _ = self.execute('body_position_fault')
        self.assertFalse(report['diagnostic_completed'])
        self.assertFalse(report['pose_envelope_passed'])
        self.assertIn('位姿包络超限', report['error'])
        self.assertTrue(report['final_paused'])

    def test_surface_corner_envelope_is_independent_hard_rejection(self):
        report, _ = self.execute('surface_fault')
        self.assertFalse(report['diagnostic_completed'])
        self.assertFalse(report['pose_envelope_passed'])
        self.assertIn('corners=', report['error'])

    def test_actual_finite_velocity_cap_is_not_replaced_by_diagnostic_gate(self):
        report, _ = self.execute('finite_velocity_cap_fault')
        self.assertFalse(report['diagnostic_completed'])
        self.assertFalse(report['pose_envelope_passed'])
        self.assertFalse(report['all_recorded_frames_have_fk_coverage'])
        self.assertIn('finite runtime上限', report['error'])
        self.assertIn('failed_frame', report)

    def test_actual_q_caps_stay_hard_rejection(self):
        report, _ = self.execute('finite_q_cap_fault')
        self.assertFalse(report['diagnostic_completed'])
        self.assertFalse(report['pose_envelope_passed'])
        self.assertIn('runtime硬限', report['error'])

    def test_metadata_violation_cannot_be_reported_as_pose_pass(self):
        report, _ = self.execute('metadata_fault')
        self.assertFalse(report['diagnostic_completed'])
        self.assertFalse(report['pose_envelope_passed'])
        self.assertFalse(report['preserved']['velocity_targets'])
        self.assertIn('failed_frame', report)

    def test_configured_camera_zero_caps_remain_zero_and_raw_cap_failure_is_explicit(self):
        report, executor = self.execute('zero_camera_projection')
        self.assertTrue(report['diagnostic_completed'], report)
        self.assertTrue(report['actual_trajectory_completed'])
        self.assertTrue(report['pose_envelope_passed'])
        self.assertFalse(report['raw_velocity_caps_gate_passed'])
        self.assertTrue(report['configured_zero_cap_velocity_exceeded'])
        self.assertEqual(report['configured_zero_cap_joint_names'], ['left_camera_rojoint', 'right_camera_rojoint'])
        self.assertEqual(report['runtime_velocity_caps'][24:26], [0., 0.])
        self.assertEqual(executor.articulation._physics_view.get_dof_max_velocities()[0, 24:26].tolist(), [0., 0.])
        self.assertEqual(set(report['configured_zero_cap_velocity_violations']),
                         {'left_camera_rojoint', 'right_camera_rojoint'})
        self.assertTrue(all(r['sample_count'] > 0 for r in report['configured_zero_cap_velocity_violations'].values()))
        self.assertTrue(all(report['preserved'].values()))

    def test_zero_cap_camera_body_pose_is_not_exempted(self):
        report, _ = self.execute('zero_camera_body_fault')
        self.assertFalse(report['diagnostic_completed'])
        self.assertFalse(report['pose_envelope_passed'])
        self.assertTrue(report['configured_zero_cap_velocity_exceeded'])
        self.assertIn('位姿包络超限', report['error'])

    def test_commanded_head_zero_cap_is_hard_rejection(self):
        report, executor = self.execute('zero_head_cap')
        self.assertFalse(report['diagnostic_completed'])
        self.assertFalse(report['pose_envelope_passed'])
        self.assertIn('受控头部速度上限', report['error'])
        self.assertFalse(executor.articulation.calls)

    def test_one_degree_free_wheel_spin_is_raw_angle_but_not_axis_swing(self):
        reference = np.tile(np.eye(3), (61, 1, 1))
        actual = reference.copy()
        actual[3] = Rotation.from_euler('z', 1., degrees=True).as_matrix()
        task, raw, swings = diagnostic._angle_metrics(actual, reference, {3: np.array([0., 0., 1.])})
        self.assertAlmostEqual(raw[3], 1., places=10)
        self.assertAlmostEqual(task[3], 0., places=10)
        self.assertAlmostEqual(swings[0], 0., places=10)
        # 该断言只针对角语义，未豁免真实center/96 raw corners的独立门。

    def test_wheel_transverse_axis_swing_still_hard_rejects_at_point11_degree(self):
        report, _ = self.execute('wheel_transverse_fault')
        self.assertFalse(report['diagnostic_completed'])
        self.assertFalse(report['pose_envelope_passed'])
        self.assertIn('位姿包络超限', report['error'])

    def cylinder(self, matrix=None, axis='Z'):
        radius, half_height = .159395, .093601 / 2
        corners = np.array([[x, y, z] for x in (-radius, radius) for y in (-radius, radius)
                            for z in (-half_height, half_height)])
        return {'path': '/synthetic/wheel/Cylinder', 'body': PATHS[3], 'type': 'Cylinder',
                'matrix': np.eye(4) if matrix is None else matrix,
                'attributes': {'axis': axis, 'radius': str(radius), 'height': str(half_height * 2)},
                'corners': corners}

    def test_verified_circular_cylinder_uses_same_2mm_geometry_bound_not_fictitious_markers(self):
        shape = self.cylinder()
        rules, mask, records = diagnostic._cylinder_rules([shape], PATHS, {3: np.array([0., 0., 1.])})
        self.assertEqual(len(rules), 1)
        self.assertFalse(mask.any())
        actual, expected = {PATHS[3]: np.eye(4)}, {PATHS[3]: np.eye(4)}
        actual[PATHS[3]][:3, :3] = Rotation.from_euler('z', 1., degrees=True).as_matrix()
        task, raw, *_ = diagnostic._shape_metrics(actual, expected, [shape], rules)
        self.assertGreater(raw[0], .002)
        self.assertLess(task[0], 1e-12)
        self.assertTrue(records[0]['radial_circle_and_joint_axis_verified'])
        actual[PATHS[3]][0, 3] = .0021
        task, raw, *_ = diagnostic._shape_metrics(actual, expected, [shape], rules)
        self.assertGreater(task[0], .002)  # 真实center变化仍用同一个2mm门。

    def test_elliptical_or_joint_misaligned_cylinder_keeps_raw_corner_guard(self):
        elliptical = np.diag([1., 1.1, 1., 1.])
        for shape in (self.cylinder(elliptical), self.cylinder(axis='X')):
            with self.subTest(axis=shape['attributes']['axis']):
                rules, mask, _ = diagnostic._cylinder_rules([shape], PATHS, {3: np.array([0., 0., 1.])})
                self.assertEqual(rules, {})
                self.assertTrue(mask.all())
                actual, expected = {PATHS[3]: np.eye(4)}, {PATHS[3]: np.eye(4)}
                actual[PATHS[3]][:3, :3] = Rotation.from_euler('z', 1., degrees=True).as_matrix()
                task, raw, *_ = diagnostic._shape_metrics(actual, expected, [shape], rules)
                np.testing.assert_array_equal(task, raw)
                self.assertGreater(task[0], .002)

    def test_nonwheel_geometry_never_receives_cylinder_symmetry_rule(self):
        shape = self.cylinder()
        shape['body'] = PATHS[2]
        rules, mask, _ = diagnostic._cylinder_rules([shape], PATHS, {3: np.array([0., 0., 1.])})
        self.assertEqual(rules, {})
        self.assertTrue(mask.all())

    def test_mid_motion_failure_holds_finite_actual_without_original_target_step(self):
        report, executor = self.execute('mid_motion_hold_fault')
        art = executor.articulation
        original = report['original_head_target_rad']
        actual = float(art.q[0, 0])
        self.assertFalse(report['diagnostic_completed'])
        self.assertTrue(report['originaltarget_pending_return'])
        self.assertTrue(report['head_safe_hold_written'])
        self.assertAlmostEqual(report['head_safe_hold_target'], actual, places=9)
        self.assertEqual(art.calls[-1]['value'], float(np.float32(actual)))
        self.assertNotEqual(art.calls[-1]['value'], original)
        self.assertFalse(report['original_head_target_restored'])
        self.assertTrue(report['final_paused'])
        self.assertTrue(all(report['preserved'].values()))

    def test_nonfinite_actual_head_after_failure_never_writes_hold_or_original(self):
        report, executor = self.execute('mid_motion_nan_head')
        self.assertFalse(report['diagnostic_completed'])
        self.assertTrue(report['originaltarget_pending_return'])
        self.assertFalse(report['head_safe_hold_written'])
        self.assertIsNone(report['head_safe_hold_target'])
        self.assertIn('未写任何目标', report['head_safe_hold_error'])
        self.assertNotEqual(executor.articulation.calls[-1]['value'], report['original_head_target_rad'])
        self.assertTrue(report['final_paused'])

    def test_completed_trajectory_does_not_write_redundant_final_restore(self):
        report, executor = self.execute('raw_velocity_only')
        self.assertTrue(report['diagnostic_completed'], report)
        self.assertTrue(report['original_head_target_restored'])
        self.assertFalse(report['originaltarget_pending_return'])
        self.assertFalse(report['head_safe_hold_written'])
        # 最后到位窗口仍推进.25s；finally若补写同值，最后call时间会等于最终time。
        self.assertGreater(executor.world.current_time - executor.articulation.calls[-1]['time'], .24)

    def test_relative_joint_tracking_uses_measured_root_and_keeps_world_diagnostic(self):
        report, _ = self.execute('legal_root_motion')
        self.assertTrue(report['diagnostic_completed'], report)
        self.assertEqual(report['internal_frame'], 'current_measured_base_link')
        self.assertLess(report['residual_peaks']['max_body_position_error_m'], 1e-9)
        self.assertGreater(report['residual_peaks']['world_body_position_error_m'], .001)
        self.assertAlmostEqual(report['residual_peaks']['root_position_error_m'], .0005, places=9)
        self.assertAlmostEqual(report['residual_peaks']['root_full_angle_error_deg'], .05, places=9)

    def test_root_translation_over_1mm_remains_independent_hard_rejection(self):
        report, _ = self.execute('root_translation_fault')
        self.assertFalse(report['diagnostic_completed'])
        self.assertFalse(report['pose_envelope_passed'])
        self.assertIn('root=', report['error'])
        self.assertGreater(report['residual_peaks']['root_position_error_m'], .001)

    def test_root_full_rotation_point11_degree_is_rejected_even_without_yaw(self):
        report, _ = self.execute('root_full_rotation_fault')
        self.assertFalse(report['diagnostic_completed'])
        self.assertFalse(report['pose_envelope_passed'])
        self.assertGreater(report['residual_peaks']['root_full_angle_error_deg'], .1)

    def test_real_nonhead_deformation_is_not_removed_with_legal_root_motion(self):
        report, _ = self.execute('legal_root_and_deformation')
        self.assertFalse(report['diagnostic_completed'])
        self.assertFalse(report['pose_envelope_passed'])
        self.assertGreater(report['residual_peaks']['max_body_position_error_m'], .001)

    def test_startup_internal_settling_excursion_is_recorded_then_two_seconds_stable(self):
        report, executor = self.execute('warm_internal_transient')
        self.assertTrue(report['diagnostic_completed'], report)
        plant = report['plant_settling_excursions']
        self.assertGreater(plant['internal_pose_excursion_samples'], 0)
        self.assertGreater(plant['max_internal_body_position_m'], .001)
        self.assertGreaterEqual(executor.articulation.calls[0]['time'], 2.5)
        self.assertTrue(report['motion_pose_passed'])
        self.assertTrue(report['motion_phase_caps_passed'])
        self.assertLess(report['motion_residual_peaks']['max_body_position_error_m'], .001)

    def test_startup_positive_cap_excursion_never_becomes_overall_raw_pass(self):
        report, executor = self.execute('warm_positive_cap_transient')
        self.assertTrue(report['diagnostic_completed'], report)
        self.assertGreater(report['plant_settling_excursions']['positive_velocity_cap_violation_samples'], 0)
        self.assertGreaterEqual(executor.articulation.calls[0]['time'], 2.29)
        self.assertIs(report['raw_velocity_caps_gate_passed'], False)
        self.assertIs(report['strict_velocity_gate_passed'], False)
        self.assertTrue(report['motion_phase_caps_passed'])

    def test_startup_q_excursion_requires_actual_return_inside_bounds_before_motion(self):
        report, executor = self.execute('warm_q_transient')
        self.assertTrue(report['diagnostic_completed'], report)
        self.assertGreater(report['plant_settling_excursions']['q_cap_violation_samples'], 0)
        self.assertIs(report['raw_q_caps_gate_passed'], False)
        self.assertTrue(report['motion_phase_caps_passed'])
        self.assertGreaterEqual(executor.articulation.calls[0]['time'], 2.29)

    def test_same_positive_cap_excursion_after_motion_still_hard_rejects(self):
        report, _ = self.execute('positive_small_motion_cap_fault')
        self.assertFalse(report['diagnostic_completed'])
        self.assertFalse(report['motion_phase_caps_passed'])
        self.assertIn('finite runtime上限', report['error'])

    def test_startup_root_1point1mm_is_immediate_rejection_without_targets(self):
        report, executor = self.execute('warm_root_fault')
        self.assertFalse(report['diagnostic_completed'])
        self.assertIn('root=', report['error'])
        self.assertFalse(executor.articulation.calls)

    def test_startup_actual_geometry_failure_is_immediate_rejection_without_targets(self):
        report, executor = self.execute('warm_collision')
        self.assertFalse(report['diagnostic_completed'])
        self.assertIn('实时96形状', report['error'])
        self.assertFalse(executor.articulation.calls)

    def test_never_settled_caps_timeout_has_no_actuator_targets(self):
        report, executor = self.execute('never_stable_caps')
        self.assertFalse(report['diagnostic_completed'])
        self.assertIn('6s内', report['error'])
        self.assertFalse(executor.articulation.calls)


if __name__ == '__main__':
    unittest.main(verbosity=2)
