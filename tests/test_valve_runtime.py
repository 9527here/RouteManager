#!/usr/bin/env python3
"""阀门运行时的纯 mock 安全回归：不导入 Kit，不访问 HTTP，不控制仿真。

快照均为 synthetic。验证门控失败时的行为及下发边界，不能替代现场接触/碰撞验收。
用法：python3 tests/test_valve_runtime.py
"""

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

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO), str(REPO / 'scripts')]
import valve_runtime as runtime  # noqa: E402


ARM = [f'right_arm_{i}_joint' for i in range(1, 7)]
LEADER = 'right_gripper_right_2_joint'
ALL_NAMES = ['lift_joint', 'front_left_wheel_joint', 'left_arm_1_joint', *ARM, LEADER]
WHEEL = '/World/SyntheticValve/wheel_link'


def pose(position=(0., 0., 0.)):
    return {'position': list(position), 'orientation': [1., 0., 0., 0.]}


def snapshot():
    """构造状态只用于门控故障注入，不冒充测量或重新构建 URDF。"""
    return {
        'source': 'synthetic runtime regression only', 'sim_time': 0., 'physics_dt': .05,
        'base_pose': pose(), 'arm_base': pose(),
        'robot_native_fixed_base': True,
        'robot_parking_anchor': {'type':'world', 'joint_chain':['/humanoid/SyntheticParkingJoint']},
        'base_fixed_to_world': True,
        'fixed_anchor': {'type': 'kinematic_body', 'body': '/World/SyntheticMount',
                         'pose': pose(), 'joint_chain': ['/World/SyntheticFixedJoint']},
        'wheel_joint': {'body1': WHEEL, 'drive_schemas': [], 'attributes': {},
                        'anchor_gap_mm': 0., 'axis_alignment_deg': 0.,
                        'off_axis_swing_deg': 0.},
        'wheel_angle_deg_wrapped': 0.,
        'geometry': {'radius_mm': 40., 'wheel_pose': pose(),
                     'axis_world': [0., 0., 1.], 'axis_point_world': [0., 0., 0.]},
        'contacts': {'available': True, 'read_succeeded': True, 'load_verifiable': True,
                     'left_pad_touching': True, 'right_pad_touching': True,
                     'left_pad_normal_impulse_Ns': .001, 'right_pad_normal_impulse_Ns': .001,
                     'left_pad_load_bearing': True, 'right_pad_load_bearing': True,
                     'unexpected_robot_normal_impulse_Ns': 0., 'unexpected_robot_load_bearing': False,
                     'contact_count': 2, 'impulse_sum_Ns': .002,
                     'source': 'synthetic finite positive normal impulses; not inferred from patch count'},
        'state': {'playing': True, 'joint_names': list(ALL_NAMES),
                  'positions': [0.] * len(ALL_NAMES), 'velocities': [0.] * len(ALL_NAMES),
                  'cmd_positions': [0.] * len(ALL_NAMES),
                  'sides': {'right': {'flange': pose(), 'tcp': pose((0., 0., .25)),
                                      'pad_poses': {'left': pose((-.04, 0., .25)),
                                                    'right': pose((.04, 0., .25))},
                                      'inner_face_midpoint_world': [0., 0., .25],
                                      'inner_face_gap_mm': 80.,
                                      'gripper': {'position': 1000.}}}},
    }


class FakeExecutor:
    def __init__(self):
        self.world = SimpleNamespace(current_time=0.)
        self.commands = []
        self.steps = 0
        self.measured = np.arange(len(ALL_NAMES), dtype=float) * .01
        self.articulation = SimpleNamespace(dof_names=list(ALL_NAMES),
                                            get_joint_positions=lambda: self.measured.copy())
        self._motion = self._auto_pick = self._base_cmd = self._base_parking = None
        self._udp_targets = self._ros2_contract = self._ros_controller = self._vr_teleop = None
        self._l3_station = self._l3station = None

    def send_joint_command(self, positions, duration, joint_names):
        self.commands.append(('joints', list(joint_names), list(positions), duration))

    def set_gripper_position(self, **kwargs):
        self.commands.append(('gripper', kwargs))

    def set_base_velocity(self, **kwargs):
        self.commands.append(('base', kwargs))

    def step(self, **kwargs):
        self.world.current_time += .05
        self.steps += 1


class FakePlanner:
    names, leader_name, side = ARM, LEADER, 'right'
    joints = {}
    limits = np.array([[-3., 3.]] * 6)

    def __init__(self, *args, **kwargs):
        self.plan_calls = 0

    def independent_anchor(self, value):
        if value.get('synthetic_fk_invalid'):
            raise runtime.PlanningError('synthetic independent FK mismatch')
        return {'q': np.zeros(6), 'tcp': np.array([0., 0., .25]),
                'R_flange': np.eye(3), 'tcp_off': np.array([0., 0., .25]),
                'R_ab': np.eye(3), 't_ab': np.zeros(3)}

    def plan(self, value, **kwargs):
        self.plan_calls += 1
        return {'angle_deg': kwargs['angle'], 'close_position_model': 990., 'metrics': {},
                'stages': {'pre': [{'q': [0.] * 6}],
                           'descent': [{'q': [0.] * 6}],
                           'turn_model': [{'q': [0.] * 6, 'angle_deg': kwargs['angle']}]}}

    def plan_arc(self, *args, **kwargs):
        return {'waypoints': [{'q': [0.] * 6, 'angle_deg': 90.}]}

    def grasp_support(self, value):
        return {'width_mm': 80., 'jaw_axis_world': [1., 0., 0.],
                'midpoint_world_projection_m': 0., 'source': 'synthetic exact support for contact-loop tests'}


class FakeSafety:
    def __init__(self, *args, **kwargs):
        self.q = np.zeros(len(ALL_NAMES))
        self.configuration_calls = []
        self.path_calls = []
        self.model = SimpleNamespace(names=list(ALL_NAMES), poses0={
            '/humanoid/right_gripper_left_pad_link': np.eye(4),
            '/humanoid/right_gripper_right_pad_link': np.eye(4),
            '/humanoid/right_gripper_left_in_link': np.eye(4),
            '/humanoid/right_gripper_right_in_link': np.eye(4), WHEEL: np.eye(4)})

    def full_q(self, partial, q=None):
        result = self.q.copy() if q is None else np.asarray(q).copy()
        for name, value in partial.items():
            result[ALL_NAMES.index(name)] = value
        return result

    def check_configuration(self, **kwargs):
        self.configuration_calls.append(copy.deepcopy(kwargs))
        return {'safe': True, 'globally_collision_free': True}

    def check_path(self, configurations, **kwargs):
        self.path_calls.append(copy.deepcopy(kwargs))
        return {'safe': True, 'sample_count': len(configurations)}

    def refresh(self):
        return self.q.copy(), dict(self.model.poses0)


class ValveRuntimeRegression(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='synthetic_valve_runtime_')
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name)
        self.executor = FakeExecutor()
        self.data = snapshot()
        self.addCleanup(mock.patch.stopall)
        mock.patch('urllib.request.urlopen', side_effect=AssertionError('禁止 HTTP')).start()
        mock.patch('socket.create_connection', side_effect=AssertionError('禁止 socket')).start()
        self.capture = mock.patch.object(runtime, 'capture', side_effect=lambda *a, **k: copy.deepcopy(self.data)).start()
        self.planner = FakePlanner()
        mock.patch.object(runtime, 'ValvePlanner', return_value=self.planner).start()
        self.safety = FakeSafety()
        mock.patch.object(runtime, 'ValveSafety', return_value=self.safety).start()

    def internal(self):
        return {'snapshot': copy.deepcopy(self.data), 'planner': self.planner, 'safety': self.safety,
                'plan': self.planner.plan(self.data, angle=90.), 'contact_pairs': [], 'collision_kwargs': {}}

    def instance(self):
        return runtime._Runtime(self.executor, '/World/SyntheticValve', self.output,
                                self.internal(), dict(runtime.REFERENCE_POLICY))

    def assert_no_motor_commands(self):
        self.assertEqual(self.executor.commands, [])

    def test_fast_closing_stops_at_slow_boundary_even_with_large_time_step(self):
        policy = runtime._policy(None)
        step = runtime._closing_step(1000., 100., 9., 86., {}, 2., policy)
        self.assertEqual(step['mode'], 'free_fast')
        self.assertAlmostEqual(step['position'], step['switch_position'])
        self.assertGreater(step['position'], 100.)
        slow = runtime._closing_step(step['position'], 100., 9., 20., {}, .01, policy)
        self.assertTrue(slow['slow_latched'])
        self.assertAlmostEqual(step['position'] - slow['position'], 1.8)

    def test_actual_narrow_gap_or_zero_load_contact_latches_slow(self):
        policy = runtime._policy(None)
        for gap, contact in [(11., {}), (86., {'left_pad_touching': True}),
                             (86., {'right_pad_load_bearing': True})]:
            step = runtime._closing_step(1000., 100., 9., gap, contact, .01, policy)
            self.assertEqual(step['mode'], 'contact_slow')
            following = runtime._closing_step(step['position'], 100., 9., 86., {}, .01,
                                              policy, step['slow_latched'])
            self.assertEqual(following['mode'], 'contact_slow')

    def test_fast_closing_rejects_missing_finite_gap_and_invalid_margins(self):
        with self.assertRaises(runtime.OperationRejected):
            runtime._closing_step(1000., 100., 9., float('nan'), {}, .01, runtime._policy(None))
        with self.assertRaises(ValueError):runtime._policy({'grip_approach_margin_mm': 1.})

    def test_initial_preposition_follows_planned_aperture_without_forcing_full_open(self):
        operation = self.instance();operation.plan['approach_position_model'] = 980.
        original = self.executor.step
        def follow(**kwargs):
            original(**kwargs)
            commands = [x for x in self.executor.commands if x[0] == 'gripper']
            if commands:self.data['state']['sides']['right']['gripper']['position'] = commands[-1][1]['position']
        self.executor.step = follow
        with mock.patch.object(runtime, '_grip_map', return_value={}):operation.grip(False, maintain_tcp=False)
        positions = [x[1]['position'] for x in self.executor.commands if x[0] == 'gripper']
        self.assertEqual(positions[-1], 980.)
        self.assertTrue(all(980. <= p < 1000. for p in positions))

    def test_run_failed_preflight_never_sends_even_stop_command(self):
        with mock.patch.object(runtime, '_prepare', return_value=({'safe': False}, None)):
            result = runtime.run(self.executor, output=self.output)
        self.assertFalse(result['success'])
        self.assertFalse(result['motion_started'])
        self.assertEqual(self.executor.steps, 0)
        self.assert_no_motor_commands()

    def test_unparked_robot_rejected_before_any_motor_command(self):
        for fault in ('native_free_base', 'missing_constraint_proof'):
            self.data = snapshot()
            if fault == 'native_free_base':self.data['robot_native_fixed_base'] = False
            else:self.data['robot_parking_anchor'] = None
            with self.assertRaisesRegex(runtime.OperationRejected, '底盘尚未驻车固定'):
                self.instance()._guard(self.data)
            self.assert_no_motor_commands()

    def test_parked_robot_drift_still_rejected_at_one_mm(self):
        operation = self.instance()
        self.data['base_pose']['position'][0] = .0011
        with self.assertRaisesRegex(runtime.OperationRejected, '底盘发生漂移'):
            operation._guard(self.data)
        self.assert_no_motor_commands()

    def test_run_prepare_exception_never_sends(self):
        with mock.patch.object(runtime, '_prepare', side_effect=RuntimeError('bad geometry')):
            result = runtime.run(self.executor, output=self.output)
        self.assertFalse(result['success'])
        self.assertEqual(result['phase'], 'preflight')
        self.assertFalse(result['motion_started'])
        self.assert_no_motor_commands()

    def test_stop_file_rejects_before_capture_or_preparation(self):
        (self.output / 'STOP').touch()
        with mock.patch.object(runtime, '_prepare', side_effect=AssertionError('不应准备')):
            result = runtime.run(self.executor, output=self.output)
        self.assertEqual(result['phase'], 'stopped')
        self.assertFalse(result['motion_started'])
        self.capture.assert_not_called()
        self.assert_no_motor_commands()

    def test_stop_file_rechecked_before_next_arm_command(self):
        operation = self.instance()
        operation._arm_command(np.zeros(6))
        (self.output / 'STOP').touch()
        with self.assertRaises(runtime.OperationStopped):
            operation._arm_command(np.ones(6) * .01)
        self.assertEqual(len(self.executor.commands), 1)
        self.assertEqual(self.executor.commands[0][1], ARM)

    def test_passive_sensor_and_joint_faults_reject_without_mutation(self):
        cases = [
            ('missing contacts', lambda s: s['contacts'].update(available=False)),
            ('active wheel schema', lambda s: s['wheel_joint'].update(drive_schemas=['PhysicsDriveAPI:angular'])),
            ('active wheel attribute', lambda s: s['wheel_joint']['attributes'].update({'drive:angular:physics:stiffness': 10.})),
            ('unfixed valve', lambda s: s.update(base_fixed_to_world=False)),
            ('bad anchor', lambda s: s['wheel_joint'].update(anchor_gap_mm=1.01)),
            ('bad axis', lambda s: s['wheel_joint'].update(axis_alignment_deg=.51)),
            ('bad swing', lambda s: s['wheel_joint'].update(off_axis_swing_deg=.51)),
            ('paused simulation', lambda s: s['state'].update(playing=False)),
        ]
        for name, change in cases:
            with self.subTest(fault=name):
                self.data = snapshot()
                change(self.data)
                result = runtime.run(self.executor, output=self.output / name)
                self.assertFalse(result['success'])
                self.assertFalse(result['motion_started'])
                self.assertEqual(result['phase'], 'preflight')
                self.assert_no_motor_commands()

    def test_independent_fk_failure_rejects_before_motion(self):
        self.data['synthetic_fk_invalid'] = True
        result = runtime.run(self.executor, output=self.output)
        self.assertFalse(result['success'])
        self.assertIn('independent FK mismatch', result['error'])
        self.assert_no_motor_commands()

    def test_unimplemented_body_or_lift_placement_is_explicit(self):
        for xy, dz in (((.01, 0.), 0.), ((0., .01), 0.), ((0., 0.), -.01)):
            with self.subTest(body_xy=xy, lift_delta=dz):
                result = runtime.preflight(self.executor, output=self.output,
                                           body_xy=xy, lift_delta=dz, urdf_path='synthetic.urdf')
                self.assertFalse(result['safe'])
                self.assertTrue(result['requires_placement'])
                self.assertEqual(self.planner.plan_calls, 0)
                self.assert_no_motor_commands()

    def test_competing_controllers_all_reject(self):
        cases = {
            '_motion': {}, '_auto_pick': {}, '_base_cmd': {}, '_base_parking': True,
            '_udp_targets': [], '_ros2_contract': object(),
            '_ros_controller': SimpleNamespace(is_initialized=lambda: True),
            '_vr_teleop': SimpleNamespace(_running=True),
            '_l3_station': SimpleNamespace(queue=['pending']),
            '_l3station': SimpleNamespace(_approach=SimpleNamespace(busy=True)),
        }
        for field, value in cases.items():
            with self.subTest(controller=field):
                setattr(self.executor, field, value)
                with self.assertRaises(runtime.OperationRejected):
                    runtime._competing_controllers(self.executor)
                setattr(self.executor, field, None)
                self.assert_no_motor_commands()

    def test_wrong_wheel_direction_rejects(self):
        operation = self.instance()
        operation.turn_origin = 0.
        self.data['wheel_angle_deg_wrapped'] = -3.
        with self.assertRaisesRegex(runtime.OperationRejected, '方向相反'):
            operation._guard(self.data, contact_required=True, desired_angle=3.)
        self.assert_no_motor_commands()

    def test_wheel_lag_rejects(self):
        operation = self.instance()
        operation.turn_origin = 0.
        with self.assertRaisesRegex(runtime.OperationRejected, '没有跟随'):
            operation._guard(self.data, contact_required=True, desired_angle=9.)
        self.assert_no_motor_commands()

    def test_sustained_contact_loss_rejects(self):
        operation = self.instance()
        self.data['contacts']['left_pad_touching'] = False
        self.data['contacts'].update(left_pad_load_bearing=False, left_pad_normal_impulse_Ns=0.)
        operation._guard(self.data, contact_required=True)
        self.executor.world.current_time += .20
        with self.assertRaisesRegex(runtime.OperationRejected, '失去双指垫接触'):
            operation._guard(self.data, contact_required=True)
        self.assert_no_motor_commands()

    def test_brief_contact_loss_recovers_without_false_completion(self):
        operation = self.instance()
        self.data['contacts']['right_pad_touching'] = False
        self.data['contacts'].update(right_pad_load_bearing=False, right_pad_normal_impulse_Ns=0.)
        operation._guard(self.data, contact_required=True)
        self.executor.world.current_time += .05
        self.data['contacts']['right_pad_touching'] = True
        self.data['contacts'].update(right_pad_load_bearing=True, right_pad_normal_impulse_Ns=.001)
        operation._guard(self.data, contact_required=True)
        self.assertIsNone(operation.contact_lost_at)
        self.assertNotEqual(operation.phase, 'complete')
        self.assert_no_motor_commands()

    def test_single_step_wheel_angle_jump_rejects(self):
        operation = self.instance()
        self.data['wheel_angle_deg_wrapped'] = 16.
        with self.assertRaisesRegex(runtime.OperationRejected, '单步角度跳变'):
            operation._guard(self.data)
        self.assert_no_motor_commands()

    def test_moving_kinematic_mount_rejects(self):
        operation = self.instance()
        self.data['fixed_anchor']['pose']['position'][0] += .002
        with self.assertRaisesRegex(runtime.OperationRejected, '固定工位发生漂移'):
            operation._guard(self.data)
        self.assert_no_motor_commands()

    def test_actual_arm_tracking_error_rejects(self):
        operation = self.instance()
        operation.command = np.zeros(6)
        self.data['state']['positions'][ALL_NAMES.index(ARM[2])] = .13
        with self.assertRaisesRegex(runtime.OperationRejected, '手臂跟踪误差过大'):
            operation._guard(self.data)
        self.assert_no_motor_commands()

    def test_failure_hold_only_right_arm_and_leader(self):
        internal = self.internal()
        def fail_after_start(operation):
            operation.phase, operation.started = 'turn', True
            raise runtime.OperationRejected('injected post-command failure')
        with mock.patch.object(runtime, '_prepare', return_value=({'safe': True}, internal)), \
                mock.patch.object(runtime._Runtime, 'execute', fail_after_start):
            result = runtime.run(self.executor, output=self.output)
        self.assertFalse(result['success'])
        self.assertTrue(result['motion_started'])
        base_commands = [c for c in self.executor.commands if c[0] == 'base']
        holds = [c for c in self.executor.commands if c[0] == 'joints']
        self.assertEqual(base_commands, [('base', {'vx': 0., 'wz': 0., 'vy': 0.})])
        self.assertEqual(len(holds), 1)
        self.assertEqual(holds[0][1], ARM + [LEADER])
        self.assertLess(len(holds[0][1]), len(ALL_NAMES))
        self.assertTrue(np.isfinite(holds[0][2]).all())
        expected = [self.executor.measured[ALL_NAMES.index(n)] for n in ARM + [LEADER]]
        np.testing.assert_allclose(holds[0][2], expected)

    def test_nonfinite_measured_hold_never_writes_nan(self):
        operation = self.instance()
        self.executor.measured[ALL_NAMES.index(ARM[0])] = float('nan')
        errors = operation.stop()
        self.assertTrue(any('非有限' in e for e in errors))
        self.assertFalse(any(c[0] == 'joints' for c in self.executor.commands))

    def test_arm_command_rejects_full_body_or_nan_vector(self):
        operation = self.instance()
        for bad in (np.zeros(len(ALL_NAMES)), np.array([0., 0., 0., 0., 0., np.nan])):
            with self.subTest(shape=bad.shape), self.assertRaises(runtime.OperationRejected):
                operation._arm_command(bad)
        self.assert_no_motor_commands()

    def test_arm_velocity_feedforward_is_partial_bounded_and_cleared(self):
        operation=self.instance();setter=mock.Mock()
        self.executor.articulation.set_joint_velocity_targets=setter
        with self.assertRaisesRegex(runtime.OperationRejected,'速度前馈'):
            operation._arm_command(np.zeros(6),velocity=np.ones(6)*10.)
        self.assert_no_motor_commands()
        operation._arm_command(np.zeros(6),velocity=np.ones(6)*.1)
        self.assertEqual(setter.call_args.kwargs['joint_indices'],[ALL_NAMES.index(n)for n in ARM])
        operation.stop()
        np.testing.assert_array_equal(setter.call_args.args[0],np.zeros((1,6)))

    def test_initial_passive_stability_failure_never_starts_targets(self):
        internal = self.internal()
        self.data['base_pose']['position'][0] = .002
        with mock.patch.object(runtime, '_prepare', return_value=({'safe': True}, internal)):
            result = runtime.run(self.executor, output=self.output)
        self.assertFalse(result['success'])
        self.assertFalse(result['motion_started'])
        self.assertRegex(result['error'], '初始无目标稳定窗口未通过|底盘发生漂移')
        self.assertGreater(self.executor.steps, 0)
        self.assert_no_motor_commands()

    def test_preflight_preview_is_not_operation_success(self):
        result = runtime.preflight(self.executor, output=self.output, urdf_path='synthetic.urdf')
        self.assertTrue(result['safe'])
        self.assertNotIn('success', result)
        self.assertFalse((self.output / 'result.json').exists())
        self.assertEqual(self.executor.steps, 0)
        self.assert_no_motor_commands()

    def test_tool_backing_override_only_two_positive_clearances(self):
        result = runtime.preflight(self.executor, output=self.output, urdf_path='synthetic.urdf')
        self.assertTrue(result['safe'])
        overrides = result['tool_backing_clearance_overrides']
        expected = {frozenset((f'/humanoid/right_gripper_{side}_in_link', WHEEL))
                    for side in ('left', 'right')}
        actual = {frozenset(item['bodies']) for item in overrides}
        self.assertEqual(actual, expected)
        self.assertEqual(len(overrides), 2)
        for item in overrides:
            self.assertEqual(item['margin_m'], .001)
            self.assertGreater(item['margin_m'], 0.)
        contacts = {frozenset(pair) for pair in result['contact_pairs']}
        self.assertEqual(contacts, {frozenset((f'/humanoid/right_gripper_{side}_pad_link', WHEEL))
                                    for side in ('left', 'right')})
        self.assertTrue(actual.isdisjoint(contacts))
        # 统一5mm原始诊断先运行且保留；只对准确两对体指定正净空，不能降全局门槛。
        self.assertIn('raw_actual_uniform_environment_margin', result)
        self.assertEqual(len(self.safety.configuration_calls), 2)
        raw, corrected = self.safety.configuration_calls
        self.assertNotIn('clearance_overrides', raw)
        self.assertEqual(corrected['clearance_overrides'], overrides)
        for call in [raw, corrected, *self.safety.path_calls]:
            self.assertEqual(call['environment_margin_m'], .005)
            self.assertEqual(call['self_margin_m'], .008)
            self.assertEqual(call['contact_penetration_m'], .001)
            self.assertEqual({frozenset(pair) for pair in call['contact_pairs']}, contacts)
        for call in self.safety.path_calls:
            self.assertEqual(call['clearance_overrides'], overrides)
        self.assert_no_motor_commands()

    def test_missing_tool_backing_body_is_not_silently_ignored(self):
        del self.safety.model.poses0['/humanoid/right_gripper_left_in_link']
        result = runtime.preflight(self.executor, output=self.output, urdf_path='synthetic.urdf')
        self.assertFalse(result['safe'])
        self.assertIn('无法唯一定位实际夹爪背板刚体', result['error'])
        self.assert_no_motor_commands()

    def test_duplicate_tool_backing_body_is_ambiguous(self):
        self.safety.model.poses0['/humanoid/another/right_gripper_right_in_link'] = np.eye(4)
        result = runtime.preflight(self.executor, output=self.output, urdf_path='synthetic.urdf')
        self.assertFalse(result['safe'])
        self.assertIn('无法唯一定位实际夹爪背板刚体', result['error'])
        self.assert_no_motor_commands()

    def test_unmoved_wheel_cannot_be_reported_as_success(self):
        operation = self.instance()
        with mock.patch.object(operation, 'grip'), mock.patch.object(operation, 'stream'):
            with self.assertRaisesRegex(runtime.OperationRejected, '真实轮轴转角没有到达目标'):
                operation.execute()
        self.assertNotEqual(operation.phase, 'complete')
        self.assertEqual(operation.accumulated, 0.)
        self.assert_no_motor_commands()

    def test_settle_timeout_never_releases_or_advances(self):
        operation = self.instance()
        operation.policy['settle_timeout_s'] = .1
        def bad_step(**kwargs):
            self.executor.world.current_time += .05
            return np.zeros(6), np.zeros(6)
        with mock.patch.object(operation, '_step', side_effect=bad_step), \
                self.assertRaisesRegex(runtime.OperationRejected, '收敛超时'):
            operation._settle(np.ones(6) * .1)
        self.assert_no_motor_commands()

    def test_tcp_settle_replans_world_goal_with_actual_anchor_at_bounded_speed(self):
        operation = self.instance()
        self.data['state']['sides']['right']['tcp']['position'][0] = .018
        self.planner._solve = mock.Mock(return_value=np.ones(6)*.01)
        original = self.executor.step
        def feedback(**kwargs):
            original(**kwargs)
            if operation.command is not None:
                for name, value in zip(ARM, operation.command):
                    self.data['state']['positions'][ALL_NAMES.index(name)] = float(value)
                self.data['state']['sides']['right']['tcp']['position'] = [0.,0.,.25]
        self.executor.step = feedback
        operation._settle(np.zeros(6), tcp_target=np.array([0.,0.,.25]), R_target=np.eye(3))
        self.planner._solve.assert_called_once()
        self.assertEqual(operation.metrics['tcp_feedback_corrections'], 1)
        calls = [r for r in self.executor.commands if r[0]=='joints']
        self.assertEqual(calls[0][1], ARM)
        self.assertLessEqual(max(abs(np.array(calls[0][2]))), .20*.05+1e-12)
        self.assertTrue(self.safety.path_calls)

    def test_tcp_feedback_correction_collision_rejects_before_arm_command(self):
        operation = self.instance()
        self.data['state']['sides']['right']['tcp']['position'][0] = .018
        self.planner._solve = mock.Mock(return_value=np.ones(6)*.01)
        with mock.patch.object(self.safety, 'check_path', return_value={'safe':False}), \
                self.assertRaisesRegex(runtime.OperationRejected, '全身扫掠'):
            operation._settle(np.zeros(6), tcp_target=np.array([0.,0.,.25]), R_target=np.eye(3))
        self.assert_no_motor_commands()

    def test_stream_uses_measured_anchor_for_every_cartesian_target(self):
        operation = self.instance()
        anchor = self.planner.independent_anchor(self.data)
        anchor['t_ab'] = np.array([.018, 0., 0.])
        self.planner.independent_anchor = mock.Mock(return_value=anchor)
        self.planner._solve = mock.Mock(return_value=np.zeros(6))
        rows = [{'q':[.01]*6, 'tcp':[0.,0.,.25], 'R':np.eye(3).tolist()}]
        operation.stream(rows, 'pre')
        self.assertGreaterEqual(self.planner._solve.call_count,
                                int(operation.policy['phase_min_duration_s']/self.data['physics_dt']))
        for call in self.planner._solve.call_args_list:
            np.testing.assert_allclose(call.args[-1], [.018,0.,0.])
        calls = [r for r in self.executor.commands if r[0]=='joints']
        self.assertEqual(len(self.safety.path_calls), len(calls)+1)
        self.assertTrue(all(r[1] == ARM for r in calls))

    def test_stream_corrected_path_rechecked_before_first_motor_command(self):
        operation = self.instance()
        self.planner._solve = mock.Mock(return_value=np.ones(6)*.1)
        rows = [{'q':[.01]*6, 'tcp':[0.,0.,.25], 'R':np.eye(3).tolist()}]
        with mock.patch.object(self.safety, 'check_path', side_effect=[{'safe':True},{'safe':False}]), \
                self.assertRaisesRegex(runtime.OperationRejected, '全身扫掠'):
            operation.stream(rows, 'pre')
        self.assert_no_motor_commands()

    def test_control_batch_keeps_each_physics_step_feedback_and_slew_bound(self):
        self.data['physics_dt'] = 1./240.
        operation = self.instance()
        self.planner._solve = mock.Mock(side_effect=lambda seed,*args,**kwargs:seed)
        def step(**kwargs):
            self.executor.world.current_time += self.data['physics_dt'];self.executor.steps += 1
            if operation.command is not None:
                for name,value in zip(ARM,operation.command):
                    self.data['state']['positions'][ALL_NAMES.index(name)]=float(value)
        self.executor.step=step
        rows=[{'q':[.1]*6,'tcp':[0.,0.,.25],'R':np.eye(3).tolist()}]
        operation.stream(rows,'pre')
        commands=np.array([r[2]for r in self.executor.commands if r[0]=='joints'])
        self.assertGreater(len(commands),self.planner._solve.call_count*3)
        self.assertEqual(operation.metrics['physics_steps'],self.executor.steps)
        self.assertLessEqual(abs(np.diff(commands,axis=0)).max(),operation.policy['joint_path_speed_rad_s']/240.+1e-12)
        self.assertEqual(len(self.safety.path_calls),self.planner._solve.call_count+1)

    def test_grasped_wheel_prediction_uses_current_angle_as_origin(self):
        operation=self.instance();operation.turn_origin=2.;operation.accumulated=12.
        poses=operation._coupled_wheel_poses([10.,11.])
        np.testing.assert_allclose(poses[0][WHEEL],np.eye(4),atol=1e-12)
        np.testing.assert_allclose(poses[1][WHEEL][:3,:3],runtime.axis_rotation([0,0,1],1.),atol=1e-12)

    def test_contact_motion_checks_actual_full_geometry_each_step(self):
        operation=self.instance()
        for _ in range(4):operation._step(contact_required=True)
        self.assertEqual(len(self.safety.configuration_calls),4)

    def closing_loop(self, mode):
        self.data['contacts'].update(left_pad_normal_impulse_Ns=0., right_pad_normal_impulse_Ns=0.,
                                     left_pad_load_bearing=False, right_pad_load_bearing=False)
        operation = self.instance()
        first_loaded_at = None
        count = 0
        def step(**kwargs):
            nonlocal first_loaded_at, count
            count += 1
            self.executor.world.current_time += .05
            commands = [c for c in self.executor.commands if c[0] == 'gripper']
            command_position = commands[-1][1]['position']
            self.data['state']['sides']['right']['gripper']['position'] = command_position
            left = mode == 'one_loaded' or (mode == 'both_loaded_later' and count >= 35)
            right = mode == 'both_loaded_later' and count >= 35
            self.data['contacts'].update(left_pad_load_bearing=left, right_pad_load_bearing=right,
                left_pad_normal_impulse_Ns=.001 if left else 0., right_pad_normal_impulse_Ns=.002 if right else 0.)
            if left and right and first_loaded_at is None:
                first_loaded_at = self.executor.world.current_time
            return operation._guard(self.data, **kwargs)
        with mock.patch.object(operation, '_step', side_effect=step):
            if mode == 'both_loaded_later':
                operation.grip(True, maintain_tcp=False)
            else:
                with self.assertRaisesRegex(runtime.OperationRejected, '没有双指垫承载接触'):
                    operation.grip(True, maintain_tcp=False)
        positions = [c[1]['position'] for c in self.executor.commands if c[0] == 'gripper']
        return operation, count, first_loaded_at, positions

    def test_both_zero_impulse_patches_keep_closing_to_original_bound_then_reject(self):
        operation, count, _, positions = self.closing_loop('zero_patches')
        self.assertGreater(count, 30)
        self.assertGreater(len(set(positions)), 2)
        bound = 1. + 999. * (80. - operation.policy['grip_reference_squeeze_mm']) / runtime.gripper_model.STROKE_MM
        self.assertGreaterEqual(min(positions), bound)
        self.assertAlmostEqual(min(positions), bound)

    def test_positive_bilateral_load_requires_original_dwell_before_grip_finishes(self):
        operation, count, first, positions = self.closing_loop('both_loaded_later')
        self.assertGreaterEqual(count, 38)
        self.assertGreaterEqual(self.executor.world.current_time - first, operation.policy['contact_dwell_s'] - 1e-9)
        self.assertTrue(all(runtime._load_bearing_pads(operation.current['contacts'])))
        self.assertTrue(all(value == positions[-1] for value in positions[35:]))

    def test_one_loaded_pad_does_not_finish_closing(self):
        _, count, _, positions = self.closing_loop('one_loaded')
        self.assertGreater(count, 30)
        self.assertGreater(len(set(positions)), 2)

    def test_missing_or_nonfinite_load_cannot_fall_back_to_touching(self):
        for fault in ('missing', 'nonfinite'):
            with self.subTest(fault=fault):
                self.data = snapshot()
                if fault == 'missing':
                    del self.data['contacts']['left_pad_normal_impulse_Ns']
                else:
                    self.data['contacts']['left_pad_normal_impulse_Ns'] = float('nan')
                with self.assertRaises(runtime.OperationRejected):
                    self.instance()._guard(self.data, contact_required=True)
                self.assert_no_motor_commands()

    def test_positive_nonpad_load_rejects_but_zero_force_nonpad_patch_does_not(self):
        operation = self.instance()
        self.data['contacts'].update(unexpected_robot_load_bearing=True,
                                     unexpected_robot_normal_impulse_Ns=.001)
        with self.assertRaisesRegex(runtime.OperationRejected, '非指垫'):
            operation._guard(self.data)
        self.data['contacts'].update(unexpected_robot_load_bearing=False,
                                     unexpected_robot_normal_impulse_Ns=0.)
        operation._guard(self.data)
        self.assert_no_motor_commands()

    def test_legal_empty_report_does_not_block_approach_but_cannot_prove_loading(self):
        self.data['contacts'].update(load_verifiable=False,
            left_pad_touching=False, right_pad_touching=False,
            left_pad_load_bearing=False, right_pad_load_bearing=False,
            left_pad_normal_impulse_Ns=0., right_pad_normal_impulse_Ns=0.,
            contact_count=0, impulse_sum_Ns=0., absence_reason='synthetic legal empty buffer')
        runtime._passive(self.data, runtime.REFERENCE_POLICY)
        self.assertEqual(runtime._load_bearing_pads(self.data['contacts']), (False, False))
        operation = self.instance()
        operation._guard(self.data, contact_required=True)
        self.executor.world.current_time += .20
        with self.assertRaisesRegex(runtime.OperationRejected, '失去双指垫接触'):
            operation._guard(self.data, contact_required=True)

    def sampled_grip(self, leader_velocity):
        """真实_Runtime._step路径，仅替换执行器为读写内存的synthetic实现。"""
        leader_idx = ALL_NAMES.index(LEADER)
        self.data['state']['positions'][leader_idx] = .6170322299003601
        self.data['state']['velocities'][leader_idx] = leader_velocity
        targets = np.zeros((1, len(ALL_NAMES)))
        self.executor.articulation._physics_view = SimpleNamespace(
            get_dof_position_targets=lambda: targets.copy())
        original_gripper = self.executor.set_gripper_position
        def command(**kwargs):
            original_gripper(**kwargs)
            targets[0, leader_idx] = runtime.gripper_model.leader_for_fraction(
                (kwargs['position'] - 1.) / 999.)
        self.executor.set_gripper_position = command
        original_step = self.executor.step
        def step(**kwargs):
            original_step(**kwargs)
            self.data['sim_time'] = self.executor.world.current_time
        self.executor.step = step
        return self.instance(), targets

    def test_grip_diagnostic_keeps_sdk_velocity_and_mechanical_hold_separate(self):
        operation, targets = self.sampled_grip(-.2775666415691376)
        operation.grip(True, maintain_tcp=False)
        saved = json.loads((self.output / 'contact_close_grip_diagnostics.json').read_text())
        self.assertEqual(saved['exit_reason'], 'completed')
        self.assertEqual(saved['physics_step_sample_count'], self.executor.steps)
        self.assertEqual(saved['sample_count'], self.executor.steps + 1)
        self.assertIsNone(operation._grip_diagnostic)
        last = saved['samples'][-1]
        self.assertEqual(last['raw_leader_q_rad'], .6170322299003601)
        self.assertEqual(last['raw_leader_velocity_rad_s'], -.2775666415691376)
        self.assertEqual(last['left_pad_normal_impulse_Ns'], .001)
        self.assertEqual(last['right_pad_normal_impulse_Ns'], .001)
        self.assertTrue(last['both_load_bearing'])
        self.assertFalse(last['stopped'])
        self.assertEqual(last['native_leader_target_rad'], targets[0, ALL_NAMES.index(LEADER)])
        self.assertIsNone(last['native_target_read_error'])
        self.assertTrue(last['mechanical_hold']['stopped'])
        self.assertGreaterEqual(last['mechanical_hold']['dwell_s'], .15)
        self.assertEqual(last['mechanical_hold']['joint_motion_speed_rad_s'], 0.)
        positions = [row[1]['position'] for row in self.executor.commands if row[0] == 'gripper']
        self.assertTrue(positions)
        self.assertEqual(set(positions), {1000.})  # 合成双载期间目标冻结，原始SDK速度仍完整保留。
        self.assertTrue(saved['does_not_reconstruct_previous_bench_contact_history'])

    def test_grip_diagnostic_success_records_original_continuous_dwell(self):
        operation, _ = self.sampled_grip(0.)
        operation.grip(True, maintain_tcp=False)
        saved = json.loads((self.output / 'contact_close_grip_diagnostics.json').read_text())
        self.assertEqual(saved['exit_reason'], 'completed')
        self.assertEqual(saved['physics_step_sample_count'], self.executor.steps)
        last = saved['samples'][-1]
        self.assertTrue(last['control_evaluated'])
        self.assertTrue(last['stopped'])
        self.assertTrue(last['both_load_bearing'])
        self.assertGreaterEqual(last['control_dwell_elapsed_s'], runtime.REFERENCE_POLICY['contact_dwell_s'])
        self.assertEqual(last['sim_time_s'], self.executor.world.current_time)

    def test_local_lever_preload_uses_existing_bound_before_contact_dwell(self):
        self.data['geometry']['grasp_region']={'preload_to_bound':True}
        operation,_=self.sampled_grip(0.)
        operation.grip(True,maintain_tcp=False)
        positions=[r[1]['position']for r in self.executor.commands if r[0]=='gripper']
        bound=1.+999.*(80.-operation.policy['grip_reference_squeeze_mm'])/runtime.gripper_model.STROKE_MM
        self.assertAlmostEqual(min(positions),bound)
        self.assertGreaterEqual(min(positions),bound-1e-9)
        saved=json.loads((self.output/'contact_close_grip_diagnostics.json').read_text())
        last=saved['samples'][-1]
        self.assertTrue(last['mechanical_hold']['stopped'])
        self.assertGreaterEqual(last['mechanical_hold']['dwell_s'],operation.policy['contact_dwell_s'])

    def test_grip_diagnostic_read_and_flush_errors_do_not_mask_control_failure(self):
        operation, _ = self.sampled_grip(.277)
        original_step = self.executor.step
        def moving_step(**kwargs):
            original_step(**kwargs)
            self.data['state']['positions'][ALL_NAMES.index(LEADER)] += .01
        self.executor.step = moving_step
        self.executor.articulation._physics_view.get_dof_position_targets = mock.Mock(
            side_effect=RuntimeError('synthetic target read failure'))
        original_write = runtime._write
        def write(path, data):
            if str(path).endswith('_grip_diagnostics.json'):
                self.assertIn('synthetic target read failure', data['samples'][-1]['native_target_read_error'])
                raise OSError('synthetic diagnostic flush failure')
            return original_write(path, data)
        with mock.patch.object(runtime, '_write', side_effect=write), \
                self.assertRaisesRegex(runtime.OperationRejected, '渐进夹爪阶段超时'):
            operation.grip(True, maintain_tcp=False)
        self.assertIsNone(operation._grip_diagnostic)
        self.assertEqual(operation.metrics['grip_diagnostic_write_errors'], ['synthetic diagnostic flush failure'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
