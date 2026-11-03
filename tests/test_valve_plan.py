#!/usr/bin/env python3
"""纯离线阀门规划回归：独立 FK、偏心轴、升降坐标和接触后重规划。

用法：python tests/test_valve_plan.py。网络被故障注入封锁，不启动 Isaac。
历史快照仅作模型一致性回归，构造的坐标变化不能当作现场量测或避碰证据。
"""

from _bootstrap import ROOT as _TEST_ROOT  # 统一离线测试的项目/工具导入路径
import copy
import json
import math
from pathlib import Path
import sys
import unittest
from unittest import mock
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import arm_kin
from valve_plan import PlanningError, ValvePlanner, axis_rotation, grasp_rotation


class ValvePlanRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.snapshot = json.loads((REPO / 'outputs/valve_ik_20261102/input.json').read_text())
        cfg = json.loads((REPO / 'config.json').read_text())
        cls.planner = ValvePlanner(cfg['robot']['presets'][cfg['robot']['name']]['urdf_path'])
        cls.preview = cls.planner.plan(cls.snapshot, body_xy=(.30, .15), lift_delta=-.20, roll_deg=0,
                                       allow_legacy_grasp_reference=True)

    def setUp(self):
        mock.patch('urllib.request.urlopen', side_effect=AssertionError('禁止网络')).start()
        self.addCleanup(mock.patch.stopall)

    def test_complete_preview_independent_fk_and_snapshot_unchanged(self):
        before = copy.deepcopy(self.snapshot)
        plan = self.preview
        A, b = np.asarray(plan['R_ab']), np.asarray(plan['t_ab'])
        for rows in plan['stages'].values():
            for row in rows:
                p, R = arm_kin.ee_world(self.planner.chain, row['q'], A, b)
                self.assertLess(np.linalg.norm(p + R @ row['tcp_off'] - row['tcp']), .005)
                ae = math.degrees(math.acos(np.clip((np.trace(R.T @ np.asarray(row['R'])) - 1) / 2, -1, 1)))
                self.assertLess(ae, 3.)
        self.assertGreaterEqual(len(plan['stages']['turn_model']), 90)
        self.assertGreater(plan['metrics']['min_joint_limit_margin_deg'], 25.)
        self.assertLess(plan['metrics']['max_adjacent_joint_delta_deg'], 8.)
        self.assertTrue(plan['requires_contact_replan'])
        self.assertFalse(plan['contact_validated'])
        self.assertFalse(plan['collision_validated'])
        self.assertGreater(plan['close_position_model'], 820.)
        self.assertEqual(self.snapshot, before)
        json.dumps(plan, allow_nan=False)

    def test_tcp_model_agrees_with_simultaneous_open_feedback(self):
        measured = self.planner.independent_anchor(self.snapshot)['tcp_off']
        self.assertLess(np.linalg.norm(self.planner.tcp_offset(position=1000) - measured), 1e-5)
        a = self.planner.tcp_offset(position=820)
        b = self.planner.tcp_offset(position=1000)
        self.assertAlmostEqual((a[2] - b[2]) * 1000, 4.580125, places=5)
        state = dict(zip(self.snapshot['state']['joint_names'], self.snapshot['state']['positions']))
        # 实测跟随关节优先，不会强制把它们改成主动关节的理想 mimic。
        self.assertLess(np.linalg.norm(self.planner.tcp_offset(joint_positions=state) - measured), 1e-5)

    def test_narrow_lever_approach_and_full_release_use_their_own_tcp_offsets(self):
        snapshot = json.loads((REPO / 'outputs/valve_sequence_20261103/run_154059/valve_3/snapshot.json').read_text())
        before = copy.deepcopy(snapshot)
        plan = self.planner.plan(snapshot, roll_deg=270., angle=-90., grasp_axial_offset_m=.006,
                                 approach_open_margin_mm=8.)
        opening = (plan['approach_position_model']-1.) / 999. * 86.2
        self.assertAlmostEqual(opening, plan['grasp_support_width_mm'] + 8.)
        self.assertLess(plan['approach_position_model'], 300.)
        np.testing.assert_allclose(plan['stages']['pre'][0]['tcp_off'],
                                   self.planner.tcp_offset(position=plan['approach_position_model']))
        np.testing.assert_allclose(plan['stages']['ascent_model'][-1]['tcp_off'],
                                   self.planner.tcp_offset(position=1000.))
        self.assertNotEqual(plan['tcp_open_model'], plan['tcp_release_model'])
        self.assertEqual(snapshot, before)

    def test_invalid_preposition_margin_is_rejected_before_ik(self):
        for margin in [0., -1., float('nan')]:
            with self.assertRaises(PlanningError):self.planner.plan(self.snapshot, approach_open_margin_mm=margin)

    def test_line_subdivides_without_relaxing_joint_jump_limit(self):
        q=np.zeros(len(self.planner.names))
        def solve(seed,tcp,*args,**kwargs):return np.ones_like(seed)*tcp[0]*20.
        def row(name,qn,tcp,*args,**kwargs):
            return {'q':qn.tolist(),'tcp':tcp.tolist(),'flange_error_mm':0.,'tcp_error_mm':0.,
                    'rotation_error_deg':0.,'min_joint_margin_deg':1.}
        with mock.patch.object(self.planner,'_solve',side_effect=solve),mock.patch.object(self.planner,'_row',side_effect=row):
            result=self.planner.plan_line(q,np.zeros(3),np.array([.01,0,0]),np.eye(3),np.zeros(3),np.eye(3),np.zeros(3),include_start=False)
        self.assertEqual(len(result['waypoints']),2)
        qs=np.vstack([q,*[r['q']for r in result['waypoints']]])
        self.assertLess(np.rad2deg(abs(np.diff(qs,axis=0))).max(),8.)

    def test_axial_grasp_offset_preserves_measured_geometry(self):
        before = copy.deepcopy(self.snapshot)
        plan = self.planner.plan(self.snapshot, body_xy=(.30, .15), lift_delta=-.20,
                                 roll_deg=0, grasp_axial_offset_m=.006, allow_legacy_grasp_reference=True)
        measured = np.asarray(self.snapshot['geometry']['center'])
        axis = np.asarray(plan['axis_world'])
        np.testing.assert_allclose(plan['measured_center'], measured, atol=0, rtol=0)
        np.testing.assert_allclose(plan['center'], measured, atol=0, rtol=0)
        np.testing.assert_allclose(plan['grasp_tcp'], measured + .006 * axis, atol=1e-12)
        np.testing.assert_allclose(plan['stages']['descent'][-1]['tcp'], plan['grasp_tcp'], atol=1e-12)
        self.assertEqual(plan['grasp_axial_offset_m'], .006)
        self.assertEqual(self.snapshot, before)
        with self.assertRaisesRegex(PlanningError, '轴向偏移必须有限'):
            self.planner.plan(self.snapshot, grasp_axial_offset_m=float('nan'))

    def test_measured_base_rotation_controls_lift_world_axis(self):
        snapshot = copy.deepcopy(self.snapshot)
        R = Rotation.from_euler('xyz', [.2, -.4, .7]).as_matrix()
        snapshot['base_pose'] = {'position': [0., 0., 0.],
                                 'orientation': Rotation.from_matrix(R).as_quat(scalar_first=True).tolist()}
        placed = self.planner.placement(snapshot, body_xy=(.1, -.1), lift_delta=-.2)
        np.testing.assert_allclose(placed['placement']['lift_axis_world'], R[:, 2], atol=1e-7)
        self.assertEqual(placed['placement']['base_rotation_source'], 'measured_base_pose')
        with self.assertRaisesRegex(PlanningError, '升降目标'):
            self.planner.placement(snapshot, lift_delta=-1.)

    def test_requested_tight_precision_rejects_unqualified_fallback(self):
        q=np.zeros(len(self.planner.names));identity=np.eye(3);zero=np.zeros(3)
        import valve_plan
        with mock.patch.object(valve_plan, 'least_squares', return_value=SimpleNamespace(x=q)), \
             mock.patch.object(arm_kin, 'ee_world', return_value=(np.array([.0002,0,0]),identity)):
            # 普通离线模型仍保留既有best行为；显式tight请求必须按实际FK拒绝。
            self.planner._solve(q,zero,identity,zero,identity,zero,multistart=True)
            with self.assertRaisesRegex(PlanningError,'指定位置精度未满足'):
                self.planner._solve(q,zero,identity,zero,identity,zero,multistart=True,
                                    position_precision_m=.0001)

    def test_qualified_fit_returned_even_if_older_score_was_smaller(self):
        q=np.zeros(len(self.planner.names));good=q.copy();good[0]=1.
        identity=np.eye(3);zero=np.zeros(3)
        def fk(_chain,value,*unused):
            if value[0] == 0:
                return np.array([.000100001,0,0]),identity
            return np.array([.00008,0,0]),Rotation.from_euler('z',.16,degrees=True).as_matrix()
        import valve_plan
        with mock.patch.object(valve_plan, 'least_squares', side_effect=[SimpleNamespace(x=q),SimpleNamespace(x=good)]), \
             mock.patch.object(arm_kin, 'ee_world', side_effect=fk):
            actual=self.planner._solve(q,zero,identity,zero,identity,zero,multistart=True,
                                       position_precision_m=.0001)
        np.testing.assert_array_equal(actual,good)

    def test_sub_half_budget_midpoint_still_defines_grasp_center(self):
        snapshot=copy.deepcopy(self.snapshot);geometry=snapshot['geometry']
        measured=np.array(geometry['center']);half=np.array(geometry['size_mm'])/2000.
        geometry['wheel_pose']={'position':measured.tolist(),'orientation':[1.,0.,0.,0.]}
        geometry['support_vertices_local_m']=[(np.array([x,y,z])+[.00024,0,0]).tolist()
            for x in (-half[0],half[0]) for y in (-half[1],half[1]) for z in (-half[2],half[2])]
        plan=self.planner.plan(snapshot,body_xy=(.30,.15),lift_delta=-.20,roll_deg=0,
                               allow_legacy_grasp_reference=True)
        self.assertAlmostEqual(plan['grasp_midpoint_alignment']['applied_jaw_axis_correction_m'],.00024,places=10)
        self.assertFalse(plan['grasp_midpoint_alignment']['misalignment_exceeds_half_squeeze_budget'])
        alignment=plan['grasp_midpoint_alignment']['applied_jaw_axis_correction_m']
        np.testing.assert_allclose(plan['grasp_tcp'],measured+alignment*np.array(plan['grasp_support']['jaw_axis_world']),atol=1e-10)

    def test_corrupt_independent_anchor_rejected(self):
        bad = copy.deepcopy(self.snapshot)
        bad['arm_base']['position'][0] += .02
        with self.assertRaisesRegex(PlanningError, '独立 FK 锚点失败'):
            self.planner.independent_anchor(bad)
        duplicate = copy.deepcopy(self.snapshot)
        duplicate['state']['joint_names'][0] = duplicate['state']['joint_names'][1]
        with self.assertRaisesRegex(PlanningError, '名字重复'):
            self.planner.independent_anchor(duplicate)

    def test_true_axis_orbit_preserves_radius_and_axial_component(self):
        plan = self.preview
        c, axis = np.asarray(plan['axis_point']), np.asarray(plan['axis_world'])
        start = np.asarray(plan['center']) - c
        for row in plan['stages']['turn_model']:
            offset = np.asarray(row['tcp']) - c
            self.assertAlmostEqual(np.linalg.norm(offset), np.linalg.norm(start), places=12)
            self.assertAlmostEqual(np.dot(offset, axis), np.dot(start, axis), places=12)
        end = np.asarray(plan['stages']['turn_model'][-1]['tcp'])
        # 偏心位置必须移动；固定 bbox 中心只转朝向会漏掉这段弧。
        self.assertGreater(np.linalg.norm(end - plan['center']), .0008)

    def test_arc_replan_is_equivariant_for_arbitrary_world_axis(self):
        plan = self.preview
        row = plan['stages']['close_model'][-1]
        q, p, R, off = (np.asarray(row[k]) for k in ('q', 'tcp', 'R', 'tcp_off'))
        A, b, c = (np.asarray(plan[k]) for k in ('R_ab', 't_ab', 'axis_point'))
        a1 = self.planner.plan_arc(q, p, R, off, c, [0, 0, 1], 12., A, b)
        H = Rotation.from_euler('xyz', [.6, -.4, .8]).as_matrix()
        translation = np.array([.5, -.3, 1.])
        a2 = self.planner.plan_arc(q, H @ p + translation, H @ R, off,
                                   H @ c + translation, H[:, 2], 12., H @ A,
                                   H @ b + translation)
        np.testing.assert_allclose([r['q'] for r in a1['waypoints']],
                                   [r['q'] for r in a2['waypoints']], atol=1e-6, rtol=0)
        final = a2['waypoints'][-1]
        expected = H @ (c + axis_rotation([0, 0, 1], 12) @ (p - c)) + translation
        np.testing.assert_allclose(final['tcp'], expected, atol=1e-12)

    def test_bad_replan_feedback_and_step_are_rejected(self):
        plan = self.preview
        row = plan['stages']['close_model'][-1]
        args = [np.asarray(row[k]) for k in ('q', 'tcp', 'R', 'tcp_off')]
        tail = [plan['axis_point'], plan['axis_world'], 10., plan['R_ab'], plan['t_ab']]
        bad = [x.copy() for x in args]
        bad[1][0] += .02
        with self.assertRaisesRegex(PlanningError, 'arc_anchor IK 失败'):
            self.planner.plan_arc(*bad, *tail)
        with self.assertRaisesRegex(PlanningError, '旋转步长'):
            self.planner.plan_arc(*args, *tail, angular_step_deg=2.)
        bad_R = [x.copy() for x in args]
        bad_R[2] *= .5
        with self.assertRaisesRegex(PlanningError, '旋转矩阵'):
            self.planner.plan_arc(*bad_R, *tail)
        with self.assertRaisesRegex(PlanningError, '关节跳变'):
            self.planner._check_step(args[0], args[0] + .3, 8., 'test')

    def test_tool_axis_faces_opposite_valve_axis(self):
        axis = np.array([.3, .4, .5])
        axis /= np.linalg.norm(axis)
        R = grasp_rotation(axis, 35.)
        np.testing.assert_allclose(R[:, 2], -axis, atol=1e-12)
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(R), 1.)


if __name__ == '__main__':
    unittest.main(verbosity=2)
