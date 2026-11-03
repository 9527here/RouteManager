#!/usr/bin/env python3
"""真实夹持支撑宽/中点的纯数学回归；无Isaac、HTTP或动力学放行。"""

from _bootstrap import ROOT as _TEST_ROOT  # 统一离线测试的项目/工具导入路径
import copy
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO), str(REPO/'scripts')]
from valve_feedback import wheel_support_interval, projected_wheel_width_mm
from valve_plan import ValvePlanner


def pose(position, R):
    return {'position': np.asarray(position).tolist(),
            'orientation': Rotation.from_matrix(R).as_quat(scalar_first=True).tolist()}


def nonround_points():
    xy = np.array([[.038585,0],[-.038585,0],[0,.0386],[0,-.0386],
                   [.04/np.sqrt(2),.04/np.sqrt(2)],[-.04/np.sqrt(2),-.04/np.sqrt(2)]])
    return np.array([[x,y,z] for x,y in xy for z in (-.016,.016)])


class SupportGeometry(unittest.TestCase):
    def geometry(self, points=None):
        points = nonround_points() if points is None else points
        return {'support_vertices_local_m': points.tolist(), 'wheel_pose': pose([.3,-1.2,.887],np.eye(3)),
                'radius_mm': 40., 'center': [.3,-1.2,.887], 'size_mm':[77.17,77.2,32.], 'yaw_deg':0.}

    def test_nonround_circumdiameter80_but_jaw_width77p17(self):
        geometry=self.geometry()
        self.assertAlmostEqual(max(np.linalg.norm(nonround_points()[:,:2],axis=1))*2000,80.)
        self.assertAlmostEqual(projected_wheel_width_mm(geometry,[1,0,0]),77.17,places=9)
        self.assertGreater(2*geometry['radius_mm']-projected_wheel_width_mm(geometry,[1,0,0]),2.8)

    def test_world_rigid_transform_preserves_width_and_midpoint_delta(self):
        geometry=self.geometry();axis=np.array([1.,.2,.1]);axis/=np.linalg.norm(axis)
        tcp=np.array([.301,-1.198,.889]);a=wheel_support_interval(geometry,axis)
        H=Rotation.from_euler('xyz',[.6,-.3,1.2]).as_matrix();translation=np.array([2.,-3.,.5])
        transformed=copy.deepcopy(geometry);transformed['wheel_pose']=pose(H@np.array(geometry['wheel_pose']['position'])+translation,H)
        b=wheel_support_interval(transformed,H@axis)
        self.assertAlmostEqual(a['width_mm'],b['width_mm'],places=9)
        self.assertAlmostEqual(a['midpoint_world_projection_m']-axis@tcp,
                               b['midpoint_world_projection_m']-(H@axis)@(H@tcp+translation),places=12)

    def test_eccentric_mid_alignment_is_necessary_for_both_faces(self):
        g=self.geometry();s=wheel_support_interval(g,[1,0,0]);width=s['width_mm']/1000
        mid=s['midpoint_world_projection_m'];opening=width-.0005
        wrong_mid=mid+.0007
        right_air_gap=wrong_mid+opening/2-s['maximum_world_projection_m']
        self.assertGreater(right_air_gap,0.)
        corrected=wrong_mid+(mid-wrong_mid)
        left_gap=s['minimum_world_projection_m']-(corrected-opening/2)
        right_gap=corrected+opening/2-s['maximum_world_projection_m']
        self.assertAlmostEqual(left_gap,-.00025,places=12)
        self.assertAlmostEqual(right_gap,-.00025,places=12)

    def test_circle_axis_case_retains80(self):
        theta=np.linspace(0,2*np.pi,64,endpoint=False)
        points=np.array([[.04*np.cos(a),.04*np.sin(a),z] for a in theta for z in (-.005,.005)])
        self.assertAlmostEqual(projected_wheel_width_mm(self.geometry(points),[1,0,0]),80.,places=9)

    def test_plan_uses_target_flange_axis_instead_of_transport_world_axis(self):
        planner=ValvePlanner.__new__(ValvePlanner);planner.side='right'
        planner._pad_positions=lambda **kwargs:np.array([[-.05,0,0],[.05,0,0]])
        snapshot={'geometry':self.geometry(),'state':{'sides':{'right':{'jaw_axis_world':[0,1,0]}}}}
        target=planner.grasp_support(snapshot,R_grasp=np.eye(3))
        measured=planner.grasp_support(snapshot)
        self.assertAlmostEqual(target['width_mm'],77.17,places=9)
        self.assertAlmostEqual(measured['width_mm'],77.2,places=9)
        np.testing.assert_allclose(target['jaw_axis_world'],[1,0,0])

    def test_actual_missing_support_refuses_legacy_is_explicit(self):
        old=self.geometry();del old['support_vertices_local_m']
        with self.assertRaisesRegex(ValueError,'实际运行禁止'):
            wheel_support_interval(old,[1,0,0])
        fallback=wheel_support_interval(old,[1,0,0],allow_legacy_fallback=True)
        self.assertTrue(fallback['legacy_reference_only'])
        with self.assertRaisesRegex(ValueError,'轴不对齐'):
            wheel_support_interval(old,[1,1,0],allow_legacy_fallback=True)

    def test_axis_input_not_mutated_and_bad_values_refuse(self):
        axis=np.array([2.,0.,0.]);before=axis.copy()
        wheel_support_interval(self.geometry(),axis)
        np.testing.assert_array_equal(axis,before)
        for bad in ([0,0,0],[float('nan'),0,0]):
            with self.assertRaises((RuntimeError,ValueError)):
                wheel_support_interval(self.geometry(),bad)


if __name__=='__main__':
    unittest.main(verbosity=2)
