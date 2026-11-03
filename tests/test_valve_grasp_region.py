"""局部夹持区域几何验证，原完整碰撞凸包必须保留。"""

from _bootstrap import ROOT as _TEST_ROOT  # 统一离线测试的项目/工具导入路径
import unittest
import numpy as np
from valve_grasp_region import apply_region
from valve_feedback import projected_wheel_width_mm


class RegionRegression(unittest.TestCase):
    def geometry(self):
        points=[[x,y,z]for y,half in [(0.,.0075),(.08,.003)]for x in(-half,half)for z in(-.005,.005)]
        return {'support_vertices_local_m':points,'wheel_pose':{'position':[0,0,0],'orientation':[1,0,0,0]}}
    def test_tapered_lever_uses_only_finite_pad_footprint_for_width(self):
        original=self.geometry();copy=np.array(original['support_vertices_local_m'])
        region=apply_region(original,{'point_local_m':[0,.06,0],'axis':'Y','half_span_m':.01})
        self.assertAlmostEqual(projected_wheel_width_mm(original,[1,0,0]),15.)
        self.assertAlmostEqual(projected_wheel_width_mm(region,[1,0,0]),9.375)
        np.testing.assert_array_equal(original['support_vertices_local_m'],copy)
        np.testing.assert_array_equal(region['support_vertices_local_m'],copy)
    def test_outside_or_bad_region_is_rejected(self):
        for p,span in [([0,1,0],.01),([1,.06,0],.01),([0,.06,0],-1)]:
            with self.assertRaises(ValueError):apply_region(self.geometry(),{'point_local_m':p,'axis':'Y','half_span_m':span})


if __name__=='__main__':unittest.main(verbosity=2)
