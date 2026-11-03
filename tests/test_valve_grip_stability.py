"""机械夹持停稳窗口的故障注入；仅synthetic，不代替真实接触验收。"""

from _bootstrap import ROOT as _TEST_ROOT  # 统一离线测试的项目/工具导入路径
import copy
import unittest
from valve_grip_stability import ContactHoldWindow
from valve_runtime import REFERENCE_POLICY


def sample():
    return {'physics_dt': .01, 'state': {'joint_names': ['right_gripper_right_2_joint'],
        'positions': [.6], 'velocities': [-.28], 'sides': {'right': {'pad_poses': {
            side: {'position': [x, 0., 0.], 'orientation': [1., 0., 0., 0.]}
            for side, x in [('left', -.04), ('right', .04)]}}}}}


class ContactHoldRegression(unittest.TestCase):
    def test_actual_hold_with_sdk_bias_keeps_raw_diagnostic(self):
        window = ContactHoldWindow(REFERENCE_POLICY)
        for i in range(20):
            row = window.update(sample(), now=i*.01, both_loaded=True)
        self.assertTrue(row['stopped'])
        self.assertEqual(row['all_gripper_sdk_speed_peak_rad_s'], .28)
        self.assertEqual(row['joint_motion_speed_rad_s'], 0.)

    def test_moving_joint_cannot_pass_even_if_sdk_reports_zero(self):
        window = ContactHoldWindow(REFERENCE_POLICY)
        for i in range(40):
            s = sample();s['state']['positions'][0] += .001*i;s['state']['velocities'] = [0.]
            self.assertFalse(window.update(s, now=i*.01, both_loaded=True)['stopped'])

    def test_moving_pad_cannot_pass_with_stationary_joint(self):
        for mode in ('translation', 'rotation'):
            window = ContactHoldWindow(REFERENCE_POLICY)
            for i in range(40):
                s = sample();pad = s['state']['sides']['right']['pad_poses']['right']
                if mode == 'translation':pad['position'][1] = i*.001
                else:
                    import math
                    pad['orientation'] = [math.cos(i*.001/2), 0., 0., math.sin(i*.001/2)]
                self.assertFalse(window.update(s, now=i*.01, both_loaded=True)['stopped'])

    def test_contact_loss_restarts_dwell(self):
        window = ContactHoldWindow(REFERENCE_POLICY)
        for i in range(15):window.update(sample(), now=i*.01, both_loaded=True)
        window.update(sample(), now=.15, both_loaded=False)
        for i in range(16, 30):
            self.assertFalse(window.update(sample(), now=i*.01, both_loaded=True)['stopped'])

    def test_missing_step_cannot_prove_continuous_hold(self):
        window = ContactHoldWindow(REFERENCE_POLICY)
        window.update(sample(), now=0., both_loaded=True)
        row = window.update(sample(), now=.5, both_loaded=True)
        self.assertFalse(row['stopped']);self.assertEqual(row['reason'], 'missing_physics_step_feedback')

    def test_bad_pose_time_or_velocity_rejected(self):
        for fault in ('nan_velocity', 'missing_pad', 'duplicate_time'):
            window = ContactHoldWindow(REFERENCE_POLICY);window.update(sample(), now=0., both_loaded=True)
            s = copy.deepcopy(sample())
            if fault == 'nan_velocity':s['state']['velocities'] = [float('nan')]
            if fault == 'missing_pad':del s['state']['sides']['right']['pad_poses']
            with self.assertRaises((ValueError, KeyError)):
                window.update(s, now=0. if fault=='duplicate_time' else .01, both_loaded=True)


if __name__ == '__main__':unittest.main(verbosity=2)
