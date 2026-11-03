"""接触阻挡状态的机械停稳窗口；只使用实测位姿，不写状态或目标。

PhysX接触时DOF速度可能与逐帧关节/刚体位移不一致。保留该原始读数，
用双侧连续承载、全部夹爪关节及双指垫的实际运动共同判断机械停稳。
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation


class ContactHoldWindow:
    def __init__(self, policy):
        self.policy = policy
        self.previous = self.anchor = None
        self.since = None

    def update(self, snapshot, *, now, both_loaded):
        state = snapshot['state']
        names = state['joint_names']
        if len(names) != len(set(names)):
            raise ValueError('夹持关节名重复')
        indices = [i for i, name in enumerate(names) if name.startswith('right_gripper_')]
        if not indices:
            raise ValueError('缺少实际右夹爪关节')
        q = np.asarray(state['positions'], float)
        velocity = np.asarray(state['velocities'], float)
        if q.shape != (len(names),) or velocity.shape != q.shape:
            raise ValueError('夹持反馈维度不符')
        pads = snapshot['state']['sides']['right']['pad_poses']
        p = np.array([pads[k]['position'] for k in ('left', 'right')], float)
        quat = np.array([pads[k]['orientation'] for k in ('left', 'right')], float)
        if (not math.isfinite(now) or not np.isfinite(q).all() or not np.isfinite(velocity).all()
                or p.shape != (2, 3) or quat.shape != (2, 4) or not np.isfinite(p).all()
                or not np.isfinite(quat).all() or np.any(np.linalg.norm(quat, axis=1) < 1e-9)):
            raise ValueError('实际夹爪/指垫反馈无效')
        rotation = Rotation.from_quat(quat, scalar_first=True)
        sample = (now, q[indices].copy(), p.copy(), rotation)
        row = {'method': 'continuous_bilateral_load_and_measured_joint_pad_motion',
               'all_gripper_sdk_speed_peak_rad_s': float(abs(velocity[indices]).max()),
               'joint_motion_speed_rad_s': None, 'pad_motion_speed_m_s': None,
               'pad_motion_speed_rad_s': None, 'dwell_s': 0., 'stopped': False}
        previous, self.previous = self.previous, sample
        if previous is not None and now <= previous[0]:
            raise ValueError('夹持停稳窗口物理时钟未推进')
        if not both_loaded:
            self.since = self.anchor = None
            row['reason'] = 'waiting_bilateral_load'
            return row
        if previous is None:
            self.anchor, self.since = sample, now
            row['reason'] = 'starting_motion_window'
            return row
        dt = now - previous[0]
        nominal_dt = float(snapshot['physics_dt'])
        if not math.isfinite(nominal_dt) or nominal_dt <= 0:
            raise ValueError('夹持物理步长无效')
        if dt > nominal_dt * 1.01 + 1e-6:
            self.since = self.anchor = None
            row['reason'] = 'missing_physics_step_feedback'
            return row
        qspeed = float(abs(sample[1] - previous[1]).max() / dt)
        pspeed = float(np.linalg.norm(p - previous[2], axis=1).max() / dt)
        rspeed = float((rotation * previous[3].inv()).magnitude().max() / dt)
        row.update(joint_motion_speed_rad_s=qspeed, pad_motion_speed_m_s=pspeed,
                   pad_motion_speed_rad_s=rspeed)
        # 使用现有速度/到位预算；不能只用一段位姿总极差放过来回快速振荡。
        vlimit = self.policy['settle_velocity_rad_s']
        plimit = self.policy['settle_tcp_position_m'] / self.policy['settle_dwell_s']
        moving = qspeed >= vlimit or rspeed >= vlimit or pspeed >= plimit
        if moving:
            self.since = self.anchor = None
            row['reason'] = 'actual_joint_or_pad_moving'
            return row
        if self.anchor is None:
            self.anchor, self.since = sample, now
        anchor = self.anchor
        drift = (abs(sample[1] - anchor[1]).max() >= self.policy['settle_position_rad']
                 or np.linalg.norm(p - anchor[2], axis=1).max() >= self.policy['settle_tcp_position_m']
                 or np.rad2deg((rotation * anchor[3].inv()).magnitude().max()) >= self.policy['settle_tcp_rotation_deg'])
        if drift:
            self.since = self.anchor = None
            row['reason'] = 'actual_joint_or_pad_drifting'
            return row
        row['dwell_s'] = now - self.since
        row['stopped'] = row['dwell_s'] >= self.policy['contact_dwell_s']
        row['reason'] = 'qualified' if row['stopped'] else 'waiting_continuous_dwell'
        return row
