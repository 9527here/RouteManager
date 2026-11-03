#!/usr/bin/env python3
"""使用当前 RM65 人形真实指垫验证新按钮，弹簧目标保持不变。"""
import json
from pathlib import Path
from types import SimpleNamespace

from cabinet_contact import CabinetContact


class L3RobotContact(CabinetContact):
    def __init__(self, executor):
        self.executor = executor
        self.station = executor._l3_station
        self.output = self.station.output
        self.side = 'right'
        self.events = []
        original = executor._cabinet_probe
        root = '/World/L3Station/Controls/object_0_variant'
        self.probe = SimpleNamespace(stage=self.station.runtime.stage, iface=original.iface,
            controls={'Btn05': {'body0': root + '/Base', 'body1': root + '/Cap',
                               'joint': root + '/Joints/button_slide', 'kind': 'linear'}},
            _pose=original._pose, _write=self.write, capture=self.capture)

    def write(self, name, data):
        (self.output / name).write_text(json.dumps(data, indent=2) + '\n')

    def capture(self, name):
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        capture_viewport_to_file(get_active_viewport(), str(self.output / name))

    def _guard(self):
        # 用户已明确暂缓旧裸机模型的自碰撞验证；新外壳URDF到位后可重新启用。
        if getattr(self.executor, '_defer_self_collision', False):
            return None
        guard = getattr(self.executor, '_collision_guard', None)
        return guard if guard is not None and guard.enabled else None

    def _contact_allowed(self, allowed):
        guard = self._guard()
        if guard is not None:
            guard.contact_allowed = allowed

    def plan_button(self, name='Btn05', approach_gap=.035):
        # 移动底盘漂移后，纯正对法兰路点可能超出臂展；仅在±20°内搜索接近方向。
        # 所有路点都先通过1mm/1°的IK检查，成功后才允许发手臂指令。
        guard = self._guard()
        base_q = guard.refresh() if guard is not None else None
        attempts = []
        for yaw in (0., -10., 10., -20., 20.):
            self._contact_step()
            try:
                plan = super().plan_button(name, approach_gap, approach_yaw_deg=yaw,
                    candidate_validator=(lambda arm: guard.arm_clear(arm, base_q)) if guard is not None else None)
            except RuntimeError as error:
                if not str(error).startswith('gap='):
                    raise
                attempts.append({'yaw_deg': yaw, 'error': str(error)})
                continue
            plan['approach_attempts'] = attempts
            return plan
        self.write('plugin_approach_failure.json', attempts)
        raise RuntimeError('当前站位超出手臂可达范围，请调整机器人与按钮的相对位置')

    def _contact_step(self):
        self.executor.step(render=True)
        if getattr(self.station, '_robot_cancel', False):
            raise InterruptedError('用户停止人形动作')

    def _motion_profile(self):
        """演示速度与模型上限分开记录，不能把参考速度误当成实物额定值。"""
        if hasattr(self, '_speed_profile'):
            return self._speed_profile
        import numpy as np
        import xml.etree.ElementTree as ET
        path = Path(__file__).resolve().parents[1] / 'assets/l3_operable/mobile_operation_profile.json'
        profile = json.loads(path.read_text()) if path.is_file() else {}
        reference = float(profile.get('arm_max_joint_speed_rad_s', .5))
        tcp_speed = float(profile.get('press_tcp_speed_m_s', .02))
        if not np.isfinite([reference, tcp_speed]).all() or min(reference, tcp_speed) <= 0.:
            raise ValueError('人形参考运动速度必须为有限正数')
        names = [f'right_arm_{i}_joint' for i in range(1, 7)]
        source = self.executor._config['robot']['presets']['v3_ctag2f120_rm65']['urdf_path']
        joints = {joint.get('name'): joint for joint in ET.parse(source).getroot().findall('joint')}
        try:
            urdf_limits = np.array([float(joints[name].find('limit').get('velocity')) for name in names])
        except (KeyError, AttributeError, TypeError, ValueError) as error:
            raise RuntimeError('当前右臂URDF缺少有效速度上限，不能生成提速轨迹') from error
        robot = self.executor.articulation
        if hasattr(robot, 'get_joint_max_velocities'):
            physics_limits = np.asarray(robot.get_joint_max_velocities(joint_names=names), dtype=float).reshape(-1)
        else:
            values = np.asarray(robot._physics_view.get_dof_max_velocities(), dtype=float).reshape(-1)
            physics_limits = values[[robot.dof_names.index(name) for name in names]]
        if (urdf_limits.shape != (6,) or physics_limits.shape != (6,)
                or not np.isfinite(urdf_limits).all() or not np.isfinite(physics_limits).all()
                or np.any(urdf_limits <= 0.) or np.any(physics_limits <= 0.)):
            raise RuntimeError('当前右臂模型或物理速度上限异常，不能生成提速轨迹')
        effective = np.minimum(np.minimum(urdf_limits, physics_limits), reference)
        self._speed_profile = {'status': '仿真参考速度，待实测', 'calibrated_to_user_hardware': False,
            'arm_max_joint_speed_rad_s': reference, 'press_tcp_speed_m_s': tcp_speed,
            'urdf_joint_velocity_limits_rad_s': urdf_limits.tolist(),
            'physics_joint_velocity_limits_rad_s': physics_limits.tolist(),
            'effective_joint_speed_limits_rad_s': effective.tolist(),
            'settle_position_tolerance_rad': .005, 'settle_velocity_tolerance_rad_s': .05,
            'settle_stable_time_s': .1, 'urdf_source': source}
        return self._speed_profile

    def _segment_frames(self, start, goal, dt, *, tcp_distance_m=None):
        import numpy as np
        if not np.isfinite(dt) or dt <= 0.:
            raise RuntimeError('仿真渲染步长异常')
        profile = self._motion_profile()
        limits = np.asarray(profile['effective_joint_speed_limits_rad_s'])
        duration = float(np.max(1.5 * abs(np.asarray(goal)-np.asarray(start)) / limits))
        minimum = 2
        if tcp_distance_m is not None:
            duration = max(duration, 1.5 * abs(float(tcp_distance_m)) / profile['press_tcp_speed_m_s'])
            minimum = 12
        return max(minimum, int(np.ceil(duration/dt)))

    def _arm_feedback(self):
        import numpy as np
        robot = self.executor.articulation
        names = [f'right_arm_{i}_joint' for i in range(1, 7)]
        indices = [robot.dof_names.index(name) for name in names]
        position = np.asarray(robot.get_joint_positions()).reshape(-1)[indices]
        try:
            velocity = np.asarray(robot.get_joint_velocities()).reshape(-1)[indices]
        except (AttributeError, IndexError, TypeError, RuntimeError):
            velocity = None
        if not np.isfinite(position).all() or (velocity is not None and not np.isfinite(velocity).all()):
            self.station.stop()
            raise RuntimeError('手臂位置或速度反馈异常，已停止后续动作')
        return position, velocity

    def _wait_arm_settled(self, target, max_frames):
        """到位并低速保持约0.1秒即可继续；原settle参数只保留为等待上限。"""
        import numpy as np
        profile = self._motion_profile()
        dt = float(self.executor.world.get_rendering_dt())
        target = np.asarray(target)
        stable = 0.
        previous, velocity = self._arm_feedback()
        actual = previous
        count = 0
        velocity_source = '物理关节速度' if velocity is not None else '相邻位置差分'
        for count in range(1, max(0, int(max_frames)) + 1):
            self._contact_step()
            actual, velocity = self._arm_feedback()
            if velocity is None:
                velocity = (actual-previous)/dt
            position_error = float(np.max(abs(actual-target)))
            speed = float(np.max(abs(velocity)))
            if (position_error <= profile['settle_position_tolerance_rad']
                    and speed <= profile['settle_velocity_tolerance_rad_s']):
                stable += dt
                if stable + 1e-12 >= profile['settle_stable_time_s']:
                    break
            else:
                stable = 0.
            previous = actual
        error = float(np.max(abs(actual-target)))
        self._last_settle = {'frames': count, 'max_frames': int(max_frames), 'stable_time_s': stable,
                             'position_error_rad': error, 'velocity_source': velocity_source}
        if error > .025:
            self.station.stop()
            raise RuntimeError('手臂跟踪误差过大，已停止后续动作')
        return error

    def _move_arm(self, target, frames=180, settle=120):
        import numpy as np
        guard = self._guard()
        names = [f'right_arm_{i}_joint' for i in range(1, 7)]
        if guard is not None:
            path = guard.plan(target)
            self.motion_paths.append(dict(guard.last_path))
        else:
            start, _ = self._arm_feedback()
            goal = np.asarray(target, dtype=float)
            if goal.shape != start.shape or not np.isfinite(goal).all():
                raise ValueError('手臂目标维度或数值异常')
            path = [start, goal]
            self.motion_paths.append({'collision_check_enabled': False,
                'method': 'joint_interpolation', 'start': start.tolist(), 'goal': goal.tolist()})
        dt = float(self.executor.world.get_rendering_dt())
        if not hasattr(self, 'motion_timing'):
            self.motion_timing = []
        for start, goal in zip(path, path[1:]):
            # frames保留旧调用兼容，不再把短段强制拉长到180帧；长段仍服从模型/参考限速。
            count = self._segment_frames(start, goal, dt)
            self.motion_timing.append({'kind': 'arm_interpolation', 'frames': count,
                'legacy_reference_frames': int(frames), 'rendering_dt_s': dt,
                'command_peak_joint_speed_rad_s': (1.5*abs(goal-start)/(count*dt)).tolist()})
            for frame in range(1, count + 1):
                if getattr(self.station, '_robot_cancel', False):
                    raise InterruptedError('用户停止人形动作')
                u = frame / count
                q = start + (goal - start) * u * u * (3 - 2 * u)
                self.executor.send_joint_command(q.tolist(), duration=0, joint_names=names)
                self._contact_step()
        error = self._wait_arm_settled(target, settle)
        self.motion_timing.append({'kind': 'arm_feedback_settle', **self._last_settle})
        return error

    def _retarget_button_waypoint(self, plan, waypoint, cycle):
        """保持未按下按钮的世界基准，只按当前机身/底盘反馈重算法兰关节目标。"""
        import numpy as np
        from scipy.spatial.transform import Rotation
        import arm_kin
        chain, seed, A, base, limits, names = self._kinematics()
        if list(names) != list(plan['joint_names']):
            raise RuntimeError('按压规划与当前手臂关节顺序不一致')
        surface = np.asarray(plan['surface'], dtype=float)
        R = np.asarray(plan['R'], dtype=float)
        tip = np.asarray(plan['tip_local'], dtype=float)
        gap = float(waypoint['gap'])
        # 不能读取正在被压动的Cap作为新基准，否则目标会随按钮退让而继续向内追赶。
        target_tcp = surface - np.array([gap, 0., 0.])
        target_flange = target_tcp - R @ tip
        row = {'cycle': int(cycle), 'gap_m': gap, 'surface_reference_frozen': True,
               'unpressed_surface_world': surface.tolist(), 'target_tcp_world': target_tcp.tolist(),
               'target_flange_world': target_flange.tolist(), 'arm_base_world': np.asarray(base).tolist(),
               'arm_base_rotation_world': np.asarray(A).tolist(), 'seed_actual_q': np.asarray(seed).tolist()}
        if not hasattr(self, 'retarget_history'):
            self.retarget_history = []
        try:
            goal = self._solve_pose(chain, A, base, limits, target_flange, R, seed)
            position, rotation = arm_kin.ee_world(chain, goal, A, base)
            position_error = float(np.linalg.norm(position-target_flange))
            orientation_error = float(Rotation.from_matrix(R @ rotation.T).magnitude())
            if (not np.isfinite(goal).all() or position_error > .001
                    or orientation_error > np.deg2rad(1.)):
                raise RuntimeError('按钮路点重规划未通过1毫米/1度门限')
        except RuntimeError as error:
            row.update(pass_ik=False, error=str(error))
            self.retarget_history.append(row)
            self.write('robot_button_retarget.json', self.retarget_history)
            raise RuntimeError('按压路点重规划失败：' + str(error).replace('旋钮', '按钮')) from error
        row.update(pass_ik=True, goal_q=np.asarray(goal).tolist(), position_error_mm=position_error*1000.,
                   orientation_error_deg=float(np.rad2deg(orientation_error)),
                   max_joint_change_rad=float(np.max(abs(np.asarray(goal)-np.asarray(seed)))))
        self.retarget_history.append(row)
        self.write('robot_button_retarget.json', self.retarget_history)
        return np.asarray(goal)

    def button(self, cycles=3):
        import numpy as np
        from pxr import UsdPhysics
        if self.station.queue:
            raise RuntimeError('请先停止机构测试负载，不能和人形接触同时加载')
        guard = self._guard()
        self.motion_paths = []
        self.motion_timing = []
        self.retarget_history = []
        speed_profile = self._motion_profile()
        if guard is not None:
            guard.contact_allowed = False
            guard.events = []
            if guard.violations(guard.refresh()):
                raise RuntimeError('起始姿态碰撞或净空不足，已禁止人形动作')
        runtime = self.station.runtime
        view = runtime.devices['object_0_variant']
        drive = UsdPhysics.DriveAPI(self.probe.stage.GetPrimAtPath(self.probe.controls['Btn05']['joint']), 'linear')
        original_target = float(drive.GetTargetPositionAttr().Get())
        original_state = self._state()
        arm_names = [f'right_arm_{i}_joint' for i in range(1, 7)]
        saved_q = [original_state['positions'][original_state['joint_names'].index(n)] for n in arm_names]
        self.executor.set_gripper_position(side='right', position=7)
        for _ in range(120):
            self._contact_step()
        result = {'method': 'RM65 actual finger pad contact; no button actuation command',
                  'preload_drive_target_m': original_target, 'cycles': [], 'pass': False,
                  'motion_speed_reference': speed_profile,
                  'collision_check_enabled': guard is not None,
                  'collision_validation': 'enabled' if guard is not None else 'deferred_pending_shell_urdf'}
        self._listen('Btn05')
        try:
            for cycle in range(cycles):
                self.station._status = '人形动作：正在规划接近姿态'
                # 移动底盘在接触反力下会有毫米级位移，每次用最新基座/指垫重规划。
                self._contact_allowed(True)
                plan = self.plan_button('Btn05', approach_gap=.035)
                self._contact_allowed(False)
                self.write('new_button_plan.json', plan)
                self.write(f'robot_button_plan_{cycle + 1}.json', plan)
                self.contact_total = 0
                self.max_contact_impulse = 0.
                row = {'cycle': cycle + 1, 'samples': []}
                self.station._status = '人形动作：正在接近绿色按钮'
                self._move_arm(plan['waypoints'][0]['q'], frames=180, settle=90)
                self.station._status = '人形动作：正在按压按钮'
                self._contact_allowed(True)
                previous_gap = float(plan['waypoints'][0]['gap'])
                for waypoint in plan['waypoints'][1:]:
                    # 触发到位立即退出，避免用手臂伺服继续挤压4.3mm硬止挡。
                    if waypoint['gap'] < -.004:
                        break
                    goal = self._retarget_button_waypoint(plan, waypoint, cycle + 1)
                    start, _ = self._arm_feedback()
                    if guard is not None:
                        if len(guard.plan(goal)) != 2:
                            raise RuntimeError('按压直线路径受阻，已停止')
                        self.motion_paths.append(dict(guard.last_path))
                    dt = float(self.executor.world.get_rendering_dt())
                    count = self._segment_frames(start, goal, dt,
                        tcp_distance_m=abs(float(waypoint['gap'])-previous_gap))
                    for frame in range(1, count + 1):
                        u = frame / count
                        command = start + (goal - start) * u * u * (3 - 2 * u)
                        self.executor.send_joint_command(command.tolist(), duration=0, joint_names=arm_names)
                        self._contact_step()
                        runtime.step(0.)
                        feedback = runtime.states['object_0_variant']
                        if feedback['pressed'] and feedback['stroke_m'] >= .0029:
                            break
                    self.motion_timing.append({'kind': 'button_contact_approach', 'planned_frames': count,
                        'executed_frames': frame, 'from_gap_m': previous_gap, 'to_gap_m': float(waypoint['gap']),
                        'rendering_dt_s': dt,
                        'command_peak_joint_speed_rad_s': (1.5*abs(goal-start)/(count*dt)).tolist()})
                    previous_gap = float(waypoint['gap'])
                    self._wait_arm_settled(command, 30)
                    self.motion_timing.append({'kind': 'button_feedback_settle', **self._last_settle})
                    runtime.step(0.)
                    sample = dict(runtime.states['object_0_variant'])
                    row['samples'].append(sample)
                    if sample['stroke_m'] >= .0028 and sample['pressed']:
                        break
                row['pressed'] = dict(runtime.states['object_0_variant'])
                self.capture(f'robot_button_pressed_{cycle + 1}.png')
                self.station._status = '人形动作：正在松开并回撤'
                self._move_arm(plan['waypoints'][0]['q'], frames=180, settle=180)
                self._contact_allowed(False)
                runtime.step(0.)
                row['released'] = dict(runtime.states['object_0_variant'])
                row['max_contact_impulse_Ns'] = self.max_contact_impulse
                row['contact_events'] = self.contact_total
                row['pass'] = bool(row['pressed']['pressed'] and row['pressed']['indicator_on']
                                   and not row['released']['pressed'] and row['released']['stroke_m'] < .0002
                                   and self.max_contact_impulse > 1e-8
                                   and abs(float(drive.GetTargetPositionAttr().Get()) - original_target) < 1e-8)
                result['cycles'].append(row)
                self.write('robot_button_contact.json', result)
                if not row['pass']:
                    break
            result['pass'] = all(r['pass'] for r in result['cycles']) and len(result['cycles']) == cycles
        finally:
            self.subscription = None
            if getattr(self.station, '_robot_cancel', False):
                # 停止时保持实测手臂姿态，不自动执行尚未确认的回程。
                state = self._state()
                hold = [state['positions'][state['joint_names'].index(n)] for n in arm_names]
                self.executor.send_joint_command(hold, duration=0, joint_names=arm_names)
                result['cancelled'] = True
            else:
                self.station._status = '人形动作：正在恢复手臂姿态'
                self._move_arm(saved_q, frames=180, settle=180)
            self._contact_allowed(False)
            result['collision_events'] = list(guard.events) if guard is not None else []
            result['motion_paths'] = self.motion_paths
            result['motion_timing'] = self.motion_timing
            result['button_retarget_history'] = self.retarget_history
            result['collision_checked_paths'] = self.motion_paths if guard is not None else []
            result['contact_samples'] = self.events
            result['final_drive_target_m'] = float(drive.GetTargetPositionAttr().Get())
            result['robot_finite'] = bool(np.isfinite(self.executor.articulation.get_joint_positions()).all())
            self.write('robot_button_contact.json', result)
            self.capture('robot_button_released.png')
        return result
