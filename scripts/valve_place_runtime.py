"""固定当前舵向的受限退离执行器，只能由已有仿真主线程显式调用。

没有网络/启动入口；唯一运动出口是 set_base_velocity(vx=0,vy=参考值,wz=0)。
几何阈值完全来自本次 fresh plan_safe_retreat，不另加穿深容差或跳过96形状。
"""
import json
import math
from pathlib import Path
import sys
import time
import threading

import numpy as np
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[1]
for path in (str(REPO), str(REPO / 'scripts')):
    if path not in sys.path:
        sys.path.insert(0, path)
from valve_safety import ValveSafety
from valve_placement import plan_safe_retreat, validate_retreat_step
import valve_placement as placement


CONTROL_REFERENCE = {
    'max_body_speed_m_s': .015, 'max_acceleration_m_s2': .020,
    'goal_error_m': .003, 'cross_track_error_m': .003,
    'root_z_error_m': .003, 'orientation_change_deg': .5,
    'steering_error_deg': .5, 'nonwheel_joint_error_rad': .02,
    'rolling_error_rad': .05, 'wheel_speed_limit_rad_s': 1.,
    'stationary_linear_m_s': .002, 'stationary_angular_rad_s': .01,
    'stationary_wheel_rad_s': .03, 'stationary_position_window_m': .001,
    'stationary_angle_window_deg': .10,
    'initial_stability_s': .30, 'stop_dwell_s': .50,
    'stop_timeout_s': 8., 'move_timeout_s': 45.,
    'wall_timeout_s': 600., 'progress_period_s': .20,
    'target_inverse_numerical_m_s': 1e-6,
}

LOW_SPEED_POLICY_ID = 'low_speed_contact_escape_10mm_v1'
LOW_SPEED_APPROVED_MAXIMA = {
    'max_body_speed_m_s': .010, 'max_acceleration_m_s2': .005,
    'orientation_change_deg': .2, 'cross_track_error_m': .002,
    'root_z_error_m': .001, 'goal_error_m': .0015,
}


class RetreatRejected(RuntimeError):
    pass


class RetreatStopped(RuntimeError):
    pass


def _write(path, value):
    temporary = Path(path).with_suffix('.json.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))
    temporary.replace(path)


def _policy(override):
    policy = dict(CONTROL_REFERENCE)
    for key, value in (override or {}).items():
        if key not in policy or not math.isfinite(value) or value <= 0:
            raise ValueError('未知或无效退离参考控制阈值: ' + str(key))
        policy[key] = float(value)
    return policy


def _controllers(executor, owned_command=None):
    command = getattr(executor, '_base_cmd', None)
    if owned_command is None:
        if command is not None:
            raise RetreatRejected('已有底盘控制，不能竞争接管')
    elif command is not None:
        if command.get('parking') or any(abs(float(command.get(k, 0.)) - expected) > 1e-9
                                         for k, expected in zip(('vx', 'vy', 'wz'), owned_command)):
            raise RetreatRejected('底盘目标被外部控制改变，停止退离')
    parking = getattr(executor, '_base_parking', None)
    if parking is not None and not callable(parking) and bool(parking):
        raise RetreatRejected('外部回正驻车状态机仍激活')
    if (getattr(executor, '_motion', None) is not None or getattr(executor, '_auto_pick', None) is not None or
            getattr(executor, '_udp_targets', None) is not None or getattr(executor, '_ros2_contract', None) is not None):
        raise RetreatRejected('已有手臂/UDP/ROS控制输入，禁止退离')
    ros = getattr(executor, '_ros_controller', None)
    if ros is not None and ros.is_initialized():
        raise RetreatRejected('ROS关节控制器激活')
    vr = getattr(executor, '_vr_teleop', None)
    if vr is not None and getattr(vr, '_running', False):
        raise RetreatRejected('VR遥操作仍激活')
    for attribute in ('_l3_station', '_l3station'):
        owner = getattr(executor, attribute, None)
        if owner is None:
            continue
        if (getattr(owner, '_robot_busy', False) or getattr(owner, '_pending_action', None) is not None or
                getattr(owner, 'queue', ()) or getattr(owner, '_action_job', None) is not None or
                getattr(owner, '_legacy_job', None) is not None):
            raise RetreatRejected('其他工位待办或动作仍激活')
        approach = getattr(owner, '_approach', None)
        if approach is not None and approach.busy:
            raise RetreatRejected('其他底盘导航任务仍激活')


def _capture(executor, safety):
    q, poses = safety.refresh()
    velocities = np.asarray(executor.articulation.get_joint_velocities()).reshape(-1)
    velocity_targets = np.asarray(executor.articulation._physics_view.get_dof_velocity_targets()).reshape(-1)
    root_velocity = np.asarray(executor.articulation._physics_view.get_root_velocities()).reshape(-1, 6)[0]
    root = np.asarray(poses[safety.model.root], float)
    if (q.shape != velocities.shape or q.shape != velocity_targets.shape or
            not np.isfinite(q).all() or not np.isfinite(velocities).all() or not np.isfinite(velocity_targets).all() or
            not np.isfinite(root_velocity).all() or root.shape != (4, 4) or not np.isfinite(root).all()):
        raise RetreatRejected('实际全身关节/根速度/姿态反馈无效')
    if not executor.world.is_playing():
        raise RetreatRejected('仿真未在运行')
    return {'q': q, 'velocities': velocities, 'velocity_targets': velocity_targets,
            'root_velocity': root_velocity, 'root': root,
            'sim_time': float(executor.world.current_time)}


def _sealed_control_check(profile, policy):
    """独立低速模式只能使用不松于封存上限的显式控制参数；不替调用方改值。"""
    identity = profile.get('policy_id')
    if identity != LOW_SPEED_POLICY_ID:
        if profile.get('recovery_control_caps') is not None or profile.get('recovery_environment_margin_m', .012) < .012:
            raise RetreatRejected('不支持的profile试图携带低速控制上限/降低环境净空')
        return {'policy_id': identity, 'low_speed_reference_active': False}
    caps = profile.get('recovery_control_caps')
    if (not isinstance(caps, dict) or set(caps) != set(LOW_SPEED_APPROVED_MAXIMA) or
            profile.get('ordinary_environment_margin_m') != .012 or
            profile.get('recovery_environment_margin_m') != .010):
        raise RetreatRejected('低速10mm profile的policy/caps/普通12mm语义不完整')
    for key, approved in LOW_SPEED_APPROVED_MAXIMA.items():
        value = caps[key]
        if not math.isfinite(value) or value <= 0 or value > approved:
            raise RetreatRejected('封存低速控制cap超出已核定参考上界: ' + key)
        if policy[key] > value:
            raise RetreatRejected('control_policy松于封存低速profile，必须显式收紧: ' + key)
    return {'policy_id': identity, 'low_speed_reference_active': True,
            'sealed_control_caps': dict(caps),
            'effective_controls': {key: policy[key] for key in caps},
            'ordinary_environment_margin_m': .012, 'recovery_environment_margin_m': .010,
            'control_caps_automatically_changed': False,
            'normal_mode_defaults_changed': False}


def _final_clearance_check(chosen, record, profile, explicit_mode, policy):
    scope = chosen.get('goal_clearance_scope', 'all_environment')
    actual_environment = float(record['minimum_environment_clearance_m'])
    actual_leg = float(record['recovering_pair']['distance_m'])
    if not math.isfinite(actual_environment) or not math.isfinite(actual_leg):
        raise RetreatRejected('终点实际净空不是有限值')
    if scope == 'recovering_leg_pair':
        binding = _sealed_control_check(profile or {}, policy)
        if not explicit_mode or not binding['low_speed_reference_active']:
            raise RetreatRejected('只有明确封存的低速10mm模式可用Leg终点12mm语义')
        required_leg = max(.012, float(chosen['goal_clearance_m']))
        required_environment = profile['recovery_environment_margin_m']
        if chosen.get('recovery_environment_margin_m') != required_environment:
            raise RetreatRejected('低速计划终点环境阈值与profile不匹配')
        if actual_leg < required_leg or actual_environment < required_environment:
            raise RetreatRejected('停稳后Leg未清出12mm或其余环境未达到封存10mm')
    else:
        required_environment = float(chosen['goal_clearance_m'])
        required_leg = required_environment
        if actual_environment < required_environment:
            raise RetreatRejected('停稳后的全身环境净空未达到普通计划目标')
    return {'scope': scope, 'required_leg_clearance_m': required_leg,
            'required_environment_clearance_m': required_environment,
            'actual_leg_clearance_m': actual_leg, 'actual_environment_clearance_m': actual_environment}


def _observed_motion_check(safety, modules, profile, sample, *, control_policy=None, authorized_motion=False):
    """显式脱困起点仅接受自己的实测包络，不乘系数、不提升任一raw cap。"""
    validator = getattr(placement, 'validate_observed_contact_profile', None)
    if not callable(validator):
        raise RetreatRejected('观察接触profile验证器尚未就绪，禁止显式脱困')
    validated = validator(safety, profile, now_sim_time=sample['sim_time'])
    if not isinstance(validated, dict) or validated.get('valid') is not True:
        raise RetreatRejected('观察接触profile未严格验证通过')
    envelope = profile['start_motion_envelope']
    control = CONTROL_REFERENCE if control_policy is None else control_policy
    control_binding = _sealed_control_check(profile, control)
    wheel_limits = envelope['wheel_abs_velocity_max_rad_s']
    expected_names = [module['wheel_joint'] for module in modules]
    if set(wheel_limits) != set(expected_names):
        raise RetreatRejected('实测抖动包络的四轮名字不匹配')
    wheel_indices = [safety.model.names.index(name) for name in expected_names]
    wheel_actual = np.array([sample['velocities'][i] for i in wheel_indices], float)
    targets = np.asarray(sample['velocity_targets'], float)[wheel_indices]
    if not np.isfinite(targets).all() or (not authorized_motion and np.any(targets != 0.)):
        raise RetreatRejected('初始观察接触窗口必须保持原零轮速目标')
    bounds = np.array([wheel_limits[name] for name in expected_names], float)
    headings = np.array([module['heading_zero'] + module['steering_sign'] *
                          sample['q'][safety.model.names.index(module['steering_joint'])] for module in modules])
    radii = np.array([module['radius_m'] for module in modules], float)
    signs = np.array([module['wheel_sign'] for module in modules], float)
    urdf_caps = np.array([module['wheel_max_velocity_rad_s'] for module in modules], float)
    if (not np.isfinite(radii).all() or np.any(radii <= 0) or not np.isfinite(urdf_caps).all() or
            np.any(urdf_caps <= 0) or np.any(np.abs(signs) < 1e-9)):
        raise RetreatRejected('缺少合法URDF轮半径/正号/速度上限')
    lateral = np.sin(headings)
    if np.dot(lateral, lateral) < 1e-8:
        raise RetreatRejected('当前轮向不支持保持舵向body Y退离')
    linear_targets = targets * signs * radii
    vy_target = float(np.dot(lateral, linear_targets) / np.dot(lateral, lateral))
    inverse_error = float(np.max(np.abs(linear_targets - lateral * vy_target)))
    if (inverse_error > control['target_inverse_numerical_m_s'] or
            abs(vy_target) > control['max_body_speed_m_s'] + control['target_inverse_numerical_m_s']):
        raise RetreatRejected('native四轮目标不能解释为本次授权的纯body Y速度')
    authorized_world_velocity = sample['root'][:3, :3] @ np.array([0., vy_target, 0.])
    wheels = np.abs(wheel_actual - targets)
    linear = np.abs(sample['root_velocity'][:3] - authorized_world_velocity)
    angular = np.abs(sample['root_velocity'][3:])
    roll_pitch = np.abs(Rotation.from_matrix(sample['root'][:3, :3]).as_euler('xyz')[:2])
    groups = (
        ('wheel_velocity_disturbance_abs_rad_s', wheels, bounds),
        ('root_linear_velocity_disturbance_component_abs_m_s', linear,
         np.asarray(envelope['root_linear_velocity_component_abs_max_m_s'], float)),
        ('root_angular_velocity_component_abs_rad_s', angular,
         np.asarray(envelope['root_angular_velocity_component_abs_max_rad_s'], float)),
        ('roll_pitch_abs_rad', roll_pitch, np.asarray(envelope['roll_pitch_abs_max_rad'], float)),
    )
    measurements = {}
    for name, values, limits in groups:
        if values.shape != limits.shape or not np.isfinite(limits).all() or np.any(limits < 0):
            raise RetreatRejected('实测起点包络无效: ' + name)
        measurements[name] = {'actual': values.tolist(), 'observed_maximum': limits.tolist()}
        if np.any(values > limits):
            raise RetreatRejected('实际抖动超过自己预采实测包络，不能放宽: ' + name)
    # 显式模式的总速度界是原观测噪声与已授权确定性rolling信号相加，非噪声倍率。
    # 普通模式仍使用独立的wheel_speed_limit_rad_s=1，URDF物理上限绝不写改。
    authorized_wheel_signal_cap = control['max_body_speed_m_s'] / (np.abs(signs) * radii)
    explicit_raw_caps = np.minimum(urdf_caps, bounds + authorized_wheel_signal_cap)
    if np.any(np.abs(wheel_actual) > explicit_raw_caps):
        raise RetreatRejected('显式模式实际轮速超过原噪声+授权信号/URDF总速度界')
    return {'valid': True, 'profile_validation': validated, 'measurements': measurements,
            'threshold_expansion_factor': 1., 'observed_noise_caps_automatically_increased': False,
            'ordinary_mode_raw_cap_unchanged': True,
            'sealed_control_policy_validation': control_binding,
            'deterministic_signal': {'native_wheel_targets_rad_s': targets.tolist(),
                                     'raw_actual_wheel_velocity_rad_s': wheel_actual.tolist(),
                                     'inverse_body_vy_target_m_s': vy_target,
                                     'inverse_target_residual_m_s': inverse_error,
                                     'authorized_world_linear_velocity_m_s': authorized_world_velocity.tolist()},
            'explicit_total_wheel_speed_caps': {
                'joint_names': expected_names, 'original_noise_caps_rad_s': bounds.tolist(),
                'authorized_rolling_signal_caps_rad_s': authorized_wheel_signal_cap.tolist(),
                'actual_urdf_caps_rad_s': urdf_caps.tolist(), 'total_caps_rad_s': explicit_raw_caps.tolist(),
                'formula': 'min(actual_URDF_cap, original_observed_noise_max + authorized_maxBodySpeed/(abs(sign)*radius))',
                'ordinary_mode_raw_cap_rad_s': control['wheel_speed_limit_rad_s'],
                'urdf_modified': False}}


def _prepare(executor, output, distances_m, placement_options, policy,
             explicit_contact_recovery=False, observed_contact_profile=None, prepared_safety=None):
    if not isinstance(explicit_contact_recovery, bool):
        raise RetreatRejected('explicit_contact_recovery 必须是显式布尔值')
    if any(key in (placement_options or {}) for key in ('explicit_contact_recovery', 'observed_contact_profile')):
        raise RetreatRejected('接触脱困模式/profile不能通过通用规划参数隐式开启')
    if explicit_contact_recovery:
        if not isinstance(observed_contact_profile, dict) or prepared_safety is None:
            raise RetreatRejected('显式接触脱困必须同时有fresh profile和同实例prepared_safety')
        if prepared_safety.executor is not executor:
            raise RetreatRejected('prepared_safety不属于同一个当前executor')
    elif observed_contact_profile is not None or prepared_safety is not None:
        raise RetreatRejected('默认严格模式不接受观察profile/预备guard；必须显式开启接触脱困')
    _controllers(executor)
    # 显式模式不得重新capture覆写profile绑定的geometry/capture_id/hash。
    # guard必须由root本次fresh创建并用于同一实例预采时间序列，随后这里刷新实际反馈。
    safety = prepared_safety if explicit_contact_recovery else ValveSafety(executor, directory=output / 'geometry')
    model = executor._get_swerve_model()
    if model is None or len(model.modules) != 4:
        raise RetreatRejected('没有实际四轮八驱模型，禁止用旧差速参数退离')
    modules = [vars(module).copy() for module in model.modules]
    sample = _capture(executor, safety)
    envelope_check = (_observed_motion_check(safety, modules, observed_contact_profile, sample, control_policy=policy)
                      if explicit_contact_recovery else None)
    recovery_kwargs = ({'explicit_contact_recovery': True, 'observed_contact_profile': observed_contact_profile}
                       if explicit_contact_recovery else {})
    plan = plan_safe_retreat(safety, modules, distances_m=distances_m,
                            **(placement_options or {}), **recovery_kwargs)
    _write(output / 'plan.json', plan)
    public = {'safe': bool(plan.get('safe')), 'motion_started': False,
              'plan': plan, 'control_reference': policy,
              'thresholds_calibrated_to_hardware': False,
              'explicit_contact_recovery': explicit_contact_recovery,
              'requires_separate_user_authorization': explicit_contact_recovery,
              'observed_start_validation': envelope_check,
              'observed_contact_profile': observed_contact_profile,
              'sealed_control_policy_validation': envelope_check.get('sealed_control_policy_validation') if envelope_check else None,
              'geometry_policy_source': '本次fresh plan_safe_retreat.reference，无运行时补充穿深容差'}
    if not plan.get('safe') or plan.get('chosen') is None:
        return public, None
    chosen = plan['chosen']
    if (not chosen.get('completed_path') or not chosen.get('safe') or not chosen.get('samples') or
            any(not sample.get('safe_for_recovery_step') for sample in chosen['samples']) or
            not plan.get('no_chassis_rotation') or plan.get('steering_changed')):
        public.update(safe=False, reason='纯计划缺少逐点通过、固定车身/舵向的完整证据')
        return public, None
    for key in ('environment_margin_m', 'hand_clearance_m', 'distance_numerical_tolerance_m'):
        if key not in plan['reference'] or not math.isfinite(plan['reference'][key]):
            raise RetreatRejected('规划缺少明确几何阈值: ' + key)
    sample = _capture(executor, safety)
    record = validate_retreat_step(safety, plan, modules, refresh=False, now_sim_time=sample['sim_time'])
    if not record['safe_for_recovery_step']:
        public.update(safe=False, reason='fresh计划后的实际初态复核未通过', actual=record)
        return public, None
    public['actual'] = record
    return public, {'safety': safety, 'modules': modules, 'plan': plan, 'initial': sample, 'record': record,
                    'explicit_contact_recovery': explicit_contact_recovery,
                    'observed_contact_profile': observed_contact_profile}


def preflight_retreat(executor, output=None, distances_m=(.05, .10, .15, .30),
                      placement_options=None, control_policy=None, explicit_contact_recovery=False,
                      observed_contact_profile=None, prepared_safety=None):
    """只读fresh采样和重规划；不推进物理，不发送任何目标。"""
    output = Path(output or REPO / 'outputs/valve_operation_20261102/retreat')
    output.mkdir(parents=True, exist_ok=True)
    policy = _policy(control_policy)
    try:
        result, _ = _prepare(executor, output, distances_m, placement_options, policy,
                             explicit_contact_recovery, observed_contact_profile, prepared_safety)
    except Exception as error:
        result = {'safe': False, 'motion_started': False, 'error': str(error),
                  'error_type': type(error).__name__, 'control_reference': policy}
    _write(output / 'preflight.json', result)
    return result


class _RetreatRuntime:
    def __init__(self, executor, output, context, policy):
        self.executor, self.output, self.policy = executor, output, policy
        self.safety, self.modules, self.plan = (context[k] for k in ('safety', 'modules', 'plan'))
        self.initial, self.previous = context['initial'], context['record']
        self.explicit_contact_recovery = bool(context.get('explicit_contact_recovery', False))
        self.observed_contact_profile = context.get('observed_contact_profile')
        self.observed_start_checks = []
        self.observed_motion_checks = []
        self.last_observed_motion_check = None
        self.current = self.initial
        self.root0 = self.initial['root'].copy()
        self.q0 = self.initial['q'].copy()
        self.steering_indices = [self.safety.model.names.index(m['steering_joint']) for m in self.modules]
        self.wheel_indices = [self.safety.model.names.index(m['wheel_joint']) for m in self.modules]
        self.other_indices = [i for i in range(len(self.q0)) if i not in self.wheel_indices]
        self.previous_wheels = self.q0[self.wheel_indices].copy()
        self.wheel_accumulated = np.zeros(4)
        self.owned_command = None
        self.started, self.stop_verified = False, False
        self.wall_started, self.last_progress = time.monotonic(), -float('inf')
        self.phase, self.trace = 'initial_stability', []
        self._physics_time = float(self.initial['sim_time'])
        self._callback_uid, self._callback_holder, self._callback_manager = None, None, None
        self._callback_enabled = False
        self._in_physics_callback = False
        self._last_observed_time, self._last_actual_delta = None, None
        self.guard_error = None
        self._last_raw_world_time = float(self.initial['sim_time'])
        self._owner_thread = threading.get_ident()
        self._main_thread = threading.main_thread().ident
        self.physical_guard = {'physical_guard_callbacks_count': 0, 'maximum_callback_gap_s': 0.,
            'minimum_callback_gap_s': None, 'expected_physics_dt_s': float(executor.world.get_physics_dt()),
            'owner_thread_id': self._owner_thread, 'main_thread_id': self._main_thread,
            'callback_thread_ids': [], 'all_callbacks_on_main_thread': True,
            'callback_registered': False, 'callback_removed': False, 'first_guard_error': None,
            'clock_source': 'initial world time + every PHYSICS_POST_STEP dt, same as baseline sampling',
            'motion_commands_in_callback': 0}

    def _clock(self):
        return self._physics_time if self._callback_uid is not None else float(self.executor.world.current_time)

    def _remember_guard_error(self, error):
        if self.guard_error is None:
            self.guard_error = error
            self.physical_guard['first_guard_error'] = {
                'error': str(error), 'error_type': type(error).__name__, 'phase': self.phase,
                'physics_time': self._physics_time,
                'raw_world_time': self._last_raw_world_time}

    def _post_physics_guard(self, dt, context=None):
        """native边界内不抛异常、不写运动目标；首个失败交给外层循环刹停。"""
        if not self._callback_enabled:
            return
        self.physical_guard['physical_guard_callbacks_count'] += 1
        thread_id = threading.get_ident()
        if thread_id not in self.physical_guard['callback_thread_ids']:
            self.physical_guard['callback_thread_ids'].append(thread_id)
        if thread_id != self._owner_thread or thread_id != self._main_thread:
            self.physical_guard['all_callbacks_on_main_thread'] = False
            self._remember_guard_error(RetreatRejected('物理guard回调不在原仿真主线程，拒绝读取Isaac'))
            return
        try:
            if not math.isfinite(dt) or dt <= 0:
                raise RetreatRejected('post physics guard没有有效物理dt')
            expected = self.physical_guard['expected_physics_dt_s']
            if abs(float(dt) - expected) > max(1e-9, expected * 1e-6):
                raise RetreatRejected('post physics guard采样dt与当前physics_dt不匹配')
            self._physics_time += float(dt)
            self.physical_guard['maximum_callback_gap_s'] = max(self.physical_guard['maximum_callback_gap_s'], float(dt))
            old = self.physical_guard['minimum_callback_gap_s']
            self.physical_guard['minimum_callback_gap_s'] = float(dt) if old is None else min(old, float(dt))
            self._in_physics_callback = True
            self._observe(allow_stop_file=self.phase == 'stopping')
        except Exception as error:
            self._remember_guard_error(error)
        finally:
            self._in_physics_callback = False

    def _install_physics_guard(self):
        if self._owner_thread != self._main_thread:
            raise RetreatRejected('run_retreat不在Python主线程，禁止安装物理guard')
        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
        self._callback_manager = SimulationManager
        # 用普通closure保留SDK的(dt,context)签名，不依赖旧bound-method兼容包装。
        def post_guard(dt=None, context=None):
            self._post_physics_guard(dt, context)
        self._callback_holder = post_guard
        self._callback_enabled = True
        uid = SimulationManager.register_callback(post_guard, SimulationEvent.PHYSICS_POST_STEP)
        if uid is None:
            raise RetreatRejected('未取得物理guard的唯一注册UID')
        self._callback_uid = uid
        self.physical_guard['callback_registered'] = True
        self.physical_guard['callback_uid'] = int(uid)

    def _remove_physics_guard(self):
        self._callback_enabled = False
        if self._callback_uid is None:
            return
        uid = self._callback_uid
        try:
            removed = self._callback_manager.deregister_callback(uid)
            self.physical_guard['callback_removed'] = bool(removed)
            if removed is False:
                self.physical_guard['callback_cleanup_error'] = '本对象UID注销返回False'
        except Exception as error:
            self.physical_guard['callback_cleanup_error'] = str(error)
        finally:
            self._callback_uid = None
            self._callback_holder = None

    def _observe(self, *, allow_stop_file=False):
        if not allow_stop_file and (self.output / 'STOP').exists():
            raise RetreatStopped('合作停止文件存在；不会自动恢复')
        if time.monotonic() - self.wall_started > self.policy['wall_timeout_s']:
            raise RetreatRejected('退出执行墙钟超时')
        _controllers(self.executor, self.owned_command)
        if (not self._in_physics_callback and self._callback_uid is not None and
                self._last_observed_time == self._clock() and self._last_actual_delta is not None):
            # 外层rendered step后使用最后一次kernel的读回，不再向接触窗口插入dt=0样本。
            return self._last_actual_delta.copy()
        sample = _capture(self.executor, self.safety)
        self._last_raw_world_time = sample['sim_time']
        sample['raw_world_sim_time'] = sample['sim_time']
        sample['sim_time'] = self._clock()
        record = validate_retreat_step(self.safety, self.plan, self.modules, previous=self.previous,
                                       refresh=False, now_sim_time=sample['sim_time'])
        if not record['safe_for_recovery_step']:
            _write(self.output / 'actual_rejection.json', record)
            raise RetreatRejected('实际微扰/穿深/全身净空反馈拒绝，禁止继续移动')
        q, root = sample['q'], sample['root']
        delta = self.root0[:3, :3].T @ (root[:3, 3] - self.root0[:3, 3])
        relative = Rotation.from_matrix(root[:3, :3] @ self.root0[:3, :3].T).as_rotvec()
        if (abs(delta[0]) > self.policy['cross_track_error_m'] or abs(delta[2]) > self.policy['root_z_error_m'] or
                np.linalg.norm(relative) > math.radians(self.policy['orientation_change_deg'])):
            raise RetreatRejected('实际侧移横向误差/高度/车身旋转超过参考控制阈值')
        if np.max(np.abs(q[self.steering_indices] - self.q0[self.steering_indices])) > math.radians(self.policy['steering_error_deg']):
            raise RetreatRejected('实际舵角改变，拒绝用转向代替保持当前轮向退出')
        if np.max(np.abs(q[self.other_indices] - self.q0[self.other_indices])) > self.policy['nonwheel_joint_error_rad']:
            raise RetreatRejected('运输手臂/升降/其他关节发生非计划变化')
        wheel_q = q[self.wheel_indices]
        change = (wheel_q - self.previous_wheels + math.pi) % (2*math.pi) - math.pi
        self.wheel_accumulated += change
        self.previous_wheels = wheel_q.copy()
        expected = []
        for i, module in enumerate(self.modules):
            heading = module['heading_zero'] + module['steering_sign'] * self.q0[self.steering_indices[i]]
            along = math.cos(heading) * delta[0] + math.sin(heading) * delta[1]
            expected.append(along / (module['wheel_sign'] * module['radius_m']))
        rolling_error = float(np.max(np.abs(self.wheel_accumulated - expected)))
        if rolling_error > self.policy['rolling_error_rad']:
            raise RetreatRejected('四轮实际滚动与实际位移不一致，停止')
        if self.explicit_contact_recovery:
            self.last_observed_motion_check = _observed_motion_check(
                self.safety, self.modules, self.observed_contact_profile, sample,
                control_policy=self.policy, authorized_motion=self.phase != 'initial_stability')
        elif np.max(np.abs(sample['velocities'][self.wheel_indices])) > self.policy['wheel_speed_limit_rad_s']:
            raise RetreatRejected('轮速反馈超过普通模式参考上限')
        self.current, self.previous = sample, record
        self._last_observed_time, self._last_actual_delta = self._clock(), delta.copy()
        if self._clock() - self.last_progress >= self.policy['progress_period_s']:
            row = {'phase': self.phase, 'sim_time': sample['sim_time'],
                   'actual_body_displacement_m': delta.tolist(), 'orientation_change_rad': relative.tolist(),
                   'steering_actual_rad': q[self.steering_indices].tolist(),
                   'wheel_velocities_rad_s': sample['velocities'][self.wheel_indices].tolist(),
                   'rolling_error_rad': rolling_error, 'recovering_pair': record['recovering_pair'],
                   'minimum_environment_clearance_m': record['minimum_environment_clearance_m'],
                   'motion_started': self.started, 'stop_verified': self.stop_verified}
            _write(self.output / 'progress.json', row)
            self.trace.append(row)
            if self.last_observed_motion_check is not None:
                self.observed_motion_checks.append({'sim_time': sample['sim_time'], 'phase': self.phase,
                                                   **self.last_observed_motion_check})
            self.last_progress = self._clock()
        return delta

    def _step(self, **kwargs):
        before = self._clock()
        callbacks_before = self.physical_guard['physical_guard_callbacks_count']
        self.executor.step()
        if self.guard_error is not None:
            raise self.guard_error
        if (self._callback_uid is not None and
                self.physical_guard['physical_guard_callbacks_count'] <= callbacks_before):
            raise RetreatRejected('外层step没有产生任何post physics guard回调')
        if self._clock() <= before:
            raise RetreatRejected('物理时钟未推进')
        if self._callback_uid is not None:
            if self._last_actual_delta is None:
                raise RetreatRejected('post physics回调没有有效实际读回')
            return self._last_actual_delta.copy()
        return self._observe(**kwargs)

    def _command(self, vy, *, stopping=False):
        if not stopping and (self.output / 'STOP').exists():
            raise RetreatStopped('发送前收到停止文件')
        if not math.isfinite(vy) or abs(vy) > self.policy['max_body_speed_m_s'] + 1e-12:
            raise RetreatRejected('拒绝非有限或超参考上限的底盘速度')
        _controllers(self.executor, self.owned_command)
        # setter若部分施加后抛错，仍必须进入零速制动，不把“尚未返回”当作没动。
        self.started = self.started or abs(vy) > 0.
        self.executor.set_base_velocity(vx=0., vy=float(vy), wz=0.)
        self.owned_command = (0., float(vy), 0.)

    def _stationary(self, sample):
        return (np.linalg.norm(sample['root_velocity'][:3]) < self.policy['stationary_linear_m_s'] and
                np.linalg.norm(sample['root_velocity'][3:]) < self.policy['stationary_angular_rad_s'] and
                np.max(np.abs(sample['velocities'][self.wheel_indices])) < self.policy['stationary_wheel_rad_s'])

    def _stop(self):
        """只零速刹停，不回正舵角；几何异常也不能恢复非零速度。"""
        self.phase = 'stopping'
        errors = [str(self.guard_error)] if self.guard_error is not None else []
        self.executor.set_base_velocity(vx=0., vy=0., wz=0.)
        self.owned_command = (0., 0., 0.)
        start, stable, stable_root = self._clock(), None, None
        while self._clock() - start < self.policy['stop_timeout_s']:
            before = self._clock()
            callbacks_before = self.physical_guard['physical_guard_callbacks_count']
            self.executor.step()
            if self.guard_error is not None and str(self.guard_error) not in errors:
                errors.append(str(self.guard_error))
            if self._callback_uid is not None and self.physical_guard['physical_guard_callbacks_count'] <= callbacks_before:
                errors.append('刹停期间未收到post physics guard回调')
                break
            if self._clock() <= before:
                errors.append('刹停期间物理时钟停止')
                break
            try:
                self._observe(allow_stop_file=True)
            except Exception as error:
                if str(error) not in errors:
                    errors.append(str(error))
                self.executor.set_base_velocity(vx=0., vy=0., wz=0.)
                # 仍读取实际速度确认制动；失败保持失败，不借此放行下一动作。
                self.current = _capture(self.executor, self.safety)
            sample = self.current
            if self._stationary(sample):
                if stable is None:
                    stable, stable_root = self._clock(), sample['root'].copy()
                movement = np.linalg.norm(sample['root'][:3, 3] - stable_root[:3, 3])
                rotation = Rotation.from_matrix(sample['root'][:3, :3] @ stable_root[:3, :3].T).magnitude()
                if movement > self.policy['stationary_position_window_m'] or rotation > math.radians(self.policy['stationary_angle_window_deg']):
                    stable, stable_root = self._clock(), sample['root'].copy()
                if (self._clock() - stable >= self.policy['stop_dwell_s'] and
                        getattr(self.executor, '_base_cmd', None) is None):
                    self.stop_verified = True
                    break
            else:
                stable = None
        if not self.stop_verified:
            errors.append('实际速度/位姿窗口未确认停稳，不能声称退离完成')
        return errors

    def execute(self):
        if (self.output / 'STOP').exists():
            raise RetreatStopped('停止文件存在，拒绝自行恢复')
        t0 = self._clock()
        while self._clock() - t0 < self.policy['initial_stability_s']:
            self._step()
            if self.explicit_contact_recovery:
                self.observed_start_checks.append(self.last_observed_motion_check)
            elif not self._stationary(self.current):
                raise RetreatRejected('初始实际速度仍在振荡，未发送非零速度')
        self.phase = 'retreat'
        distance = float(self.plan['chosen']['body_y_distance_m'])
        sign, speed, t0 = math.copysign(1., distance), 0., self._clock()
        while True:
            delta = self._observe()
            progress = sign * float(delta[1])
            remaining = abs(distance) - progress
            if progress < -self.policy['goal_error_m'] or remaining < -self.policy['goal_error_m']:
                raise RetreatRejected('实际退出方向错误或超出规划终点')
            if remaining <= self.policy['goal_error_m']:
                break
            if self._clock() - t0 > self.policy['move_timeout_s']:
                raise RetreatRejected('实际退离超时，未完成')
            dt = float(self.executor.world.get_physics_dt())
            limit = min(self.policy['max_body_speed_m_s'],
                        math.sqrt(2 * self.policy['max_acceleration_m_s2'] * max(0., remaining - self.policy['goal_error_m'] / 2)))
            speed = min(speed + self.policy['max_acceleration_m_s2'] * dt, limit)
            self._command(sign * speed)
            self._step()
        errors = self._stop()
        if errors:
            raise RetreatRejected('刹停/反馈复核失败: ' + '; '.join(errors))
        delta = self._observe(allow_stop_file=True)
        if abs(float(delta[1]) - distance) > self.policy['goal_error_m']:
            raise RetreatRejected('停稳后的实际位移未到计划终点')
        final_clearance = _final_clearance_check(self.plan['chosen'], self.previous, self.observed_contact_profile,
                                                self.explicit_contact_recovery, self.policy)
        self.phase = 'complete'
        return {'success': True, 'phase': self.phase, 'motion_started': self.started,
                'actual_body_displacement_m': delta.tolist(), 'planned_body_y_distance_m': distance,
                'stopped_verified': self.stop_verified, 'final_geometry_check': self.previous,
                'final_clearance_contract': final_clearance,
                'globally_collision_free': False}


def run_retreat(executor, output=None, distances_m=(.05, .10, .15, .30),
                placement_options=None, control_policy=None, explicit_contact_recovery=False,
                observed_contact_profile=None, prepared_safety=None):
    """必须重新采样并纯计划通过才执行；暂停/停止后不会自动清除STOP或重启。"""
    output = Path(output or REPO / 'outputs/valve_operation_20261102/retreat')
    output.mkdir(parents=True, exist_ok=True)
    policy, runtime = _policy(control_policy), None
    try:
        if (output / 'STOP').exists():
            raise RetreatStopped('停止文件存在，禁止自动恢复')
        check, context = _prepare(executor, output, distances_m, placement_options, policy,
                                  explicit_contact_recovery, observed_contact_profile, prepared_safety)
        _write(output / 'preflight.json', check)
        if not check['safe']:
            result = {'success': False, 'phase': 'preflight', 'motion_started': False, 'preflight': check}
        else:
            runtime = _RetreatRuntime(executor, output, context, policy)
            runtime._install_physics_guard()
            result = runtime.execute()
    except Exception as error:
        result = {'success': False, 'phase': 'stopped' if isinstance(error, RetreatStopped) else
                  (runtime.phase if runtime else 'preflight'), 'motion_started': bool(runtime and runtime.started),
                  'error': str(error), 'error_type': type(error).__name__}
        if runtime is not None and runtime.started:
            try:
                result['stop_errors'] = runtime._stop()
            except Exception as stop_error:
                result['stop_errors'] = [str(stop_error)]
            result['stopped_verified'] = runtime.stop_verified
            if runtime.current:
                delta = runtime.root0[:3, :3].T @ (runtime.current['root'][:3, 3] - runtime.root0[:3, 3])
                result['actual_body_displacement_m'] = delta.tolist()
    finally:
        if runtime is not None:
            runtime._remove_physics_guard()
    result['control_reference'] = policy
    result['explicit_contact_recovery'] = bool(explicit_contact_recovery)
    result['observed_contact_profile'] = observed_contact_profile
    result['sealed_control_policy_validation'] = (
        runtime.last_observed_motion_check.get('sealed_control_policy_validation') if
        runtime is not None and runtime.last_observed_motion_check is not None else None)
    result['initial_stillness_gate_replaced_by_observed_envelope'] = bool(runtime and runtime.explicit_contact_recovery)
    result['thresholds_calibrated_to_hardware'] = False
    if runtime is not None:
        result['physical_guard'] = runtime.physical_guard
        if runtime.physical_guard.get('callback_cleanup_error'):
            result['success'] = False
            result['callback_cleanup_error'] = runtime.physical_guard['callback_cleanup_error']
        _write(output / 'trace.json', runtime.trace)
        _write(output / 'observed_start_checks.json', runtime.observed_start_checks)
        _write(output / 'observed_motion_checks.json', runtime.observed_motion_checks)
    _write(output / 'result.json', result)
    _write(output / 'progress.json', {k: v for k, v in result.items() if k != 'preflight'})
    return result
