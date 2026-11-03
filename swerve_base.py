"""四轮独立转向/独立驱动运动学；几何从URDF读取，不依赖Isaac。"""
from dataclasses import asdict, dataclass, replace
from collections import deque
from itertools import product
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


MODULE_NAMES = ('front_left', 'rear_left', 'front_right', 'rear_right')
STEERING_JOINTS = tuple('caster_' + name + '_joint' for name in MODULE_NAMES)
WHEEL_JOINTS = tuple(name + '_wheel_joint' for name in MODULE_NAMES)
BASE_JOINTS = STEERING_JOINTS + WHEEL_JOINTS


class BaseSettlingWindow:
    """以物理位置变化判定停稳，避免原生速度伺服的高频微振永远不能过门限。

    这些是用户授权的仿真停稳判据，不是真机规格或电机速度标定。
    """
    def __init__(self, window_s=.25, xy_span_m=.002, yaw_span_rad=.01,
                 wheel_span_rad=.015, steering_span_rad=.01):
        self.window_s = float(window_s)
        self.xy_span_m, self.yaw_span_rad = float(xy_span_m), float(yaw_span_rad)
        self.wheel_span_rad, self.steering_span_rad = float(wheel_span_rad), float(steering_span_rad)
        self.samples = deque()

    def update(self, timestamp, base_xy_yaw, steering, wheels):
        position = np.r_[base_xy_yaw, steering, wheels].astype(float)
        if position.shape != (11,) or not np.isfinite(position).all() or not math.isfinite(timestamp):
            raise ValueError('停稳验证需要有限的根XY/yaw与8关节物理位置')
        if self.samples and timestamp <= self.samples[-1][0]:
            self.samples.clear()
        self.samples.append((float(timestamp), position))
        cutoff = timestamp-self.window_s
        # 保留一个窗口边界之前的采样，确保确实覆盖完整时间而非短片段。
        while len(self.samples) > 1 and self.samples[1][0] <= cutoff:
            self.samples.popleft()
        values = np.asarray([sample[1] for sample in self.samples])
        values[:, 2] = (values[:, 2]-values[0, 2]+math.pi) % (2*math.pi)-math.pi
        spans = np.ptp(values, axis=0)
        duration = timestamp-self.samples[0][0]
        settled = (duration >= self.window_s-1e-9 and np.max(spans[:2]) < self.xy_span_m
                   and spans[2] < self.yaw_span_rad and np.max(spans[3:7]) < self.steering_span_rad
                   and np.max(spans[7:]) < self.wheel_span_rad)
        return {'settled': bool(settled), 'window_s': self.window_s, 'duration_s': float(duration),
                'xy_span_m': spans[:2].tolist(), 'yaw_span_rad': float(spans[2]),
                'steering_span_rad': spans[3:7].tolist(), 'wheel_span_rad': spans[7:].tolist(),
                'calibrated_to_user_hardware': False, 'method': 'actual_physics_position_window'}


def _numbers(element, attribute, default):
    return np.fromstring(element.get(attribute, default) if element is not None else default, sep=' ')


def _rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


@dataclass(frozen=True)
class SwerveModule:
    name: str
    steering_joint: str
    wheel_joint: str
    pivot_xy: tuple
    contact_offset_xy: tuple
    heading_zero: float
    steering_sign: float
    wheel_sign: float
    radius_m: float
    steering_limits_rad: tuple
    wheel_max_velocity_rad_s: float
    steering_max_velocity_rad_s: float = math.inf


class SteeringPrealignment:
    """对舵期间零轮速；同步限速轨迹结束且实际稳定后才许可平移。

    轨迹速度和稳定门限是仿真参考控制设置，未标定为硬件规格。
    """
    def __init__(self, start, target, timestamp, max_rate_rad_s=.6,
                 angle_tolerance_rad=math.radians(1.), stable_time_s=.10):
        self.start = np.asarray(start, dtype=float).copy()
        self.target = np.asarray(target, dtype=float).copy()
        if self.start.shape != (4,) or self.target.shape != (4,) or not np.isfinite(np.r_[self.start,self.target,timestamp,max_rate_rad_s]).all():
            raise ValueError('预对舵需要4个有限起始角、目标角和时间')
        if max_rate_rad_s <= 0.:
            raise ValueError('预对舵参考速度必须大于0')
        self.started = float(timestamp)
        self.rate = float(max_rate_rad_s)
        self.duration = 1.5*float(np.max(abs(self.target-self.start)))/self.rate
        self.tolerance = float(angle_tolerance_rad)
        self.initial_stop = BaseSettlingWindow(window_s=stable_time_s, xy_span_m=.0015,
            yaw_span_rad=.005, wheel_span_rad=.01, steering_span_rad=.005)
        self.trajectory_started = None
        self.stable = BaseSettlingWindow(window_s=stable_time_s, xy_span_m=.0015,
            yaw_span_rad=.005, wheel_span_rad=.01, steering_span_rad=.005)
        self.ready = False
        self.last_verification = None
        self.verified_at = None

    def update(self, timestamp, actual_steering, wheel_positions, base_pose):
        actual = np.asarray(actual_steering, dtype=float)
        elapsed = max(0., float(timestamp)-self.started)
        stop_verification = None
        if self.trajectory_started is None:
            stop_verification = self.initial_stop.update(timestamp,base_pose,actual,wheel_positions)
            if stop_verification['settled']:
                # 等实际停稳再从当前反馈起步，不回追刹车前的姿态。
                self.start = actual.copy()
                self.duration = 1.5*float(np.max(abs(self.target-self.start)))/self.rate
                self.trajectory_started = float(timestamp)
        steering_elapsed = 0. if self.trajectory_started is None else max(0.,float(timestamp)-self.trajectory_started)
        u = (1. if self.duration <= 1e-12 else float(np.clip(steering_elapsed/self.duration,0.,1.))) if self.trajectory_started is not None else 0.
        command = self.start+(self.target-self.start)*u*u*(3.-2.*u)
        error = float(np.max(abs(self.target-actual)))
        # 行驶后根位置自然变化，只在未放行阶段要求位置窗口稳定。
        if self.ready and error > max(self.tolerance*2., math.radians(2.)):
            self.ready = False
            self.last_verification,self.verified_at = None,None
            self.stable.samples.clear()
            self.initial_stop.samples.clear()
            self.trajectory_started = None
            self.start = actual.copy()
            command = actual.copy()
        verification = None
        if not self.ready:
            if self.trajectory_started is None or u < 1. or error > self.tolerance:
                self.stable.samples.clear()
            else:
                verification = self.stable.update(timestamp, base_pose, actual, wheel_positions)
                self.ready = verification['settled']
                if self.ready:
                    self.last_verification,self.verified_at = verification,float(timestamp)
        return command, {'phase': 'translating' if self.ready else 'prealign',
            'ready': bool(self.ready), 'trajectory_complete': bool(self.trajectory_started is not None and u>=1.),
            'elapsed_s': elapsed, 'trajectory_duration_s': self.duration,
            'stage': ('ready' if self.ready else ('braking' if self.trajectory_started is None else
                      ('steering' if u<1. else 'stabilizing'))),
            'initial_stop_verification':stop_verification,
            'max_error_rad': error, 'angle_tolerance_rad': self.tolerance,
            'reference_max_rate_rad_s': self.rate,
            'stable_verification': self.last_verification or verification,
            'verified_at_sim_time':self.verified_at,
            'calibrated_to_user_hardware': False}


class SwerveModel:
    def __init__(self, modules, source_urdf):
        self.modules = list(modules)
        self.source_urdf = str(source_urdf)
        self.original_limits = {m.steering_joint: list(m.steering_limits_rad) for m in modules}

    @classmethod
    def from_urdf(cls, path):
        path = Path(path)
        root = ET.parse(path).getroot()
        joints = {j.get('name'): j for j in root.findall('joint')}
        links = {link.get('name'): link for link in root.findall('link')}
        modules = []
        for name, steering_name, wheel_name in zip(MODULE_NAMES, STEERING_JOINTS, WHEEL_JOINTS):
            if steering_name not in joints or wheel_name not in joints:
                raise ValueError('URDF缺少四轮八驱关节：' + steering_name + '/' + wheel_name)
            steering, wheel = joints[steering_name], joints[wheel_name]
            if steering.find('parent').get('link') != 'base_link' or wheel.find('parent').get('link') != steering.find('child').get('link'):
                raise ValueError('四轮八驱关节拓扑与已支持模型不一致：' + name)
            so, wo = steering.find('origin'), wheel.find('origin')
            sr = _rotation(_numbers(so, 'rpy', '0 0 0'))
            wr = _rotation(_numbers(wo, 'rpy', '0 0 0'))
            steering_axis = sr @ _numbers(steering.find('axis'), 'xyz', '1 0 0')
            wheel_axis = sr @ wr @ _numbers(wheel.find('axis'), 'xyz', '1 0 0')
            if abs(abs(steering_axis[2])-1.) > 1e-6 or np.linalg.norm(steering_axis[:2]) > 1e-6:
                raise ValueError('转向轴不是底盘竖直轴：' + steering_name)
            if abs(wheel_axis[2]) > 1e-6:
                raise ValueError('滚轮轴不是底盘水平轴：' + wheel_name)
            # 滚动方向 = 轮轴×上方向；轴向正号和左右镜像都来自模型。
            wheel_heading = np.cross(wheel_axis, np.array([0., 0., 1.]))
            heading = math.atan2(wheel_heading[1], wheel_heading[0])
            # 选择接近底盘+X的零位方向，另一种朝向由轮速负号表达。
            wheel_sign = 1.
            if abs(heading) > math.pi/2 + 1e-6:
                heading = math.atan2(-wheel_heading[1], -wheel_heading[0])
                wheel_sign = -1.
            collision = links[wheel.find('child').get('link')].find('collision')
            cylinder = collision.find('geometry/cylinder') if collision is not None else None
            if cylinder is None:
                raise ValueError('滚轮没有可确认半径的圆柱碰撞体：' + wheel_name)
            radius = float(cylinder.get('radius'))
            if not math.isfinite(radius) or radius <= 0:
                raise ValueError('滚轮半径无效：' + wheel_name)
            # 使用轮胎接触圆柱的中心，避免把轮轴原点误当轮胎中心造成轮距误差。
            center = sr @ (_numbers(wo, 'xyz', '0 0 0') + wr @ _numbers(collision.find('origin'), 'xyz', '0 0 0'))
            h = np.array([[math.cos(heading), -math.sin(heading)], [math.sin(heading), math.cos(heading)]])
            offset = h.T @ center[:2]
            limit = steering.find('limit')
            bounds = (-math.inf, math.inf) if steering.get('type') == 'continuous' else (float(limit.get('lower')), float(limit.get('upper')))
            velocity = float(wheel.find('limit').get('velocity'))
            modules.append(SwerveModule(name, steering_name, wheel_name,
                tuple(_numbers(so, 'xyz', '0 0 0')[:2]), tuple(offset), heading,
                float(steering_axis[2]), wheel_sign, radius, bounds, velocity,
                float(limit.get('velocity'))))
        return cls(modules, path.resolve())

    def set_steering_limits(self, limits):
        limits = np.asarray(limits, dtype=float)
        if limits.shape != (4, 2) or not np.isfinite(limits).all() or (limits[:, 0] >= limits[:, 1]).any():
            raise ValueError('四个转向限位必须是有限且递增的4×2数组')
        self.modules = [replace(m, steering_limits_rad=tuple(bounds)) for m, bounds in zip(self.modules, limits)]

    def metadata(self):
        return {'mode': 'four_wheel_independent_steering_drive', 'source_urdf': self.source_urdf,
                'modules': [asdict(m) for m in self.modules], 'original_urdf_limits_rad': self.original_limits,
                'velocity_frame': 'base_link; +X前进、+Y向左、+Z逆时针'}

    @staticmethod
    def cardinal_axis(vx, vy, wz):
        if abs(wz) > 1e-12:
            return None
        if abs(vx)>1e-12 and abs(vy)<=1e-12:
            return 'x'
        if abs(vy)>1e-12 and abs(vx)<=1e-12:
            return 'y'
        return None

    def solve_cardinal(self, vx, vy, current_steering=None, locked_steering=None):
        """纯车身轴平移：锁定等价舵角，反向/变速不重新换支。

        ±90是同一轮胎滚动平面，轮速反号后地面速度仍同向。等距解优先
        平衡四舵净转角，减小同向反扭矩；这不保证实际车身零偏航，仍须反馈验收。
        """
        axis = self.cardinal_axis(vx,vy,0.)
        current = np.zeros(4) if current_steering is None else np.asarray(current_steering,dtype=float)
        if axis is None or current.shape!=(4,) or not np.isfinite(np.r_[vx,vy,current]).all():
            raise ValueError('单轴平移仅允许非零vx或vy其中一个，不含车身旋转')
        if locked_steering is None:
            direction = math.atan2(vy,vx)
            choices=[]
            for module in self.modules:
                low,high=module.steering_limits_rad
                values=[]
                for heading in (direction,direction+math.pi):
                    raw=(heading-module.heading_zero)/module.steering_sign
                    for winding in range(-2,3):
                        value=raw+2*math.pi*winding
                        if low-1e-7<=value<=high+1e-7:
                            value=float(np.clip(value,low,high))
                            if not any(abs(value-old)<1e-8 for old in values):values.append(value)
                if not values:raise ValueError('现有舵角限位不能完成单轴平移：'+module.name)
                choices.append(values)
            signs=np.array([m.steering_sign for m in self.modules])
            preferred=np.array([-1.,1.,1.,-1.])*math.pi/2
            def cost(values):
                values=np.asarray(values);delta=values-current
                return (round(abs(float(np.sum(signs*delta))),9),
                        float(np.max(abs(delta))),float(np.sum(delta*delta)),
                        float(np.sum(abs(values-preferred))))
            target=np.asarray(min(product(*choices),key=cost),dtype=float)
        else:
            target=np.asarray(locked_steering,dtype=float).copy()
            if target.shape!=(4,) or not np.isfinite(target).all():raise ValueError('锁定舵角需要4个有限值')
        velocities=[];residual=[];headings=[]
        for module,q in zip(self.modules,target):
            low,high=module.steering_limits_rad
            if not low-1e-7<=q<=high+1e-7:raise ValueError('锁定舵角超出现有参考限位')
            heading=module.heading_zero+module.steering_sign*q
            sideways=-math.sin(heading)*vx+math.cos(heading)*vy
            if abs(sideways)>1e-8:raise ValueError('锁定舵角与请求平移方向不一致')
            residual.append(sideways);headings.append(heading)
            velocities.append((math.cos(heading)*vx+math.sin(heading)*vy)/(module.radius_m*module.wheel_sign))
        ratio=max([1.]+[abs(v)/m.wheel_max_velocity_rad_s for m,v in zip(self.modules,velocities)])
        scale=1./ratio
        return {'steering':target,'wheel_velocity':np.asarray(velocities)*scale,
                'lateral_residual_m_s':np.asarray(residual)*scale,'scale':scale,
                'executed_twist':np.array([vx,vy,0.])*scale,'cardinal_axis':axis,
                'rolling_plane_heading_rad_mod_pi':(np.asarray(headings)+math.pi/2)%math.pi-math.pi/2,
                'net_steering_travel_rad':float(np.sum(np.array([m.steering_sign for m in self.modules])*(target-current)))}

    def solve(self, vx, vy, wz, current_steering=None):
        twist = np.array([vx, vy, wz], dtype=float)
        if not np.isfinite(twist).all():
            raise ValueError('底盘速度必须为有限数')
        current = np.zeros(4) if current_steering is None else np.asarray(current_steering, dtype=float)
        if current.shape != (4,) or not np.isfinite(current).all():
            raise ValueError('转向反馈必须是4个有限角度')
        angles, velocities, lateral = [], [], []
        for module, actual in zip(self.modules, current):
            px, py = module.pivot_xy
            ox, oy = module.contact_offset_xy
            v = np.array([vx-wz*py, vy+wz*px])
            speed = float(np.linalg.norm(v))
            lower, upper = module.steering_limits_rad
            if speed < 1e-12 and abs(wz*ox) < 1e-12:
                q = float(np.clip(actual, lower, upper))
                wheel_speed = 0.
            else:
                if speed < abs(wz*ox)-1e-10:
                    raise ValueError('当前速度无法满足舵轮接触点无侧滑约束：' + module.name)
                ratio = float(np.clip(wz*ox/max(speed, 1e-15), -1., 1.))
                direction, adjustment = math.atan2(v[1], v[0]), math.asin(ratio)
                candidates = []
                for heading in (direction+adjustment, direction+math.pi-adjustment):
                    raw = (heading-module.heading_zero)/module.steering_sign
                    nearest = int(round((actual-raw)/(2*math.pi)))
                    for winding in range(nearest-2, nearest+3):
                        candidate = raw+winding*2*math.pi
                        if lower-1e-7 <= candidate <= upper+1e-7:
                            candidates.append(float(np.clip(candidate, lower, upper)))
                if not candidates:
                    raise ValueError(f'{module.name}所需舵角超出现有{math.degrees(lower):.1f}～{math.degrees(upper):.1f}度限位；该侧移/转弯速度不可执行')
                q = min(candidates, key=lambda candidate: abs(candidate-actual))
                heading = module.heading_zero+module.steering_sign*q
                linear = math.cos(heading)*v[0]+math.sin(heading)*v[1]-wz*oy
                wheel_speed = linear/(module.radius_m*module.wheel_sign)
            heading = module.heading_zero+module.steering_sign*q
            lateral.append(-math.sin(heading)*v[0]+math.cos(heading)*v[1]+wz*ox)
            angles.append(q)
            velocities.append(wheel_speed)
        maximum_ratio = max([1.] + [abs(v)/m.wheel_max_velocity_rad_s for m, v in zip(self.modules, velocities)])
        scale = 1./maximum_ratio
        return {'steering': np.asarray(angles), 'wheel_velocity': np.asarray(velocities)*scale,
                'lateral_residual_m_s': np.asarray(lateral)*scale,
                'scale': scale, 'executed_twist': twist*scale}
