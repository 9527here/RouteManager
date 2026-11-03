"""阀门任务的受限净空退离规划与反馈复核；无网络、控制或场景修改。

仅在唯一初始环境对处于正间隙但低于buffer时允许非下降退出。四轮按真实舵角
和URDF半径/正号同步滚动；底盘不转向，双臂保持当前实际姿态。
"""
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

LOW_SPEED_POLICY_ID='low_speed_contact_escape_10mm_v1'
LOW_SPEED_CONTROL_CAPS={
    'max_body_speed_m_s':.010,'max_acceleration_m_s2':.005,
    'orientation_change_deg':.2,'cross_track_error_m':.002,
    'root_z_error_m':.001,'goal_error_m':.0015,
}


def _effective_recovery_margin(profile):
    if 'policy_id' not in profile:
        if any(key in profile for key in ('recovery_environment_margin_m','recovery_control_caps','low_speed_reference')):
            raise ValueError('缺少合法封存的低速policy_id')
        return profile['ordinary_environment_margin_m']
    if (profile.get('policy_id')!=LOW_SPEED_POLICY_ID or profile.get('low_speed_reference') is not True
            or profile.get('recovery_environment_margin_m')!=.010
            or profile.get('recovery_final_leg_clearance_m')!=.012
            or profile.get('recovery_control_caps')!=LOW_SPEED_CONTROL_CAPS):
        raise ValueError('未知或被篡改的低速脱困参考policy')
    return .010


def _hash_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _profile_digest(profile):
    return hashlib.sha256(json.dumps({k:v for k,v in profile.items() if k != 'profile_sha256'},
                                    sort_keys=True, allow_nan=False).encode()).hexdigest()


def _geometry_binding(safety):
    return {'capture_id': safety.capture_id,
            'snapshot_path': str((safety.directory/'snapshot.json').resolve()),
            'snapshot_sha256': _hash_file(safety.directory/'snapshot.json'),
            'geometry_path': str((safety.directory/'geometry.npz').resolve()),
            'geometry_sha256': _hash_file(safety.directory/'geometry.npz'),
            'stage_identifier': safety.model.data.get('stage_identifier'),
            'robot_root': safety.model.root,
            'shape_paths': sorted(s['path'] for s in safety.model.shapes)}


def _box_sat_distance(safety, index, poses):
    """两真实Cube的独立SAT有符号间隙；拒绝把约1um的EPA误差当动态接触量。"""
    cache = getattr(safety, '_retreat_box_sat_cache', None)
    if cache is None:
        cache = {};safety._retreat_box_sat_cache = cache
    if index not in cache:
        _,_,a,b,*_ = safety._pair_records[index]
        if a['type'] != 'Cube' or b['type'] != 'Cube':
            raise RuntimeError('观测接触模式只支持已核实的Cube↔Cube Leg对')
        with np.load(safety.directory/'geometry.npz') as arrays:
            rows=[]
            for shape in (a,b):
                vertices=arrays[shape['key']+'_v'];faces=arrays[shape['key']+'_f'];axes=[]
                for face in faces:
                    points=vertices[face];axis=np.cross(points[1]-points[0],points[2]-points[0]);axis/=np.linalg.norm(axis)
                    if not any(abs(np.dot(axis,old))>1-1e-8 for old in axes):axes.append(axis)
                if len(axes)!=3:raise RuntimeError('接触对不是可验证的正交Cube')
                rows.append((shape['body'],vertices.copy(),np.asarray(axes)))
        cache[index]=rows
    transformed=[];bases=[]
    for body,vertices,axes in cache[index]:
        matrix=poses[body] if body else np.eye(4)
        transformed.append(vertices@matrix[:3,:3].T+matrix[:3,3]);bases.append(axes@matrix[:3,:3].T)
    axes=list(bases[0])+list(bases[1])
    for a in bases[0]:
        for b in bases[1]:
            cross=np.cross(a,b);length=np.linalg.norm(cross)
            if length>1e-8:axes.append(cross/length)
    return max(float(max((transformed[0]@a).min()-(transformed[1]@a).max(),
                         (transformed[1]@a).min()-(transformed[0]@a).max())) for a in axes)


def _surface_bounds(safety, poses, frames):
    values={}
    for wheel in frames:
        matrix=np.linalg.inv(poses[safety.model.root])@poses[wheel['body']]@np.asarray(wheel['local_matrix'])
        original=np.asarray(wheel['matrix_in_base']);a,b=matrix[:3,2],original[:3,2]
        a,b=a/np.linalg.norm(a),b/np.linalg.norm(b)
        if np.dot(a,b)<0:a=-a
        angle=math.atan2(np.linalg.norm(np.cross(a,b)),np.dot(a,b))
        values[wheel['shape_path']]=float(np.linalg.norm(matrix[:3,3]-original[:3,3])+
            2*math.hypot(wheel['radius_m'],wheel['height_m']/2)*math.sin(angle/2))
    return values


def _contact_window(history, window_s):
    end=history[-1][0];start=end-window_s
    duration=sum(max(0.,min(b[0],end)-max(a[0],start)) for a,b in zip(history,history[1:]) if a[1]<0)
    return float(duration/window_s)


def build_observed_contact_profile(safety, modules, measurement_path, samples_path, *, minimum_duration_s=2.,
                                   low_speed_reference=False):
    """从同一几何capture后的密采实际帧建立待授权经验包络；不启用或执行恢复。"""
    modules,exact,indices,by_shape=_prepare(safety,modules)
    metadata=json.loads(Path(measurement_path).read_text())
    archive=np.load(samples_path)
    t,q,v,links,root_v=(archive[name] for name in ('t','q','v','links','root_v'))
    names=metadata['all_joint_names'];bodies=metadata['all_body_names']
    if names!=safety.model.names or links.shape!=(len(t),len(bodies),7) or q.shape!=v.shape or q.shape!=(len(t),len(names)):
        raise ValueError('观测profile的关节/link schema不匹配')
    if not all(np.isfinite(a).all() for a in (t,q,v,links,root_v)) or len(t)<2 or np.any(np.diff(t)<=0):
        raise ValueError('观测profile含非有限值或非递增时间')
    physics_dt=float(metadata.get('flags',{}).get('physics_dt',safety.model.data.get('physics_dt',0.)))
    if physics_dt<=0 or np.max(np.diff(t))>2*physics_dt+1e-6 or t[-1]-t[0]<minimum_duration_s:
        raise ValueError('需要至少2秒且最大采样间隔不超过2个physics_dt的最新密采profile')
    if float(t[0])<float(safety.model.data.get('capture_time',float('inf')))-physics_dt:
        raise ValueError('观测帧早于几何capture，拒绝混用旧基线')
    if any(metadata.get('command_calls',{}).values()) or not metadata.get('command_calls'):
        raise ValueError('profile必须证明采样期间没有新增运动命令')
    flags=metadata.get('flags',{})
    if any(flags.get(key) not in (None,'None') for key in ('base_cmd','motion','auto_pick')):
        raise ValueError('profile不是无运动任务的静止接触基线')
    if np.any(np.asarray(flags.get('velocity_targets',[]),float)):
        raise ValueError('profile存在非零速度目标')
    path_by_leaf={p.split('/')[-1]:p for p in safety.model.cluster}
    paths=[path_by_leaf[name] for name in bodies]
    if len(set(paths))!=len(safety.model.cluster):raise ValueError('profile没有覆盖全部实际机器人刚体')
    leg_pair=tuple(sorted(('/World/L3Station/Leg',safety.model.root+'/tn__base_link_bumper_xp_028_kY0l0')))
    if leg_pair not in by_shape:raise ValueError('当前场景没有已核实的Leg接触形状对')
    leg_index=by_shape[leg_pair]
    initial_exact=[_distance(safety,k,safety.poses,exact) for k in indices]
    allowed_assembly={_pair(row) for row in initial_exact if row['distance_m']<0}
    assembly_min={key:float('inf') for key in allowed_assembly}
    frames=[];surface_max={};history=[];max_duty=0.;negative_start=None;max_run=0.
    for frame_no,(instant,current_q,current_links) in enumerate(zip(t,q,links)):
        poses=dict(safety.poses)
        for path,link in zip(paths,current_links):
            matrix=np.eye(4);matrix[:3,:3]=Rotation.from_quat(link[3:]).as_matrix();matrix[:3,3]=link[:3];poses[path]=matrix
        if frame_no==0:
            for index,(geometry,local) in exact.items():
                shape=safety.model.shapes[index]
                frames.append({'body':shape['body'],'shape_path':shape['path'],'local_matrix':local.tolist(),
                    'matrix_in_base':(np.linalg.inv(poses[safety.model.root])@poses[shape['body']]@local).tolist(),
                    'radius_m':float(geometry.radius),'height_m':float(geometry.halfLength)*2})
        distance=_box_sat_distance(safety,leg_index,poses);history.append([float(instant),distance])
        if distance<0 and negative_start is None:negative_start=float(instant)
        if negative_start is not None:
            max_run=max(max_run,float(instant)-negative_start)
            if distance>=0:negative_start=None
        if instant-t[0]>=.25:max_duty=max(max_duty,_contact_window(history,.25))
        for key,value in _surface_bounds(safety,poses,frames).items():surface_max[key]=max(surface_max.get(key,0.),value)
        for index in indices:
            row=_distance(safety,index,poses,exact);key=_pair(row)
            if key in allowed_assembly:assembly_min[key]=min(assembly_min[key],row['distance_m'])
            elif row['distance_m']<0:raise ValueError('profile中出现新的底盘自身穿透shape对，不能纳入既存例外')
        report=safety.check_configuration(q=current_q,poses=poses,environment_margin_m=.012)
        for row in report['violations']:
            key=_pair(row)
            if (row['self'] and key not in allowed_assembly) or (not row['self'] and key!=leg_pair):
                raise ValueError('profile中存在其他碰撞或低于12mm环境净空')
        nearest,_=safety._nearest(poses,safety._external_pairs,('left_arm','left_hand','right_arm','right_hand'))
        if any(row is None or row['distance_m']<.05 for row in nearest.values()):
            raise ValueError('profile中四组手臂净空未保持50mm')
    wheel_indices=[names.index(m['wheel_joint']) for m in modules]
    root_index=bodies.index(safety.model.root.split('/')[-1])
    tilts=Rotation.from_quat(links[:,root_index,3:]).as_euler('xyz')[:,:2]
    profile={'schema':'routemanager.observed_contact_profile.v1','mode':'explicit_observed_contact_recovery',
        'enabled_by_default':False,'requires_separate_user_authorization':True,'empirical_not_theoretical_guarantee':True,
        'geometry_binding':_geometry_binding(safety),'leg_shape_pair':list(leg_pair),
        'sources':[{'path':str(Path(p).resolve()),'sha256':_hash_file(p)} for p in (measurement_path,samples_path)],
        'start_sim_time':float(t[0]),'end_sim_time':float(t[-1]),'sample_count':len(t),
        'physics_dt_s':physics_dt,'maximum_sample_interval_s':float(np.diff(t).max()),
        'leg_sat_minimum_m':min(row[1] for row in history),'contact_window_s':.25,
        'maximum_contact_time_fraction':max_duty,'maximum_negative_run_duration_s':max_run,
        'cylinder_reference_frames_first_actual':frames,'cylinder_surface_bound_max_m':surface_max,
        'assembly_solver_minimum_by_shape_pair':[{'shapes':list(key),'minimum_m':value} for key,value in sorted(assembly_min.items())],
        'distance_numerical_tolerance_m':1e-7,
        'start_motion_envelope':{
            'wheel_abs_velocity_max_rad_s':{m['wheel_joint']:float(np.max(abs(v[:,i]))) for m,i in zip(modules,wheel_indices)},
            'root_linear_velocity_component_abs_max_m_s':np.max(abs(root_v[:,:3]),0).tolist(),
            'root_angular_velocity_component_abs_max_rad_s':np.max(abs(root_v[:,3:]),0).tolist(),
            'roll_pitch_abs_max_rad':np.max(abs(tilts),0).tolist()},
        'ordinary_environment_margin_m':.012,'arm_hand_margin_m':.05,
        'normal_guard_pass':False,'execution_authorized':False}
    if low_speed_reference:
        profile.update({'policy_id':LOW_SPEED_POLICY_ID,'low_speed_reference':True,
            'recovery_environment_margin_m':.010,'recovery_final_leg_clearance_m':.012,
            'recovery_control_caps':dict(LOW_SPEED_CONTROL_CAPS),
            'recovery_reference_notice':'独立低速接触脱困仿真参考；非硬件标定、非数学安全保证；普通12mm与观测原始caps保持'})
    profile['profile_sha256']=_profile_digest(profile)
    # 同一Safety实例封存从源帧计算出的原始caps；调用方改值再重算hash也不能扩张。
    if not hasattr(safety,'_observed_contact_profiles'):safety._observed_contact_profiles={}
    safety._observed_contact_profiles[profile['profile_sha256']]=json.dumps(profile,sort_keys=True,allow_nan=False)
    return profile


def validate_observed_contact_profile(safety,profile,*,now_sim_time=None,max_age_sim_s=30.):
    """无效则抛错；成功只说明profile匹配且新鲜，不代表授权或执行检查通过。"""
    if not isinstance(profile,dict) or profile.get('schema')!='routemanager.observed_contact_profile.v1':
        raise ValueError('缺少合法observed_contact_profile')
    if profile.get('profile_sha256')!=_profile_digest(profile):raise ValueError('profile内容hash改变')
    sealed=getattr(safety,'_observed_contact_profiles',{}).get(profile['profile_sha256'])
    if sealed!=json.dumps(profile,sort_keys=True,allow_nan=False):
        raise ValueError('profile不是当前Safety由实际源帧封存的原始caps，禁止改值扩张')
    binding=profile.get('geometry_binding',{})
    if binding!=_geometry_binding(safety):raise ValueError('profile的capture/shape/id/geometry hash不匹配')
    for source in profile['sources']:
        if not Path(source['path']).is_file() or _hash_file(source['path'])!=source['sha256']:
            raise ValueError('profile来源证据缺失或hash改变')
    if now_sim_time is None:
        if safety.executor is None:raise ValueError('离线profile校验必须给出实际当前sim_time')
        now_sim_time=float(safety.executor.world.current_time)
    age=float(now_sim_time)-profile['end_sim_time']
    if not math.isfinite(age) or age < -1e-6 or age>max_age_sim_s:raise ValueError('profile已过期或来自未来')
    if profile['sample_count']<2 or profile['end_sim_time']-profile['start_sim_time']<2. or profile['maximum_sample_interval_s']>2*profile['physics_dt_s']+1e-6:
        raise ValueError('profile不满足密采/时长条件')
    if profile['leg_shape_pair']!=sorted(('/World/L3Station/Leg',safety.model.root+'/tn__base_link_bumper_xp_028_kY0l0')):
        raise ValueError('profile目标Leg对不匹配')
    if profile.get('ordinary_environment_margin_m')!=.012 or profile.get('arm_hand_margin_m')!=.05 or profile.get('distance_numerical_tolerance_m')!=1e-7:
        raise ValueError('profile试图改变普通净空或数值界')
    effective_margin=_effective_recovery_margin(profile)
    envelope=profile['start_motion_envelope']
    arrays=[list(envelope['wheel_abs_velocity_max_rad_s'].values()),
            envelope['root_linear_velocity_component_abs_max_m_s'],envelope['root_angular_velocity_component_abs_max_rad_s'],
            envelope['roll_pitch_abs_max_rad'],list(profile['cylinder_surface_bound_max_m'].values())]
    if not all(np.isfinite(a).all() and np.min(a)>=0 for a in arrays):raise ValueError('profile动态包络无效')
    return {'valid':True,'age_sim_s':age,'profile_sha256':profile['profile_sha256'],
            'policy_id':profile.get('policy_id'),'ordinary_environment_margin_m':.012,
            'effective_recovery_environment_margin_m':effective_margin,
            'requires_separate_user_authorization':True,'normal_guard_pass':False}


def _pair(row):
    return tuple(sorted((row['shape_a'], row['shape_b'])))


def _prepare(safety, modules):
    safety._prepare_pair_cache()
    if hasattr(modules, 'modules'):
        modules = [vars(m) for m in modules.modules]
    elif isinstance(modules, dict):
        modules = modules.get('base_info', modules).get('modules')
    modules = list(modules)
    if len(modules) != 4:
        raise ValueError('需要实际四轮模块的几何常量')
    wheel_bodies = {j['body1'] for j in safety.model.joints
                    if j['name'] in [m['wheel_joint'] for m in modules]}
    if len(wheel_bodies) != 4:
        raise RuntimeError('轮模块与碰撞模型不匹配')
    exact = {}
    for index, shape in enumerate(safety.model.shapes):
        if shape['body'] not in wheel_bodies:
            continue
        if shape['type'] != 'Cylinder':
            raise RuntimeError('真实滚轮不是可核实的Cylinder，不能用圆柱旋转不变性')
        matrix = np.asarray(shape['matrix'], float)
        linear = matrix[:3, :3]
        scale = np.linalg.norm(linear, axis=0)
        if not np.allclose(scale, scale[0], atol=1e-8) or not np.allclose(
                (linear / scale).T @ (linear / scale), np.eye(3), atol=1e-7):
            raise RuntimeError('滚轮非均匀缩放/剪切，不能用原生Cylinder')
        frame = matrix.copy()
        frame[:3, :3] /= scale
        axis = shape['attributes']['axis']
        align = np.eye(4)
        if axis == 'X': align[:3, :3] = Rotation.from_rotvec([0., math.pi/2, 0.]).as_matrix()
        elif axis == 'Y': align[:3, :3] = Rotation.from_rotvec([-math.pi/2, 0., 0.]).as_matrix()
        elif axis != 'Z': raise ValueError('未知滚轮圆柱轴')
        radius = float(shape['attributes']['radius']) * scale[0]
        height = float(shape['attributes']['height']) * scale[0]
        exact[index] = (safety.model.coal.Cylinder(radius, height), frame @ align)
    if len(exact) != 4:
        raise RuntimeError('没有取得四个真实滚轮Cylinder')
    base = safety.model.root
    assembly_indices = [k for k, row in enumerate(safety._pair_records)
                        if row[7] and ((row[4] == base and row[5] in wheel_bodies)
                                       or (row[5] == base and row[4] in wheel_bodies))]
    by_shape = {tuple(sorted((row[2]['path'], row[3]['path']))): k
                for k, row in enumerate(safety._pair_records)}
    return modules, exact, assembly_indices, by_shape


def _distance(safety, index, poses, exact=None):
    ia, ib, a, b, *_ = safety._pair_records[index]
    coal = safety.model.coal
    arguments = []
    for i, shape in ((ia, a), (ib, b)):
        matrix = np.asarray(poses[shape['body']]) if shape['body'] else np.eye(4)
        geometry = shape['geometry']
        if exact and i in exact:
            geometry, local = exact[i]
            matrix = matrix @ local
            # 圆柱绕自身轴的相位不改变几何；规范化径向基向量，避免EPA因等价
            # 旋转相位选择不同而给深装配交叠返回不同数值分支。轴向与中心完全保留。
            axis = matrix[:3, 2] / np.linalg.norm(matrix[:3, 2])
            basis = np.array([0., 0., 1.]) if abs(axis[2]) < .9 else np.array([1., 0., 0.])
            tangent = np.cross(basis, axis);tangent /= np.linalg.norm(tangent)
            matrix = matrix.copy();matrix[:3, :3] = np.column_stack((tangent, np.cross(axis, tangent), axis))
        arguments.extend((geometry, coal.Transform3s(matrix[:3, :3], matrix[:3, 3])))
    result = coal.DistanceResult()
    request = getattr(safety, '_retreat_exact_distance_request', None)
    if request is None:
        request = coal.DistanceRequest()
        request.enable_signed_distance = True
        # 深CAD装配交叠的EPA距离只作原始诊断；其约1um精度不能用于判断滚动
        # 几何的亚微米变化，后面另用中心/轴线的解析表面位移上界证明不变性。
        request.gjk_tolerance = 1e-6
        request.epa_tolerance = 1e-6
        safety._retreat_exact_distance_request = request
    value = float(coal.distance(*arguments, request, result))
    if not math.isfinite(value):
        raise RuntimeError('退离精确距离非有限')
    return {'shape_a': a['path'], 'shape_b': b['path'], 'body_a': a['body'], 'body_b': b['body'],
            'distance_m': value}


def _evaluate(safety, context, q, root, poses=None, previous_distance=None):
    poses = safety.predict(q, root) if poses is None else poses
    observed=context.get('observed_contact_profile')
    ordinary_margin=(observed['ordinary_environment_margin_m'] if observed is not None
                     else context['environment_margin_m'])
    raw = safety.check_configuration(q=q, root_pose=root, poses=poses,
                                     environment_margin_m=ordinary_margin)
    exact = [_distance(safety, i, poses, context['_exact']) for i in context['_assembly_indices']]
    initial = context['exact_initial_assembly_overlaps']
    baseline = {_pair(row): row['distance_m'] for row in initial}
    all_baseline = {_pair(row): row['distance_m'] for row in context['exact_initial_assembly_distances']}
    observed_actual=observed is not None and context.get('_actual_feedback',False)
    frames=(observed['cylinder_reference_frames_first_actual'] if observed_actual
            else context['cylinder_reference_frames'])
    frame_bounds = {}
    for wheel in frames:
        matrix = np.linalg.inv(poses[safety.model.root]) @ poses[wheel['body']] @ np.asarray(wheel['local_matrix'])
        original = np.asarray(wheel['matrix_in_base'])
        displacement = float(np.linalg.norm(matrix[:3, 3] - original[:3, 3]))
        a, b = matrix[:3, 2], original[:3, 2]
        a, b = a/np.linalg.norm(a), b/np.linalg.norm(b)
        if np.dot(a,b) < 0: a = -a
        angle = float(math.atan2(np.linalg.norm(np.cross(a,b)), np.dot(a,b)))
        extent = math.hypot(wheel['radius_m'], wheel['height_m']/2)
        frame_bounds[wheel['shape_path']] = displacement + 2*extent*math.sin(angle/2)
    failures = []
    max_worsening = 0.
    new_self = []
    observed_minima={tuple(row['shapes']):row['minimum_m'] for row in observed['assembly_solver_minimum_by_shape_pair']} if observed_actual else {}
    for row in exact:
        key = _pair(row)
        bound = max(frame_bounds.get(row['shape_a'],0.), frame_bounds.get(row['shape_b'],0.))
        row['cylinder_surface_change_bound_m'] = bound
        row['reference_solver_distance_m'] = all_baseline[key]
        row['solver_distance_is_diagnostic'] = True
        if observed_actual and key not in observed_minima and row['distance_m']<0:
            new_self.append(row)
        elif not observed_actual and key not in baseline and all_baseline[key] <= bound + 2e-6:
            new_self.append(row)  # 初始距离的数值不确定区间不能被当成无新碰撞证据。
        if observed_actual and key in observed_minima and row['distance_m']<observed_minima[key]-1e-7:
            failures.append({'reason':'既有装配深度劣于封存的实测baseline最坏值','pair':row,
                             'observed_floor_m':observed_minima[key]})
        if key in baseline:
            max_worsening = max(max_worsening, baseline[key] - row['distance_m'])
    if new_self:
        failures.append({'reason': '新增真实Cylinder底盘自身穿透shape对', 'pairs': new_self})
    surface_bound = max(frame_bounds.values(), default=0.)
    dynamic_measurements=[]
    if observed_actual:
        for path,value in frame_bounds.items():
            limit=observed['cylinder_surface_bound_max_m'].get(path)
            dynamic_measurements.append({'shape':path,'actual_surface_bound_m':value,
                                         'observed_max_m':limit,'numerical_bound_m':1e-7})
            if limit is None or value>limit+1e-7:
                failures.append({'reason':'实际轮组件波动超出封存的实测包络','shape':path,
                                 'actual_surface_bound_m':value,'observed_max_m':limit})
    elif surface_bound > context['distance_numerical_tolerance_m']:
        failures.append({'reason': '原有圆柱装配几何变化超过解析数值界，不能证明穿深不变',
                         'maximum_surface_change_bound_m': surface_bound})
    assembly_keys = {_pair(safety._pair_records[i][2] | {'shape_a': safety._pair_records[i][2]['path'],
                                                       'shape_b': safety._pair_records[i][3]['path']})
                     for i in context['_assembly_indices']}
    for row in raw['violations']:
        if row['self']:
            if _pair(row) not in assembly_keys:
                failures.append({'reason': '非已审计底盘组件自身碰撞/净空不足', 'pair': row})
        elif (_pair(row) != tuple(context['recovering_shape_pair'])
              and row['distance_m']<context['environment_margin_m']):
            failures.append({'reason': '新环境碰撞或低于导航12mm buffer', 'pair': row})
    leg = _distance(safety, context['_leg_index'], poses)
    if observed is not None:
        leg['solver_distance_m']=leg['distance_m']
        leg['distance_m']=_box_sat_distance(safety,context['_leg_index'],poses)
        leg['distance_method']='independent_Cube_SAT'
    distance = leg['distance_m']
    tolerance = context['distance_numerical_tolerance_m']
    minimum = observed['leg_sat_minimum_m'] if observed is not None else context['initial_recovering_distance_m']
    if ((observed is None and distance < -tolerance) or distance < minimum - tolerance):
        failures.append({'reason': '初始受限对新增穿透或低于起点净空', 'pair': leg})
    if previous_distance is not None and not observed_actual and distance < previous_distance - tolerance:
        failures.append({'reason': '初始受限对净空下降', 'previous_distance_m': previous_distance, 'pair': leg})
    groups = ('left_hand', 'left_arm', 'right_hand', 'right_arm')
    hands, calls = safety._nearest(poses, safety._external_pairs, groups)
    if any(row is None or row['distance_m'] < context['hand_clearance_m'] for row in hands.values()):
        failures.append({'reason': '双手/双臂运输净空不足50mm', 'nearest': hands})
    polygon = [row for row in raw['violations'] if row['self'] and _pair(row) in assembly_keys]
    record = {'safe_for_recovery_step': not failures, 'failures': failures,
              'recovering_pair': leg, 'minimum_environment_clearance_m': raw['minimum_environment_clearance_m'],
              'nearest_by_arm_group': hands, 'exact_cylinder_assembly_distances': exact,
              'original_polygon_assembly_distances': polygon,
              'maximum_exact_assembly_penetration_worsening_m': max_worsening,
              'maximum_assembly_surface_change_bound_m': surface_bound,
              'assembly_invariance_method': 'Cylinder中心/轴线给出表面Hausdorff上界；不使用EPA波纹放宽物理碰撞',
              'new_exact_assembly_shape_pairs': new_self,
              'guard_mode':'observed_contact_recovery' if observed is not None else 'nominal_nonpenetrating_retreat',
              'normal_guard_pass':False if observed is not None else not failures,
              'requires_separate_user_authorization':observed is not None,
              'nominal_geometry_numerical_bound_m':context['distance_numerical_tolerance_m'],
              'observed_dynamic_assembly_measurements':dynamic_measurements,
              'ordinary_environment_margin_m':ordinary_margin,
              'effective_recovery_environment_margin_m':context['environment_margin_m'],
              'raw_ordinary_environment_violations':[r for r in raw['violations'] if not r['self']],
              'recovery_policy_id':observed.get('policy_id') if observed is not None else None,
              'globally_collision_free': False, 'robot_collision_shapes': safety.capture_counts['robot_shapes']}
    points = np.concatenate([shape['corners'] @ poses[shape['body']][:3, :3].T + poses[shape['body']][:3, 3]
                             for shape in safety.model.shapes if shape['robot']])
    return record, points


def plan_safe_retreat(safety, modules, distances_m=(.05, .10, .15, .30), *,
                      environment_margin_m=.012, hand_clearance_m=.05,
                      final_clearance_m=.012, surface_step_m=.002,
                      distance_numerical_tolerance_m=1e-7,
                      explicit_contact_recovery=False,observed_contact_profile=None,now_sim_time=None):
    """保持当前舵角和车身朝向，双向body Y退离；返回纯计划，不执行。

    仅允许一个初始正净空Leg对低于buffer。1e-7m容差用于输入float32位姿和距离
    求解数值精度，报告每个原始穿深；不能把96边形近似波纹当实际Cylinder变形。
    """
    modules, exact, indices, by_shape = _prepare(safety, modules)
    if bool(explicit_contact_recovery)!=(observed_contact_profile is not None):
        raise ValueError('接触脱困模式与fresh observed_contact_profile必须显式同时给出')
    if explicit_contact_recovery:
        validate_observed_contact_profile(safety,observed_contact_profile,now_sim_time=now_sim_time)
        if (environment_margin_m!=_effective_recovery_margin(observed_contact_profile)
                or hand_clearance_m!=observed_contact_profile['arm_hand_margin_m']
                or distance_numerical_tolerance_m!=observed_contact_profile['distance_numerical_tolerance_m']
                or final_clearance_m<observed_contact_profile.get('recovery_final_leg_clearance_m',observed_contact_profile['ordinary_environment_margin_m'])
                or surface_step_m>.002):
            raise ValueError('observed模式的环境/手臂/数值界/终点与扫掠参数必须绑定封存profile，禁止另传参数放宽')
    q0, poses0 = safety.refresh()
    root0 = np.array(poses0[safety.model.root], copy=True)
    wheel_joints = {m['wheel_joint']: next(j for j in safety.model.joints if j['name'] == m['wheel_joint'])
                    for m in modules}
    initial_report = safety.check_configuration(environment_margin_m=environment_margin_m)
    environment = [r for r in initial_report['violations'] if not r['self']]
    if (len(environment) != 1 or (environment[0]['distance_m'] < 0 and not explicit_contact_recovery)
            or '/World/L3Station/Leg' not in (environment[0]['shape_a'], environment[0]['shape_b'])):
        return {'safe': False, 'pass': False, 'reason': '仅支持已审计的唯一正间隙Leg受限对，不允许泛化恢复',
                'initial_environment_violations': environment, 'candidates': [], 'stage_modified': False}
    leg = environment[0]
    exact0 = [_distance(safety, i, poses0, exact) for i in indices]
    cylinder_frames = []
    for i,(geometry,local) in exact.items():
        shape = safety.model.shapes[i]
        cylinder_frames.append({'body':shape['body'],'shape_path':shape['path'],
            'local_matrix':local.tolist(),
            'matrix_in_base':(np.linalg.inv(root0)@poses0[shape['body']]@local).tolist(),
            'radius_m':float(geometry.radius),'height_m':float(geometry.halfLength)*2})
    context = {'version': 1, 'capture_id': safety.capture_id,
               'recovering_shape_pair': list(_pair(leg)), 'initial_recovering_distance_m': leg['distance_m'],
               'exact_initial_assembly_overlaps': [r for r in exact0 if r['distance_m'] < 0],
               'exact_initial_assembly_distances': exact0, 'cylinder_reference_frames':cylinder_frames,
               'environment_margin_m': environment_margin_m, 'hand_clearance_m': hand_clearance_m,
               'distance_numerical_tolerance_m': distance_numerical_tolerance_m,
               'cylinder_geometry_method': 'coal原生Cylinder；同一USD半径/高度/local刚体矩阵，未修改实际几何',
               'initial_q': q0.tolist(), 'initial_root': root0.tolist(),
               'observed_contact_profile':observed_contact_profile,
               'explicit_contact_recovery':bool(explicit_contact_recovery),
               'goal_clearance_scope':('recovering_leg_pair' if observed_contact_profile and observed_contact_profile.get('policy_id')==LOW_SPEED_POLICY_ID else 'all_environment'),
               'required_final_leg_clearance_m':final_clearance_m,
               'recovery_control_caps':(dict(observed_contact_profile['recovery_control_caps']) if observed_contact_profile and observed_contact_profile.get('policy_id')==LOW_SPEED_POLICY_ID else None),
               '_exact': exact, '_assembly_indices': indices, '_leg_index': by_shape[_pair(leg)]}
    candidates = []
    yaw = math.atan2(root0[1, 0], root0[0, 0])
    planar = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    for sign in (1., -1.):
        for length in distances_m:
            if not math.isfinite(length) or length <= 0:
                raise ValueError('退出距离必须是有限正数')
            steps = max(1, int(math.ceil(length / min(.0008, surface_step_m / 3))))
            samples, previous, previous_points = [], None, None
            success = True
            for k in range(steps+1):
                travel = sign * length * k / steps
                delta = np.array([0., travel])
                root = root0.copy();root[:2, 3] += planar @ delta
                q = q0.copy()
                rolling = []
                for module in modules:
                    steering = q0[safety.model.names.index(module['steering_joint'])]
                    heading = module['heading_zero'] + module['steering_sign'] * steering
                    tangent = np.array([math.cos(heading), math.sin(heading)])
                    along = float(tangent @ delta)
                    wheel_delta = along / (module['wheel_sign'] * module['radius_m'])
                    q[safety.model.names.index(module['wheel_joint'])] += wheel_delta
                    rolling.append({'wheel_joint': module['wheel_joint'], 'steering_joint': module['steering_joint'],
                                    'steering_target_rad': float(steering), 'wheel_delta_rad': wheel_delta,
                                    'wheel_target_rad': float(q[safety.model.names.index(module['wheel_joint'])]),
                                    'lateral_no_slip_residual_m': float(abs(tangent[0]*delta[1]-tangent[1]*delta[0]))})
                # 只平移整机且只让四轮滚动：直接从同批物理刚体姿态推进，不把
                # 全树FK约1um的残差误认为起点立即发生了亚微米装配变形。
                root_delta = root @ np.linalg.inv(root0)
                predicted = dict(poses0)
                for body in safety.model.cluster:
                    predicted[body] = root_delta @ poses0[body]
                for module in modules:
                    joint = wheel_joints[module['wheel_joint']]
                    difference = q[joint['index']] - q0[joint['index']]
                    vector = np.zeros(3);vector['XYZ'.index(joint['axis'])] = difference
                    rotation = np.eye(4);rotation[:3, :3] = Rotation.from_rotvec(vector).as_matrix()
                    predicted[joint['body1']] = (root_delta @ poses0[joint['body1']]
                        @ np.linalg.inv(joint['B']) @ rotation @ joint['B'])
                record, points = _evaluate(safety, context, q, root, poses=predicted, previous_distance=previous)
                record.update({'fraction': k/steps, 'distance_body_y_m': travel, 'root_pose': root.tolist(),
                               'q': q.tolist(), 'wheels': rolling})
                if previous_points is not None:
                    jump = float(np.linalg.norm(points - previous_points, axis=1).max())
                    record['maximum_surface_step_m'] = jump
                    if jump > surface_step_m + 1e-9:
                        record['safe_for_recovery_step'] = False
                        record['failures'].append({'reason': '退离表面扫掠步幅超过2mm', 'distance_m': jump})
                samples.append(record)
                if not record['safe_for_recovery_step']:
                    success = False
                    break
                previous = record['recovering_pair']['distance_m']; previous_points = points
            end = samples[-1]
            low_policy=bool(observed_contact_profile and observed_contact_profile.get('policy_id')==LOW_SPEED_POLICY_ID)
            goal = (success and end['recovering_pair']['distance_m']>=final_clearance_m
                    and end['minimum_environment_clearance_m']>=environment_margin_m) if low_policy else (success and end['minimum_environment_clearance_m']>=final_clearance_m)
            candidates.append({'safe': bool(goal), 'body_y_distance_m': sign*length,
                               'goal_clearance_m': final_clearance_m, 'samples': samples,
                               'goal_clearance_scope':context['goal_clearance_scope'],
                               'recovery_environment_margin_m':environment_margin_m,
                               'completed_path': success, 'final_clearance_m': end['minimum_environment_clearance_m'],
                               'reason': (('低速10mm参考及Leg12mm终点通过；普通12mm未通过' if low_policy else '退出全路径与终点12mm检查通过') if goal else
                                          '终点尚未清出12mm' if success else '路径出现新的碰撞/净空或穿深恶化'),
                               'globally_collision_free': False})
    safe_candidates = [c for c in candidates if c['safe']]
    chosen = min(safe_candidates, key=lambda c: abs(c['body_y_distance_m'])) if safe_candidates else None
    reference = {k: v for k, v in context.items() if not k.startswith('_')}
    return {'safe': chosen is not None, 'pass': chosen is not None, 'reference': reference,
            'guard_mode':'observed_contact_recovery' if explicit_contact_recovery else 'nominal_nonpenetrating_retreat',
            'requires_separate_user_authorization':bool(explicit_contact_recovery),'execution_authorized':False,
            'normal_guard_pass':False if explicit_contact_recovery else chosen is not None,
            'chosen': chosen, 'candidates': candidates, 'stage_modified': False,
            'no_chassis_rotation': True, 'steering_changed': False,
            'scope': '只读真实轮角同步的全身退出方案；实际反馈必须逐步重复复核，未驱动'}


def validate_retreat_step(safety, plan, modules, *, previous=None, refresh=True,now_sim_time=None):
    """由运行时在主线程对实际全身反馈复核；不是执行入口。"""
    modules, exact, indices, by_shape = _prepare(safety, modules)
    reference = plan['reference']
    if reference['capture_id'] != safety.capture_id:
        raise ValueError('退出参考不属于当前采样；必须重新规划')
    observed=reference.get('observed_contact_profile')
    profile_check=None
    if observed is not None:
        if reference.get('explicit_contact_recovery') is not True:
            raise ValueError('观测接触profile不能隐式启用')
        profile_check=validate_observed_contact_profile(safety,observed,now_sim_time=now_sim_time)
        if (reference['environment_margin_m']!=_effective_recovery_margin(observed)
                or reference['hand_clearance_m']!=observed['arm_hand_margin_m']
                or reference['distance_numerical_tolerance_m']!=observed['distance_numerical_tolerance_m']):
            raise ValueError('实际observed guard参数与封存profile不一致')
        if observed.get('policy_id')==LOW_SPEED_POLICY_ID:
            if (reference.get('goal_clearance_scope')!='recovering_leg_pair'
                    or reference.get('required_final_leg_clearance_m',0)<.012
                    or reference.get('recovery_control_caps')!=observed['recovery_control_caps']):
                raise ValueError('低速参考的Leg终点或控制caps被改变')
    if refresh:
        safety.refresh()
    context = dict(reference)
    context.update({'_exact': exact, '_assembly_indices': indices,
                    '_leg_index': by_shape[tuple(reference['recovering_shape_pair'])],
                    '_actual_feedback':observed is not None})
    previous_distance = previous['recovering_pair']['distance_m'] if previous else None
    record, _ = _evaluate(safety, context, safety.q, safety.poses[safety.model.root],
                           poses=safety.poses, previous_distance=previous_distance)
    if observed is not None:
        instant=float(now_sim_time if now_sim_time is not None else safety.executor.world.current_time)
        old=(previous or {}).get('observed_contact_monitor')
        if old is not None and old['profile_sha256']!=observed['profile_sha256']:
            raise ValueError('不能拼接不同profile的接触窗口')
        history=list(old['history']) if old else []
        first=old['first_sim_time'] if old else instant
        negative_start=old['negative_run_start_sim_time'] if old else None
        distance=record['recovering_pair']['distance_m']
        if history and instant<history[-1][0]-1e-9:raise ValueError('接触监控时间倒退')
        if history and instant==history[-1][0]:history[-1]=[instant,distance]
        else:history.append([instant,distance])
        if distance<0 and negative_start is None:negative_start=instant
        run=0. if negative_start is None else instant-negative_start
        if run>observed['maximum_negative_run_duration_s']+1e-7:
            record['failures'].append({'reason':'Leg连续接触持续时间超过封存baseline最坏值',
                                      'actual_duration_s':run,'observed_max_s':observed['maximum_negative_run_duration_s']})
        if distance>=0:negative_start=None
        window=observed['contact_window_s']
        complete=instant-first>=window-1e-9
        duty=_contact_window(history,window) if complete else None
        if duty is not None and duty>observed['maximum_contact_time_fraction']+1e-7:
            record['failures'].append({'reason':'Leg接触窗口占比超过封存baseline最坏值',
                                      'actual_fraction':duty,'observed_max':observed['maximum_contact_time_fraction']})
        while len(history)>1 and history[1][0]<=instant-window:history.pop(0)
        record['observed_contact_monitor']={'profile_sha256':observed['profile_sha256'],
            'profile_age_sim_s':profile_check['age_sim_s'],'first_sim_time':first,
            'history':history,'negative_run_start_sim_time':negative_start,
            'actual_negative_run_duration_s':run,'window_complete':complete,'contact_time_fraction':duty,
            'leg_hard_floor_m':observed['leg_sat_minimum_m'],'distance_numerical_tolerance_m':1e-7,
            'empirical_not_theoretical_guarantee':True,'normal_guard_pass':False}
        record['safe_for_recovery_step']=not record['failures']
    root0 = np.asarray(reference['initial_root']);root = safety.poses[safety.model.root]
    delta = root0[:3, :3].T @ (root[:3, 3] - root0[:3, 3])
    orientation = float(Rotation.from_matrix(root[:3, :3] @ root0[:3, :3].T).magnitude())
    record.update({'actual_body_displacement_m': delta.tolist(), 'actual_orientation_change_rad': orientation,
                   'actual_physics_pose': True})
    return record
