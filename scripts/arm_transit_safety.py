"""从当前物理真值检查双臂/手与环境净空；只读建模与规划，不驱动机器人。"""
import itertools
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation


def _matrix(position, quaternion_xyzw):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(quaternion_xyzw).as_matrix()
    result[:3, 3] = position
    return result


def _group(body):
    leaf = (body or '').split('/')[-1]
    for side in ('left', 'right'):
        if leaf.startswith(side+'_gripper_'):
            return side+'_hand'
        if leaf.startswith(side+'_arm_'):
            return side+'_arm'
    return None


def _capture_current(executor, directory):
    """复用导出器的几何/锚点解析，去掉昂贵USD导出；每次必须重新采当前场景。"""
    from pxr import Usd, UsdGeom, UsdPhysics
    source_path = Path(__file__).with_name('scene')/'snapshot_collision_scene.py'
    source = source_path.read_text()
    source = source.replace("    stage.GetRootLayer().Export(str(output/'scene_root.usda'))\n", '')
    source = source.replace("    stage.Flatten().Export(str(output/'scene_flat.usdc'))\n", '')
    if 'stage.Flatten().Export' in source or 'stage.GetRootLayer().Export' in source:
        raise RuntimeError('几何导出器布局改变，拒绝意外导出完整USD，请同步只读采样器')
    namespace = {}
    exec(compile(source, str(source_path), 'exec'), namespace)
    counts = namespace['snapshot'](executor, directory)
    data = json.loads((directory/'snapshot.json').read_text())
    arrays_file = np.load(directory/'geometry.npz')
    arrays = {key: arrays_file[key] for key in arrays_file.files}
    arrays_file.close()
    stage = executor._l3_station.runtime.stage
    cache = UsdGeom.XformCache()
    # 主柜壳曾没有CollisionAPI；显式保留可见柜壳保护，不能因此声称壳内空间可通行。
    for prim in Usd.PrimRange(stage.GetPrimAtPath('/World/Cabinet'), Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if UsdGeom.Imageable(prim).ComputeVisibility() == 'invisible':
            continue
        if 'CabinetBody' not in str(prim.GetPath()):
            continue
        mesh = UsdGeom.Mesh(prim)
        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
        if len(points) < 4:
            raise RuntimeError('柜壳可视几何不足，无法建立非操作臂包络')
        world = np.asarray(cache.GetLocalToWorldTransform(prim)).T
        vertices = points@world[:3, :3].T+world[:3, 3]
        key = 'visual_shell_'+str(len(data['shapes']))
        arrays[key+'_v'] = vertices
        arrays[key+'_f'] = np.zeros((0, 3), dtype=np.int32)
        data['shapes'].append({'path': str(prim.GetPath()), 'body': None, 'type': 'Mesh',
            'robot': False, 'approximation': 'convexHull', 'key': key, 'source': 'visual_envelope'})
    # 静态几何原来是世界坐标顶点，后续以当前/捕获Xform之差更新，避免冻结障碍位置。
    static_frames = {}
    for shape in data['shapes']:
        shape.setdefault('source', 'collision_api')
        if shape['body'] is None and shape['type'] != 'Plane':
            path = shape['path']
            static_frames[path] = np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T.tolist()
            frame_id = '__static__:'+path
            shape['body'] = frame_id
            data['poses'][frame_id] = np.eye(4).tolist()
    data['static_frames'] = static_frames
    data['capture_time'] = float(executor.world.current_time)
    data['capture_wall_time'] = time.time()
    data['description'] = '当前场景重新采样；未导出或修改USD，不包含旧快照几何'
    np.savez_compressed(directory/'geometry.npz', **arrays)
    (directory/'snapshot.json').write_text(json.dumps(data, ensure_ascii=False, indent=2))
    return counts


class ArmTransitSafety:
    """环境净空与收臂只读规划器；调用方负责停止/执行，类本身绝不下发关节目标。"""
    def __init__(self, executor, directory=None):
        self.executor = executor
        self.stage = executor._l3_station.runtime.stage
        self.directory = Path(directory) if directory else Path(__file__).resolve().parents[1]/'outputs/lateral_correction/arm_safety_geometry'
        self.directory.mkdir(parents=True, exist_ok=True)
        self.capture_counts = _capture_current(executor, self.directory)
        # Kit导入缓存可能未见新脚本，因此使用项目文件显式加载已有几何构造器。
        import sys
        import types
        path = Path(__file__).with_name('robot_collision_guard.py')
        module = sys.modules.get('arm_transit_collision_model') or types.ModuleType('arm_transit_collision_model')
        module.__file__ = str(path)
        sys.modules[module.__name__] = module
        exec(compile(path.read_text(), str(path), 'exec'), module.__dict__)
        policy_path = Path(__file__).resolve().parents[1]/'assets/robot_safety/policy.json'
        exclusions = json.loads(policy_path.read_text()).get('extra_exclusions', []) if policy_path.exists() else []
        self.model = module.CollisionModel(self.directory, exclusions)
        self._original_pairs = self.model.pairs.copy()
        self._groups = [_group(shape['body']) for shape in self.model.shapes]
        self._external_pairs = np.asarray([(i, j) for i, j in self._original_pairs
            if self.model.shapes[i]['robot'] != self.model.shapes[j]['robot']
            and (self._groups[i] or self._groups[j])], dtype=int).reshape(-1, 2)
        self._static_frames = {path: np.array(matrix) for path, matrix in self.model.data.get('static_frames', {}).items()}
        self._rigid_paths = {shape['body'] for shape in self.model.shapes
                             if shape['body'] and not shape['body'].startswith('__static__:')}
        self._physics_iface = executor._cabinet_probe.iface
        self.last_report = None
        self.refresh()

    def _read_poses(self):
        from pxr import UsdGeom
        poses = dict(self.model.poses0)
        views = [self.executor.articulation]
        for name in ('_oxford_devices',):
            runtime = getattr(self.executor, name, None)
            if runtime:
                views.extend(runtime.devices.values())
        views.extend(self.executor._l3_station.runtime.devices.values())
        found = set()
        for view in views:
            values = np.asarray(view._physics_view.get_link_transforms())[0]
            for path, value in zip(view._physics_view.link_paths[0], values):
                poses[path] = _matrix(value[:3], value[3:])
                found.add(path)
        for path in self._rigid_paths-found:
            value = self._physics_iface.get_rigidbody_transformation(path)
            if not value or not value.get('ret_val', False):
                raise RuntimeError('环境或机器人刚体真值不可读：'+path)
            poses[path] = _matrix(value['position'], value['rotation'])
        cache = UsdGeom.XformCache()
        for path, reference in self._static_frames.items():
            prim = self.stage.GetPrimAtPath(path)
            if not prim:
                raise RuntimeError('障碍几何被删除，请重建净空模型：'+path)
            current = np.asarray(cache.GetLocalToWorldTransform(prim)).T
            delta = current@np.linalg.inv(reference)
            if not np.allclose(delta[:3,:3].T@delta[:3,:3], np.eye(3), atol=1e-6):
                raise RuntimeError('障碍缩放已改变，请重建净空模型：'+path)
            poses['__static__:'+path] = delta
        if not all(np.isfinite(value).all() for value in poses.values()):
            raise RuntimeError('机器人或环境物理位姿非有限，不能继续走位')
        return poses

    def refresh(self):
        self.poses = self._read_poses()
        self.q = np.asarray(self.executor.articulation.get_joint_positions()).reshape(-1).copy()
        if not np.isfinite(self.q).all():
            raise RuntimeError('机器人关节反馈非有限')
        self.model.poses0 = dict(self.poses)
        return self.q.copy(), self.poses

    def _shape_bounds(self, poses, error_message='碰撞几何位姿非有限'):
        """复用不变的局部角点，批量更新当前位姿包络；不缓存世界位姿。"""
        key = (id(self.model), id(self.model.shapes), len(self.model.shapes))
        if getattr(self, '_shape_cache_key', None) != key:
            self._shape_cache_key = key
            self._shape_bodies = tuple(shape['body'] for shape in self.model.shapes)
            self._shape_corners = np.stack([shape['corners'] for shape in self.model.shapes])
            self._shape_identity = np.eye(4)
            self._pair_group_cache = []
        frames = np.stack([poses[body] if body else self._shape_identity for body in self._shape_bodies])
        if not np.isfinite(frames).all():
            raise RuntimeError(error_message)
        # 与逐形状 corners @ R.T 完全相同的8角点计算，不放宽包络或减少形状。
        corners = np.matmul(self._shape_corners, frames[:, :3, :3].swapaxes(1, 2))+frames[:, None, :3, 3]
        return frames, corners.min(axis=1), corners.max(axis=1)

    def _pairs_for_groups(self, pairs, groups):
        """配对拓扑不随机器人走位变化；仍核对数组内容，避免场景切换复用旧分组。"""
        signature = tuple(self._groups)
        cache = getattr(self, '_pair_group_cache', [])
        entry = next((entry for entry in cache if entry['source'] is pairs
                      and entry['groups'] == signature and np.array_equal(entry['pairs'], pairs)), None)
        if entry is None:
            entry = {'source': pairs, 'pairs': np.array(pairs, copy=True), 'groups': signature, 'selected': {}}
            cache.append(entry)
            # 收臂候选会创建临时配对数组，避免缓存无限保留旧规划。
            self._pair_group_cache = cache[-8:]
        identifiers = np.asarray(signature, dtype=object)
        for group in groups:
            if group not in entry['selected']:
                i, j = pairs.T
                entry['selected'][group] = pairs[(identifiers[i] == group) | (identifiers[j] == group)]
        return entry['selected']

    def _nearest(self, poses, pairs, groups):
        coal = self.model.coal
        frames, low, high = self._shape_bounds(poses)
        selections = self._pairs_for_groups(pairs, groups)
        transforms = {}
        def transform(index):
            if index not in transforms:
                frame = frames[index]
                transforms[index] = coal.Transform3s(frame[:3, :3], frame[:3, 3])
            return transforms[index]
        nearest = {}
        calls = 0
        for group in groups:
            selected = selections[group]
            if not len(selected):
                nearest[group] = None
                continue
            i, j = selected.T
            gap = np.maximum(low[i]-high[j], low[j]-high[i])
            lower = np.linalg.norm(np.maximum(gap, 0.), axis=1)
            best = None
            for k in np.argsort(lower):
                if best is not None:
                    if best['distance_m'] >= 0 and lower[k] >= best['distance_m']:
                        break
                    if best['distance_m'] < 0 and lower[k] > 0:
                        break
                a, b = map(int, selected[k])
                result = coal.DistanceResult()
                distance = float(coal.distance(self.model.shapes[a]['geometry'], transform(a),
                    self.model.shapes[b]['geometry'], transform(b), self.model.request, result))
                if not math.isfinite(distance):
                    raise RuntimeError('碰撞距离查询非有限，禁止继续走位：'+self.model.shapes[a]['path'])
                calls += 1
                if best is None or distance < best['distance_m']:
                    sa, sb = self.model.shapes[a], self.model.shapes[b]
                    best = {'distance_m': distance, 'shape_a': sa['path'], 'shape_b': sb['path'],
                            'body_a': sa['body'], 'body_b': sb['body'],
                            'source_a': sa.get('source', 'collision_api'), 'source_b': sb.get('source', 'collision_api'),
                            'group': group}
            nearest[group] = best
        return nearest, calls

    def check(self, min_clearance_m=.04, sides=('left', 'right'), include_arms=True, *, refresh=True, poses=None):
        started = time.perf_counter()
        if refresh:
            self.refresh()
        groups = [side+'_'+part for side in sides for part in (('hand','arm') if include_arms else ('hand',))]
        coverage = {group:self._groups.count(group) for group in groups}
        if any(count == 0 for count in coverage.values()):
            raise RuntimeError('手臂/夹爪碰撞几何覆盖不足：'+str(coverage))
        nearest, calls = self._nearest(self.poses if poses is None else poses, self._external_pairs, groups)
        rows = [row for row in nearest.values() if row is not None]
        violations = [row for row in rows if row['distance_m'] < min_clearance_m]
        report = {'safe': not violations, 'pass':not violations, 'minimum_distance_m': min((row['distance_m'] for row in rows), default=None),
            'nearest_by_group': nearest, 'violations': violations, 'min_clearance_m': min_clearance_m,
            'query_seconds': time.perf_counter()-started, 'coal_distance_calls': calls,
            'robot_shapes': sum(shape['robot'] for shape in self.model.shapes),
            'all_shapes': len(self.model.shapes), 'visual_envelopes': sum(shape.get('source') == 'visual_envelope' for shape in self.model.shapes),
            'capture_time': self.model.data['capture_time'], 'current_time': float(self.executor.world.current_time),
            'group_shape_coverage':coverage,
            'actual_physics_pose': True, 'self_collision_solver_changed': False}
        self.last_report = report
        return report

    def check_arm_path(self, side, goal_q, min_clearance_m=.04, body_clearance_m=.008):
        """当前位姿到候选收臂的连续采样，只读；环境与非相邻身体均检查。"""
        if side not in ('left','right'):
            raise ValueError('手臂必须为left/right')
        q0, poses0 = self.refresh()
        names = [f'{side}_arm_{i}_joint' for i in range(1,7)]
        if not all(name in self.model.names for name in names):
            raise RuntimeError('当前不是已支持的六轴手臂')
        indices = [self.model.names.index(name) for name in names]
        goal_q = np.asarray(goal_q, dtype=float)
        if goal_q.shape != (6,) or not np.isfinite(goal_q).all():
            raise ValueError('收臂候选必须是6个有限关节角')
        runtime_limits = np.asarray(self.executor.articulation.get_dof_limits()).reshape(-1,2)[indices]
        if np.any(goal_q < runtime_limits[:,0]+.02) or np.any(goal_q > runtime_limits[:,1]-.02):
            return {'safe':False,'reason':'收臂目标超限','joint_names':names}
        moving = [i for i,g in enumerate(self._groups) if g in (side+'_arm',side+'_hand')]
        self_pairs = np.asarray([(i,j) for i,j in self._original_pairs if self.model.shapes[i]['robot'] and self.model.shapes[j]['robot']
            and (i in moving or j in moving)
            # 收臂不改变夹爪关节，同一夹爪的闭合指垫接触保持不变，不是手臂撞身体。
            and not (self._groups[i] == side+'_hand' and self._groups[j] == side+'_hand')], dtype=int).reshape(-1,2)
        external_pairs = np.asarray([(i,j) for i,j in self._external_pairs if i in moving or j in moving], dtype=int).reshape(-1,2)
        start = q0[indices]
        samples = []
        minimum = float('inf')
        body_minimum = float('inf')
        closest_environment = None
        closest_body = None
        root_pose = poses0[self.model.root]
        def evaluate(arm):
            full = q0.copy();full[indices] = arm
            poses = self.model.fk(full, root_pose)
            nearest,_ = self._nearest(poses, external_pairs, [side+'_arm',side+'_hand'])
            env = min((h['distance_m'] for h in nearest.values() if h), default=float('inf'))
            body,_ = self._nearest(poses, self_pairs, [side+'_arm',side+'_hand'])
            internal = min((h['distance_m'] for h in body.values() if h), default=float('inf'))
            corners = np.concatenate([self.model.shapes[i]['corners']@poses[self.model.shapes[i]['body']][:3,:3].T
                                      +poses[self.model.shapes[i]['body']][:3,3] for i in moving])
            return env, internal, corners, nearest, body
        cache = {}
        def sample(u):
            nonlocal minimum, body_minimum, closest_environment, closest_body
            if u not in cache:
                cache[u] = evaluate(start+(goal_q-start)*u)
                minimum = min(minimum,cache[u][0]);body_minimum = min(body_minimum,cache[u][1])
                for row in cache[u][3].values():
                    if row and (closest_environment is None or row['distance_m'] < closest_environment['distance_m']):
                        closest_environment = {**row,'path_fraction':u}
                for row in cache[u][4].values():
                    if row and (closest_body is None or row['distance_m'] < closest_body['distance_m']):
                        closest_body = {**row,'path_fraction':u}
            return cache[u]
        def visit(a,b,depth=0):
            ra,rb,rm = sample(a),sample(b),sample((a+b)/2)
            for value in (ra,rb,rm):
                if value[0] < min_clearance_m or value[1] < body_clearance_m:
                    return False
            displacement = max(float(np.max(np.linalg.norm(ra[2]-rm[2],axis=1))),
                               float(np.max(np.linalg.norm(rb[2]-rm[2],axis=1))))
            if displacement > .002 or np.max(abs((goal_q-start)*(b-a))) > .01:
                if depth >= 16:
                    return False
                return visit(a,(a+b)/2,depth+1) and visit((a+b)/2,b,depth+1)
            samples.append({'from':a,'to':b,'max_surface_step_m':displacement})
            return True
        safe = visit(0.,1.)
        def hand_envelope(full):
            predicted = self.model.fk(full,root_pose)
            points = np.concatenate([shape['corners']@predicted[shape['body']][:3,:3].T+predicted[shape['body']][:3,3]
                for index,shape in enumerate(self.model.shapes) if self._groups[index] == side+'_hand'])
            return {'maximum_horizontal_radius_from_base_m':float(np.linalg.norm(points[:,:2]-root_pose[:2,3],axis=1).max()),
                    'world_aabb_min':points.min(0).tolist(),'world_aabb_max':points.max(0).tolist()}
        full_goal=q0.copy();full_goal[indices]=goal_q
        before_envelope,after_envelope=hand_envelope(q0),hand_envelope(full_goal)
        return {'safe':safe,'pass':safe,'joint_names':names,'start_q':start.tolist(),'goal_q':goal_q.tolist(),
            'minimum_environment_clearance_m':minimum,'minimum_body_clearance_m':body_minimum,
            'min_clearance_m':min_clearance_m,'body_clearance_m':body_clearance_m,
            'configuration_samples':len(cache),'segments':len(samples),
            'closest_environment_pair':closest_environment,'closest_body_pair':closest_body,
            'start_hand_envelope':before_envelope,'goal_hand_envelope':after_envelope,
            'sequence':[start.tolist(),goal_q.tolist()] if safe else [],
            'path':[start.tolist(),goal_q.tolist()] if safe else [],
            'waypoints':[goal_q.tolist()] if safe else [],
            'reason':'环境与非相邻身体包络全路径检查通过' if safe else '当前姿态/路径存在环境或身体净空不足，未授权执行',
            'self_collision_solver_changed':False, 'capture_time':self.model.data['capture_time']}

    def propose_stow(self, side='left', min_clearance_m=.04, body_clearance_m=.008):
        """由当前实际六轴链求向下收臂候选，验证完整路径后才给可执行sequence。"""
        import arm_kin
        from scipy.optimize import least_squares
        q0, poses = self.refresh()
        names = [f'{side}_arm_{i}_joint' for i in range(1,7)]
        indices = [self.model.names.index(name) for name in names]
        seed = q0[indices]
        source = arm_kin.URDF
        chain = arm_kin.load_chain(source,side+'_arm_base_link',side+'_gripper_base_link')
        if arm_kin.joint_names(chain) != names:
            raise RuntimeError('当前URDF手臂链/关节顺序不匹配')
        table = self.executor._link_pose_table()
        p,quat = table[side+'_arm_base_link'];A = Rotation.from_quat(quat,scalar_first=True).as_matrix()
        fk_position,fk_rotation = arm_kin.ee_world(chain,seed,A,p)
        flange_position,flange_quaternion = table[side+'_gripper_base_link']
        flange_rotation = Rotation.from_quat(flange_quaternion,scalar_first=True).as_matrix()
        if (np.linalg.norm(fk_position-flange_position)>.002
                or Rotation.from_matrix(fk_rotation@flange_rotation.T).magnitude()>math.radians(1.)):
            raise RuntimeError('当前URDF FK与实际法兰不一致，禁止规划收臂')
        root_pose = poses[self.model.root]
        hand_points = np.concatenate([shape['corners']@poses[shape['body']][:3,:3].T+poses[shape['body']][:3,3]
            for index,shape in enumerate(self.model.shapes) if self._groups[index] == side+'_hand'])
        radius = float(np.linalg.norm(hand_points[:,:2]-root_pose[:2,3],axis=1).max())
        # 按当前几何而不是关节是否为零判断；这些是旧RM65的收纳判据，不是真机规格。
        if radius <= .55 and hand_points[:,2].max() < p[2]-.20:
            current = self.check_arm_path(side,seed,min_clearance_m,body_clearance_m)
            if current['safe']:
                return {**current,'safe':True,'pass':True,'already_stowed':True,'side':side,
                        'sequence':[],'path':[],'waypoints':[], 'goal_q':seed.tolist(),
                        'maximum_horizontal_hand_radius_m':radius,
                        'reason':'当前已处于低伸出收纳姿态，实际环境/身体净空通过，无需重复驱动',
                        'attempts':[], 'method':'当前物理几何核验；未下发关节命令'}
        bp,bq = table['base_link'];B = Rotation.from_quat(bq,scalar_first=True).as_matrix()
        B[:,2] = [0.,0.,1.]
        x = B[:,0];x[2] = 0.;x /= np.linalg.norm(x);y = np.cross([0.,0.,1.],x);B = np.column_stack((x,y,[0.,0.,1.]))
        limits = np.asarray(self.executor.articulation.get_dof_limits()).reshape(-1,2)[indices]
        sign = 1. if side=='left' else -1.
        tool_z = np.array([0.,0.,-1.]);tool_x = B[:,0];tool_y = np.cross(tool_z,tool_x)
        down = np.column_stack((tool_x,tool_y,tool_z))
        attempts=[]
        for local in [(-.10,sign*.34,1.02),(-.05,sign*.30,1.10),(-.15,sign*.38,.95),(0.,sign*.36,1.05)]:
            target = bp+B@np.array([local[0],local[1],0.]);target[2]=local[2]
            for yaw in (0.,math.pi/2,-math.pi/2,math.pi):
                R = Rotation.from_rotvec([0.,0.,yaw]).as_matrix()@down
                def error(v):
                    position,orientation=arm_kin.ee_world(chain,v,A,p)
                    return np.r_[position-target,.25*Rotation.from_matrix(R@orientation.T).as_rotvec()]
                fit = least_squares(error,np.clip(seed,limits[:,0]+.021,limits[:,1]-.021),
                    bounds=(limits[:,0]+.02,limits[:,1]-.02),max_nfev=300)
                residual=error(fit.x)
                row={'target_world':target.tolist(),'orientation_yaw_rad':yaw,
                     'position_error_m':float(np.linalg.norm(residual[:3])),
                     'angle_error_rad':float(np.linalg.norm(residual[3:])/.25)}
                if row['position_error_m']>.001 or row['angle_error_rad']>math.radians(1.):
                    attempts.append(row);continue
                path = self.check_arm_path(side,fit.x,min_clearance_m,body_clearance_m)
                compact = (path['goal_hand_envelope']['maximum_horizontal_radius_from_base_m']
                           < path['start_hand_envelope']['maximum_horizontal_radius_from_base_m']-.15)
                path['stow_reduces_horizontal_reach'] = bool(compact)
                if not compact:
                    path['safe']=path['pass']=False
                    path['sequence']=path['path']=path['waypoints']=[]
                    path['reason']='候选未充分缩小手部水平包络，不能作为收臂姿态'
                row['path']=path;attempts.append(row)
                if path['safe']:
                    return {'safe':True,'pass':True,**path,'side':side,'target_world':target.tolist(),'attempts':attempts,
                            'method':'实际六轴FK/IK与当前几何连续环境/身体检查；仅规划未执行'}
        return {'safe':False,'pass':False,'side':side,'joint_names':names,'sequence':[],'path':[],'waypoints':[],
                'reason':'未找到全路径净空通过的收臂候选，保持停止，不发送零位目标','attempts':attempts,
                'method':'实际六轴FK/IK与当前几何连续环境/身体检查；仅规划未执行'}
