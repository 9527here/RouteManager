"""阀门操作的只读全身碰撞适配器；采样、FK、扫掠均不修改仿真。

采样必须由已有 Isaac 主线程调用。离线模式只需已有 snapshot.json / geometry.npz。
不依赖 L3 工位，也没有启动、运动、reset、关节写入或 HTTP 入口。
"""
import itertools
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from arm_transit_safety import ArmTransitSafety, _group
from robot_collision_guard import CollisionModel, clearance_for


def _matrix(position, quaternion_xyzw):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(quaternion_xyzw).as_matrix()
    result[:3, 3] = position
    if not np.isfinite(result).all():
        raise RuntimeError('刚体实际位姿非有限，拒绝规划')
    return result


def _joint_frame(position, quaternion):
    return _matrix(list(position), [*quaternion.GetImaginary(), quaternion.GetReal()]).tolist()


def capture_scene(executor, directory, visual_envelopes=('/World/Cabinet/CabinetBody/CabinetBody',)):
    """读取当前全部 CollisionAPI（含不可见形状），仅写诊断文件，不导出或编辑 USD。"""
    import omni.usd
    import omni.physx
    from pxr import Usd, UsdGeom, UsdPhysics

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    stage = omni.usd.get_context().get_stage()
    robot = executor.articulation
    if robot is None:
        raise RuntimeError('机器人尚未加载')
    iface = omni.physx.get_physx_interface()
    values = np.asarray(robot._physics_view.get_link_transforms())[0]
    poses = {path: _matrix(v[:3], v[3:]).tolist()
             for path, v in zip(robot._physics_view.link_paths[0], values)}
    robot_prefix = executor.robot_prim_path.rstrip('/') + '/'
    cache = UsdGeom.XformCache()
    data = {
        'robot_root': executor.robot_prim_path, 'names': list(robot.dof_names),
        'q': np.asarray(robot.get_joint_positions()).reshape(-1).tolist(),
        'limits': np.asarray(robot.get_dof_limits()).reshape(-1, 2).tolist(),
        'poses': poses, 'bodies': {}, 'joints': [], 'shapes': [], 'static_frames': {},
        'capture_time': float(executor.world.current_time), 'capture_wall_time': time.time(),
        'stage_identifier': stage.GetRootLayer().identifier,
        'description': '主线程只读当前全场景；包含隐藏碰撞体；未导出/修改USD或物理状态',
        'visual_envelopes': list(visual_envelopes), 'conservative_shapes': [], 'filtered_pairs': [],
        'body_authored_scales': {},
    }
    arrays = {}
    collision_paths = []
    prims = list(Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()))
    for prim in prims:
        if not prim.IsActive():
            continue
        path = str(prim.GetPath())
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            data['bodies'][path] = {'name': prim.GetName(), 'robot': path.startswith(robot_prefix)}
        if prim.HasAPI(UsdPhysics.FilteredPairsAPI):
            for target in UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets():
                data['filtered_pairs'].append({'bodies': [path, str(target)],
                                               'reason': '当前USD显式FilteredPairs关系'})
        if prim.IsA(UsdPhysics.Joint) and path.startswith(robot_prefix):
            joint = UsdPhysics.Joint(prim)
            body0, body1 = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
            a, b = str(body0[0]) if body0 else None, str(body1[0]) if body1 else None
            # 世界固定根不属于机器人内部 FK 树；根姿态由物理实测给出。
            if a and b and a.startswith(robot_prefix) and b.startswith(robot_prefix):
                data['joints'].append({
                    'name': prim.GetName(), 'path': path, 'type': prim.GetTypeName(),
                    'body0': a, 'body1': b,
                    'T0': _joint_frame(joint.GetLocalPos0Attr().Get(), joint.GetLocalRot0Attr().Get()),
                    'T1': _joint_frame(joint.GetLocalPos1Attr().Get(), joint.GetLocalRot1Attr().Get()),
                    'axis': prim.GetAttribute('physics:axis').Get(),
                    'collision_enabled': bool(joint.GetCollisionEnabledAttr().Get()),
                })
        collision = prim.HasAPI(UsdPhysics.CollisionAPI) and prim.GetAttribute('physics:collisionEnabled').Get() is not False
        visual = path in visual_envelopes and not collision
        if not collision and not visual:
            continue
        if collision:
            collision_paths.append(path)
        owner = prim
        while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
            owner = owner.GetParent()
        body = str(owner.GetPath()) if owner else None
        world_matrix = np.asarray(cache.GetLocalToWorldTransform(prim)).T
        local = world_matrix
        if body:
            if body not in poses:
                observed = iface.get_rigidbody_transformation(body)
                if not observed or not observed.get('ret_val', False):
                    raise RuntimeError('碰撞体所属环境刚体没有实际反馈：' + body)
                poses[body] = _matrix(observed['position'], observed['rotation']).tolist()
            authored = np.asarray(cache.GetLocalToWorldTransform(owner)).T
            # PhysX pose只有米制刚体旋转/平移，不含上游0.01缩放。直接 inv(authored)
            # 会把该缩放消掉，厘米资产因而放大100倍；只去刚体部分，缩放留给几何。
            rigid = np.eye(4)
            rigid[:3, :3] = Rotation.from_matrix(authored[:3, :3]).as_matrix()
            rigid[:3, 3] = authored[:3, 3]
            local = np.linalg.inv(rigid) @ world_matrix
            data['body_authored_scales'][body] = np.linalg.norm(authored[:3, :3], axis=0).tolist()
        row = {'path': path, 'body': body, 'type': prim.GetTypeName(),
               'robot': bool(body and body.startswith(robot_prefix)),
               'approximation': (UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
                                 if prim.HasAPI(UsdPhysics.MeshCollisionAPI) else None),
               'source': 'collision_api' if collision else 'visual_envelope'}
        if row['type'] == 'Plane':
            row['matrix'] = local.tolist()
            row['axis'] = prim.GetAttribute('axis').Get() or 'Z'
            data['shapes'].append(row)
            continue
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            vertices = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
            counts, indices = mesh.GetFaceVertexCountsAttr().Get(), mesh.GetFaceVertexIndicesAttr().Get()
            faces, start = [], 0
            for count in counts:
                for k in range(1, count - 1):
                    faces.append([indices[start], indices[start + k], indices[start + k + 1]])
                start += count
        elif prim.IsA(UsdGeom.Cube):
            half = float(UsdGeom.Cube(prim).GetSizeAttr().Get()) / 2
            vertices = np.array(list(itertools.product((-half, half), repeat=3)))
            faces = ConvexHull(vertices).simplices
        elif prim.IsA(UsdGeom.Cylinder):
            row['matrix'] = local.tolist()
            row['attributes'] = {'radius': str(UsdGeom.Cylinder(prim).GetRadiusAttr().Get()),
                                 'height': str(UsdGeom.Cylinder(prim).GetHeightAttr().Get()),
                                 'axis': UsdGeom.Cylinder(prim).GetAxisAttr().Get()}
            data['shapes'].append(row)
            continue
        elif row['type'] in ('Sphere', 'Capsule', 'Cone'):
            # 当前人形是 Mesh/Cube/Cylinder。其他基本体先用外包盒，不能静默漏掉。
            extent = np.asarray(prim.GetAttribute('extent').Get(), dtype=float)
            if extent.shape != (2, 3) or not np.isfinite(extent).all():
                raise RuntimeError('基本碰撞体包络不可读：' + path)
            vertices = np.array(list(itertools.product(*zip(extent[0], extent[1]))))
            faces = ConvexHull(vertices).simplices
            row['approximation'] = 'convexHull'
            row['source'] = 'conservative_primitive_envelope'
            data['conservative_shapes'].append(path)
        else:
            raise RuntimeError('未支持碰撞体类型，不能省略：' + path + ' ' + row['type'])
        if vertices.ndim != 2 or vertices.shape[1:] != (3,) or len(vertices) < 4 or not np.isfinite(vertices).all():
            raise RuntimeError('碰撞网格顶点不完整或非有限：' + path)
        vertices = vertices @ local[:3, :3].T + local[:3, 3]
        key = 'shape_' + str(len(data['shapes']))
        row['key'] = key
        arrays[key + '_v'], arrays[key + '_f'] = vertices, np.asarray(faces, dtype=np.int32)
        data['shapes'].append(row)
    # 静态几何的顶点已经在世界系；用相对捕获时的变换更新其当前位置。
    for shape in data['shapes']:
        if shape['body'] is None and shape['type'] != 'Plane':
            path = shape['path']
            data['static_frames'][path] = np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T.tolist()
            shape['body'] = '__static__:' + path
            poses[shape['body']] = np.eye(4).tolist()
    data['collision_paths'] = collision_paths
    data['coverage'] = {'robot_shapes': sum(s['robot'] for s in data['shapes']),
                        'all_shapes': len(data['shapes']),
                        'robot_shape_types': {kind: sum(s['robot'] and s['type'] == kind for s in data['shapes'])
                                              for kind in sorted({s['type'] for s in data['shapes'] if s['robot']})}}
    np.savez_compressed(directory / 'geometry.npz', **arrays)
    (directory / 'snapshot.json').write_text(json.dumps(data, ensure_ascii=False, indent=2))
    return data['coverage']


class ValveSafety(ArmTransitSafety):
    """全机器人三维窄相检查；保留原收臂器接口，但不依赖任何工位类。"""
    def __init__(self, executor=None, directory=None, expected_robot_shapes=96,
                 policy_path=None, visual_envelopes=('/World/Cabinet/CabinetBody/CabinetBody',)):
        self.executor = executor
        if directory is None:
            raise ValueError('必须指定本次独立的几何输出目录')
        self.directory = Path(directory)
        if executor is not None:
            self.capture_counts = capture_scene(executor, self.directory, visual_envelopes)
        policy_path = Path(policy_path) if policy_path else Path(__file__).resolve().parents[1] / 'assets/robot_safety/policy.json'
        self.policy = json.loads(policy_path.read_text()) if policy_path.exists() else {}
        source = json.loads((self.directory / 'snapshot.json').read_text())
        self.model = CollisionModel(self.directory, self.policy.get('extra_exclusions', []) + source.get('filtered_pairs', []))
        self.capture_counts = self.model.data.get('coverage', {'robot_shapes': sum(s['robot'] for s in self.model.shapes),
                                                             'all_shapes': len(self.model.shapes)})
        count = sum(s['robot'] for s in self.model.shapes)
        if expected_robot_shapes is not None and count != expected_robot_shapes:
            raise RuntimeError(f'机器人碰撞体覆盖不完整：期望{expected_robot_shapes}，实际{count}')
        self._original_pairs = self.model.pairs.copy().reshape(-1, 2)
        self._groups = [_group(s['body']) for s in self.model.shapes]
        for group in ('left_arm', 'left_hand', 'right_arm', 'right_hand'):
            if group not in self._groups:
                raise RuntimeError('缺少实际手臂/夹爪碰撞形状：' + group)
        self._external_pairs = np.array([(i, j) for i, j in self._original_pairs
                                         if self.model.shapes[i]['robot'] != self.model.shapes[j]['robot']], dtype=int).reshape(-1, 2)
        self._static_frames = {p: np.array(t) for p, t in self.model.data.get('static_frames', {}).items()}
        self._rigid_paths = {s['body'] for s in self.model.shapes if s['body'] and not s['body'].startswith('__static__:')}
        self._support = {frozenset(p['bodies']) for p in self.policy.get('environment_allowed_pairs', [])}
        self._planes = [p for p in self.model.data['shapes'] if p['type'] == 'Plane']
        # 地面渲染geom与实际collisionPlane可不同；只许可四个wheel刚体的支撑，不能忽略整地面。
        ground_paths = [s['path'] for s in self.model.data['shapes']
                        if s['path'].startswith('/World/ground_plane/')]
        wheel_bodies = [j['body1'] for j in self.model.data['joints']
                        if j['name'] in ('front_left_wheel_joint', 'front_right_wheel_joint',
                                         'rear_left_wheel_joint', 'rear_right_wheel_joint')]
        self.ground_support_pairs = [{'bodies': [body, path], 'reason': '四个URDF轮关节子刚体与当前地面支撑'}
                                     for body in wheel_bodies for path in ground_paths]
        self._support.update(frozenset(p['bodies']) for p in self.ground_support_pairs)
        self.stage = None
        if executor is not None:
            import omni.usd
            import omni.physx
            self.stage = omni.usd.get_context().get_stage()
            self._physics_iface = omni.physx.get_physx_interface()
        self.q = self.model.q0.copy()
        self.poses = dict(self.model.poses0)
        self.last_report = None
        self.capture_id = hashlib.sha256(json.dumps({
            'root': self.model.root, 'capture_time': self.model.data.get('capture_time'),
            'shapes': sorted(s['path'] for s in self.model.shapes),
            'q': self.model.q0.tolist(),
        }, sort_keys=True).encode()).hexdigest()
        self.refresh()
        errors = self.model.check_fk()
        self.fk_error = {'max_position_m': max((e['position_m'] for e in errors), default=0.),
                         'max_angle_rad': max((e['angle_rad'] for e in errors), default=0.)}
        if self.fk_error['max_position_m'] > .008 or self.fk_error['max_angle_rad'] > math.radians(3):
            raise RuntimeError('碰撞模型 FK 与独立物理位姿不符：' + str(self.fk_error))

    def _read_poses(self):
        if self.executor is None:
            return dict(self.poses)
        reader = getattr(self, 'feedback_reader', None)
        if reader is not None:
            # 同一主线程同一物理步复用新鲜读数；几何/约束变化由USD notice立即失效。
            reader.guard.check()
            if reader.frame_time != float(self.executor.world.current_time):
                raise RuntimeError('共享碰撞反馈不是当前物理步，拒绝复用')
            poses = dict(self.model.poses0)
            for path, value in zip(reader.paths, reader.robot_transforms):
                poses[path] = _matrix(value[:3], value[3:])
            for path in self._rigid_paths - set(reader.paths):
                observed = reader.rigid_poses.get(path)
                if observed is None:
                    raise RuntimeError('共享反馈缺少实际环境刚体: ' + path)
                poses[path] = _matrix(observed['position'], np.asarray(observed['orientation'])[[1,2,3,0]])
            return poses
        from pxr import Usd, UsdGeom, UsdPhysics
        if self.stage.GetRootLayer().identifier != self.model.data['stage_identifier']:
            raise RuntimeError('场景已切换，必须重新采样碰撞几何')
        paths = {str(p.GetPath()) for p in Usd.PrimRange.Stage(self.stage, Usd.TraverseInstanceProxies())
                 if p.IsActive() and p.HasAPI(UsdPhysics.CollisionAPI)
                 and p.GetAttribute('physics:collisionEnabled').Get() is not False}
        if paths != set(self.model.data['collision_paths']):
            raise RuntimeError('碰撞体清单已改变，必须重新采样，不能使用旧保护')
        poses = dict(self.model.poses0)
        robot = self.executor.articulation
        values = np.asarray(robot._physics_view.get_link_transforms())[0]
        found = set()
        for path, value in zip(robot._physics_view.link_paths[0], values):
            poses[path] = _matrix(value[:3], value[3:])
            found.add(path)
        for path in self._rigid_paths - found:
            observed = self._physics_iface.get_rigidbody_transformation(path)
            if not observed or not observed.get('ret_val', False):
                raise RuntimeError('环境/机器人刚体实际反馈不可读：' + path)
            poses[path] = _matrix(observed['position'], observed['rotation'])
        cache = UsdGeom.XformCache()
        for path, reference in self._static_frames.items():
            prim = self.stage.GetPrimAtPath(path)
            if not prim or not prim.IsActive():
                raise RuntimeError('静态障碍已消失，必须重新采样：' + path)
            delta = np.asarray(cache.GetLocalToWorldTransform(prim)).T @ np.linalg.inv(reference)
            if not np.allclose(delta[:3, :3].T @ delta[:3, :3], np.eye(3), atol=1e-6):
                raise RuntimeError('障碍缩放已改变，必须重新采样：' + path)
            poses['__static__:' + path] = delta
        if not all(np.isfinite(t).all() for t in poses.values()):
            raise RuntimeError('物理位姿出现非有限数')
        return poses

    def refresh(self):
        self.poses = self._read_poses()
        if self.executor is not None:
            self.q = np.asarray(self.executor.articulation.get_joint_positions()).reshape(-1).copy()
        if self.q.shape != self.model.q0.shape or not np.isfinite(self.q).all():
            raise RuntimeError('全身关节反馈无效')
        self.model.poses0 = dict(self.poses)
        return self.q.copy(), dict(self.poses)

    def full_q(self, partial=None, q=None):
        """只在本地数组合并部分关节；绝不把整机反馈回灌为控制指令。"""
        full = np.asarray(self.q if q is None else q, dtype=float).copy()
        for name, value in (partial or {}).items():
            full[self.model.names.index(name)] = value
        if full.shape != self.model.q0.shape or not np.isfinite(full).all():
            raise ValueError('候选关节角无效')
        return full

    def predict(self, q=None, root_pose=None):
        full = self.full_q(q=q)
        root = self.poses[self.model.root] if root_pose is None else np.asarray(root_pose, float)
        if (root.shape != (4, 4) or not np.isfinite(root).all()
                or not np.allclose(root[3], [0., 0., 0., 1.], atol=1e-8)
                or not np.allclose(root[:3, :3].T @ root[:3, :3], np.eye(3), atol=1e-6)
                or np.linalg.det(root[:3, :3]) < 0):
            raise ValueError('候选底座位姿无效')
        predicted = self.model.fk(full, root)
        predicted.update({p: t for p, t in self.poses.items() if p not in self.model.cluster})
        return predicted

    def establish_invariant_assembly_reference(self, controlled_joint_names):
        """为指定运动审计已有底盘装配重叠；只保存证明，不更改任何碰撞过滤。

        只允许base_link与四轮刚体的既存shape重叠，且控制关节必须与相对FK依赖
        完全无交集。其他重叠仍然阻止requested motion，不能借本证书放行。
        """
        self.refresh()
        controlled = sorted(set(controlled_joint_names))
        if not controlled or any(n not in self.model.names for n in controlled):
            raise ValueError('审计必须给出实际存在的受控关节名')
        parents = {j['body1']: j for j in self.model.joints}

        def dependencies(body):
            result = set()
            while body in parents:
                joint = parents[body]
                if joint['index'] is not None:
                    result.add(joint['name'])
                body = joint['body0']
            return result

        wheel_names = {'front_left_wheel_joint', 'front_right_wheel_joint',
                       'rear_left_wheel_joint', 'rear_right_wheel_joint'}
        wheels = {j['body1'] for j in self.model.joints if j['name'] in wheel_names}
        if len(wheels) != 4 or self.model.root.split('/')[-1] != 'base_link':
            raise RuntimeError('现有装配审计只支持已确认的base_link与四轮结构')
        legal = {frozenset((self.model.root, w)) for w in wheels}
        report = self.check_configuration()
        overlaps, other = [], []
        pair_records = {}
        for row in report['violations']:
            pair = frozenset((row['body_a'], row['body_b']))
            if not row['self'] or pair not in legal or row['distance_m'] >= 0:
                other.append(row)
                continue
            a, b = sorted(pair)
            deps = dependencies(a) ^ dependencies(b)
            if set(controlled) & deps:
                raise RuntimeError('受控关节会改变已有装配对相对位姿，不能建立不变证明：' + str(sorted(deps)))
            overlaps.append(dict(row))
            pair_records[(a, b)] = {'bodies': [a, b], 'dependency_joint_names': sorted(deps),
                                    'relative_matrix': (np.linalg.inv(self.poses[a]) @ self.poses[b]).tolist()}
        protected = sorted({n for row in pair_records.values() for n in row['dependency_joint_names']})
        reference = {
            'version': 1, 'capture_id': self.capture_id, 'capture_time': self.model.data.get('capture_time'),
            'controlled_joint_names': controlled,
            'existing_shape_overlaps': overlaps, 'body_pairs': list(pair_records.values()),
            'protected_joint_positions': {n: float(self.q[self.model.names.index(n)]) for n in protected},
            'position_tolerance_m': .0002, 'angle_tolerance_rad': math.radians(.1),
            'penetration_worsening_tolerance_m': .0002,
            'other_initial_violations': other,
            'globally_collision_free': False if overlaps else report['safe'],
            'scope': '仅指定受控运动的已有底盘装配重叠不变证明；不是全身无碰撞或装配修复',
            'stage_modified': False,
        }
        return reference

    def _validate_invariant_reference(self, reference, predicted, full_q):
        if not isinstance(reference, dict) or reference.get('version') != 1 or reference.get('capture_id') != self.capture_id:
            raise ValueError('装配不变证明与当前几何采样不匹配')
        # 容差固定，不接受调用方通过修改JSON扩大许可范围。
        issues = []
        for pair in reference['body_pairs']:
            a, b = pair['bodies']
            relative = np.linalg.inv(predicted[a]) @ predicted[b]
            original = np.asarray(pair['relative_matrix'])
            dp = float(np.linalg.norm(relative[:3, 3] - original[:3, 3]))
            da = float(Rotation.from_matrix(relative[:3, :3] @ original[:3, :3].T).magnitude())
            if dp > .0002 or da > math.radians(.1):
                issues.append({'reason': '已有装配对相对位姿改变', 'bodies': [a, b],
                               'position_change_m': dp, 'angle_change_rad': da})
        for name, value in reference['protected_joint_positions'].items():
            difference = float(abs(full_q[self.model.names.index(name)] - value))
            if difference > math.radians(.1):
                issues.append({'reason': '本次未授权的轮/舵关节角改变', 'joint': name, 'difference_rad': difference})
        return issues

    def _prepare_pair_cache(self):
        """只缓存不随位姿变化的元数据；不缓存物理反馈或距离判定。"""
        if getattr(self, '_pair_cache_identity', None) == id(self._original_pairs):
            return
        rows, self_indices, environment_indices = [], [], []
        by_body_pair = {}
        for index, (ia, ib) in enumerate(self._original_pairs):
            ia, ib = int(ia), int(ib)
            a, b = self.model.shapes[ia], self.model.shapes[ib]
            ba, bb = a['body'] or a['path'], b['body'] or b['path']
            pair = frozenset((ba, bb))
            support = (pair in self._support or frozenset((ba, b['path'])) in self._support
                       or frozenset((bb, a['path'])) in self._support)
            is_self = bool(a['robot'] and b['robot'])
            margin = clearance_for({'a': ba, 'b': bb, 'self': True}) if is_self else None
            rows.append((ia, ib, a, b, ba, bb, pair, is_self, margin))
            if not support:
                (self_indices if is_self else environment_indices).append(index)
                by_body_pair.setdefault(pair, []).append(index)
        self._pair_records = rows
        self._self_pair_indices = np.asarray(self_indices, dtype=int)
        self._environment_pair_indices = np.asarray(environment_indices, dtype=int)
        self._indices_by_body_pair = by_body_pair
        self._shape_corners = np.stack([s['corners'] for s in self.model.shapes])
        self._shape_bodies = [s['body'] for s in self.model.shapes]
        self._pair_cache_identity = id(self._original_pairs)

    def check_configuration(self, q=None, root_pose=None, *, poses=None, refresh=False,
                            environment_margin_m=.005, self_margin_m=.008,
                            contact_pairs=(), contact_penetration_m=.001,
                            invariant_assembly_reference=None, clearance_overrides=(), return_surface_points=False,
                            _robot_fk_poses=False):
        """对完整机器人而非仅操作臂检查；接触仅允许明确列出的身体对，且限穿深。"""
        if refresh:
            self.refresh()
        reader = getattr(self, 'feedback_reader', None)
        if reader is not None:reader.guard.check()
        distance_cache = getattr(self, '_exact_distance_cache', None)
        if distance_cache is None:self._exact_distance_cache = distance_cache = {}
        if not all(math.isfinite(v) and v >= 0 for v in (environment_margin_m, self_margin_m, contact_penetration_m)):
            raise ValueError('净空与接触容差必须非负且有限')
        measured = poses is self.poses or (poses is None and q is None and root_pose is None)
        predicted = self.poses if measured else (self.predict(q, root_pose) if poses is None else poses)
        full = self.full_q(q=q)
        reference_issues, original_overlaps = [], {}
        if invariant_assembly_reference is not None:
            reference_issues = self._validate_invariant_reference(invariant_assembly_reference, predicted, full)
            original_overlaps = {tuple(sorted((row['shape_a'], row['shape_b']))): row
                                 for row in invariant_assembly_reference['existing_shape_overlaps']}
        contacts = {frozenset(pair) for pair in contact_pairs}
        coal = self.model.coal
        self._prepare_pair_cache()
        if not hasattr(self, '_relative_pair_indices'):
            # 同一祖先的刚体运动在相对变换中严格相消；只用于predict生成的FK位姿。
            ancestors = {self.model.root: set()}
            for joint in self.model.joints:
                ancestors[joint['body1']] = ancestors[joint['body0']] | (
                    {joint['index']} if joint['index'] is not None else set())
            self._relative_pair_indices = {
                k:np.asarray(sorted(ancestors[record[4]] ^ ancestors[record[5]]),dtype=int)
                for k,record in enumerate(self._pair_records)
                if record[4] in ancestors and record[5] in ancestors}
        override_records, override_indices = [], []
        seen_overrides = set()
        for source in clearance_overrides:
            bodies = source.get('bodies', [])
            margin = float(source.get('margin_m', float('nan')))
            reason = source.get('reason')
            if (len(bodies) != 2 or bodies[0] == bodies[1] or not math.isfinite(margin)
                    or margin < 0 or not isinstance(reason, str) or not reason.strip()):
                raise ValueError('净空覆盖必须有两个明确刚体、有限非负margin_m与理由')
            pair = frozenset(bodies)
            indices = self._indices_by_body_pair.get(pair, [])
            if pair in seen_overrides or pair in contacts or not indices:
                raise ValueError('净空覆盖重复、与接触许可冲突或不属于当前实际碰撞对')
            if any(self._pair_records[index][7] for index in indices):
                raise ValueError('工具净空覆盖不允许改变机器人自碰撞阈值')
            seen_overrides.add(pair)
            override_indices.extend(indices)
            override_records.append({'bodies': list(bodies), 'margin_m': margin, 'reason': reason,
                                     'indices': indices})
        matrices = np.asarray([predicted[body] if body else np.eye(4) for body in self._shape_bodies])
        if not np.isfinite(matrices).all():
            raise RuntimeError('候选碰撞体位姿非有限')
        corners_world = self._shape_corners @ matrices[:, :3, :3].transpose(0, 2, 1) + matrices[:, None, :3, 3]
        low, high = corners_world.min(1), corners_world.max(1)
        transforms = {}

        def transform(index):
            if index not in transforms:
                t = matrices[index]
                transforms[index] = coal.Transform3s(t[:3, :3], t[:3, 3])
            return transforms[index]
        i, j = self._original_pairs.T
        lower = np.linalg.norm(np.maximum(np.maximum(low[i] - high[j], low[j] - high[i]), 0.), axis=1)
        nearest = {'environment': None, 'self': None, 'intentional_contact': None}
        violations = []
        existing_invariant = []
        calls, distance_cache_hits = 0, 0

        def inspect(index, kind, margin):
            nonlocal calls, distance_cache_hits
            ia, ib, a, b, ba, bb, pair, _, _ = self._pair_records[index]
            # 必须是本次运行的不变几何、当前完整世界矩阵逐字节相同，才复用精确距离。
            key = (matrices[ia].tobytes(), matrices[ib].tobytes(), id(a['geometry']), id(b['geometry']))
            if _robot_fk_poses and kind == 'self' and index in self._relative_pair_indices:
                key = ('relative_fk',full[self._relative_pair_indices[index]].tobytes(),id(a['geometry']),id(b['geometry']))
            cached = distance_cache.get(index) if reader is not None else None
            if cached is not None and cached[0] == key:
                distance = cached[1];distance_cache_hits += 1
            else:
                result = coal.DistanceResult()
                distance = float(coal.distance(a['geometry'], transform(ia), b['geometry'], transform(ib), self.model.request, result))
                calls += 1
                if reader is not None:distance_cache[index] = (key, distance)
            if not math.isfinite(distance):
                raise RuntimeError('精确碰撞距离无效：' + a['path'])
            row = {'distance_m': distance, 'required_margin_m': margin, 'shape_a': a['path'], 'shape_b': b['path'],
                   'body_a': ba, 'body_b': bb, 'self': kind == 'self', 'kind': kind}
            best = nearest[kind]
            if best is None or distance < best['distance_m']:
                nearest[kind] = row
            if distance < margin:
                original = original_overlaps.get(tuple(sorted((a['path'], b['path']))))
                if (kind == 'self' and original is not None and not reference_issues
                        and pair == frozenset((original['body_a'], original['body_b']))
                        and distance >= original['distance_m'] - .0002):
                    existing_invariant.append({**row, 'reference_distance_m': original['distance_m'],
                                                'classification': 'existing_invariant_assembly_overlap'})
                else:
                    violations.append(row)
            return row

        # 显式工具净空每次都给出精确测距；许可仅是正净空buffer，绝非穿透豁免。
        override_measurements = []
        for record in override_records:
            rows = [inspect(index, 'environment', record['margin_m']) for index in record['indices']]
            best = min(rows, key=lambda row: row['distance_m'])
            override_measurements.append({k: v for k, v in record.items() if k != 'indices'} |
                                         {'minimum_distance_m': best['distance_m'], 'nearest_shape_pair': best,
                                          'pass': best['distance_m'] >= record['margin_m'],
                                          'permits_penetration': False})
        contact_indices = np.asarray(sorted({index for pair in contacts
                                             for index in self._indices_by_body_pair.get(pair, [])}), dtype=int)
        is_contact = np.zeros(len(self._original_pairs), dtype=bool)
        is_contact[contact_indices] = True
        is_override = np.zeros(len(self._original_pairs), dtype=bool)
        is_override[override_indices] = True
        groups = [('self', self._self_pair_indices[~is_contact[self._self_pair_indices]], max(self_margin_m, .001)),
                  ('environment', self._environment_pair_indices[~(is_contact | is_override)[self._environment_pair_indices]], environment_margin_m),
                  ('intentional_contact', contact_indices, -contact_penetration_m)]
        # 每类按AABB距离下界排序；超过该类所有许可阈值与已知最近距离后整组退出。
        # 所有潜在违例仍逐对精确查询，不能把AABB重叠直接当碰撞。
        for kind, indices, group_margin in groups:
            for index in indices[np.argsort(lower[indices])]:
                best = nearest[kind]
                if best is not None and lower[index] > max(group_margin, best['distance_m'], 0.):
                    break
                normal_margin = self._pair_records[index][8]
                margin = (self_margin_m if normal_margin >= .005 else normal_margin) if kind == 'self' else group_margin
                if best is not None and lower[index] > max(margin, best['distance_m'], 0.):
                    continue
                inspect(index, kind, margin)
        # CollisionModel跳过Plane，因此单独保留有限机器人形状对无限平面的保守检查。
        for plane in self._planes:
            t = predicted.get(plane['body'], np.eye(4)) @ np.asarray(plane['matrix'])
            normal = np.linalg.inv(t[:3, :3]).T[:, 'XYZ'.index(plane.get('axis', 'Z'))]
            normal /= np.linalg.norm(normal)
            indices = [i for i,s in enumerate(self.model.shapes) if s['robot'] and
                       frozenset((s['body'] or s['path'],plane['path'])) not in self._support]
            if not indices:continue
            distances = ((corners_world[indices]-t[:3,3]) @ normal).min(axis=1)
            nearest_index = int(np.argmin(distances))
            needed = set(np.flatnonzero(distances < environment_margin_m).tolist())
            needed.add(nearest_index)
            for k in sorted(needed):
                index = indices[k];shape = self.model.shapes[index];body = shape['body'] or shape['path']
                distance = float(distances[k])
                row = {'distance_m': distance, 'required_margin_m': environment_margin_m,
                       'shape_a': shape['path'], 'shape_b': plane['path'], 'body_a': body,
                       'body_b': plane['body'] or plane['path'], 'self': False, 'kind': 'environment',
                       'method': '保守形状角点对无限平面'}
                best = nearest['environment']
                if best is None or distance < best['distance_m']:
                    nearest['environment'] = row
                if distance < environment_margin_m:
                    violations.append(row)
        for issue in reference_issues:
            violations.append({'distance_m': None, 'kind': 'invariant_reference_violation', **issue})
        safe = not violations
        report = {'safe': safe, 'pass': safe, 'pass_for_requested_motion': safe,
                  'globally_collision_free': (False if existing_invariant or reference_issues else
                                              not any(h['distance_m'] is not None and h['distance_m'] < 0 for h in violations)),
                  'existing_invariant_assembly_overlaps': existing_invariant,
                  'invariant_assembly_reference': invariant_assembly_reference,
                  'clearance_override_measurements': override_measurements,
                  'minimum_environment_clearance_m': nearest['environment']['distance_m'] if nearest['environment'] else None,
                  'minimum_self_clearance_m': nearest['self']['distance_m'] if nearest['self'] else None,
                  'nearest': nearest, 'violations': sorted(violations, key=lambda h: (h['distance_m'] is not None, h['distance_m'] or 0.)),
                  'coal_distance_calls': calls, 'exact_pose_distance_cache_hits': distance_cache_hits,
                  'coverage': self.capture_counts, 'fk_error': self.fk_error,
                  'capture_time': self.model.data.get('capture_time'), 'actual_physics_pose': measured,
                  'explicit_ground_support_pairs': self.ground_support_pairs,
                  'method': 'fresh_all_robot_environment_and_self_coal_3d', 'stage_modified': False}
        self.last_report = report
        if return_surface_points:
            # 扫掠已需要同一批角点，避免再逐形状做一次矩阵乘法。
            report['_surface_points'] = corners_world
        return report

    def check_path(self, configurations, root_poses=None, *, environment_poses=None,
                   surface_step_m=.002, joint_step_rad=.01,
                   max_depth=16, **kwargs):
        """完整关节/根位姿路径自适应扫掠，含不运动手臂；不只检查路点端点。"""
        qs = [self.full_q(q=q) for q in configurations]
        if not qs:
            raise ValueError('轨迹不能为空')
        roots = ([np.array(self.poses[self.model.root], copy=True) for _ in qs] if root_poses is None
                 else [np.asarray(t, float) for t in root_poses])
        if len(roots) != len(qs):
            raise ValueError('底座路径与关节路径长度不一致')
        envs = ([{} for _ in qs] if environment_poses is None else
                [{p: np.asarray(t, float) for p, t in row.items()} for row in environment_poses])
        if len(envs) != len(qs):
            raise ValueError('环境刚体路径与关节路径长度不一致')
        moving_environment = set(envs[0])
        moving_shape_mask = np.array([s['robot'] or s['body'] in moving_environment for s in self.model.shapes])
        for row in envs:
            if set(row) != moving_environment:
                raise ValueError('每个路点必须给出同一组运动环境刚体')
            for path, matrix in row.items():
                if path not in self.poses or path in self.model.cluster or path.startswith('__static__:'):
                    raise ValueError('环境轨迹必须指向已采样的实际外部刚体：' + path)
                if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
                        or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8)
                        or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
                        or np.linalg.det(matrix[:3, :3]) < 0):
                    raise ValueError('运动环境轨迹必须是有限刚体变换')
        if surface_step_m <= 0 or joint_step_rad <= 0:
            raise ValueError('扫掠采样步长必须为正')
        count, minimum_environment, minimum_self = 0, float('inf'), float('inf')
        worst = None
        max_surface_step = 0.
        invariant_sample_count = 0
        max_existing_overlap_count = 0
        max_existing_overlap_worsening = 0.
        evaluations, cache_hits = {}, 0
        override_minima = {}

        def interpolate(a, b, u):
            root = np.eye(4)
            delta = Rotation.from_matrix(b[:3, :3] @ a[:3, :3].T).as_rotvec()
            root[:3, :3] = Rotation.from_rotvec(delta * u).as_matrix() @ a[:3, :3]
            root[:3, 3] = a[:3, 3] * (1-u) + b[:3, 3] * u
            return root

        def evaluate(q, root, environment):
            nonlocal count, minimum_environment, minimum_self, worst
            nonlocal invariant_sample_count, max_existing_overlap_count, max_existing_overlap_worsening
            nonlocal cache_hits
            # 缓存严格限于本次纯几何扫掠；不同捕获/运行帧不会复用旧距离。
            cache_key = (q.tobytes(), root.tobytes(),
                   tuple((p, environment[p].tobytes()) for p in sorted(environment)))
            if cache_key in evaluations:
                cache_hits += 1
                return evaluations[cache_key]
            predicted = self.predict(q, root)
            predicted.update(environment)
            report = self.check_configuration(q=q, root_pose=root, poses=predicted,
                                              return_surface_points=True, _robot_fk_poses=True, **kwargs)
            points = report.pop('_surface_points')[moving_shape_mask].reshape(-1, 3)
            count += 1
            for row in report['clearance_override_measurements']:
                pair = tuple(sorted(row['bodies']))
                if pair not in override_minima or row['minimum_distance_m'] < override_minima[pair]['minimum_distance_m']:
                    override_minima[pair] = row
            existing = report['existing_invariant_assembly_overlaps']
            if kwargs.get('invariant_assembly_reference') is not None:
                invariant_sample_count += 1
                max_existing_overlap_count = max(max_existing_overlap_count, len(existing))
                max_existing_overlap_worsening = max(max_existing_overlap_worsening,
                    max((v['reference_distance_m'] - v['distance_m'] for v in existing), default=0.))
            for key, value in (('environment', report['minimum_environment_clearance_m']),
                               ('self', report['minimum_self_clearance_m'])):
                if value is not None:
                    if key == 'environment': minimum_environment = min(minimum_environment, value)
                    else: minimum_self = min(minimum_self, value)
            if not report['safe']:
                worst = report
            evaluations[cache_key] = (report, points)
            return evaluations[cache_key]

        def visit(a, b, ra, rb, ea, eb, depth=0):
            nonlocal max_surface_step, worst
            mid, rm = (a+b)/2, interpolate(ra, rb, .5)
            em = {path: interpolate(ea[path], eb[path], .5) for path in moving_environment}
            va, vm, vb = evaluate(a, ra, ea), evaluate(mid, rm, em), evaluate(b, rb, eb)
            if not all(v[0]['safe'] for v in (va, vm, vb)):
                return False
            movement = max(float(np.linalg.norm(va[1] - vm[1], axis=1).max()),
                           float(np.linalg.norm(vb[1] - vm[1], axis=1).max()))
            if movement > surface_step_m or np.abs(b-a).max() > joint_step_rad:
                if depth >= max_depth:
                    worst = {'safe': False, 'reason': '轨迹扫掠采样未收敛'}
                    return False
                return (visit(a, mid, ra, rm, ea, em, depth+1)
                        and visit(mid, b, rm, rb, em, eb, depth+1))
            max_surface_step = max(max_surface_step, movement)
            return True

        safe = evaluate(qs[0], roots[0], envs[0])[0]['safe']
        if safe:
            safe = all(visit(a, b, ra, rb, ea, eb) for a, b, ra, rb, ea, eb
                       in zip(qs, qs[1:], roots, roots[1:], envs, envs[1:]))
        return {'safe': safe, 'pass': safe, 'configuration_samples': count,
                'sample_count_basis': '本次扫掠内唯一完整q/root/环境位姿；未跨调用缓存',
                'reused_endpoint_evaluations': cache_hits,
                'clearance_override_measurements': list(override_minima.values()),
                'pass_for_requested_motion': safe,
                'globally_collision_free': False if (kwargs.get('invariant_assembly_reference') or {}).get('existing_shape_overlaps') else safe,
                'invariant_assembly_reference': kwargs.get('invariant_assembly_reference'),
                'invariant_reference_checked_samples': invariant_sample_count,
                'max_existing_assembly_overlap_shape_pairs': max_existing_overlap_count,
                'max_existing_assembly_overlap_worsening_m': max_existing_overlap_worsening,
                'environment_sweep_included': bool(moving_environment),
                'moving_environment_bodies': sorted(moving_environment),
                'minimum_environment_clearance_m': minimum_environment if math.isfinite(minimum_environment) else None,
                'minimum_self_clearance_m': minimum_self if math.isfinite(minimum_self) else None,
                'max_surface_step_m': max_surface_step, 'coverage': self.capture_counts,
                'failure': worst, 'method': '全身真实3D形状自适应扫掠；未写仿真', 'stage_modified': False}
