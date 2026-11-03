"""阀门与人形机器人同一物理步的只读反馈，不创建传感器或修改 USD。

只能在已启动的 RouteManager 仿真主线程调用 ``capture(executor, valve_root)``。
导入本模块不会加载 Isaac；所有 omni/pxr 导入均延迟到实际采样函数中。
"""
import math
import time
import hashlib
import json
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

_SUPPORT_VERTEX_CACHE = {}


def _support_vertices(points, stage_identifier, wheel_path):
    """线性支撑宽只需同一真实网格凸包顶点；保留几何精度，不把2r当夹持宽。"""
    from scipy.spatial import ConvexHull
    # 仅cache键去除USD局部变换相消产生的pm级抖动；返回的仍是原始支持点。
    key = (stage_identifier, wheel_path, hashlib.sha256(np.round(points, 12).tobytes()).hexdigest())
    if key not in _SUPPORT_VERTEX_CACHE:
        _SUPPORT_VERTEX_CACHE[key] = points[ConvexHull(points).vertices].copy()
        if len(_SUPPORT_VERTEX_CACHE) > 16:
            del _SUPPORT_VERTEX_CACHE[next(iter(_SUPPORT_VERTEX_CACHE))]
    return _SUPPORT_VERTEX_CACHE[key]


def wheel_support_interval(geometry, jaw_axis_world, *, allow_legacy_fallback=False):
    """同一真实mesh在世界jaw轴上的[min,max]；动态变换只用native wheel_pose。"""
    axis = _vector(jaw_axis_world, 3, '夹爪闭合轴').copy()
    norm = float(np.linalg.norm(axis))
    if norm < 1e-9:
        raise ValueError('夹爪闭合轴不能为零')
    axis /= norm
    if 'wheel_pose' in geometry:
        R = _rotation(geometry['wheel_pose'])
    else:
        # 历史快照只保存world-Z/yaw；只能用于下方明确轴对齐的bbox回退。
        R = Rotation.from_euler('z', float(geometry['yaw_deg']), degrees=True).as_matrix()
    local = R.T @ axis
    if 'support_vertices_local_m' in geometry:
        points = np.asarray(geometry.get('grasp_support_vertices_local_m',geometry['support_vertices_local_m']), float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 4 or not np.isfinite(points).all():
            raise ValueError('手轮支撑顶点无效')
        origin = _vector(geometry['wheel_pose']['position'], 3, 'native手轮位置')
        projected = points @ local + float(axis @ origin)
        lo, hi = float(projected.min()), float(projected.max())
        source = ('explicit_local_contact_region_of_full_mesh_hull' if 'grasp_region' in geometry else
                  'exact_same_mesh_convex_support_with_native_wheel_pose')
    else:
        if not allow_legacy_fallback:
            raise ValueError('缺少真实支撑顶点，实际运行禁止用radius/bbox猜夹宽')
        if abs(local).max() < 1 - 1e-6:
            raise ValueError('历史快照缺支撑顶点且夹持轴不对齐；请重新capture')
        width = float(np.abs(local) @ _vector(geometry['size_mm'], 3, '历史mesh bbox尺寸')) / 1000.
        mid = float(axis @ _vector(geometry['center'], 3, '历史mesh中心'))
        lo, hi = mid - width/2, mid + width/2
        source = 'explicit_offline_legacy_axis_aligned_bbox_reference_only'
    return {'minimum_world_projection_m': lo, 'maximum_world_projection_m': hi,
            'midpoint_world_projection_m': .5 * (lo + hi), 'width_mm': (hi - lo) * 1000.,
            'jaw_axis_world': axis.tolist(), 'source': source,
            'legacy_reference_only': source.startswith('explicit_offline')}


def projected_wheel_width_mm(geometry, jaw_axis_world, *, allow_legacy_fallback=False):
    """radius是外接半径，实际夹持宽由支撑投影给出；二者不混用。"""
    return wheel_support_interval(geometry, jaw_axis_world, allow_legacy_fallback=allow_legacy_fallback)['width_mm']


def _vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise RuntimeError(name + ' 反馈无效')
    return result


def _quaternion(value, name):
    result = _vector(value, 4, name)
    norm = float(np.linalg.norm(result))
    if norm < 1e-9:
        raise RuntimeError(name + ' 为零四元数')
    return result / norm


def _pose(position, quaternion_wxyz):
    p = _vector(position, 3, '刚体位置')
    q = _quaternion(quaternion_wxyz, '刚体 wxyz 四元数')
    return {'position': p.tolist(), 'orientation': q.tolist()}


def _rotation(pose):
    return Rotation.from_quat(pose['orientation'], scalar_first=True).as_matrix()


def _rigid_pose(interface, path):
    value = interface.get_rigidbody_transformation(path)
    if not value or not value.get('ret_val', False):
        raise RuntimeError('PhysX 刚体反馈不可用: ' + path)
    q = _quaternion(value['rotation'], path + ' xyzw 四元数')
    return _pose(value['position'], q[[3, 0, 1, 2]])


def _joint_quat(value):
    if value is None:
        return np.eye(3)
    q = [float(value.GetReal()), *[float(v) for v in value.GetImaginary()]]
    return Rotation.from_quat(_quaternion(q, '关节局部四元数'), scalar_first=True).as_matrix()


def _scalar(value):
    if value is None:
        return None
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return str(value)


def _scale(cache, prim):
    # 动态位姿从 PhysX 读；USD 只用于没有物理动态的固有缩放，不读其世界平移。
    matrix = np.asarray(cache.GetLocalToWorldTransform(prim), dtype=float).T
    scale = np.linalg.norm(matrix[:3, :3], axis=0)
    if not np.isfinite(scale).all() or np.any(scale < 1e-10):
        raise RuntimeError('刚体 authored 缩放无效: ' + str(prim.GetPath()))
    return scale


def _mesh_vertices_in_body(mesh, body):
    from pxr import Gf, UsdGeom
    points = np.asarray(UsdGeom.Mesh(mesh).GetPointsAttr().Get(), dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
        raise RuntimeError('手轮 mesh 没有有效顶点: ' + str(mesh.GetPath()))
    if not np.isfinite(points).all():
        raise RuntimeError('手轮 mesh 顶点含 NaN/无穷值')
    transform = Gf.Matrix4d(1)
    current = mesh
    # 只连乘 mesh 到刚体的局部变换，不用曾返回局部原点的世界平移缓存。
    while current and current != body:
        xf = UsdGeom.Xformable(current)
        if xf:
            if xf.GetResetXformStack():
                raise RuntimeError('手轮 mesh 到刚体之间存在 resetXformStack，不能直接量测')
            local = xf.GetLocalTransformation()
            if isinstance(local, tuple):
                local = local[0]
            transform = transform * local
        current = current.GetParent()
    if current != body:
        raise RuntimeError('手轮 mesh 不在 wheel 刚体层级内')
    matrix = np.asarray(transform, dtype=float).T
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def _joint_info(stage, joint_prim, interface, cache):
    from pxr import UsdPhysics
    joint = UsdPhysics.Joint(joint_prim)
    paths0, paths1 = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
    if len(paths1) != 1 or len(paths0) > 1:
        raise RuntimeError('阀门轮轴 body0/body1 关系无效')
    body0, body1 = (str(paths0[0]) if paths0 else None), str(paths1[0])
    pose0 = _rigid_pose(interface, body0) if body0 else _pose([0, 0, 0], [1, 0, 0, 0])
    pose1 = _rigid_pose(interface, body1)
    scale0 = _scale(cache, stage.GetPrimAtPath(body0)) if body0 else np.ones(3)
    scale1 = _scale(cache, stage.GetPrimAtPath(body1))
    R0, R1 = _rotation(pose0), _rotation(pose1)
    local_R0, local_R1 = _joint_quat(joint.GetLocalRot0Attr().Get()), _joint_quat(joint.GetLocalRot1Attr().Get())
    local_p0 = _vector(joint.GetLocalPos0Attr().Get(), 3, '轮轴 localPos0')
    local_p1 = _vector(joint.GetLocalPos1Attr().Get(), 3, '轮轴 localPos1')
    axis_token = str(joint_prim.GetAttribute('physics:axis').Get())
    if axis_token not in ('X', 'Y', 'Z'):
        raise RuntimeError('轮轴轴向应为 X/Y/Z')
    axis = np.eye(3)['XYZ'.index(axis_token)]
    axis0 = R0 @ (scale0 * (local_R0 @ axis))
    axis1 = R1 @ (scale1 * (local_R1 @ axis))
    axis0 /= np.linalg.norm(axis0)
    axis1 /= np.linalg.norm(axis1)
    anchor0 = np.asarray(pose0['position']) + R0 @ (scale0 * local_p0)
    anchor1 = np.asarray(pose1['position']) + R1 @ (scale1 * local_p1)
    joint_R0, joint_R1 = R0 @ local_R0, R1 @ local_R1
    relative = joint_R0.T @ joint_R1
    quat = Rotation.from_matrix(relative).as_quat()
    # 取约束轴上的 twist；保持带符号角，跨 ±180° 的展开由执行状态机保存历史。
    along = float(np.dot(quat[:3], axis))
    angle = math.degrees(2 * math.atan2(along, float(quat[3])))
    wrapped_angle = (angle + 180.) % 360. - 180.
    pure_twist = Rotation.from_rotvec(axis * math.radians(wrapped_angle)).as_matrix()
    swing = math.degrees(float(np.linalg.norm(Rotation.from_matrix(relative @ pure_twist.T).as_rotvec())))
    attrs = {}
    for attr in joint_prim.GetAttributes():
        name = attr.GetName()
        if name.startswith(('drive:', 'physxJoint:')) or name in (
                'physics:lowerLimit', 'physics:upperLimit', 'physics:jointEnabled',
                'physics:breakForce', 'physics:breakTorque'):
            attrs[name] = _scalar(attr.Get())
    drive_schemas = [str(v) for v in joint_prim.GetAppliedSchemas() if 'DriveAPI' in str(v)]
    info = {
        'path': str(joint_prim.GetPath()), 'body0': body0, 'body1': body1,
        'axis_token': axis_token, 'axis_world': axis1.tolist(),
        'axis_point_world': anchor1.tolist(),
        'anchor0_world': anchor0.tolist(), 'anchor1_world': anchor1.tolist(),
        'anchor_gap_mm': float(np.linalg.norm(anchor1 - anchor0) * 1000),
        'axis_alignment_deg': math.degrees(math.acos(float(np.clip(axis0 @ axis1, -1, 1)))),
        'angle_deg_wrapped': wrapped_angle, 'off_axis_swing_deg': swing,
        'angle_method': 'twist of physical body0/body1 joint frames; runtime must unwrap deltas',
        'local_pos0': local_p0.tolist(), 'local_pos1': local_p1.tolist(),
        'authored_body0_scale': scale0.tolist(), 'authored_body1_scale': scale1.tolist(),
        'drive_schemas': drive_schemas, 'attributes': attrs,
    }
    return info, pose1, scale1


def _materials(stage, mesh_prims):
    from pxr import UsdShade
    result = []
    seen = set()
    for mesh in mesh_prims:
        binding = UsdShade.MaterialBindingAPI(mesh)
        for purpose in ('physics', ''):
            try:
                material, _ = binding.ComputeBoundMaterial(purpose)
                if not material or str(material.GetPath()) in seen:
                    continue
                path = str(material.GetPath())
                seen.add(path)
                prim = stage.GetPrimAtPath(path)
                result.append({'path': path, 'purpose': purpose or 'allPurpose',
                    'static_friction': _scalar(prim.GetAttribute('physics:staticFriction').Get()),
                    'dynamic_friction': _scalar(prim.GetAttribute('physics:dynamicFriction').Get()),
                    'restitution': _scalar(prim.GetAttribute('physics:restitution').Get())})
            except Exception as exc:
                result.append({'mesh': str(mesh.GetPath()), 'purpose': purpose,
                               'metadata_error': str(exc)})
    return result


def _normal_contact_sample(sample):
    """安装版6.2 ContactData的世界法线/冲量/分离量；不把patch存在当成载荷。"""
    raw = {}
    for name in ('position', 'normal', 'impulse', 'separation'):
        try:
            raw[name] = getattr(sample, name)
        except Exception as error:
            raw[name] = None
            raw[name + '_read_error'] = str(error)
    try:
        normal = _vector(raw['normal'], 3, 'ContactData.normal')
        impulse = _vector(raw['impulse'], 3, 'ContactData.impulse')
        separation = float(raw['separation'])
        norm = math.hypot(*normal)
        if norm <= 0. or not math.isfinite(norm) or not math.isfinite(separation):
            raise ValueError('ContactData法线为零/非有限或separation非有限')
        unit = normal / norm
        magnitude = abs(float(np.dot(impulse, unit)))
        vector_magnitude = math.hypot(*impulse)
        if not math.isfinite(magnitude) or not math.isfinite(vector_magnitude):
            raise ValueError('ContactData冲量投影/幅值非有限')
        position = None
        if raw['position'] is not None:
            value = np.asarray(raw['position'], float)
            if value.shape == (3,) and np.isfinite(value).all():
                position = value.tolist()
        return {'load_verifiable': True, 'position_world': position,
                'normal_world': normal.tolist(), 'unit_normal_world': unit.tolist(),
                'separation_m': separation, 'impulse_Ns': impulse.tolist(),
                'normal_impulse_magnitude_Ns': magnitude,
                'vector_impulse_magnitude_Ns': vector_magnitude}
    except Exception as error:
        # 保留不可读来源，不把NaN写成JSON数值或回退为“有力”。
        return {'load_verifiable': False, 'read_error': str(error),
                'raw_unverifiable_source': {name: repr(value) for name, value in raw.items()}}


def _contacts(stage, wheel_path, robot_root, report_configuration=None):
    import omni.physx
    from pxr import PhysicsSchemaTools, PhysxSchema
    if report_configuration is None:
        wheel = stage.GetPrimAtPath(wheel_path)
        wheel_reports = bool(wheel.HasAPI(PhysxSchema.PhysxContactReportAPI))
        pad_reports = {}
        for prim in stage.Traverse():
            name = prim.GetName()
            if str(prim.GetPath()).startswith(robot_root + '/') and name in (
                    'right_gripper_left_pad_link', 'right_gripper_right_pad_link'):
                pad_reports[name] = bool(prim.HasAPI(PhysxSchema.PhysxContactReportAPI))
    else:
        # 仅由带USD变更失效保护的FeedbackReader传入；接触事件和冲量仍每步新读。
        wheel_reports = report_configuration['wheel_report_api']
        pad_reports = report_configuration['pad_report_apis']
    available = wheel_reports or (len(pad_reports) == 2 and all(pad_reports.values()))
    result = {'available': available, 'wheel_report_api': wheel_reports,
              'pad_report_apis': pad_reports, 'sample_scope': 'current physics step only',
              'read_succeeded': False, 'load_verifiable': False,
              'left_pad_touching': False, 'right_pad_touching': False,
              'left_pad_normal_impulse_Ns': 0., 'right_pad_normal_impulse_Ns': 0.,
              'left_pad_load_bearing': False, 'right_pad_load_bearing': False,
              'unexpected_robot_normal_impulse_Ns': 0., 'unexpected_robot_load_bearing': False,
              'contact_count': 0, 'impulse_sum_Ns': 0., 'events': [],
              'load_measurement_source': {
                  'normal': 'omni.physx.bindings.ContactData.normal; world-space vector',
                  'impulse': 'omni.physx.bindings.ContactData.impulse; N*s vector',
                  'separation': 'omni.physx.bindings.ContactData.separation; meter',
                  'criterion': 'finite abs(dot(impulse, normalized normal)) > 0; no guessed force threshold'},
              'note': '未创建 ContactSensor 或 Apply ContactReportAPI；缺报告时空事件不是无接触证据'}
    if not available:
        result['absence_reason'] = 'contact_report_api_not_configured'
        return result
    try:
        buffer = omni.physx.get_physx_simulation_interface().get_contact_report()
        if isinstance(buffer, dict):
            headers, data = buffer['contact_headers'], buffer['contact_data']
        else:
            headers, data = buffer[0], buffer[1]
        errors = []
        for header in headers:
            actors = [str(PhysicsSchemaTools.intToSdfPath(header.actor0)),
                      str(PhysicsSchemaTools.intToSdfPath(header.actor1))]
            if wheel_path not in actors or not any(p.startswith(robot_root + '/') for p in actors):
                continue
            count = int(header.num_contact_data)
            if count == 0:
                continue
            pad_sides = [side for side in ('left', 'right')
                         if any(p.endswith('/right_gripper_' + side + '_pad_link') for p in actors)]
            for side in pad_sides:
                result[side + '_pad_touching'] = True
            event = {'actors': actors, 'num_contacts': count, 'pad_sides': pad_sides,
                     'samples': [], 'normal_impulse_magnitude_Ns': 0.}
            try:
                event['colliders'] = [str(PhysicsSchemaTools.intToSdfPath(header.collider0)),
                                      str(PhysicsSchemaTools.intToSdfPath(header.collider1))]
            except Exception:
                event['colliders'] = None
            for i in range(int(header.contact_data_offset), int(header.contact_data_offset) + count):
                sample = _normal_contact_sample(data[i])
                if sample['load_verifiable']:
                    magnitude = sample['normal_impulse_magnitude_Ns']
                    result['impulse_sum_Ns'] += sample['vector_impulse_magnitude_Ns']
                    event['normal_impulse_magnitude_Ns'] += magnitude
                    if pad_sides:
                        for side in pad_sides:
                            result[side + '_pad_normal_impulse_Ns'] += magnitude
                    else:
                        result['unexpected_robot_normal_impulse_Ns'] += magnitude
                elif len(errors) < 16:
                    errors.append(sample['read_error'])
                if len(event['samples']) < 8:
                    event['samples'].append(sample)
            result['contact_count'] += count
            if len(result['events']) < 32:
                result['events'].append(event)
        if errors:
            result['available'] = False
            result['read_error'] = '; '.join(errors)
        else:
            result['read_succeeded'] = True
            result['load_verifiable'] = result['contact_count'] > 0
            if not result['load_verifiable']:
                result['absence_reason'] = 'no_current_wheel_robot_contact_data'
                result['load_verification_note'] = '当前物理步读取成功但无接触样本，法线/冲量承载尚不可验证'
            else:
                for side in ('left', 'right'):
                    result[side + '_pad_load_bearing'] = result[side + '_pad_normal_impulse_Ns'] > 0.
                result['unexpected_robot_load_bearing'] = result['unexpected_robot_normal_impulse_Ns'] > 0.
    except Exception as exc:
        result['available'] = False
        result['read_error'] = str(exc)
    return result


def _resolve_fixed_graph(base_path, joints, kinematic_paths):
    """只沿启用的固定约束寻找 world/运动学基座，不能把任意 fixedJoint 当固定。"""
    world = '__physics_world__'
    edges = {}
    for joint in joints:
        if not joint['enabled']:
            continue
        bodies = joint['bodies']
        if not 1 <= len(bodies) <= 2:
            continue
        a, b = (world, bodies[0]) if len(bodies) == 1 else bodies
        edges.setdefault(a, []).append((b, joint['path']))
        edges.setdefault(b, []).append((a, joint['path']))
    queue = deque([(base_path, [], [base_path])])
    visited = set()
    while queue:
        node, joint_chain, body_chain = queue.popleft()
        if node in visited:
            continue
        visited.add(node)
        if node == world or node in kinematic_paths:
            return {'type': 'world' if node == world else 'kinematic_body',
                    'body': None if node == world else node,
                    'joint_chain': joint_chain, 'body_chain': body_chain,
                    'proof': 'enabled FixedJoint graph reaches ' +
                             ('world' if node == world else 'RigidBodyAPI kinematicEnabled=true')}
        for next_node, joint_path in edges.get(node, []):
            queue.append((next_node, joint_chain + [joint_path], body_chain + [next_node]))
    return None


def _fixed_anchor(stage, base_path, valve_root, interface):
    from pxr import Usd, UsdPhysics
    joints, kinematic_paths = [], set()
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if not prim.IsActive():
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            body = UsdPhysics.RigidBodyAPI(prim)
            if body.GetKinematicEnabledAttr().Get() is True:
                kinematic_paths.add(str(prim.GetPath()))
        if not prim.IsA(UsdPhysics.FixedJoint):
            continue
        joint = UsdPhysics.Joint(prim)
        b0, b1 = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
        valid_relations = len(b0) <= 1 and len(b1) <= 1
        bodies = [str(v) for v in [*b0, *b1]]
        enabled = valid_relations and joint.GetJointEnabledAttr().Get() is not False
        joints.append({'path': str(prim.GetPath()), 'bodies': bodies, 'enabled': enabled,
                       'connected_to_world': enabled and len(bodies) == 1})
    proof = _resolve_fixed_graph(base_path, joints, kinematic_paths)
    if proof is not None:
        if proof['type'] == 'kinematic_body':
            # Kinematic 可以被别的任务移动。保留同帧真实锚点，运行时应逐帧拒绝漂移。
            proof['pose'] = _rigid_pose(interface, proof['body'])
            proof['stability'] = 'current physical pose only; runtime must reject anchor drift'
        else:
            proof['pose'] = _pose([0, 0, 0], [1, 0, 0, 0])
            proof['stability'] = 'world frame'
    local = [j for j in joints if j['path'].startswith(valve_root + '/')]
    return proof, local


def capture(executor, valve_root='/World/vla_table/valve_asset/valve_1'):
    """返回同物理步的 JSON snapshot；调用本函数不会推进物理或创建/修改 prim。"""
    import omni.usd
    import omni.physx
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(valve_root)
    if not root or not root.IsActive():
        raise RuntimeError('阀门路径不存在或未激活: ' + valve_root)
    joints = [p for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
              if p.IsA(UsdPhysics.RevoluteJoint)]
    if len(joints) != 1:
        raise RuntimeError(f'阀门应有唯一旋转关节，实际 {len(joints)} 个: {valve_root}')
    interface = omni.physx.get_physx_interface()
    cache = UsdGeom.XformCache()
    joint, wheel_pose, wheel_scale = _joint_info(stage, joints[0], interface, cache)
    wheel = stage.GetPrimAtPath(joint['body1'])
    meshes = [p for p in Usd.PrimRange(wheel, Usd.TraverseInstanceProxies())
              if p.IsA(UsdGeom.Mesh)]
    if not meshes:
        raise RuntimeError('手轮刚体下面没有 mesh: ' + joint['body1'])
    points = np.concatenate([_mesh_vertices_in_body(p, wheel) for p in meshes]) * wheel_scale
    support_meshes = [p for p in meshes if p.HasAPI(UsdPhysics.CollisionAPI)
                      and p.GetAttribute('physics:collisionEnabled').Get() is not False]
    if not support_meshes:
        raise RuntimeError('手轮缺少实际碰撞mesh，不能用可视外形猜夹持宽')
    support_points = np.concatenate([_mesh_vertices_in_body(p, wheel) for p in support_meshes]) * wheel_scale
    mn, mx = points.min(axis=0), points.max(axis=0)
    center_local = (mn + mx) * .5
    R = _rotation(wheel_pose)
    center = np.asarray(wheel_pose['position']) + R @ center_local
    # 半径投影到轮轴法平面，兼容轴不沿世界 Z 的阀门。
    axis_local = R.T @ np.asarray(joint['axis_world'])
    radial = points - center_local
    radial -= np.outer(radial @ axis_local, axis_local)
    radius = float(np.linalg.norm(radial, axis=1).max())
    if not math.isfinite(radius) or radius <= 0:
        raise RuntimeError('手轮网格半径无效')

    state = executor.get_sim_state(ref_link='', with_velocity=True, with_effort=True)
    table = executor._link_pose_table()
    if 'right_arm_base_link' not in table or 'base_link' not in table:
        raise RuntimeError('机器人实测臂座/底盘反馈缺失')
    # 使用同一 table 的全精度法兰/指垫；get_sim_state 的默认位姿会取六位小数。
    for side in ('right', 'left'):
        names = [f'{side}_gripper_base_link', f'{side}_gripper_left_pad_link', f'{side}_gripper_right_pad_link']
        if all(name in table for name in names):
            p, q = table[names[0]]
            entry = state['sides'][side]
            entry['flange'] = _pose(p, q)
            tcp = .5 * (np.asarray(table[names[1]][0]) + np.asarray(table[names[2]][0]))
            entry['tcp'] = _pose(tcp, q)
            entry['tcp_local'] = (_rotation(entry['flange']).T @ (tcp - p)).tolist()
            difference = np.asarray(table[names[2]][0]) - np.asarray(table[names[1]][0])
            norm = float(np.linalg.norm(difference))
            if not math.isfinite(norm) or norm < 1e-9:
                raise RuntimeError('实测双指垫不能定义闭合轴')
            entry['jaw_axis_world'] = (difference / norm).tolist()
            entry['pad_poses'] = {'left': _pose(*table[names[1]]), 'right': _pose(*table[names[2]])}
            jaw = difference / norm
            face_projections = []
            pad_sources = []
            for index, finger in ((1, 'left'), (2, 'right')):
                # tensor link path是真实刚体路径，不假设URDF导入层级平铺。
                paths = [path for row in executor.articulation._physics_view.link_paths for path in row
                         if str(path).rsplit('/', 1)[-1] == names[index]]
                if len(paths) != 1:
                    raise RuntimeError('无法唯一定位实际指垫mesh刚体')
                pad = stage.GetPrimAtPath(str(paths[0]))
                pad_meshes = [p for p in Usd.PrimRange(pad, Usd.TraverseInstanceProxies())
                              if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI)
                              and p.GetAttribute('physics:collisionEnabled').Get() is not False]
                if not pad_meshes:
                    raise RuntimeError('实际指垫缺少碰撞mesh，不能猜inner-face中点')
                pad_points = np.concatenate([_mesh_vertices_in_body(mesh, pad) for mesh in pad_meshes]) * _scale(cache, pad)
                pad_R = Rotation.from_quat(table[names[index]][1], scalar_first=True).as_matrix()
                scalar = pad_points @ (pad_R.T @ jaw) + float(jaw @ table[names[index]][0])
                face_projections.append(float(scalar.max() if finger == 'left' else scalar.min()))
                pad_sources.append({'body': str(paths[0]), 'finger': finger,
                                    'mesh_paths': [str(p.GetPath()) for p in pad_meshes],
                                    'points_sha256': hashlib.sha256(pad_points.tobytes()).hexdigest(), 'units': 'm',
                                    'coordinate_frame': 'pad rigid body local',
                                    'support_vertices_local_m': _support_vertices(pad_points, stage.GetRootLayer().identifier, str(paths[0])).tolist()})
            inner_mid = .5 * sum(face_projections)
            midpoint_delta = inner_mid - float(jaw @ tcp)
            entry['inner_face_midpoint_world'] = (tcp + jaw * midpoint_delta).tolist()
            entry['inner_face_gap_mm'] = (face_projections[1] - face_projections[0]) * 1000.
            entry['inner_face_midpoint_tcp_projection_delta_m'] = midpoint_delta
            entry['pad_support_sources'] = pad_sources

    fixed_anchor, fixed = _fixed_anchor(stage, joint['body0'], valve_root, interface)

    robot_root = str(getattr(executor, 'robot_prim_path', '/humanoid'))
    robot_bases = [str(path) for row in executor.articulation._physics_view.link_paths for path in row
                   if str(path).rsplit('/', 1)[-1] == 'base_link']
    if len(robot_bases) != 1:
        raise RuntimeError('无法唯一定位驻车底盘刚体')
    parking_anchor, _ = _fixed_anchor(stage, robot_bases[0], robot_root, interface)
    geometry = {'center': center.tolist(), 'center_local_m': center_local.tolist(),
                'link_world': wheel_pose['position'], 'radius_mm': radius * 1000,
                'size_mm': ((mx - mn) * 1000).tolist(),
                'yaw_deg': math.degrees(math.atan2(R[1, 0], R[0, 0])),
                'axis_world': joint['axis_world'], 'axis_point_world': joint['axis_point_world'],
                'axis_point': joint['axis_point_world'], 'wheel_pose': wheel_pose,
                'mesh_count': len(meshes), 'vertex_count': int(len(points))}
    geometry['support_vertices_local_m'] = _support_vertices(support_points, stage.GetRootLayer().identifier, joint['body1']).tolist()
    geometry['support_width_source'] = 'exact linear support of the same measured mesh convex hull, metres in wheel body frame'
    geometry['support_metadata'] = {'body_path': joint['body1'], 'mesh_paths': [str(p.GetPath()) for p in support_meshes],
                                    'source_points_sha256': hashlib.sha256(support_points.tobytes()).hexdigest(),
                                    'source_vertex_count': int(len(support_points)), 'support_vertex_count': len(geometry['support_vertices_local_m']),
                                    'coordinate_frame': 'wheel rigid body local, before native wheel pose', 'units': 'm'}
    region = root.GetCustomDataByKey('grasp_region')
    if region:
        from valve_grasp_region import apply_region
        geometry = apply_region(geometry,json.loads(region))
    geometry['grasp_support_width_mm'] = {
        side: projected_wheel_width_mm(geometry, state['sides'][side]['jaw_axis_world'])
        for side in ('left', 'right') if 'jaw_axis_world' in state['sides'][side]}
    return {'schema': 'valve_feedback/v1', 'source': 'current PhysX read-only sample',
            'capture_wall_time': time.time(), 'sim_time': float(state['sim_time']),
            'physics_dt': float(executor.world.get_physics_dt()),
            'valve_root': valve_root, 'robot_root': robot_root,
            'state': state, 'arm_base': _pose(*table['right_arm_base_link']),
            'base_pose': _pose(*table['base_link']), 'geometry': geometry,
            'robot_native_fixed_base': bool(executor.articulation._physics_view.shared_metatype.fixed_base),
            'robot_parking_anchor': parking_anchor,
            'wheel_joint': joint, 'wheel_angle_deg_wrapped': joint['angle_deg_wrapped'],
            # 保留旧字段；证明可能经启用的固定约束到达运动学夹具，来源须看 fixed_anchor。
            'base_fixed_to_world': fixed_anchor is not None, 'fixed_anchor': fixed_anchor,
            'fixed_joints': fixed,
            'materials': _materials(stage, meshes),
            'contacts': _contacts(stage, joint['body1'], robot_root),
            'notes': ['角度来自物理关节帧相对 twist，累计转角必须从样本间有符号差展开',
                      '接触冲量是 N·s，不能直接当成法向力、夹持力矩或阀门所需扭矩']}
