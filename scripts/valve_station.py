"""在现有场景添加可移除阀门验证工位；不重载机器人，不写回来源 USD。"""
import json
from pathlib import Path


def rebind_existing_views(executor):
    """Kit 删除任意 prim 会删掉旧 Articulation._physics_view；只重绑定，不 reset 状态。"""
    views = [executor.articulation]
    for attribute in ('_oxford_devices', '_l3_station'):
        owner = getattr(executor, attribute, None)
        if owner is not None:
            devices = getattr(owner, 'devices', None) or getattr(getattr(owner, 'runtime', None), 'devices', {})
            views.extend(devices.values())
    for view in views:
        if not hasattr(view, '_physics_view'):
            view._physics_view = None
        view.initialize()
        if not view.is_physics_handle_valid():
            raise RuntimeError('已有物理视图无法重新绑定，禁止 reset/重载机器人继续')


def setup(executor, position, output, source, root='/World/ValveStation'):
    """position 是阀体根的世界坐标；支架尺寸是仿真验证夹具，不代表真机规格。"""
    import numpy as np
    import omni.usd
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade
    from scipy.spatial.transform import Rotation
    stage = omni.usd.get_context().get_stage()
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    position = np.asarray(position, float)
    if position.shape != (3,) or not np.isfinite(position).all() or position[2] <= .113:
        raise ValueError('验证工位位置必须有限，且支架高度须为正')
    if getattr(executor, '_base_cmd', None) is not None:
        raise RuntimeError('底盘仍在运动，禁止添加工位')
    owner = getattr(executor, '_l3_station', None)
    approach = getattr(owner, '_approach', None)
    if owner is not None and (getattr(owner, '_robot_busy', False) or
                              getattr(owner, '_pending_action', None) is not None or
                              getattr(owner, 'queue', ()) or
                              (approach is not None and approach.busy)):
        raise RuntimeError('其他工位任务仍激活，禁止改变场景')
    if stage.GetPrimAtPath(root):
        raise RuntimeError('阀门工位已存在，先读取现有状态，禁止覆盖')
    q0 = np.asarray(executor.articulation.get_joint_positions()).copy()
    if not np.isfinite(q0).all() or getattr(executor, '_motion', None) is not None:
        raise RuntimeError('机器人状态非有限或仍有运动任务，禁止添加工位')
    source_stage = Usd.Stage.Open(str(source))
    source_root = source_stage.GetDefaultPrim()
    if not source_root:raise RuntimeError('来源USD没有defaultPrim')
    source_prim = str(source_root.GetPath())
    unit = float(UsdGeom.GetStageMetersPerUnit(source_stage))
    if not np.isfinite(unit) or unit <= 0:raise RuntimeError('来源单位无效')
    joints = [p for p in Usd.PrimRange(source_root,Usd.TraverseInstanceProxies()) if p.IsA(UsdPhysics.RevoluteJoint)]
    if len(joints)!=1:raise RuntimeError('来源必须有唯一被动转轴')
    source_joint = UsdPhysics.Joint(joints[0])
    if any('DriveAPI' in str(x) for x in joints[0].GetAppliedSchemas()):
        raise RuntimeError('来源轮轴有驱动，禁止作为被动阀门操作')
    b0,b1 = source_joint.GetBody0Rel().GetTargets(),source_joint.GetBody1Rel().GetTargets()
    if len(b0)!=1 or len(b1)!=1:raise RuntimeError('轮轴刚体关系无效')
    fixeds = [p for p in Usd.PrimRange(source_root,Usd.TraverseInstanceProxies())
              if p.IsA(UsdPhysics.FixedJoint) and UsdPhysics.Joint(p).GetBody1Rel().GetTargets()==b0
              and not UsdPhysics.Joint(p).GetBody0Rel().GetTargets()]
    if len(fixeds)!=1:raise RuntimeError('来源阀体缺少唯一世界固定约束')
    def mapped(path):
        path=str(path)
        if not path.startswith(source_prim+'/'):raise RuntimeError('物理关系指向来源根外部')
        return root+'/Valve/Asset'+path[len(source_prim):]
    base_path,wheel_path = mapped(b0[0]),mapped(b1[0])
    fixed_path,joint_path = mapped(fixeds[0].GetPath()),mapped(joints[0].GetPath())
    cache = UsdGeom.XformCache()
    source_body = source_stage.GetPrimAtPath(b0[0])
    body_matrix = np.asarray(cache.GetLocalToWorldTransform(source_body)).T
    scales = np.linalg.norm(body_matrix[:3,:3],axis=0)
    body_rotation = body_matrix[:3,:3]/scales
    if not np.allclose(body_rotation.T@body_rotation,np.eye(3),atol=1e-6):raise RuntimeError('阀体根存在剪切变换')
    base_points=[]
    for mesh in Usd.PrimRange(source_body,Usd.TraverseInstanceProxies()):
        if mesh.IsA(UsdGeom.Mesh):
            M=np.asarray(cache.GetLocalToWorldTransform(mesh)).T
            points=np.asarray(UsdGeom.Mesh(mesh).GetPointsAttr().Get(),float)
            base_points.append((points@M[:3,:3].T+M[:3,3])*unit)
    if not base_points:raise RuntimeError('阀体缺少实际网格')
    bottom_local=float(np.concatenate(base_points)[:,2].min())
    support_top=float(position[2])+bottom_local
    if support_top <= 0:raise RuntimeError('工位高度不足以容纳原尺寸阀体')
    stage.GetSessionLayer().Export(str(output / 'session_before_valve.usda'))
    stage.GetRootLayer().Export(str(output / 'root_before_valve.usda'))
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        station = UsdGeom.Xform.Define(stage, root)
        station.GetPrim().SetCustomDataByKey('description', '阀门验证工位：参考尺寸支架，真实接触转动，无轮轴驱动')
        # 支架只提供已固定阀体的视觉/碰撞包络，不用它连接夹爪或驱动手轮。
        width = .22
        support = UsdGeom.Cube.Define(stage, root + '/Support')
        support.CreateSizeAttr(1.)
        support.AddTranslateOp().Set(Gf.Vec3d(position[0], position[1], support_top / 2))
        support.AddScaleOp().Set(Gf.Vec3d(width, width, support_top))
        support.CreateDisplayColorAttr([Gf.Vec3f(.22, .25, .29)])
        UsdPhysics.CollisionAPI.Apply(support.GetPrim())
        mount = UsdGeom.Xform.Define(stage, root + '/Mount')
        mount_position=position+body_matrix[:3,3]*unit
        mount.AddTranslateOp().Set(Gf.Vec3d(*map(float,mount_position)))
        quat=Rotation.from_matrix(body_rotation).as_quat(scalar_first=True)
        mount.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(quat[0]),Gf.Vec3d(*quat[1:].tolist())))
        UsdPhysics.RigidBodyAPI.Apply(mount.GetPrim()).CreateKinematicEnabledAttr(True)
        valve = UsdGeom.Xform.Define(stage, root + '/Valve')
        valve.AddTranslateOp(opSuffix='station').Set(Gf.Vec3d(*map(float, position)))
        valve.AddScaleOp(opSuffix='units').Set(Gf.Vec3d(unit))
        # 外层只放置/换单位；引用defaultPrim保留来源父级0.0011等缩放，不能只引用子模型。
        asset=UsdGeom.Xform.Define(stage,root+'/Valve/Asset')
        asset.GetPrim().GetReferences().AddReference(str(source),source_prim)
        for prim in list(Usd.PrimRange(asset.GetPrim(),Usd.TraverseInstanceProxies())):
            for schema in (UsdPhysics.ArticulationRootAPI,PhysxSchema.PhysxArticulationAPI):
                if prim.HasAPI(schema):prim.RemoveAPI(schema)
        fixed=UsdPhysics.FixedJoint(stage.GetPrimAtPath(fixed_path))
        fixed.CreateBody0Rel().SetTargets([root + '/Mount'])
        fixed.CreateLocalPos0Attr(Gf.Vec3f(0))
        fixed.CreateLocalPos1Attr(Gf.Vec3f(0))
        fixed.CreateLocalRot0Attr(Gf.Quatf(1))
        fixed.CreateLocalRot1Attr(Gf.Quatf(1))
        fixed.CreateExcludeFromArticulationAttr(True)
        wheel = stage.GetPrimAtPath(wheel_path)
        joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(joint_path))
        joint.CreateExcludeFromArticulationAttr(True)
        PhysxSchema.PhysxContactReportAPI.Apply(wheel).CreateThresholdAttr(0.)
        # 沿用旧阀门场景的摩擦参考值1.0，记录来源，不把它当作真机标定。
        material = UsdShade.Material.Define(stage, root + '/ContactMaterial')
        physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr(1.)
        physics_material.CreateDynamicFrictionAttr(1.)
        physics_material.CreateRestitutionAttr(0.)
        collision_meshes=[]
        for body in (base_path,wheel_path):
            for mesh in Usd.PrimRange(stage.GetPrimAtPath(body),Usd.TraverseInstanceProxies()):
                if not mesh.IsA(UsdGeom.Mesh):continue
                if not mesh.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI.Apply(mesh)
                    UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr('convexHull')
                elif not mesh.HasAPI(UsdPhysics.MeshCollisionAPI):
                    UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr('convexHull')
                UsdShade.MaterialBindingAPI.Apply(mesh).Bind(material,materialPurpose='physics')
                collision_meshes.append(str(mesh.GetPath()))
        report = {'valve_root': root + '/Valve', 'source': str(source), 'source_prim': source_prim,
                  'position_world': list(map(float, position)), 'source_meters_per_unit': unit,'base_path':base_path,'wheel_path':wheel_path,'joint_path':joint_path,'collision_meshes':collision_meshes,
                  'fixture_reference_size_m': [width, width, support_top],
                  'friction_reference': {'value': 1., 'source': 'scripts/scene/static_scene.usda',
                                         'calibrated_to_hardware': False},
                  'source_joint_friction': joint.GetPrim().GetAttribute('physxJoint:jointFriction').Get(),
                  'wheel_drive_added': False, 'robot_reloaded': False,
                  'original_assets_modified': False, 'session_layer_only': True}
    # 已有导航器缓存了工位添加前的形状清单；下次使用时必须重新采当前环境。
    # 只在前面已确认没有待办/运行任务后失效缓存，不改变任何机器人目标。
    if hasattr(executor, '_arm_transit_safety'):
        executor._arm_transit_safety = None
    if owner is not None:
        owner._approach = None
    report['navigation_geometry_cache_invalidated'] = True
    (output / 'station_setup.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    return report


def relocate_empty(executor, position, output, source, root='/World/ValveStation'):
    """仅重排本工具新增且未被夹持的工位；须先完成新布局的离线环境查询。"""
    import omni.usd
    from pxr import Usd
    from valve_feedback import capture
    stage = omni.usd.get_context().get_stage()
    snapshot = capture(executor, root + '/Valve')
    if snapshot['contacts']['left_pad_touching'] or snapshot['contacts']['right_pad_touching']:
        raise RuntimeError('阀门正在接触夹爪，禁止重排验证工位')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    stage.GetSessionLayer().Export(str(output / 'session_before_relayout.usda'))
    (output / 'fixture_before_relayout.json').write_text(json.dumps(snapshot, ensure_ascii=False, indent=2))
    # PhysX/USD 同步可能在匿名根层留下此新增工位的 over；只清本工具拥有的路径。
    # 仅删 session def 会留下空 prim，使后续重建误判已有有效工位。
    for layer in (stage.GetSessionLayer(), stage.GetRootLayer()):
        with Usd.EditContext(stage, layer):
            stage.RemovePrim(root)
    rebind_existing_views(executor)
    return setup(executor, position, output / 'relayout', source, root)


def translate_empty_xy(executor, position, output):
    """暂停状态平移本轮新增空载工位，仅改三个已有session属性，不是机器人走位。

    提交只允许一次play/step/pause。失败恢复原属性后保持暂停，不再偷偷step；
    因此rollback时明确报告物理姿态可能尚待操作员另行重新同步。
    """
    import numpy as np
    import omni.usd
    import omni.physx
    from pxr import Gf, Usd
    from scipy.spatial.transform import Rotation
    from valve_feedback import capture
    from valve_runtime import _competing_controllers
    from valve_safety import ValveSafety

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    root = '/World/ValveStation'
    stage = omni.usd.get_context().get_stage()
    interface = omni.physx.get_physx_interface()
    original, attributes = {}, {}
    mutated = False
    cancellation = None
    paused_entry = not executor.world.is_playing()
    report = {'success': False, 'operation': 'translate_only_our_new_empty_valve_station_xy',
              'is_robot_navigation': False, 'root': root, 'session_layer_only': True,
              'prims_created_or_removed': False, 'views_rebound': False,
              'world_reset': False, 'robot_state_or_target_setters_used': [],
              'commit_outer_step_count': 0, 'rollback_performed': False,
              'reference_checks': {'robot_root_translation_m': .001, 'head_yaw_deg': .1,
                                   'valve_pose_translation_m': .001, 'valve_pose_rotation_deg': .1,
                                   'robot_revolute_q_change_rad': .005, 'robot_prismatic_q_change_m': .001,
                                   'hand_position_change_m': .002, 'hand_rotation_change_deg': .1,
                                   'original_mechanism_translation_m': .001, 'original_mechanism_rotation_deg': .1,
                                   'layout_environment_clearance_m': .012, 'layout_self_clearance_m': .008,
                                   'arm_hand_clearance_m': .05},
              'checks_are_simulation_references_not_zero_motion_guarantees': True}

    def write(name, value):
        (output / name).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))

    def matrix(pose):
        p, q = np.asarray(pose['position'], float), np.asarray(pose['orientation'], float)
        if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
            raise RuntimeError('工位/机器人实际位姿无效')
        return p, Rotation.from_quat(q, scalar_first=True).as_matrix()

    def rigid(path):
        value = interface.get_rigidbody_transformation(path)
        if not value or value.get('ret_val') is not True:
            raise RuntimeError('原生刚体位姿不可读: ' + path)
        q = np.asarray(value['rotation'], float)
        p = np.asarray(value['position'], float)
        if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
            raise RuntimeError('原生刚体位姿非有限: ' + path)
        return {'position': p.tolist(), 'orientation': q[[3, 0, 1, 2]].tolist()}

    def views():
        result = [executor.articulation]
        for attribute in ('_oxford_devices', '_l3_station'):
            owner = getattr(executor, attribute, None)
            if owner is not None:
                devices = getattr(owner, 'devices', None) or getattr(getattr(owner, 'runtime', None), 'devices', {})
                result.extend(devices.values())
        return list({id(view): view for view in result}.values())

    def view_snapshot(view):
        if not view.is_physics_handle_valid():
            raise RuntimeError('已有Articulation物理视图无效，禁止重绑/重置作为布局fallback')
        native = view._physics_view
        result = {'python_view_id': id(view), 'physics_view_id': id(native),
                  'dof_names': list(view.dof_names), 'valid': True}
        for key, method in (
                ('position_targets', 'get_dof_position_targets'), ('velocity_targets', 'get_dof_velocity_targets'),
                ('stiffness', 'get_dof_stiffnesses'), ('damping', 'get_dof_dampings'),
                ('max_forces', 'get_dof_max_forces')):
            values = np.asarray(getattr(native, method)()).copy()
            if not np.isfinite(values).all():
                raise RuntimeError('已有Articulation参数非有限: ' + key)
            result[key] = values.tolist()
        return result

    def contacts_empty(snapshot):
        contacts = snapshot.get('contacts', {})
        if not contacts.get('available'):
            raise RuntimeError('没有可靠接触报告，不能假设工位空载')
        if (contacts.get('left_pad_touching') or contacts.get('right_pad_touching') or
                int(contacts.get('contact_count', 0)) != 0):
            raise RuntimeError('现有新增工位正在与机器人接触，禁止平移')

    def yaw(pose):
        return float(Rotation.from_matrix(matrix(pose)[1]).as_euler('xyz')[2])

    def wrapped_deg(value):
        return float(np.degrees((value + np.pi) % (2*np.pi) - np.pi))

    def is_station_shape(shape):
        return shape['path'].startswith(root + '/')

    def is_ground_shape(shape):
        return shape['path'].startswith('/World/ground_plane/')

    def fixture_environment_check(guard, poses):
        """CollisionModel原本只枚举涉及robot的pair，因此额外完整检查移动夹具对原环境。"""
        coal = guard.model.coal
        moving = [s for s in guard.model.shapes if is_station_shape(s)]
        others = [s for s in guard.model.shapes if not s['robot'] and not is_station_shape(s) and not is_ground_shape(s)]
        if not moving:
            raise RuntimeError('fresh几何缺少新增工位碰撞体，不能以空列表当布局安全')
        measurements, violations = [], []
        for a in moving:
            ta = poses[a['body']] if a['body'] else np.eye(4)
            for b in others:
                tb = poses[b['body']] if b['body'] else np.eye(4)
                value = coal.DistanceResult()
                distance = float(coal.distance(a['geometry'], coal.Transform3s(ta[:3, :3], ta[:3, 3]),
                                               b['geometry'], coal.Transform3s(tb[:3, :3], tb[:3, 3]),
                                               guard.model.request, value))
                if not np.isfinite(distance):
                    raise RuntimeError('夹具↔原环境精确距离无效')
                row = {'fixture_shape': a['path'], 'original_environment_shape': b['path'],
                       'distance_m': distance, 'required_margin_m': .012}
                measurements.append(row)
                if distance < .012:
                    violations.append(row)
        # 无限Plane未进入coal convex形状列表；除明确地面外，保守角点检查不能漏掉。
        for plane in getattr(guard, '_planes', []):
            if is_ground_shape(plane) or is_station_shape(plane):
                continue
            transform = poses.get(plane['body'], np.eye(4)) @ np.asarray(plane['matrix'])
            normal = np.linalg.inv(transform[:3, :3]).T[:, 'XYZ'.index(plane.get('axis', 'Z'))]
            normal /= np.linalg.norm(normal)
            for shape in moving:
                body = poses[shape['body']] if shape['body'] else np.eye(4)
                points = shape['corners'] @ body[:3, :3].T + body[:3, 3]
                distance = float(np.min((points - transform[:3, 3]) @ normal))
                row = {'fixture_shape': shape['path'], 'original_environment_shape': plane['path'],
                       'distance_m': distance, 'required_margin_m': .012,
                       'method': '保守角点对原非ground无限Plane'}
                measurements.append(row)
                if distance < .012:
                    violations.append(row)
        return {'safe': not violations, 'pair_count': len(measurements),
                'fixture_shape_count': len(moving), 'original_environment_shape_count': len(others),
                'minimum_clearance_m': min((r['distance_m'] for r in measurements), default=None),
                'violations': violations, 'measurements': measurements,
                'explicitly_excluded_ground_shapes': [s['path'] for s in guard.model.data['shapes'] if is_ground_shape(s)]}

    def whole_body_and_arms(guard, poses=None):
        if guard.capture_counts['robot_shapes'] != 96:
            raise RuntimeError('fresh几何没有完整96机器人形状')
        configuration = (guard.check_configuration(environment_margin_m=.012, self_margin_m=.008)
                         if poses is None else guard.check_configuration(q=guard.q.copy(), poses=poses,
                                                   environment_margin_m=.012, self_margin_m=.008))
        arms = guard.check(min_clearance_m=.05, refresh=False, poses=guard.poses if poses is None else poses)
        return {'safe': bool(configuration['safe'] and arms['safe']),
                'whole_robot': configuration, 'four_arm_hand_groups': arms, 'coverage': guard.capture_counts}

    try:
        target = np.asarray(position, float)
        if target.shape != (3,) or not np.isfinite(target).all() or abs(target[2] - .84) > 1e-9:
            raise ValueError('本工位只允许明确给出的有限XY位置，Z必须保持0.84m')
        if not paused_entry or (hasattr(executor.world, 'is_stopped') and executor.world.is_stopped()):
            raise RuntimeError('入口必须是暂停而非运行/停止状态，不能打断其他动作来搬工位')
        _competing_controllers(executor)
        before = capture(executor, root + '/Valve')
        contacts_empty(before)
        root_prim = stage.GetPrimAtPath(root)
        if not root_prim or '阀门验证工位' not in str(root_prim.GetCustomDataByKey('description') or ''):
            raise RuntimeError('路径不是本工具新增验证工位，禁止修改已有资产')
        op_paths = {
            'valve': (root + '/Valve', 'xformOp:translate:station'),
            'mount': (root + '/Mount', 'xformOp:translate'),
            'support': (root + '/Support', 'xformOp:translate'),
        }
        for name, (path, attr_name) in op_paths.items():
            prim = stage.GetPrimAtPath(path)
            attr = prim.GetAttribute(attr_name) if prim else None
            if not attr or not attr.HasAuthoredValueOpinion():
                raise RuntimeError('工位没有所需已有平移属性，禁止创建: ' + path + '.' + attr_name)
            session = stage.GetSessionLayer()
            spec = session.GetAttributeAtPath(attr.GetPath())
            if (spec is None or not spec.HasInfo('default') or attr.GetNumTimeSamples() != 0 or
                    session.ListTimeSamplesForPath(attr.GetPath())):
                raise RuntimeError('工位平移必须本就在SessionLayer有default且无timesamples，不能精确rollback则拒绝')
            value = np.asarray(attr.Get(), float)
            if value.shape != (3,) or not np.isfinite(value).all():
                raise RuntimeError('工位原平移属性无效')
            if not np.array_equal(value, np.asarray(spec.default, float)):
                raise RuntimeError('工位组合平移与原Session default不同，拒绝不精确rollback')
            original[name] = value.copy()
            attributes[name] = attr
        if abs(original['valve'][2] - .84) > 1e-9 or abs(original['mount'][2] - .84) > 1e-9:
            raise RuntimeError('原工位Z已不是0.84m，不允许借XY函数改高度')
        if np.linalg.norm(original['valve'] - original['mount']) > 1e-9:
            raise RuntimeError('原阀门与运动学固定Mount的平移不一致')
        delta = target - original['valve']
        if abs(delta[2]) > 1e-9:
            raise RuntimeError('禁止非XY工位平移')
        before_guard = ValveSafety(executor, directory=output / 'geometry_before')
        original_mechanism_before = {path: rigid(path) for path, data in before_guard.model.data['bodies'].items()
                                     if not data['robot'] and not path.startswith(root + '/')}
        before_safety = whole_body_and_arms(before_guard)
        report['initial_geometry_check'] = before_safety
        if not before_safety['safe']:
            raise RuntimeError('原全身/自身/四臂手几何门槛未通过，禁止先平移夹具赌安全')
        predicted = {p: np.array(t, copy=True) for p, t in before_guard.poses.items()}
        moved_bodies = set()
        for shape in before_guard.model.shapes:
            if is_station_shape(shape):
                moved_bodies.add(shape['body'])
        for path in moved_bodies:
            if path is None:
                raise RuntimeError('新增工位形状没有可追溯刚体/静态frame')
            predicted[path][:3, 3] += delta
        target_safety = whole_body_and_arms(before_guard, predicted)
        target_fixture = fixture_environment_check(before_guard, predicted)
        report.update(target_geometry_check=target_safety, target_fixture_environment_check=target_fixture,
                      hypothetical_moved_bodies=sorted(moved_bodies))
        write('translate_target_preflight.json', report)
        if not target_safety['safe'] or not target_fixture['safe']:
            raise RuntimeError('仅本地预测的目标布局未通过全身/自身/原环境12mm/四臂手50mm检查')
        live_views = views()
        before_views = [view_snapshot(view) for view in live_views]
        if len(before_views[0]['dof_names']) != 50:
            raise RuntimeError('本次验证要求当前50DOF整机，拒绝更换模型后复用布局')
        before_base = rigid(before['wheel_joint']['body0'])
        before_head = executor._link_pose_table().get('head_yaw_link')
        if before_head is None:
            raise RuntimeError('缺少实际head_yaw_link反馈')
        before_head_pose = {'position': list(map(float, before_head[0])),
                            'orientation': list(map(float, before_head[1]))}
        before_paths = {str(prim.GetPath()) for prim in stage.Traverse()}
        head_index = before['state']['joint_names'].index('head_yaw_joint')
        head_joint_before = float(before['state']['positions'][head_index])
        report.update(position_before=original['valve'].tolist(), position_requested=target.tolist(),
                      delta_world_m=delta.tolist(), attributes_before={k: v.tolist() for k, v in original.items()},
                      attribute_paths={k: path + '.' + attr for k, (path, attr) in op_paths.items()})
        write('translate_before.json', {'snapshot': before, 'valve_base_pose': before_base,
                                        'head_pose': before_head_pose, 'views': before_views,
                                        'original_mechanism_poses': original_mechanism_before, **report})

        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for name, attr in attributes.items():
                updated = original[name] + delta
                mutated = True
                if not attr.Set(Gf.Vec3d(*updated.tolist())):
                    raise RuntimeError('已有平移属性Set失败: ' + name)
        # 不重绑、不reset、不改任何机器人参数；只提交一次正常物理步。
        commit_error = None
        try:
            executor.world.play()
            report['commit_outer_step_count'] = 1
            executor.step(render=False)
        except BaseException as error:
            commit_error = error
            raise
        finally:
            try:
                executor.world.pause()
            except BaseException as pause_error:
                report['commit_pause_error'] = repr(pause_error)
                if commit_error is None:
                    raise

        after = capture(executor, root + '/Valve')
        write('translate_after_capture.json', after)
        report['commit_simulation_advance_s'] = float(after['state'].get('sim_time', 0.) - before['state'].get('sim_time', 0.))
        _competing_controllers(executor)
        contacts_empty(after)
        if {id(view) for view in views()} != {id(view) for view in live_views}:
            raise RuntimeError('已有Articulation对象清单改变')
        after_views = [view_snapshot(view) for view in live_views]
        write('translate_after_views.json', after_views)
        for prior, current in zip(before_views, after_views):
            for key in ('python_view_id', 'physics_view_id', 'dof_names'):
                if prior[key] != current[key]:
                    raise RuntimeError('已有Articulation身份/关节顺序改变: ' + key)
            for key in ('position_targets', 'velocity_targets', 'stiffness', 'damping', 'max_forces'):
                if not np.array_equal(np.asarray(prior[key]), np.asarray(current[key])):
                    raise RuntimeError('提交后已有Articulation目标/参数改变: ' + key)
        if before_paths != {str(prim.GetPath()) for prim in stage.Traverse()}:
            raise RuntimeError('提交过程中prim清单改变，拒绝认为只是XY属性更新')
        if before['state']['joint_names'] != after['state']['joint_names']:
            raise RuntimeError('机器人实际关节名顺序改变')
        q_difference = np.abs(np.asarray(after['state']['positions']) - np.asarray(before['state']['positions']))
        prismatic = {joint['name'] for joint in before_guard.model.joints if joint['type'] == 'PhysicsPrismaticJoint'}
        q_limits = np.array([.001 if name in prismatic else .005 for name in before['state']['joint_names']])
        report['actual_robot_q_changes'] = {name: float(change) for name, change in zip(before['state']['joint_names'], q_difference)}
        if not np.isfinite(q_difference).all() or np.any(q_difference >= q_limits):
            raise RuntimeError('一次提交引起机器人实际关节变化超过明确参考界限')
        hands = {}
        for side in ('left', 'right'):
            for key in ('flange', 'tcp'):
                p0, R0 = matrix(before['state']['sides'][side][key])
                p1, R1 = matrix(after['state']['sides'][side][key])
                change = float(np.linalg.norm(p1 - p0))
                angle = float(np.degrees(Rotation.from_matrix(R1 @ R0.T).magnitude()))
                hands[side + '_' + key] = {'translation_m': change, 'rotation_deg': angle}
                if change >= .002 or angle >= .1:
                    raise RuntimeError('一次提交引起实际双手位姿变化超过参考界限: ' + side + '_' + key)
        report['actual_hand_pose_changes'] = hands
        robot_p0, _ = matrix(before['base_pose'])
        robot_p1, _ = matrix(after['base_pose'])
        root_error = float(np.linalg.norm(robot_p1 - robot_p0))
        after_head = executor._link_pose_table()['head_yaw_link']
        after_head_pose = {'position': list(map(float, after_head[0])),
                           'orientation': list(map(float, after_head[1]))}
        head_world_delta = abs(wrapped_deg(yaw(after_head_pose) - yaw(before_head_pose)))
        head_joint_delta = abs(wrapped_deg(float(after['state']['positions'][head_index]) - head_joint_before))
        if root_error >= .001 or head_world_delta >= .1 or head_joint_delta >= .1:
            raise RuntimeError('一次提交引起机器人root/头yaw超出检查界限')
        after_base = rigid(after['wheel_joint']['body0'])
        pose_checks = {}
        for name, prior, current in (
                ('valve_base', before_base, after_base),
                ('wheel', before['geometry']['wheel_pose'], after['geometry']['wheel_pose']),
                ('mount', before['fixed_anchor']['pose'], after['fixed_anchor']['pose'])):
            p0, R0 = matrix(prior); p1, R1 = matrix(current)
            error = float(np.linalg.norm(p1 - p0 - delta))
            rotation = float(np.degrees(Rotation.from_matrix(R1 @ R0.T).magnitude()))
            pose_checks[name] = {'translation_delta_error_m': error, 'rotation_change_deg': rotation,
                                 'actual_position_after': p1.tolist()}
            if error >= .001 or rotation >= .1:
                raise RuntimeError('工位实际刚体没有按同一delta平移/朝向改变: ' + name)
        after_guard = ValveSafety(executor, directory=output / 'geometry_after')
        if (set(before_guard.model.data['collision_paths']) != set(after_guard.model.data['collision_paths']) or
                before_guard.capture_counts != after_guard.capture_counts):
            raise RuntimeError('提交前后CollisionAPI清单/覆盖数改变，不能声称原环境保持')
        post_safety = whole_body_and_arms(after_guard)
        post_fixture = fixture_environment_check(after_guard, after_guard.poses)
        report.update(post_geometry_check=post_safety, post_fixture_environment_check=post_fixture)
        if not post_safety['safe'] or not post_fixture['safe']:
            raise RuntimeError('提交后fresh全身96/自身/四臂手50mm/夹具对原环境12mm检查未通过')
        mechanisms = {}
        for body, old_pose in original_mechanism_before.items():
            current_pose = rigid(body)
            old_p, old_R = matrix(old_pose)
            current_p, current_R = matrix(current_pose)
            shift = float(np.linalg.norm(current_p - old_p))
            angle = float(np.degrees(Rotation.from_matrix(current_R @ old_R.T).magnitude()))
            mechanisms[body] = {'translation_m': shift, 'rotation_deg': angle}
            if shift >= .001 or angle >= .1:
                raise RuntimeError('一次提交引起原机构位姿变化超过参考界限: ' + body)
        report['original_mechanism_pose_changes'] = mechanisms
        report.update(success=True, actual_robot_root_change_m=root_error,
                      actual_head_world_yaw_change_deg=head_world_delta,
                      actual_head_joint_yaw_change_deg=head_joint_delta,
                      physical_pose_checks=pose_checks, all_view_identities_valid_and_preserved=True,
                      all_targets_gains_maxforce_exactly_preserved=True,
                      stage_prim_paths_preserved=True, collision_shape_paths_preserved=True,
                      navigation_geometry_must_be_recaptured=True)
        write('translate_after.json', {'snapshot': after, 'valve_base_pose': after_base,
                                       'head_pose': after_head_pose, 'views': after_views})
    except BaseException as error:
        report.update(success=False, error=str(error), error_type=type(error).__name__,
                      cancelled=not isinstance(error, Exception))
        if not isinstance(error, Exception):
            cancellation = error
        if paused_entry:
            try:
                executor.world.pause()
            except BaseException as pause_error:
                report['pause_cleanup_error'] = repr(pause_error)
        if mutated:
            report.update(require_physical_resync=True, failure_rollback_authored_only=True,
                          no_rollback_physics_step_permitted=True)
            try:
                with Usd.EditContext(stage, stage.GetSessionLayer()):
                    for name, attr in attributes.items():
                        if not attr.Set(Gf.Vec3d(*original[name].tolist())):
                            raise RuntimeError('原平移属性rollback失败: ' + name)
                report.update(rollback_performed=True, rollback_authored_only_no_second_step=True,
                              physical_pose_may_require_later_explicit_resynchronization=True)
            except BaseException as rollback_error:
                report['rollback_error'] = str(rollback_error)
    try:
        report['paused_after'] = not executor.world.is_playing()
    except BaseException as state_error:
        report['paused_after'] = None
        report['pause_state_read_error'] = repr(state_error)
    write('translate_result.json', report)
    if cancellation is not None:
        raise cancellation
    return report
