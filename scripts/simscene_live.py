"""SimScene覆盖层与已有Kit场景的主线程适配；不启动/重置仿真。"""
import json
from pathlib import Path


def _checkpoint(executor):
    import numpy as np
    view = executor.articulation._physics_view
    if not view.check():
        raise RuntimeError('机器人物理视图无效，保持现场不变')
    methods = ['get_dof_positions', 'get_dof_velocities', 'get_root_transforms', 'get_root_velocities',
               'get_dof_position_targets', 'get_dof_velocity_targets', 'get_dof_stiffnesses',
               'get_dof_dampings', 'get_dof_max_forces', 'get_dof_max_velocities', 'get_dof_limits',
               'get_masses', 'get_inertias']
    return {name: np.asarray(getattr(view, name)()).copy() for name in methods}


def _verify(executor, before):
    import numpy as np
    view = executor.articulation._physics_view
    if not view.check():
        raise RuntimeError('附着后物理视图失效，禁止自动reset')
    same = {name: bool(np.array_equal(value, getattr(view, name)())) for name, value in before.items()}
    if not all(same.values()):
        raise RuntimeError('附着意外改变原生状态：' + str(same))
    return same


def inspect(executor):
    import numpy as np
    import omni.usd
    from pxr import UsdGeom
    stage = omni.usd.get_context().get_stage()
    saved = _checkpoint(executor)
    children = []
    cache = UsdGeom.BBoxCache(0, ['default', 'render', 'proxy'])
    for root in stage.GetPseudoRoot().GetChildren():
        for prim in (list(root.GetChildren()) if str(root.GetPath()) == '/World' else [root]):
            bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            children.append({'path': str(prim.GetPath()), 'type': prim.GetTypeName(),
                'bounds': None if bounds.IsEmpty() else {'min': list(bounds.GetMin()), 'max': list(bounds.GetMax())}})
    return {'stage_id': stage.GetRootLayer().identifier, 'pid': __import__('os').getpid(),
        'playing': executor.world.is_playing(), 'physics_dt': executor.world.get_physics_dt(),
        'meters_per_unit': UsdGeom.GetStageMetersPerUnit(stage), 'up_axis': str(UsdGeom.GetStageUpAxis(stage)),
        'robot_shapes': executor.articulation.num_shapes, 'finite_q': bool(np.isfinite(saved['get_dof_positions']).all()),
        'robot_view_valid': True, 'objects': children, 'owned': getattr(executor, '_simscene_attached', {}),
        'mode': 'attached_to_RouteManager', 'simulation_started': False}


def _paused(executor, expected_stage):
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    if stage.GetRootLayer().identifier != expected_stage:
        raise RuntimeError('场景身份已改变，请重新检查当前场景')
    if executor.world.is_playing() or any(getattr(executor, k, None) is not None for k in ('_motion', '_base_cmd', '_auto_pick')):
        raise RuntimeError('先停止任务并暂停当前场景，才能添加静态覆盖层')
    return stage


def attach(executor, scene_id, source, expected_stage, output):
    import hashlib
    import numpy as np
    import omni.usd
    from pxr import Usd, Sdf, UsdGeom, UsdPhysics, UsdLux
    stage = _paused(executor, expected_stage)
    before = _checkpoint(executor)
    source = Path(source).resolve()
    imported = Usd.Stage.Open(str(source))
    root = imported.GetDefaultPrim()
    if not root or UsdGeom.GetStageMetersPerUnit(imported) != 1 or UsdGeom.GetStageUpAxis(imported) != 'Z':
        raise RuntimeError('覆盖层必须有defaultPrim，且已归一化为Z-up米制')
    forbidden = []
    for prim in imported.Traverse():
        physical_schemas = any(x.startswith(('Physics', 'Physx', 'Newton')) for x in prim.GetAppliedSchemas())
        if (physical_schemas or prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI(UsdPhysics.CollisionAPI)
                or prim.HasAPI(UsdPhysics.ArticulationRootAPI) or prim.IsA(UsdPhysics.Joint)
                or prim.IsA(UsdPhysics.Scene) or prim.IsA(UsdGeom.Camera)
                or (prim.HasAPI(UsdLux.LightAPI) and UsdLux.LightAPI(prim).GetIntensityAttr().Get())):
            forbidden.append(str(prim.GetPath()))
    if forbidden:
        raise RuntimeError('此附着入口仅接受无相机/物理/活跃光源的静态视觉层：' + str(forbidden[:10]))
    owned = getattr(executor, '_simscene_attached', {})
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    if scene_id in owned:
        if owned[scene_id]['source_sha256'] == digest:
            return {'replayed': True, **owned[scene_id], 'state_preserved': _verify(executor, before)}
        raise RuntimeError('同名覆盖层已存在；请使用新的scene_id，不能原位覆盖现有资产')
    # 使用当前真实96形状的世界包络做保守布局检查，另保护整个人物巡检NavMesh范围。
    from valve_safety import ValveSafety
    import uuid
    safety = ValveSafety(executor, directory=Path(output) / ('geometry_' + scene_id + '_' + uuid.uuid4().hex[:8]))
    _, low, high = safety._shape_bounds(safety.poses, '当前几何反馈不可用')
    protected = [(shape['path'], low[i], high[i]) for i, shape in enumerate(safety.model.shapes)
                 if 'ground_plane' not in shape['path'].lower() and shape['type'] != 'Plane']
    cache = UsdGeom.BBoxCache(0, ['default', 'render', 'proxy'])
    nav = stage.GetPrimAtPath('/World/PeoplePatrol/NavMeshVolume')
    if nav:
        box = cache.ComputeWorldBound(nav).ComputeAlignedRange()
        protected.append((str(nav.GetPath()), np.array(box.GetMin()), np.array(box.GetMax())))
    for prim in root.GetChildren():
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if box.IsEmpty():
            continue
        lo, hi = np.array(box.GetMin()), np.array(box.GetMax())
        for path, a, b in protected:
            if np.all(hi + .05 >= a) and np.all(b + .05 >= lo):
                raise RuntimeError('新资产包络侵入现有机器人/机构/巡检区域：' + str(prim.GetPath()) + ' / ' + path)
    namespace = '/World/SimScene/' + scene_id
    if stage.GetPrimAtPath(namespace):
        raise RuntimeError('目标命名空间已有对象，不能覆盖')
    layer = Sdf.Layer.CreateAnonymous('SimScene_' + scene_id + '.usda')
    stage.GetSessionLayer().subLayerPaths.append(layer.identifier)
    record = {'namespace': namespace, 'layer': layer.identifier, 'source': str(source), 'source_sha256': digest}
    executor._simscene_attached = {**owned, scene_id: record}
    try:
        with Usd.EditContext(stage, layer):
            UsdGeom.Xform.Define(stage, namespace).GetPrim().GetReferences().AddReference(str(source))
        checks = _verify(executor, before)
    except Exception:
        # 只撤销本轮层；不自动重建物理、恢复运动或操作已有资产。
        layers = list(stage.GetSessionLayer().subLayerPaths)
        if layer.identifier in layers:
            layers.remove(layer.identifier)
            stage.GetSessionLayer().subLayerPaths = layers
        executor._simscene_attached = owned
        executor.world.pause()
        raise
    return {'attached': True, 'state_preserved': checks, 'layout_margin_reference_m': .05,
            'physics_added': False, 'camera_added': False, **record}


def remove(executor, scene_id, expected_stage):
    import logging
    stage = _paused(executor, expected_stage)
    before = _checkpoint(executor)
    owned = getattr(executor, '_simscene_attached', {})
    if scene_id not in owned:
        raise RuntimeError('只允许移除本入口记录的覆盖层')
    record = owned[scene_id]
    layers = list(stage.GetSessionLayer().subLayerPaths)
    layers.remove(record['layer'])
    stage.GetSessionLayer().subLayerPaths = layers
    # 已知旧Articulation对任意prim删除都会删view。只重绑定有效后端，不reset状态。
    executor.simulation_app.update()
    from valve_station import rebind_existing_views
    from physics_view_guard import acknowledge_rebuilt_view
    rebind_existing_views(executor)
    acknowledge_rebuilt_view(executor, logging.getLogger(__name__))
    checks = _verify(executor, before)
    executor._simscene_attached = {k: v for k, v in owned.items() if k != scene_id}
    return {'removed': scene_id, 'state_preserved': checks, 'reset': False}


def dispatch(executor, operation, arguments):
    functions = {'inspect': inspect, 'attach': attach, 'remove': remove}
    if operation not in functions:
        raise ValueError('未知附着操作')
    return functions[operation](executor, **arguments)
