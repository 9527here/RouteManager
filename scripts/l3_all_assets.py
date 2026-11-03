#!/usr/bin/env python3
"""保留三件人形操作工位，旁边补齐其余十二件可操作资产。"""
import json
from pathlib import Path


def build(asset_root):
    from pxr import Usd, UsdGeom, UsdPhysics, Gf
    root = Path(asset_root)
    path = root / 'station_all.usda'
    if path.exists():
        raise FileExistsError('请写到新目录，不能覆盖正在加载的场景组合层')
    stage = Usd.Stage.CreateNew(str(path))
    prim = UsdGeom.Xform.Define(stage, '/L3Station').GetPrim()
    prim.GetReferences().AddReference('./station.usda')
    stage.SetDefaultPrim(prim)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, 'Z')
    parts = [('CatalogBoard', [.825, -1.24, 1.055], [.025, 1., .66]),
             ('CatalogLeg', [.85, -1.24, .43], [.045, .045, .86]),
             ('CatalogFoot', [.85, -1.24, .015], [.30, .50, .03])]
    for name, position, size in parts:
        cube = UsdGeom.Cube.Define(stage, '/L3Station/' + name)
        cube.CreateSizeAttr(1.)
        cube.CreateDisplayColorAttr([(.22, .28, .32)])
        xf = UsdGeom.Xformable(cube)
        xf.AddTranslateOp().Set(Gf.Vec3d(*position))
        xf.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    layout = {}
    groups = [('button', ['object_0', 'object_1', 'object_2'], [-.90, -1.16, -1.42], 1.26),
              ('selector', ['object_4', 'object_5', 'object_6', 'object_6_variant', 'object_19'],
               [-.88, -1.04, -1.20, -1.36, -1.52], 1.08),
              ('plate', ['object_13', 'object_15', 'object_16', 'object_111'],
               [-.90, -1.10, -1.30, -1.50], .87)]
    for kind, names, ys, z in groups:
        for name, y in zip(names, ys):
            position = [.78, y, z]
            prim = UsdGeom.Xform.Define(stage, '/L3Station/Controls/' + name).GetPrim()
            prim.GetReferences().AddReference('./' + name + '.usda')
            xf = UsdGeom.Xformable(prim)
            xf.AddTranslateOp().Set(Gf.Vec3d(*position))
            xf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 180))
            layout[name] = {'kind': kind, 'position': position}
    stage.GetRootLayer().Save()
    (root / 'station_all_layout.json').write_text(json.dumps(layout, indent=2) + '\n')
    return path


def expand(station, asset_root):
    import numpy as np
    from pxr import UsdGeom, Gf, Sdf
    from l3_controls_runtime import attach
    root = Path(asset_root).resolve()
    executor, stage = station.executor, station.runtime.stage
    expected = {v['id'] for v in json.loads((root / 'catalog.json').read_text())}
    if set(station.runtime.devices) == expected:
        station.show_ui()
        station.all_view()
        return station
    path = root / 'station_all.usda'
    if not path.exists():
        build(root)
    if station.queue:
        raise RuntimeError('请先结束当前机构负载，再扩展工位')
    if hasattr(executor, '_operation_demo'):
        executor._operation_demo.stop()
    stage.GetRootLayer().Export(str(station.output / 'scene_before_all.usda'))
    robot = executor.articulation
    robot_q = np.asarray(robot.get_joint_positions()).copy()
    pos, quat = [np.asarray(v).copy() for v in robot.get_world_poses()]
    if robot_q.size != 50 or not np.isfinite(robot_q).all():
        raise RuntimeError('人形物理读回异常，未修改场景')
    robot.set_default_state(positions=pos, orientations=quat)
    robot.set_joints_default_state(positions=robot_q, velocities=np.zeros_like(robot_q))
    previous = getattr(executor, '_oxford_devices', None)
    old_q = {name: np.asarray(view.get_joint_positions()).copy()
             for name, view in station.runtime.devices.items()}
    ox_q = {name: np.asarray(view.get_joint_positions()).copy()
            for name, view in previous.devices.items()} if previous else {}
    executor.world.remove_physics_callback('l3_test_load')
    executor.world.remove_physics_callback('l3_controls')
    if previous:
        executor.world.remove_physics_callback('oxford_device_states')
    executor._motion = executor._auto_pick = executor._base_cmd = executor._wheel_pos_target = None
    executor.world.stop()
    # 旧路径保持不变，新组合层引用原三件工位，便于延用接触代码与恢复关节状态。
    stage.GetPrimAtPath('/World/L3Station').GetReferences().SetReferences([Sdf.Reference(str(path))])
    panel = stage.GetPrimAtPath('/World/OperationPanel')
    if panel:
        xf = UsdGeom.Xformable(panel)
        op = next((v for v in xf.GetOrderedXformOps()
                   if v.GetOpName() == 'xformOp:translate:l3_clearance'), None)
        if op is None:
            op = xf.AddTranslateOp(opSuffix='l3_clearance')
        op.Set(Gf.Vec3d(0, -1.65, 0))
    executor.world.reset()
    robot._on_physics_ready(None)
    robot.set_world_poses(positions=pos, orientations=quat)
    robot.set_joint_positions(robot_q)
    robot.set_joint_velocities(np.zeros_like(robot_q))
    robot.set_joint_position_targets(robot_q)
    robot.set_joint_velocity_targets(np.zeros_like(robot_q))
    # 扩展工位重置物理后不能丢掉颈部参考阻尼，否则静止头部会重新抽动。
    executor._tune_neck_drives()
    executor._mimic_targets = None
    if previous:
        for name, view in previous.devices.items():
            view._on_physics_ready(None)
            view.set_joint_positions(ox_q[name])
            view.set_joint_velocities(np.zeros_like(ox_q[name]))
        executor.world.add_physics_callback('oxford_device_states', callback_fn=previous.step)
    station.runtime = attach(executor.world, stage, root, '/World/L3Station/Controls')
    for name, q in old_q.items():
        view = station.runtime.devices[name]
        view.set_joint_positions(q)
        view.set_joint_velocities(np.zeros_like(q))
    station.selected = 'object_0_variant'
    station.completed = []
    executor.world.add_physics_callback('l3_test_load', callback_fn=station.step)
    for _ in range(240):
        executor.step(render=True)
    result = {'asset_count': len(station.runtime.devices), 'assets': list(station.runtime.devices),
              'expected_assets_present': set(station.runtime.devices) == expected,
              'finite_robot': bool(np.isfinite(robot.get_joint_positions()).all()),
              'finite_jacobian': bool(np.isfinite(robot._physics_view.get_jacobians()).all()),
              'robot_dof': int(np.asarray(robot.get_joint_positions()).size),
              'robot_base_delta_m': float(np.linalg.norm(np.asarray(robot.get_world_poses()[0]) - pos))}
    (station.output / 'all_assets_install.json').write_text(json.dumps(result, indent=2))
    if not result['expected_assets_present'] or not result['finite_robot'] or not result['finite_jacobian']:
        raise RuntimeError('全资产工位加载检查未通过，请查看all_assets_install.json')
    station.write()
    station.show_ui()
    station.all_view()
    stage.GetRootLayer().Export(str(station.output / 'scene_all_assets.usda'))
    return station
