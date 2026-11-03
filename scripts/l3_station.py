#!/usr/bin/env python3
"""给现有人形场景安装 L3 参考机构，并提供受力测试按钮。"""
import argparse
import json
import math
from pathlib import Path
from l3_action_plugin import ChineseActionsMixin


def build_station(asset_root):
    from pxr import Usd, UsdGeom, UsdPhysics, Gf
    root = Path(asset_root)
    path = root / 'station.usda'
    stage = Usd.Stage.CreateNew(str(path))
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, '/L3Station').GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, 'Z')
    parts = [('Board', [.825, -.49, 1.075], [.025, .46, .43]),
             ('Leg', [.85, -.49, .43], [.045, .045, .86]),
             ('Foot', [.85, -.49, .015], [.30, .34, .03])]
    for name, position, size in parts:
        cube = UsdGeom.Cube.Define(stage, '/L3Station/' + name)
        cube.CreateSizeAttr(1.)
        cube.CreateDisplayColorAttr([(.22, .28, .32)])
        xf = UsdGeom.Xformable(cube)
        xf.AddTranslateOp().Set(Gf.Vec3d(*position))
        xf.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    layout = {'object_0_variant': [.78, -.40, 1.035],
              'object_3': [.78, -.40, 1.20], 'object_14': [.78, -.60, 1.09]}
    for name, position in layout.items():
        prim = UsdGeom.Xform.Define(stage, '/L3Station/Controls/' + name).GetPrim()
        prim.GetReferences().AddReference('./' + name + '.usda')
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(*position))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 180))
    stage.GetRootLayer().Save()
    return path


def install(executor, asset_root, output):
    import numpy as np
    import omni.usd
    from isaacsim.core.utils.stage import add_reference_to_stage
    from l3_controls_runtime import attach
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    if hasattr(executor, '_l3_station'):
        return executor._l3_station
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath('/World/L3Station'):
        raise RuntimeError('L3Station已存在，请先检查，不能重复插入')
    if hasattr(executor, '_operation_demo'):
        executor._operation_demo.stop()
    # 毫米行程和小惯量螺母需要与独立验收一致的240Hz，60Hz会跨过锁紧终点。
    executor.world.set_simulation_dt(physics_dt=1 / 240, rendering_dt=1 / 60)
    robot = executor.articulation
    q = np.asarray(robot.get_joint_positions()).copy()
    pos, quat = [np.asarray(v).copy() for v in robot.get_world_poses()]
    if q.size != 50 or not np.isfinite(q).all():
        raise RuntimeError('需要已验证的 RM65 50 DOF 人形')
    (output / 'robot_before.json').write_text(json.dumps({'names': list(robot.dof_names), 'positions': q.tolist(),
        'base_position': pos.tolist(), 'base_orientation': quat.tolist()}, indent=2))
    stage.GetRootLayer().Export(str(output / 'scene_before.usda'))
    stage.Flatten().Export(str(output / 'scene_before_flat.usdc'))
    robot.set_default_state(positions=pos, orientations=quat)
    robot.set_joints_default_state(positions=q, velocities=np.zeros_like(q))
    executor._motion = executor._auto_pick = executor._base_cmd = executor._wheel_pos_target = None
    previous = getattr(executor, '_oxford_devices', None)
    previous_q = {name: np.asarray(view.get_joint_positions()).copy() for name, view in previous.devices.items()} if previous else {}
    if previous:
        executor.world.remove_physics_callback('oxford_device_states')
    executor.world.stop()
    _clear_previous_panel(stage)
    add_reference_to_stage(str(Path(asset_root).resolve() / 'station.usda'), '/World/L3Station')
    from pxr import UsdPhysics, PhysxSchema
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            PhysxSchema.PhysxSceneAPI.Apply(prim).CreateSolverTypeAttr('TGS')
            PhysxSchema.PhysxSceneAPI.Apply(prim).CreateTimeStepsPerSecondAttr(240)
    executor.world.reset()
    robot._on_physics_ready(None)
    robot.set_world_poses(positions=pos, orientations=quat)
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros_like(q))
    robot.set_joint_position_targets(q)
    # World.reset会回写导入时缓存的低阻尼，须恢复已验证的颈部参考设置。
    executor._tune_neck_drives()
    executor._mimic_targets = None
    if previous:
        for name, view in previous.devices.items():
            view._on_physics_ready(None)
            view.set_joint_positions(previous_q[name])
            view.set_joint_velocities(np.zeros_like(previous_q[name]))
        executor.world.add_physics_callback('oxford_device_states', callback_fn=previous.step)
    runtime = attach(executor.world, stage, asset_root, '/World/L3Station/Controls')
    station = L3Station(executor, runtime, output)
    executor._l3_station = station
    executor.world.add_physics_callback('l3_test_load', callback_fn=station.step)
    for _ in range(120):
        executor.step(render=True)
    if not np.isfinite(robot._physics_view.get_jacobians()).all():
        raise RuntimeError('人形物理视图异常，禁止操作')
    station.show_ui()
    station.overview()
    stage.GetRootLayer().Export(str(output / 'scene_with_l3.usda'))
    station.write()
    return station


def _clear_previous_panel(stage):
    """旧Oxford大手柄伸出板面，错开整板避免进入L3操作包络。"""
    from pxr import UsdGeom, Gf
    prim = stage.GetPrimAtPath('/World/OperationPanel')
    if not prim:
        return
    xf = UsdGeom.Xformable(prim)
    name = 'xformOp:translate:l3_clearance'
    op = next((v for v in xf.GetOrderedXformOps() if v.GetOpName() == name), None)
    if op is None:
        op = xf.AddTranslateOp(opSuffix='l3_clearance')
    op.Set(Gf.Vec3d(0, -.55, 0))


class L3Station(ChineseActionsMixin):
    def __init__(self, executor, runtime, output):
        self.executor, self.runtime, self.output = executor, runtime, Path(output)
        self.queue = []
        self.elapsed = 0.
        self.completed = []
        self.label = None
        self.updates = 0
        self.selected = 'object_0_variant'

    def set_solver(self, solver='TGS'):
        import numpy as np
        from pxr import UsdPhysics, PhysxSchema
        if solver not in ('PGS', 'TGS'):
            raise ValueError(solver)
        self.stop()
        executor = self.executor
        old = getattr(executor, '_oxford_devices', None)
        groups = [(self.runtime, 'l3_controls')]
        if old:
            groups.append((old, 'oxford_device_states'))
        saved = {id(view): np.asarray(view.get_joint_positions()).copy()
                 for runtime, _ in groups for view in runtime.devices.values()}
        robot = executor.articulation
        q = np.asarray(robot.get_joint_positions()).copy()
        pos, quat = [np.asarray(x).copy() for x in robot.get_world_poses()]
        robot.set_default_state(positions=pos, orientations=quat)
        robot.set_joints_default_state(positions=q, velocities=np.zeros_like(q))
        executor.world.remove_physics_callback('l3_test_load')
        for _, callback in groups:
            executor.world.remove_physics_callback(callback)
        executor.world.stop()
        for prim in self.runtime.stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                PhysxSchema.PhysxSceneAPI.Apply(prim).CreateSolverTypeAttr(solver)
        executor.world.reset()
        robot._on_physics_ready(None)
        robot.set_world_poses(positions=pos, orientations=quat)
        robot.set_joint_positions(q)
        robot.set_joint_velocities(np.zeros_like(q))
        robot.set_joint_position_targets(q)
        executor._tune_neck_drives()
        executor._mimic_targets = None
        for runtime, callback in groups:
            for name, view in runtime.devices.items():
                view._on_physics_ready(None)
                view.set_joint_positions(saved[id(view)])
                view.set_joint_velocities(np.zeros_like(saved[id(view)]))
                if hasattr(runtime, '_limits_' + name):
                    delattr(runtime, '_limits_' + name)
            executor.world.add_physics_callback(callback, callback_fn=runtime.step)
        executor.world.add_physics_callback('l3_test_load', callback_fn=self.step)
        for _ in range(120):
            executor.step(render=True)
        result = {'solver': solver, 'finite_jacobian': bool(np.isfinite(robot._physics_view.get_jacobians()).all()),
                  'finite_positions': bool(np.isfinite(robot.get_joint_positions()).all()),
                  'dof_count': int(np.asarray(robot.get_joint_positions()).size)}
        (self.output / 'solver_validation.json').write_text(json.dumps(result, indent=2))
        if not result['finite_jacobian'] or not result['finite_positions']:
            raise RuntimeError('切换求解器后人形物理视图异常')
        self.runtime.stage.GetRootLayer().Export(str(self.output / 'scene_with_l3.usda'))

    def _fixture_stop(self):
        import numpy as np
        self.queue = []
        for view in self.runtime.devices.values():
            view.set_joint_efforts(np.zeros((1, len(view.dof_names)), dtype=np.float32))

    def _fixture_step(self, dt):
        import numpy as np
        if self.queue:
            name, mode, goal, duration = self.queue[0]
            view = self.runtime.devices[name]
            q = np.asarray(view.get_joint_positions()).reshape(-1)
            force = np.zeros_like(q)
            reached = False
            if mode == 'force':
                for j, value in goal.items():
                    force[view.dof_names.index(j)] = value
            else:
                joint, value = next(iter(goal.items()))
                index = view.dof_names.index(joint)
                error = value - q[index]
                reached = bool(abs(error) < .025)
                if joint in ('upper_turn', 'lower_turn'):
                    force[index] = np.clip(.05 * error, -.12, .12)
                elif joint == 'selector_turn':
                    # 恒定正反扭矩会让小惯量旋钮越过中间档并反复振荡；近目标减力。
                    force[index] = np.clip(.12 * error, -.1, .1)
                else:
                    force[index] = math.copysign(.03, error)
            self.elapsed += dt
            if mode == 'turn' and self.elapsed >= duration and not reached:
                self._fixture_stop()
                self._status = '动作未到位，已停止；请检查机构位置'
                self._action_history.append({**(self._action_job or {}), 'pass': False, 'reason': '转动超时'})
                self._action_job = None
                return
            if self.elapsed >= duration or reached:
                view.set_joint_efforts(np.zeros((1, len(q)), dtype=np.float32))
                self.completed.append({'device': name, 'mode': mode, 'goal': goal,
                                       'reached': reached, 'state': self.runtime.states[name]})
                self.queue.pop(0)
                self.elapsed = 0.
            else:
                view.set_joint_efforts(force.reshape(1, -1).astype(np.float32))
        self.updates += 1
        if self.updates % 30 == 0:
            self.write()
            if self.label:
                self._update_label()

    def focus(self):
        from pxr import UsdGeom
        from isaacsim.core.utils.viewports import set_camera_view
        name = getattr(self, 'selected', 'object_0_variant')
        prim = self.runtime.stage.GetPrimAtPath(self.runtime.prefix + '/' + name)
        p = UsdGeom.XformCache().GetLocalToWorldTransform(prim).ExtractTranslation()
        set_camera_view(eye=[p[0]-.25, p[1]-.10, p[2]+.10], target=[p[0]-.01, p[1], p[2]])

    def write(self):
        data = self.runtime.snapshot()
        data.update(test_load_active=bool(self.queue), completed=self.completed[-20:],
                    selected=getattr(self, 'selected', 'object_0_variant'),
                    selected_asset_name=self._asset_name('l3', getattr(self, 'selected', 'object_0_variant')),
                    action_status=getattr(self, '_status', '就绪'), robot_action_active=getattr(self, '_robot_busy', False),
                    approach_state=dict(self._approach.state) if getattr(self, '_approach', None) is not None else {},
                    pending_action=getattr(self, '_pending_action', None),
                    other_action_active=getattr(self, '_legacy_job', None) is not None,
                    collection_status=getattr(self, '_collect_status', '未开始采集'),
                    collection_output=str(getattr(self, '_collect_output', '') or ''),
                    action_history=getattr(self, '_action_history', [])[-30:])
        (self.output / 'live_status.json').write_text(json.dumps(data, indent=2))

    def overview(self):
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=[-1.6, -2.8, 1.8], target=[.55, -.35, 1.03])

    def closeup(self):
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=[.27, -.97, 1.38], target=[.78, -.49, 1.09])

    def all_view(self):
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=[-.45, -.98, 1.18], target=[.78, -.98, 1.08])



def main():
    """客户端只提交代码，所有 Isaac 访问仍在服务仿真主线程。"""
    import urllib.request
    import urllib.error
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['install', 'all', 'status', 'press', 'selector', 'plate_out',
                                         'plate_in', 'stop', 'overview', 'closeup', 'focus', 'robot_button', 'protect',
                                         'approach_selected', 'stop_base'])
    parser.add_argument('--url', default='http://127.0.0.1:18765')
    parser.add_argument('--cycles', type=int, default=3)
    parser.add_argument('--asset', help='资产ID，例如object_5、object_6_variant、object_13')
    args = parser.parse_args()
    project = Path(__file__).resolve().parent.parent
    # Kit 的导入缓存可能看不到新模块；显式加载本项目文件。
    code = f"""import sys, types
from pathlib import Path
project = Path({str(project)!r})
def load_l3_module(name):
    p = project / 'scripts' / (name + '.py')
    module = sys.modules.get(name) or types.ModuleType(name)
    module.__file__ = str(p)
    sys.modules[name] = module
    exec(compile(p.read_text(), str(p), 'exec'), module.__dict__)
    return module
runtime_module = load_l3_module('l3_controls_runtime')
load_l3_module('l3_asset_labels')
load_l3_module('l3_action_plugin')
station_module = load_l3_module('l3_station')
"""
    if args.action == 'install':
        code += "station_module.install(executor, project / 'assets/l3_operable', project / 'outputs/l3_operable/live')\n"
    code += "station = executor._l3_station\nstation.__class__ = station_module.L3Station\nstation.runtime.__class__ = runtime_module.L3Controls\n"
    code += "executor.world.remove_physics_callback('l3_test_load')\nexecutor.world.add_physics_callback('l3_test_load', callback_fn=station.step)\nexecutor.world.remove_physics_callback('l3_controls')\nexecutor.world.add_physics_callback('l3_controls', callback_fn=station.runtime.step)\n"
    if args.action in ('install', 'all'):
        code += "all_module = load_l3_module('l3_all_assets')\nall_module.expand(station, project / 'assets/l3_operable')\n"
    else:
        if args.asset:
            code += f"station.select({args.asset!r})\n"
        if args.action == 'protect':
            code += "load_l3_module('robot_collision_guard')\nprotection = load_l3_module('install_robot_collision_guard')\n"
            code += "protection.activate(executor, project / 'assets/robot_safety', project / 'outputs/collision_guard')\nstation.show_ui()\n"
        elif args.action == 'robot_button':
            code += f"station.start_robot(cycles={args.cycles})\n"
        elif args.action in ('press', 'selector', 'plate_out', 'plate_in'):
            code += f"station.command({args.action!r}, device={args.asset!r})\n"
        elif args.action == 'status':
            code += "station.runtime.step(0.)\nstation.write()\n"
        else:
            code += f"station.{args.action}()\n"
    request = urllib.request.Request(args.url.rstrip('/') + '/python',
        data=json.dumps({'code': code}).encode(), headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=600) as response:
            print(response.read().decode())
    except urllib.error.HTTPError as error:
        raise SystemExit(error.read().decode())
    if args.action == 'status':
        print((project / 'outputs/l3_operable/live/live_status.json').read_text())


if __name__ == '__main__':
    main()
