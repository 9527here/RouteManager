#!/usr/bin/env python3
"""以真实物理读回检查 L3 弹簧、档位、螺旋耦合与联锁；不传送关节到目标。"""
import argparse
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('asset_root', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cycles', type=int, default=5)
    parser.add_argument('--property-checks', action='store_true')
    parser.add_argument('--solver', choices=['TGS', 'PGS'], default='TGS')
    args = parser.parse_args()
    root, output = args.asset_root.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'width': 1600, 'height': 1000,
                         'renderer': 'RayTracedLighting'})
    result = {'method': 'physical generalized force/torque, zero teleport; provisional reference parameters', 'solver': args.solver, 'tests': []}
    try:
        import numpy as np
        import omni.usd
        from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf
        from isaacsim.core.api import World
        from l3_controls_runtime import attach
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        omni.usd.get_context().open_stage(str(root / 'gallery.usda'))
        for _ in range(20):
            app.update()
        stage = omni.usd.get_context().get_stage()
        world = World(stage_units_in_meters=1., physics_dt=1 / 240, rendering_dt=1 / 60)
        scene = UsdPhysics.Scene.Get(stage, '/physicsScene')
        if not scene:
            scene = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
        px = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        px.CreateSolverTypeAttr(args.solver)
        px.CreateEnableStabilizationAttr(True)
        px.CreateTimeStepsPerSecondAttr(240)
        world.reset()
        runtime = attach(world, stage, root)
        dt = 1 / 240

        def dump():
            result['snapshot'] = runtime.snapshot()
            (output / 'validation.json').write_text(json.dumps(result, indent=2) + '\n')

        def run(seconds, device=None, force=None):
            for _ in range(round(seconds / dt)):
                if device:
                    view = runtime.devices[device]
                    view.set_joint_efforts(np.array([[force.get(j, 0.) for j in view.dof_names]], dtype=np.float32))
                world.step(render=False)
            if device:
                runtime.devices[device].set_joint_efforts(np.zeros((1, len(runtime.devices[device].dof_names)), dtype=np.float32))
            runtime.step(dt)
            return json.loads(json.dumps(runtime.states[device])) if device else runtime.snapshot()

        def rotate(device, joint, target, seconds=4., max_torque=.06):
            view = runtime.devices[device]
            index = view.dof_names.index(joint)
            reached = False
            for _ in range(round(seconds / dt)):
                q = float(np.asarray(view.get_joint_positions()).reshape(-1)[index])
                error = target - q
                if abs(error) < .025:
                    reached = True
                    break
                efforts = np.zeros((1, len(view.dof_names)), dtype=np.float32)
                if joint.endswith('_turn') and joint != 'selector_turn':
                    efforts[0, index] = np.clip(.05 * error, -max_torque, max_torque)
                else:
                    efforts[0, index] = math.copysign(max_torque, error)
                view.set_joint_efforts(efforts)
                world.step(render=False)
            view.set_joint_efforts(np.zeros((1, len(view.dof_names)), dtype=np.float32))
            run(.8)
            state = json.loads(json.dumps(runtime.states[device]))
            state['fixture_reached'] = reached
            return state

        run(3.)
        result['initial'] = runtime.snapshot()
        result['runtime_gains'] = {n: [np.asarray(x).tolist() for x in v.get_gains()]
                                   for n, v in runtime.devices.items()}
        if args.property_checks:
            checks = {'force_curve': [], 'single_nut_interlock': {}, 'pass': False}
            p = runtime.profile['button']
            spring = (p['switch_force_N'] - p['preload_N']) / p['switch_travel_m']
            for force in [1., 2., 3.8, 5.6]:
                # 定量力-位移用刚体外力；关节actuation与隐式驱动同时使用不作标定依据。
                view = runtime.devices['object_0_variant']
                loads = np.zeros((1, len(view.body_names), 3), dtype=np.float32)
                loads[0, view.body_names.index('Cap'), 0] = -force
                samples = []
                for step in range(480):
                    view._physics_view.apply_forces_and_torques_at_position(loads, None, None,
                                                                          np.array([0], dtype=np.uint32), True)
                    world.step(render=False)
                    if step >= 360:
                        samples.append([-float(np.asarray(view.get_joint_positions()).reshape(-1)[0]),
                                        float(np.asarray(view.get_joint_velocities()).reshape(-1)[0])])
                runtime.step(dt)
                state = dict(runtime.states['object_0_variant'])
                expected = float(np.clip((force - p['preload_N']) / spring, 0., p['stroke_m']))
                checks['force_curve'].append({'applied_N': force, 'expected_m': expected,
                    'measured_m': state['stroke_m'], 'range_m': [min(v[0] for v in samples), max(v[0] for v in samples)],
                    'last_velocity_m_s': samples[-1][1],
                    'pass': abs(state['stroke_m'] - expected) < .00015})
                run(1., 'object_0_variant', {})
            upper = rotate('object_14', 'upper_turn', math.pi * 4.4, 6., .12)
            blocked = run(.5, 'object_14', {'bridge_swing': .08})
            checks['single_nut_interlock'] = {'upper_loose': upper, 'attempt_swing': blocked,
                'pass': upper['upper_turns'] > 2. and blocked['lower_turns'] < .02
                        and not blocked['unlocked'] and abs(blocked['angle_rad']) < .025}
            checks['pass'] = all(c['pass'] for c in checks['force_curve']) and checks['single_nut_interlock']['pass']
            (output / 'property_checks.json').write_text(json.dumps(checks, indent=2) + '\n')
            print('PROPERTY_CHECKS', checks['pass'], flush=True)
            if not checks['pass']:
                raise RuntimeError('参考物理属性检查未通过')
            return
        for name, item in runtime.catalog.items():
            for cycle in range(args.cycles):
                if item['kind'] == 'button':
                    pressed = run(.5, name, {'button_slide': -8.})
                    released = run(1., name, {})
                    passed = pressed['pressed'] and pressed['indicator_on'] and not released['pressed'] and released['stroke_m'] < .0002
                    row = {'id': name, 'kind': 'button', 'cycle': cycle + 1, 'pressed': pressed, 'released': released, 'pass': bool(passed)}
                elif item['kind'] == 'selector':
                    records = []
                    values = list(np.radians(item['detents_deg']))
                    for target in [values[-1], values[0], 0.]:
                        records.append(rotate(name, 'selector_turn', target, max_torque=.10))
                    errors = [abs(r['angle_rad'] - v) for r, v in zip(records, [values[-1], values[0], 0.])]
                    row = {'id': name, 'kind': 'selector', 'cycle': cycle + 1, 'states': records,
                           'pass': bool(max(errors) < math.radians(3.))}
                else:
                    blocked = run(.4, name, {'bridge_swing': .08})
                    upper = rotate(name, 'upper_turn', math.pi * 4.4, seconds=6., max_torque=.12)
                    lower = rotate(name, 'lower_turn', math.pi * 4.4, seconds=6., max_torque=.12)
                    out = rotate(name, 'bridge_swing', math.pi / 2, seconds=4., max_torque=.03)
                    tighten_blocked = run(.4, name, {'upper_turn': -.08, 'lower_turn': -.08})
                    back = rotate(name, 'bridge_swing', 0., seconds=4., max_torque=.03)
                    close_upper = rotate(name, 'upper_turn', 0., seconds=6., max_torque=.12)
                    closed = rotate(name, 'lower_turn', 0., seconds=6., max_torque=.12)
                    pitch = runtime.profile['plate']['screw_pitch_m']
                    coupling_error = max(abs(lower['upper_axial_m'] - lower['upper_turns'] * pitch),
                                         abs(lower['lower_axial_m'] - lower['lower_turns'] * pitch))
                    passed = (abs(blocked['angle_rad']) < .025 and lower['unlocked']
                              and out['out'] and abs(out['angle_rad'] - math.pi / 2) < .05
                              and min(tighten_blocked['upper_turns'], tighten_blocked['lower_turns']) > 1.97
                              and closed['connected'] and coupling_error < .0001)
                    row = {'id': name, 'kind': 'plate', 'cycle': cycle + 1,
                           'blocked': blocked, 'upper_loose': upper, 'both_loose': lower,
                           'out': out, 'tighten_while_out': tighten_blocked,
                           'returned': back, 'upper_tight': close_upper, 'closed': closed,
                           'coupling_error_m': coupling_error, 'pass': bool(passed)}
                result['tests'].append(row)
                dump()
                print('TEST', name, cycle + 1, row['pass'], flush=True)
        viewport = get_active_viewport()
        viewport.set_texture_resolution((1600, 1000))
        set_camera_view(eye=[.8, -1.1, 1.60], target=[0, 0, 1.03])
        for _ in range(90):
            world.step(render=True)
        capture_viewport_to_file(viewport, str(output / 'gallery.png'))
        for _ in range(20):
            app.update()
        for name, item in runtime.catalog.items():
            for other in runtime.devices:
                imageable = UsdGeom.Imageable(stage.GetPrimAtPath('/World/Controls/' + other))
                imageable.MakeVisible() if other == name else imageable.MakeInvisible()
            position = item['gallery_position']
            size = .08 if item['kind'] == 'plate' else .048
            set_camera_view(eye=[position[0] + size * 2.4, position[1] - size * 1.3, position[2] + size * .8],
                            target=[position[0] + .006, position[1], position[2]])
            for _ in range(50):
                world.step(render=True)
            capture_viewport_to_file(viewport, str(output / (name + '.png')))
            for _ in range(12):
                app.update()
        result['pass'] = all(r['pass'] for r in result['tests']) and runtime.last_error is None
        dump()
        print('L3_VALIDATION_COMPLETE', result['pass'], flush=True)
        if not result['pass']:
            raise RuntimeError('物理机构验收未通过，见 validation.json')
    finally:
        app.close()


if __name__ == '__main__':
    main()
