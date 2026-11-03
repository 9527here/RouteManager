#!/usr/bin/env python3
"""独立 PhysX 验证：受控关节加载、实际位置读回、源状态机与复位。

这里的加载器是测试夹具，不代表人形已经夹持或达到实物扭矩。
"""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('asset_root', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--diagnostic', action='store_true')
    args = parser.parse_args()
    root, output = args.asset_root.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'width': 1600, 'height': 1000,
                         'renderer': 'RayTracedLighting'})
    results = {'method': 'switches: temporary position drive; e-stop: joint effort fixture; no human contact', 'cycles': []}
    try:
        import numpy as np
        import carb
        import omni.usd
        from pxr import Usd, UsdGeom, UsdPhysics
        from isaacsim.core.api import World
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        from oxford_device_runtime import attach_runtime
        omni.usd.get_context().open_stage(str(root / 'gallery.usda'))
        for _ in range(30):
            app.update()
        world = World(physics_dt=1 / 120, rendering_dt=1 / 60, stage_units_in_meters=1.0)
        world.reset()
        stage = omni.usd.get_context().get_stage()
        runtime = attach_runtime(world, stage)
        motor_fixture = {}
        def apply_motor_fixture(dt):
            for name, targets in motor_fixture.items():
                view = runtime.devices[name]
                view.set_joint_position_targets(np.array([[targets[j] for j in view.dof_names]], dtype=np.float32))
        world.add_physics_callback('validation_motor_fixture', callback_fn=apply_motor_fixture)
        dt = 1 / 120
        viewport = get_active_viewport()
        viewport.set_texture_resolution((1600, 1000))
        carb.settings.get_settings().set('/rtx/post/tonemap/op', 4)
        carb.settings.get_settings().set('/rtx/post/tonemap/filmIso', 200.0)

        def run(seconds, name=None, goals=None):
            saved_gains = None
            if name and name != 'e_stop_button':
                view = runtime.devices[name]
                saved_gains = view.get_gains()
                view.set_gains(kps=np.array([[5.0]]), kds=np.array([[0.5]]))
                motor_fixture[name] = goals
            for _ in range(round(seconds / dt)):
                if name == 'e_stop_button':
                    view = runtime.devices[name]
                    q = np.asarray(view.get_joint_positions()).reshape(-1)
                    v = np.asarray(view.get_joint_velocities()).reshape(-1)
                    efforts = []
                    for i, joint in enumerate(view.dof_names):
                        target = goals[joint]
                        linear = joint == 'button_joint'
                        kp, kd, limit = (5000, 20, 10) if linear else (10, 0.8, 3)
                        efforts.append(float(np.clip(kp * (target - q[i]) - kd * v[i], -limit, limit)))
                    view.set_joint_efforts(np.array([efforts], dtype=np.float32))
                world.step(render=False)
            if name:
                runtime.devices[name].set_joint_efforts(np.zeros((1, len(goals)), dtype=np.float32))
            if saved_gains is not None:
                motor_fixture.pop(name)
                view.set_gains(kps=saved_gains[0], kds=saved_gains[1])
            runtime.step(dt)
            return json.loads(json.dumps(runtime.snapshot()))

        def capture(filename, eye, target):
            set_camera_view(eye=eye, target=target)
            for _ in range(80):
                world.step(render=True)
            capture_viewport_to_file(viewport, str(output / filename))
            for _ in range(15):
                app.update()

        run(3)
        results['initial'] = runtime.snapshot()
        results['gains'] = {name: [np.asarray(x).tolist() for x in view.get_gains()]
                            for name, view in runtime.devices.items()}
        results['base_initial'] = {name: [np.asarray(x).tolist() for x in view.get_world_poses()]
                                   for name, view in runtime.devices.items()}
        if args.diagnostic:
            diagnostics = {}
            for name in ('isolator_switch', 'safety_switch'):
                view = runtime.devices[name]
                rows = []
                for mode in ('normal', 'no_drive'):
                    if mode == 'no_drive':
                        view.set_gains(kps=np.zeros((1, 1)), kds=np.zeros((1, 1)))
                    sign = 1 if name == 'isolator_switch' else -1
                    for tick in range(240):
                        view.set_joint_efforts(np.array([[sign * 1.5]], dtype=np.float32))
                        world.step(render=False)
                        if tick % 30 == 0:
                            rows.append({'mode': mode, 'tick': tick,
                                'q': np.asarray(view.get_joint_positions()).tolist(),
                                'v': np.asarray(view.get_joint_velocities()).tolist(),
                                'efforts': np.asarray(view.get_applied_joint_efforts()).tolist(),
                                'measured': np.asarray(view.get_measured_joint_efforts()).tolist(),
                                'limits': np.asarray(view.get_dof_limits()).tolist(),
                                'gains': [np.asarray(a).tolist() for a in view.get_gains()]})
                diagnostics[name] = rows
            (output / 'diagnostic.json').write_text(json.dumps(diagnostics, indent=2))
            print('DIAGNOSTIC_COMPLETE', flush=True)
            return
        capture('gallery.png', [0.80, -1.20, 1.25], [0.09, 0, 0.87])
        for name, sign in [('isolator_switch', 1), ('safety_switch', -1)]:
            for cycle in range(3):
                on = run(4, name, {'switch_joint': sign * 1.5707})['devices'][name]
                hold_on = run(4)['devices'][name]
                off = run(4, name, {'switch_joint': 0.0})['devices'][name]
                hold_off = run(4)['devices'][name]
                results['cycles'].append({'id': name, 'cycle': cycle + 1,
                    'on': on, 'hold_on': hold_on, 'off': off, 'hold_off': hold_off,
                    'pass': on['activation_state'] and hold_on['activation_state']
                            and not off['activation_state'] and not hold_off['activation_state']
                            and abs(on['positions']['switch_joint'] - sign * 1.5707) < 0.035
                            and abs(off['positions']['switch_joint']) < 0.035})
                print('CYCLE', name, cycle + 1, results['cycles'][-1]['pass'], flush=True)
        for cycle in range(3):
            name = 'e_stop_button'
            pressed = run(2, name, {'button_joint': -0.005, 'button_twist_joint': 0.0})['devices'][name]
            latched = run(3)['devices'][name]
            if cycle == 0:
                capture('e_stop_pressed.png', [-0.05, -0.24, 1.04], [-0.20, -0.04, 0.9])
            released = run(3, name, {'button_joint': -0.005, 'button_twist_joint': -1.50})['devices'][name]
            reset = run(12)['devices'][name]
            results['cycles'].append({'id': name, 'cycle': cycle + 1,
                'pressed': pressed, 'latched': latched, 'released': released, 'reset': reset,
                'pass': pressed['activation_state'] and latched['activation_state']
                        and not released['activation_state'] and not reset['activation_state']
                        and reset['positions']['button_joint'] > -0.0005
                        and abs(reset['positions']['button_twist_joint']) < 0.035})
            print('CYCLE', name, cycle + 1, results['cycles'][-1]['pass'], flush=True)
        for name, eye, target in [
                ('e_stop_button', [-0.05, -0.24, 1.04], [-0.20, -0.04, 0.9]),
                ('isolator_switch', [0.14, -0.25, 1.04], [0, -0.02, 0.9]),
                ('safety_switch', [0.82, -0.45, 1.1], [0.33, 0, 0.9])]:
            capture(name + '.png', eye, target)
        results['final'] = runtime.snapshot()
        results['base_final'] = {name: [np.asarray(x).tolist() for x in view.get_world_poses()]
                                 for name, view in runtime.devices.items()}
        results['pass'] = all(c['pass'] for c in results['cycles']) and runtime.last_error is None
        stage.GetRootLayer().Export(str(output / 'tested_scene.usda'))
        (output / 'validation.json').write_text(json.dumps(results, indent=2) + '\n')
        print('VALIDATION_COMPLETE', results['pass'], flush=True)
        if not results['pass']:
            raise RuntimeError('机构行为验收未通过，查看 validation.json')
    finally:
        app.close()


if __name__ == '__main__':
    main()
