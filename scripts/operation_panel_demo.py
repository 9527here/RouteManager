#!/usr/bin/env python3
"""在现有人形场景放置操作面板，并以明确标注的关节驱动演示机构。"""
import json
import time
from pathlib import Path


def install(executor, panel_asset, output):
    import numpy as np
    import omni.usd
    from isaacsim.core.utils.stage import add_reference_to_stage
    from oxford_device_runtime import attach_runtime
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    if hasattr(executor, '_operation_demo'):
        return executor._operation_demo
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath('/World/OperationPanel'):
        raise RuntimeError('面板已存在，请检查已有会话，不重复插入')
    robot = executor.articulation
    if robot is None:
        raise RuntimeError('需要当前项目的人形场景')
    q = np.asarray(robot.get_joint_positions()).copy()
    position, orientation = [np.asarray(v).copy() for v in robot.get_world_poses()]
    if not np.isfinite(q).all() or q.size != 50:
        raise RuntimeError('当前不是已验证的 50 DOF 人形，保持场景不变')
    snapshot = {'names': list(robot.dof_names), 'positions': q.tolist(),
                'base_position': position.tolist(), 'base_orientation': orientation.tolist(),
                'robot_prim_path': executor.robot_prim_path, 'base_z': executor._base_z}
    (output / 'robot_before.json').write_text(json.dumps(snapshot, indent=2))
    stage.GetRootLayer().Export(str(output / 'scene_before.usda'))
    stage.Flatten().Export(str(output / 'scene_before_flat.usdc'))
    # 不二次 load_robot。先保存 reset 默认值，再重建原实例的物理视图。
    robot.set_default_state(positions=position, orientations=orientation)
    robot.set_joints_default_state(positions=q, velocities=np.zeros_like(q))
    executor._motion = None
    executor._auto_pick = None
    executor._base_cmd = None
    executor._wheel_pos_target = None
    executor.world.stop()
    add_reference_to_stage(str(Path(panel_asset).resolve()), '/World/OperationPanel')
    executor.world.reset()
    robot._on_physics_ready(None)
    robot.set_world_poses(positions=position, orientations=orientation)
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros_like(q))
    robot.set_joint_position_targets(q)
    executor._mimic_targets = None
    executor._oxford_devices = attach_runtime(executor.world, stage, '/World/OperationPanel/Devices')
    for _ in range(180):
        executor.step(render=True)
    feedback = np.asarray(robot.get_joint_positions())
    links = np.asarray(robot._physics_view.get_link_transforms())
    jacobian = np.asarray(robot._physics_view.get_jacobians())
    if not all(np.isfinite(v).all() for v in [feedback, links, jacobian]):
        raise RuntimeError('重建后人形反馈不正常，禁止启动演示')
    validation = {'dof_count': feedback.size, 'finite_links': True, 'finite_jacobian': True,
                  'max_joint_drift_rad': float(np.max(np.abs(feedback - q))),
                  'robot_contact_demo': False, 'camera_count': len(executor._cameras)}
    (output / 'scene_validation.json').write_text(json.dumps(validation, indent=2))
    stage.GetRootLayer().Export(str(output / 'scene_with_panel.usda'))
    demo = OperationPanelDemo(executor, output)
    executor._operation_demo = demo
    demo.show_ui()
    demo.overview()
    return demo


class OperationPanelDemo:
    def __init__(self, executor, output):
        self.executor = executor
        self.output = Path(output)
        self.runtime = executor._oxford_devices
        self.running = False
        self.phase = 'Ready'
        self.phase_index = -1
        self.cycle = 0
        self.events = []
        self.saved_gains = {}
        self.last_ui = 0.0
        self.label = None
        self.window = None
        self.recording = False
        self.record_frames = 0
        self.last_record = 0.
        # 这些是展示的伺服目标，不是真人/人形接触，也不代表电气档位规格。
        self.segments = [
            ('Isolator: turn ON', 'isolator_switch', 4., {'switch_joint': 1.5707}),
            ('Isolator: hold ON', 'isolator_switch', 2., {'switch_joint': 1.5707}),
            ('Isolator: turn OFF', 'isolator_switch', 4., {'switch_joint': 0.}),
            ('Isolator: hold OFF', 'isolator_switch', 2., {'switch_joint': 0.}),
            ('Safety handle: ON', 'safety_switch', 4., {'switch_joint': -1.5707}),
            ('Safety handle: hold ON', 'safety_switch', 2., {'switch_joint': -1.5707}),
            ('Safety handle: OFF', 'safety_switch', 4., {'switch_joint': 0.}),
            ('Safety handle: hold OFF', 'safety_switch', 2., {'switch_joint': 0.}),
            ('E-stop: press', 'e_stop_button', 2., {'button_joint': -.005, 'button_twist_joint': 0.}),
            ('E-stop: latched', 'e_stop_button', 2., {'button_joint': -.005, 'button_twist_joint': 0.}),
            ('E-stop: twist to unlock', 'e_stop_button', 4., {'button_joint': -.005, 'button_twist_joint': -1.5}),
            ('E-stop: release', 'e_stop_button', 2., {'button_joint': 0., 'button_twist_joint': -1.5}),
            ('E-stop: twist back', 'e_stop_button', 3., {'button_joint': 0., 'button_twist_joint': 0.}),
            ('E-stop: reset', 'e_stop_button', 2., {'button_joint': 0., 'button_twist_joint': 0.}),
        ]

    def start(self, loop=True):
        import numpy as np
        if self.running:
            return
        self.loop = loop
        self.running = True
        self.phase_index = -1
        self.cycle = 0
        self.started = time.monotonic()
        self.segment_started = self.started
        self.last_targets = {n: dict(s['positions']) for n, s in self.runtime.samples.items()}
        for name, view in self.runtime.devices.items():
            self.saved_gains[name] = view.get_gains()
            p, d = [], []
            for joint in view.dof_names:
                linear = joint == 'button_joint'
                p.append(500. if linear else 5.)
                d.append(10. if linear else .5)
            view.set_gains(kps=np.array([p]), kds=np.array([d]))
        self.executor.world.add_physics_callback('operation_panel_demo', callback_fn=self.step)

    def stop(self):
        if not self.running:
            return
        self.running = False
        self.recording = False
        self.executor.world.remove_physics_callback('operation_panel_demo')
        for name, view in self.runtime.devices.items():
            p, d = self.saved_gains[name]
            view.set_gains(kps=p, kds=d)
        self.runtime.step(0.)
        self.phase = 'Stopped - source gains restored'
        self._write()

    def _write(self):
        result = {'phase': self.phase, 'cycle': self.cycle, 'running': self.running,
                  'method': 'joint drive demo; robot idle; no human contact',
                  'state': self.runtime.snapshot(), 'events': self.events[-80:]}
        (self.output / 'demo_status.json').write_text(json.dumps(result, indent=2))

    def overview(self):
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=[-2.2, -3.6, 2.05], target=[.55, -.47, .94])

    def closeup(self):
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=[-.15, -2.05, 1.52], target=[.91, -.95, 1.13])

    def estop_detail(self):
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=[.55, -1.48, 1.40], target=[.875, -1.18, 1.25])

    def record(self):
        if self.running:
            raise RuntimeError('先停止演示，再录制完整一轮')
        self.recording = True
        self.record_frames = 0
        self.last_record = 0.
        (self.output / 'frames').mkdir(exist_ok=True)
        self.start(loop=False)

    def capture(self, name='panel.png'):
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        capture_viewport_to_file(get_active_viewport(), str(self.output / name))

    def show_ui(self):
        import omni.ui as ui
        self.window = ui.Window('Operation panel demo', width=440, height=240)
        with self.window.frame:
            with ui.VStack(spacing=6):
                ui.Label('JOINT DRIVE DEMO - robot is idle', style={'color': 0xff77ddff})
                ui.Label('Breaker CAD is static. No robot contact verified.')
                self.label = ui.Label('Ready', word_wrap=True, height=65)
                with ui.HStack(height=32):
                    ui.Button('Start loop', clicked_fn=lambda: self.start(True))
                    ui.Button('Stop', clicked_fn=self.stop)
                with ui.HStack(height=32):
                    ui.Button('Overview', clicked_fn=self.overview)
                    ui.Button('Panel close-up', clicked_fn=self.closeup)
                    ui.Button('E-stop detail', clicked_fn=self.estop_detail)

    def step(self, dt):
        import math
        import numpy as np
        if not self.running:
            return
        now = time.monotonic()
        if self.phase_index < 0:
            self.phase_index = 0
            self.segment_started = now
            self.from_targets = dict(self.last_targets[self.segments[0][1]])
        label, device, duration, target = self.segments[self.phase_index]
        if now - self.segment_started >= duration:
            state = self.runtime.snapshot()['devices'][device]
            errors = {j: abs(state['positions'][j] - value) for j, value in target.items()}
            expected = None
            if 'hold ON' in label or label == 'E-stop: latched':
                expected = True
            elif 'hold OFF' in label or label in ('E-stop: reset', 'E-stop: twist to unlock'):
                expected = False
            good = all(e < (.0005 if j == 'button_joint' else .04) for j, e in errors.items())
            if expected is not None:
                good = good and state['activation_state'] == expected
            self.events.append({'phase': label, 'cycle': self.cycle + 1,
                                'state': json.loads(json.dumps(state)), 'pass': bool(good)})
            self.last_targets[device] = dict(target)
            self.phase_index += 1
            if self.phase_index >= len(self.segments):
                self.cycle += 1
                if not self.loop:
                    self.stop()
                    return
                self.phase_index = 0
            label, device, duration, target = self.segments[self.phase_index]
            self.from_targets = dict(self.last_targets[device])
            self.segment_started = now
        fraction = min((now - self.segment_started) / (duration * .8), 1.)
        smooth = fraction * fraction * (3. - 2. * fraction)
        goals = dict(self.last_targets[device])
        for joint, value in target.items():
            goals[joint] = self.from_targets[joint] + smooth * (value - self.from_targets[joint])
        for name, view in self.runtime.devices.items():
            values = goals if name == device else self.last_targets[name]
            view.set_joint_position_targets(np.array([[values[j] for j in view.dof_names]], dtype=np.float32))
        self.phase = label
        if now - self.last_ui > .2:
            state = self.runtime.snapshot()['devices']
            iso = math.degrees(state['isolator_switch']['positions']['switch_joint'])
            safety = math.degrees(state['safety_switch']['positions']['switch_joint'])
            push = -state['e_stop_button']['positions']['button_joint'] * 1000
            twist = math.degrees(state['e_stop_button']['positions']['button_twist_joint'])
            active = state['e_stop_button']['activation_state']
            if self.label is not None:
                self.label.text = (f'Cycle {self.cycle + 1}: {label}\n'
                                   f'Isolator {iso:.1f} deg | Safety {safety:.1f} deg\n'
                                   f'E-stop {push:.2f} mm / {twist:.1f} deg | active {active}')
            self._write()
            self.last_ui = now
        if self.recording and now - self.last_record > .4:
            self.capture(f'frames/{self.record_frames:04d}.png')
            self.record_frames += 1
            self.last_record = now


def main():
    import argparse
    import os
    import urllib.request
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['install', 'start', 'stop', 'overview', 'closeup', 'estop', 'status'])
    parser.add_argument('--url', default=os.environ.get('ROUTE_MANAGER_URL', 'http://127.0.0.1:18765'))
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] / 'outputs/operation_panel')
    parser.add_argument('--once', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    code = f'import sys\nsys.path.insert(0,{str(root / "scripts")!r})\nimport operation_panel_demo\n'
    if args.action == 'install':
        code += f'operation_panel_demo.install(executor,{str(root / "assets/operation_panel.usda")!r},{str(args.output.resolve())!r})\n'
    elif args.action == 'start':
        code += f'executor._operation_demo.start(loop={not args.once!r})\n'
    elif args.action in ('stop', 'overview', 'closeup', 'estop'):
        method = 'estop_detail' if args.action == 'estop' else args.action
        code += f'executor._operation_demo.{method}()\n'
    code += 'executor._operation_demo.runtime.step(0.)\nexecutor._operation_demo._write()\n'
    request = urllib.request.Request(args.url.rstrip('/') + '/python',
                                     data=json.dumps({'code': code}).encode(),
                                     headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=180) as response:
        result = json.load(response)
    print(json.dumps(result, ensure_ascii=False))
    status = args.output.resolve() / 'demo_status.json'
    if status.exists():
        data = json.loads(status.read_text())
        print(data['phase'], 'running=' + str(data['running']))


if __name__ == '__main__':
    main()
