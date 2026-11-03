#!/usr/bin/env python3
"""L3 参考机构的物理反馈/灯/档位/压板联锁，只在仿真主线程运行。"""
import json
import math
from pathlib import Path


class L3Controls:
    def __init__(self, stage, devices, catalog, profile, prefix):
        self.stage, self.devices, self.catalog = stage, devices, catalog
        self.profile, self.prefix = profile, prefix
        self.states = {}
        self.transitions = {name: 0 for name in devices}
        self.previous = {}
        self.last_error = None

    @staticmethod
    def _write_targets_if_changed(view, ordered):
        import numpy as np
        try:
            actual = np.asarray(view._physics_view.get_dof_position_targets())
        except Exception:
            # 兼容没有目标读取接口的后端，继续原来的写入；setter失败仍正常抛出。
            actual = None
        if actual is not None and actual.shape == ordered.shape and np.array_equal(actual, ordered):
            return False
        # 核对实际目标而非只记上次写入，确保外部操作覆盖或重建后能恢复参考阻力。
        view.set_joint_position_targets(ordered)
        return True

    def step(self, dt):
        import numpy as np
        from pxr import UsdShade, UsdLux, Gf
        for name, view in self.devices.items():
            item = self.catalog[name]
            q = dict(zip(view.dof_names, map(float, np.asarray(view.get_joint_positions()).reshape(-1))))
            if not all(math.isfinite(v) for v in q.values()):
                self.last_error = name + ': non-finite feedback'
                raise RuntimeError(self.last_error)
            target = {}
            if item['kind'] == 'button':
                p = self.profile['button']
                stroke = -q['button_slide']
                old = bool(self.states.get(name, {}).get('pressed', False))
                pressed = stroke >= (p['switch_travel_m'] * (.8 if old else 1.))
                spring = (p['switch_force_N'] - p['preload_N']) / p['switch_travel_m']
                target['button_slide'] = p['preload_N'] / spring
                state = {'pressed': pressed, 'indicator_on': pressed, 'stroke_m': stroke}
                marker = pressed
                if name not in self.previous or old != pressed:
                    path = self.prefix + '/' + name
                    shader = UsdShade.Shader(self.stage.GetPrimAtPath(path + item['indicator_shader'].removeprefix('/Asset')))
                    color = (.1, 2., .1) if item['source_file'] in ('object_0.glb', 'object_0 (1).glb') else ((2., .1, .1) if item['source_file'] == 'object_1.glb' else (1.5, 1.5, 1.5))
                    shader.GetInput('emissiveColor').Set(Gf.Vec3f(*(color if pressed else (0, 0, 0))))
                    light = UsdLux.SphereLight(self.stage.GetPrimAtPath(path + item['indicator_light'].removeprefix('/Asset')))
                    light.GetIntensityAttr().Set(10. if pressed else 0.)
                    light.GetColorAttr().Set(Gf.Vec3f(*color))
            elif item['kind'] == 'selector':
                angle = q['selector_turn']
                values = np.radians(item['detents_deg'])
                index = int(np.argmin(abs(values - angle)))
                target['selector_turn'] = float(values[index])
                settled = abs(angle - values[index]) < math.radians(3.)
                state = {'angle_rad': angle, 'detent_index': index if settled else None,
                         'nearest_detent': index, 'settled': bool(settled)}
                marker = state['detent_index']
            else:
                p = self.profile['plate']
                release = p['release_turns'] * 2 * math.pi
                turn = [q['upper_turn'], q['lower_turn']]
                slide = [q['upper_slide'], q['lower_slide']]
                unlocked = all(a > release - .10 and x > p['release_turns'] * p['screw_pitch_m'] - .0001
                               for a, x in zip(turn, slide))
                out = abs(q['bridge_swing']) > math.radians(5.)
                limits = np.asarray(view.get_dof_limits()).copy()
                for joint in ['upper_turn', 'lower_turn']:
                    limits[0, view.dof_names.index(joint)] = [release - .10 if out else 0., p['max_turns'] * 2 * math.pi]
                # tensor 的相等上下限不能可靠表达锁止；保留0.006度的有限间隙。
                limits[0, view.dof_names.index('bridge_swing')] = [0., math.radians(p['swing_deg']) if unlocked or out else .0001]
                key = (bool(unlocked), bool(out))
                if key != getattr(self, '_limits_' + name, None):
                    view._physics_view.set_dof_limits(limits.astype(np.float32), np.array([0], dtype=np.uint32))
                    setattr(self, '_limits_' + name, key)
                for key_name in ['upper', 'lower']:
                    target[key_name + '_slide'] = max(0., q[key_name + '_turn']) * p['screw_pitch_m'] / (2 * math.pi)
                target['upper_turn'] = target['lower_turn'] = target['bridge_swing'] = 0.
                connected = abs(q['bridge_swing']) < math.radians(3.) and max(turn) < .12
                state = {'upper_turns': turn[0] / (2 * math.pi), 'lower_turns': turn[1] / (2 * math.pi),
                         'upper_axial_m': slide[0], 'lower_axial_m': slide[1],
                         'angle_rad': q['bridge_swing'], 'unlocked': bool(unlocked),
                         'connected': bool(connected), 'out': bool(out)}
                marker = (state['connected'], state['out'], state['unlocked'])
            if name in self.previous and marker != self.previous[name]:
                self.transitions[name] += 1
            self.previous[name] = marker
            ordered = np.array([[target[j] for j in view.dof_names]], dtype=np.float32)
            self._write_targets_if_changed(view, ordered)
            state.update(positions=q, transitions=self.transitions[name], kind=item['kind'])
            self.states[name] = state

    def snapshot(self):
        return {'controls': json.loads(json.dumps(self.states)), 'last_error': self.last_error,
                'calibrated_to_user_hardware': False, 'thread_contact_model': 'equivalent helical coupling'}


def attach(world, stage, asset_root, prefix='/World/Controls'):
    from pxr import UsdPhysics, Usd
    from isaacsim.core.prims import Articulation
    root = Path(asset_root)
    items = json.loads((root / 'catalog.json').read_text())
    profile = json.loads((root / 'reference_profile.json').read_text())
    devices, selected = {}, {}
    for item in items:
        path = prefix + '/' + item['id']
        prim = stage.GetPrimAtPath(path)
        if not prim:
            continue
        roots = [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if len(roots) != 1:
            raise RuntimeError(f'{path} articulation root 数量错误')
        view = Articulation(str(roots[0].GetPath()), name='l3_' + item['id'])
        view.initialize()
        devices[item['id']] = view
        selected[item['id']] = item
    if not devices:
        raise RuntimeError('没有找到物理版 L3 机构')
    runtime = L3Controls(stage, devices, selected, profile, prefix)
    world.add_physics_callback('l3_controls', callback_fn=runtime.step)
    return runtime
