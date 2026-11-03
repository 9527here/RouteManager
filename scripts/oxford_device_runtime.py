#!/usr/bin/env python3
"""Oxford 机构状态机；所有 Isaac 读写由调用方在仿真主线程执行。"""
class DeviceState:
    """按上游阈值迁移，关节用名称映射，避免 URDF 导入后顺序变化。"""

    def __init__(self, kind, active=False):
        if kind not in ('e_stop_button', 'isolator_switch', 'safety_switch'):
            raise ValueError(kind)
        self.kind = kind
        self.active = bool(active)
        self.transitions = 0

    def update(self, positions):
        before = self.active
        if self.kind == 'e_stop_button':
            push, twist = positions['button_joint'], positions['button_twist_joint']
            if push < -0.0045 and twist > -1.047 and not self.active:
                self.active = True
            elif push < -0.0045 and twist < -1.047 and self.active:
                self.active = False
            targets = {'button_joint': -0.005 if self.active else 0.0,
                       'button_twist_joint': 0.0}
        else:
            q = positions['switch_joint']
            signed = q if self.kind == 'isolator_switch' else -q
            if signed > 1.3963 and not self.active:
                self.active = True
            elif signed < 0.1745 and self.active:
                self.active = False
            sign = 1 if self.kind == 'isolator_switch' else -1
            targets = {'switch_joint': sign * 1.5707 if self.active else 0.0}
        self.transitions += int(before != self.active)
        return targets


class OxfordDeviceRuntime:
    """可注册到 World 的物理步回调，不建立相机，不修改机器人控制。"""

    def __init__(self, devices):
        # devices: {设备名: 已初始化的 isaacsim.core.prims.Articulation}
        self.devices = devices
        self.states = {name: DeviceState(name) for name in devices}
        self.samples = {}
        self.last_error = None

    @staticmethod
    def _write_targets_if_changed(view, ordered):
        import numpy as np
        try:
            actual = np.asarray(view._physics_view.get_dof_position_targets())
        except Exception:
            actual = None
        if actual is not None and actual.shape == ordered.shape and np.array_equal(actual, ordered):
            return False
        # 相同目标由现有驱动持续保持；外部临时覆盖后必须再次写回状态机目标。
        view.set_joint_position_targets(ordered)
        return True

    def step(self, dt):
        import numpy as np
        for name, view in self.devices.items():
            values = np.asarray(view.get_joint_positions()).reshape(-1)
            if not np.isfinite(values).all():
                self.last_error = f'{name}: non-finite joint positions'
                raise RuntimeError(self.last_error)
            positions = dict(zip(view.dof_names, map(float, values)))
            targets = self.states[name].update(positions)
            ordered = np.array([[targets[j] for j in view.dof_names]], dtype=np.float32)
            self._write_targets_if_changed(view, ordered)
            self.samples[name] = {'positions': positions,
                                  'activation_state': self.states[name].active,
                                  'transitions': self.states[name].transitions,
                                  'targets': targets}

    def snapshot(self):
        return {'devices': self.samples, 'last_error': self.last_error,
                'force_calibrated': False, 'human_contact_verified': False}


def attach_runtime(world, stage, prefix='/World/Devices'):
    """资产须在机器人前加载；这里只包装已存在的 articulation，不添加 prim。"""
    from pxr import Usd, UsdPhysics
    from isaacsim.core.prims import Articulation
    devices = {}
    for name in ('e_stop_button', 'isolator_switch', 'safety_switch'):
        root = stage.GetPrimAtPath(f'{prefix}/{name}')
        if not root:
            continue
        roots = [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if len(roots) != 1:
            raise RuntimeError(f'{name} articulation root 数量错误：{len(roots)}')
        view = Articulation(str(roots[0].GetPath()), name=f'oxford_{name}')
        view.initialize()
        devices[name] = view
    if not devices:
        raise RuntimeError(f'{prefix} 下没有 Oxford 机构')
    runtime = OxfordDeviceRuntime(devices)
    world.add_physics_callback('oxford_device_states', callback_fn=runtime.step)
    return runtime
