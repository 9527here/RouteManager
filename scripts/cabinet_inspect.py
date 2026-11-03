#!/usr/bin/env python3
"""动力柜独立验收：通过 HTTP 在仿真主线程测量刚体实际运动。

先启动 scripts/run.sh，再执行本脚本 setup / exercise / capture。
所有 pxr/omni 导入留在运行时，系统 Python 客户端不依赖 Isaac。
"""
import argparse
import json
import math
from pathlib import Path
import sys
import urllib.error
import urllib.request
import uuid


class CabinetProbe:
    def __init__(self, executor, output):
        import omni.usd
        import omni.physx
        self.executor = executor
        self.stage = omni.usd.get_context().get_stage()
        self.iface = omni.physx.get_physx_interface()
        self.output = Path(output)
        self.controls = {}

    def setup(self, asset, repair=False, position=(0, 0, 0), yaw=0):
        from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, Gf
        if self.executor.articulation is not None:
            raise RuntimeError('机器人已加载；柜体验收需要独立空白服务，避免 reset 影响机器人')
        # 引用层只屏蔽资产自带的第二个物理场景，不写回原资产。
        wrapper = self.output / ('cabinet_repaired.usda' if repair else 'cabinet_test.usda')
        if wrapper.exists():
            wrapper = wrapper.with_stem(wrapper.stem + '_' + uuid.uuid4().hex[:8])
        scene = Usd.Stage.CreateNew(str(wrapper))
        root = scene.DefinePrim('/Cabinet', 'Xform')
        scene.SetDefaultPrim(root)
        UsdGeom.SetStageMetersPerUnit(scene, 1.0)
        UsdGeom.SetStageUpAxis(scene, UsdGeom.Tokens.z)
        root.GetReferences().AddReference(str(asset), '/World')
        root_xf = UsdGeom.Xformable(root)
        root_xf.AddTranslateOp().Set(Gf.Vec3d(*position))
        root_xf.AddRotateZOp().Set(float(yaw))
        scene.GetPrimAtPath('/Cabinet/PhysicsScene').SetActive(False)
        if repair:
            self._repair_scene(scene)
        scene.GetRootLayer().Save()
        del scene
        self.executor.world.stop()
        if self.stage.GetPrimAtPath('/World/Cabinet'):
            self.stage.RemovePrim('/World/Cabinet')
        self.executor.load_scene(str(wrapper), '/World/Cabinet')
        UsdLux.DomeLight.Define(self.stage, '/World/TestDome').CreateIntensityAttr(400)
        sun = UsdLux.DistantLight.Define(self.stage, '/World/TestSun')
        sun.CreateIntensityAttr(1000)
        sun_xf = UsdGeom.Xformable(sun)
        if not sun_xf.GetOrderedXformOps():
            sun_xf.AddRotateXYZOp().Set(Gf.Vec3f(20, -45, 0))
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath('/World/Cabinet')):
            if not prim.IsA(UsdPhysics.Joint):
                continue
            joint = UsdPhysics.Joint(prim)
            name = prim.GetName().removesuffix('_Joint')
            drive = 'linear' if prim.IsA(UsdPhysics.PrismaticJoint) else 'angular'
            self.controls[name] = {
                'joint': str(prim.GetPath()), 'kind': drive,
                'body0': str(joint.GetBody0Rel().GetTargets()[0]),
                'body1': str(joint.GetBody1Rel().GetTargets()[0]),
                'lower': prim.GetAttribute('physics:lowerLimit').Get(),
                'upper': prim.GetAttribute('physics:upperLimit').Get(),
            }
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=[2.5, 1.6, 1.65], target=[0, 0, 0.85])
        self.executor.world.reset()
        for _ in range(180):
            self.executor.step(render=True)
        self._write('inventory.json', self.controls)
        self.stage.GetRootLayer().Export(str(self.output / 'session.usda'))

    @staticmethod
    def _repair_scene(scene):
        from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
        cache = UsdGeom.XformCache()
        prims = list(scene.Traverse())
        # 这些部件是独立约束，不应把运动学固定端纳入 articulation。
        for prim in prims:
            for api in (UsdPhysics.ArticulationRootAPI, PhysxSchema.PhysxArticulationAPI):
                if prim.HasAPI(api):
                    prim.RemoveAPI(api)
        for prim in prims:
            if not prim.IsA(UsdPhysics.Joint):
                continue
            joint = UsdPhysics.Joint(prim)
            joint.CreateExcludeFromArticulationAttr(True)
            PhysxSchema.PhysxJointAPI.Apply(prim)
            b0 = scene.GetPrimAtPath(joint.GetBody0Rel().GetTargets()[0])
            b1 = scene.GetPrimAtPath(joint.GetBody1Rel().GetTargets()[0])
            m0, m1 = cache.GetLocalToWorldTransform(b0), cache.GetLocalToWorldTransform(b1)
            # 锁体内部还有两个刚体；reset stack 隔离父刚体，保留原世界位姿。
            parent = b1.GetParent()
            if parent.HasAPI(UsdPhysics.RigidBodyAPI):
                xf = UsdGeom.Xformable(b1)
                xf.ClearXformOpOrder()
                xf.AddTransformOp().Set(m1)
                xf.SetResetXformStack(True)
            # 原资产全部写零锚点，旋钮/锁体初始坐标并不重合，加载时会被强拉到一起。
            joint.CreateLocalPos0Attr(Gf.Vec3f(*m0.GetInverse().Transform(m1.ExtractTranslation())))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0))
            q0, q1 = m0.ExtractRotationQuat(), m1.ExtractRotationQuat()
            rel = q0.GetInverse() * q1
            joint.CreateLocalRot0Attr(Gf.Quatf(rel.GetReal(), Gf.Vec3f(*rel.GetImaginary())))
            joint.CreateLocalRot1Attr(Gf.Quatf(1))
        for prim in prims:
            if prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')

    def _write(self, filename, value):
        (self.output / filename).write_text(
            json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')

    def _pose(self, path):
        import numpy as np
        from pxr import Gf
        value = self.iface.get_rigidbody_transformation(path)
        if not value or not value.get('ret_val', False):
            raise RuntimeError('PhysX 刚体反馈不可用: ' + path)
        q = value['rotation']  # PhysX 返回 xyzw。
        return (np.array(value['position'], dtype=float),
                Gf.Quatd(float(q[3]), Gf.Vec3d(*map(float, q[:3]))))

    def _relative(self, info):
        from pxr import Gf
        p0, q0 = self._pose(info['body0'])
        p1, q1 = self._pose(info['body1'])
        rotation0 = Gf.Rotation(q0)
        displacement = rotation0.GetInverse().TransformDir(Gf.Vec3d(*(p1 - p0)))
        return float(displacement[0]), q0.GetInverse() * q1

    def _drive(self, name, target, frames=120):
        info = self.controls[name]
        if not info['lower'] <= target <= info['upper']:
            raise ValueError(f"{name} 目标超出资产限位 [{info['lower']}, {info['upper']}]")
        attr = self.stage.GetPrimAtPath(info['joint']).GetAttribute(
            f"drive:{info['kind']}:physics:targetPosition")
        attr.Set(float(target))
        for _ in range(frames):
            self.executor.step(render=True)

    def exercise(self, names=None, tag='original'):
        rows = []
        for name in names or sorted(self.controls):
            info = self.controls[name]
            try:
                self._drive(name, 0)
                x0, q0 = self._relative(info)
                targets = [info['lower'], 0.0] if info['kind'] == 'linear' else [45., -45., 0.]
                samples = []
                for cycle in range(2):
                    for target in targets:
                        self._drive(name, target)
                        x, q = self._relative(info)
                        if info['kind'] == 'linear':
                            actual = x - x0
                            tolerance = 0.0005
                        else:
                            delta = q0.GetInverse() * q
                            actual = math.degrees(2 * math.atan2(float(delta.GetImaginary()[0]), float(delta.GetReal())))
                            actual = (actual + 180) % 360 - 180
                            tolerance = 2.0
                        samples.append({'cycle': cycle + 1, 'target': target,
                                        'actual': actual, 'pass': abs(actual - target) < tolerance})
                rows.append({'name': name, 'kind': info['kind'], 'samples': samples,
                             'pass': all(s['pass'] for s in samples)})
            except Exception as exc:
                rows.append({'name': name, 'pass': False, 'error': str(exc)})
            self._write(f'results_{tag}.json', rows)
        return rows

    def capture(self, filename='cabinet.png'):
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        # 截取现有视口，不创建额外传感器或注册机器人相机。
        capture_viewport_to_file(get_active_viewport(), str(self.output / filename))
        for _ in range(20):
            self.executor.step(render=True)

    def light(self, name, on):
        from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
        if name not in [f'Btn{i:02d}' for i in range(1, 8)]:
            raise ValueError('灯名称必须为 Btn01~Btn07')
        info = self.controls[name]
        saved = getattr(self, '_lamp_bindings', {})
        colors = {'Btn03': (1., 0., 0.), 'Btn06': (1., 0., 0.),
                  'Btn04': (1., 0.7, 0.), 'Btn07': (1., 0.7, 0.)}
        color = colors.get(name, (0., 1., 0.))
        material = UsdShade.Material.Define(self.stage, f'/World/Cabinet/TestMaterials/{name}_On')
        shader = UsdShade.Shader.Define(self.stage, str(material.GetPath()) + '/Surface')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color) * 0.2)
        shader.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color) * 3)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(info['body1'])):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            key = str(prim.GetPath())
            binding = UsdShade.MaterialBindingAPI.Apply(prim)
            if key not in saved:
                original, _ = binding.ComputeBoundMaterial()
                saved[key] = str(original.GetPath()) if original else None
            if on:
                binding.Bind(material)
            elif saved[key]:
                binding.Bind(UsdShade.Material.Get(self.stage, saved[key]))
            else:
                binding.UnbindDirectBinding()
        self._lamp_bindings = saved
        self._lamp_states = getattr(self, '_lamp_states', {})
        self._lamp_states[name] = bool(on)
        self._write('lamp_states.json', self._lamp_states)
        for _ in range(30):
            self.executor.step(render=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['setup', 'exercise', 'capture', 'set', 'light'])
    parser.add_argument('--asset', type=Path)
    parser.add_argument('--url', default='http://127.0.0.1:18765')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] / 'outputs/cabinet_inspection')
    parser.add_argument('--names', nargs='+')
    parser.add_argument('--tag', default='original')
    parser.add_argument('--repair', action='store_true', help='在独立层修正关节锚点和嵌套刚体')
    parser.add_argument('--name')
    parser.add_argument('--target', type=float)
    parser.add_argument('--on', action='store_true', help='light 点亮；省略则熄灭')
    parser.add_argument('--position', nargs=3, type=float, default=(0, 0, 0))
    parser.add_argument('--yaw', type=float, default=0)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    code = f'''import importlib.util, sys
spec = importlib.util.spec_from_file_location('cabinet_probe_runtime', {str(Path(__file__).resolve())!r})
mod = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mod
exec(compile(open(spec.origin, encoding='utf-8').read(), spec.origin, 'exec'), mod.__dict__)
'''
    if args.action == 'setup':
        if args.asset is None:
            parser.error('setup 需要 --asset')
        code += f'''executor._cabinet_probe = mod.CabinetProbe(executor, {str(output)!r})
executor._cabinet_probe.setup({str(args.asset.resolve())!r}, {args.repair!r}, {tuple(args.position)!r}, {args.yaw!r})'''
    else:
        code += 'executor._cabinet_probe.__class__ = mod.CabinetProbe\n'
        if args.action == 'exercise':
            code += f'executor._cabinet_probe.exercise({args.names!r}, {args.tag!r})'
        elif args.action == 'set':
            if args.name is None or args.target is None:
                parser.error('set 需要 --name 和 --target（米或度）')
            code += f'executor._cabinet_probe._drive({args.name!r}, {args.target!r})'
        elif args.action == 'light':
            if args.name is None:
                parser.error('light 需要 --name')
            code += f'executor._cabinet_probe.light({args.name!r}, {args.on!r})'
        else:
            code += f'executor._cabinet_probe.capture({("cabinet.png" if args.tag == "original" else "cabinet_" + args.tag + ".png")!r})'
    request = urllib.request.Request(args.url + '/python',
        data=json.dumps({'code': code}).encode(), headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=600) as response:
            result = json.load(response)
    except urllib.error.HTTPError as exc:
        print(exc.read().decode(), file=sys.stderr)
        sys.exit(1)
    print(json.dumps(result, ensure_ascii=False))
    if result.get('status') != 'success':
        sys.exit(1)
    if args.action == 'exercise':
        path = output / f'results_{args.tag}.json'
        rows = json.loads(path.read_text())
        for row in rows:
            print(row['name'], 'PASS' if row['pass'] else 'FAIL', row.get('error', ''))
        print('完整反馈:', path)


if __name__ == '__main__':
    main()
