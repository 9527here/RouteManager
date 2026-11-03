#!/usr/bin/env python3
"""只读审计 GLB 并在独立 Isaac 6.2 进程转换/预览，不改活动场景。"""
import argparse
import asyncio
import hashlib
import json
import re
import struct
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--opposite', action='store_true', help='补拍相反方向，确认正背面差异')
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    items = []
    for i, path in enumerate(sorted(source.glob('*.glb'))):
        data = path.read_bytes()
        magic, version, total = struct.unpack_from('<4sII', data)
        size, kind = struct.unpack_from('<I4s', data, 12)
        assert magic == b'glTF' and version == 2 and total == len(data) and kind == b'JSON'
        gltf = json.loads(data[20:20 + size])
        items.append({'id': re.sub(r'[^a-zA-Z0-9_]', '_', path.stem),
                      'file': path.name, 'source': str(path), 'bytes': len(data),
                      'sha256': hashlib.sha256(data).hexdigest(),
                      'nodes': len(gltf.get('nodes', [])), 'meshes': len(gltf.get('meshes', [])),
                      'animations': len(gltf.get('animations', [])), 'skins': len(gltf.get('skins', [])),
                      'extensions': gltf.get('extensionsUsed', []),
                      'external_resources': [v['uri'] for kind in ('buffers', 'images') for v in gltf.get(kind, [])
                                             if 'uri' in v and not v['uri'].startswith('data:')]})
    (output / 'audit.json').write_text(json.dumps(items, indent=2) + '\n')
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'width': 1280, 'height': 960,
                         'renderer': 'RayTracedLighting'})
    try:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension('omni.kit.asset_converter')
        import omni.kit.asset_converter as converter
        import omni.usd
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdUtils
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        context = converter.AssetConverterContext()
        context.ignore_camera = True
        context.ignore_light = True
        context.export_preview_surface = True
        context.use_meter_as_world_unit = True
        # 只规范单位元数据，不假定归一化模型的真实物理尺寸。
        usd_dir = output / 'usd'
        usd_dir.mkdir(exist_ok=True)
        scene = Usd.Stage.CreateNew(str(output / 'gallery.usda'))
        world = UsdGeom.Xform.Define(scene, '/World')
        scene.SetDefaultPrim(world.GetPrim())
        UsdGeom.SetStageMetersPerUnit(scene, 1.)
        UsdGeom.SetStageUpAxis(scene, 'Z')
        UsdLux.DomeLight.Define(scene, '/World/Dome').CreateIntensityAttr(300.)
        sun = UsdLux.DistantLight.Define(scene, '/World/Sun')
        sun.CreateIntensityAttr(1200.)
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-30, 15, 20))
        for item in items:
            path = usd_dir / (item['id'] + '.usd')
            if not path.exists():
                task = converter.get_instance().create_converter_task(item['source'], str(path), None, context)
                future = asyncio.ensure_future(task.wait_until_finished())
                while not future.done():
                    app.update()
                if not future.result():
                    raise RuntimeError(f"{item['file']}: {task.get_error_message()}")
            stage = Usd.Stage.Open(str(path))
            default = stage.GetDefaultPrim()
            if not default:
                raise RuntimeError(f'无 defaultPrim：{path}')
            bound = UsdGeom.BBoxCache(0, ['default', 'render']).ComputeWorldBound(default).ComputeAlignedRange()
            item.update(usd=str(path.relative_to(output)), up_axis=str(UsdGeom.GetStageUpAxis(stage)),
                        meters_per_unit=UsdGeom.GetStageMetersPerUnit(stage),
                        bounds_min=list(bound.GetMin()), bounds_max=list(bound.GetMax()),
                        size_authored=list(bound.GetSize()),
                        usd_joints=sum(p.IsA(UsdPhysics.Joint) for p in stage.Traverse()),
                        rigid_bodies=sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in stage.Traverse()),
                        collisions=sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in stage.Traverse()),
                        unresolved=list(UsdUtils.ComputeAllDependencies(str(path))[2]))
            if item['unresolved']:
                raise RuntimeError(item['unresolved'])
            root = UsdGeom.Xform.Define(scene, '/World/' + item['id'])
            root.GetPrim().GetReferences().AddReference('./' + item['usd'])
            item['gallery_prim'] = str(root.GetPath())
            item['gallery_translate'] = [0., 0., 0.]
            print('CONVERTED', item['file'], flush=True)
        scene.GetRootLayer().Save()
        (output / 'audit.json').write_text(json.dumps(items, indent=2) + '\n')
        del scene
        omni.usd.get_context().open_stage(str(output / 'gallery.usda'))
        for _ in range(120):
            app.update()
        stage = omni.usd.get_context().get_stage()
        viewport = get_active_viewport()
        viewport.set_texture_resolution((1280, 960))
        for item in items:
            for other in items:
                imageable = UsdGeom.Imageable(stage.GetPrimAtPath(other['gallery_prim']))
                imageable.MakeVisible() if other is item else imageable.MakeInvisible()
            center = [(a + b) / 2 for a, b in zip(item['bounds_min'], item['bounds_max'])]
            size = max(item['size_authored'])
            # GLB 规范 Y-up，但源网格语义朝向不明；等轴图检查外观，不推定功能。
            sign = -1 if args.opposite else 1
            eye = [center[0] + sign * size * 1.7, center[1] - sign * size * 2.3, center[2] + size * 1.5]
            set_camera_view(eye=eye, target=center)
            for _ in range(60):
                app.update()
            suffix = '_opposite' if args.opposite else ''
            capture_viewport_to_file(viewport, str(output / (item['id'] + suffix + '.png')))
            for _ in range(15):
                app.update()
            print('RENDERED', item['file'], flush=True)
        print('L3_INSPECTION_COMPLETE', len(items), flush=True)
    finally:
        app.close()


if __name__ == '__main__':
    main()
