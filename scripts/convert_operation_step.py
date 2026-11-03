#!/usr/bin/env python3
"""Isaac Sim 6.2 HOOPS：STEP → 米制、Z-up 外观资产，保留零件名称。"""
import argparse
import asyncio
import json
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    # Kit 某些导入扩展会把位置参数误当作要打开的场景。
    sys.argv = [sys.argv[0]]
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'width': 1280, 'height': 960,
                         'renderer': 'RayTracedLighting'})
    try:
        import omni.kit.app
        from pxr import Gf, Usd, UsdGeom, UsdUtils
        manager = omni.kit.app.get_app().get_extension_manager()
        manager.set_extension_enabled_immediate('omni.kit.converter.hoops_core', True)
        from omni.kit.converter.hoops_core import get_instance
        raw = output / 'geometry.usda'
        if raw.exists():
            raise RuntimeError(f'输出已存在：{raw}')
        task = asyncio.ensure_future(get_instance().create_converter_task(
            str(source), str(raw), {'instancing': 'false', 'convertMetadata': 'true',
                                   'convertCurves': 'false', 'upAxis': '2'}))
        while not task.done():
            app.update()
        path, status = task.result()
        if not path or not raw.exists():
            raise RuntimeError(f'CAD 转换失败：{status}')
        stage = Usd.Stage.Open(str(raw))
        root = stage.GetDefaultPrim()
        if not root:
            roots = [p for p in stage.GetPseudoRoot().GetChildren() if p.GetTypeName() == 'Xform']
            if len(roots) != 1:
                raise RuntimeError('CAD 顶层不唯一，需要人工确认')
            root = roots[0]
            stage.SetDefaultPrim(root)
        original_units = UsdGeom.GetStageMetersPerUnit(stage)
        original_axis = str(UsdGeom.GetStageUpAxis(stage))
        bounds = UsdGeom.BBoxCache(0, ['default', 'render']).ComputeWorldBound(root).ComputeAlignedRange()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        stage.GetRootLayer().Save()
        wrapper_path = output / 'asset.usda'
        wrapper = Usd.Stage.CreateNew(str(wrapper_path))
        UsdGeom.SetStageMetersPerUnit(wrapper, 1.0)
        UsdGeom.SetStageUpAxis(wrapper, UsdGeom.Tokens.z)
        asset = UsdGeom.Xform.Define(wrapper, '/Asset')
        wrapper.SetDefaultPrim(asset.GetPrim())
        geom = UsdGeom.Xform.Define(wrapper, '/Asset/Geometry')
        geom.GetPrim().GetReferences().AddReference('./geometry.usda')
        xf = UsdGeom.Xformable(geom.GetPrim())
        if original_axis == 'Y':
            xf.AddRotateXOp().Set(90.0)
        xf.AddScaleOp().Set(Gf.Vec3f(original_units))
        box = UsdGeom.BBoxCache(0, ['default', 'render']).ComputeWorldBound(asset.GetPrim()).ComputeAlignedRange()
        lo, hi = box.GetMin(), box.GetMax()
        asset.AddTranslateOp().Set(Gf.Vec3d(-(lo[0] + hi[0]) / 2,
                                          -(lo[1] + hi[1]) / 2, -lo[2]))
        wrapper.GetRootLayer().Save()
        unresolved = UsdUtils.ComputeAllDependencies(str(wrapper_path))[2]
        if unresolved:
            raise RuntimeError(f'未解析依赖：{unresolved}')
        meshes = [str(p.GetPath()) for p in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
                  if p.IsA(UsdGeom.Mesh)]
        report = {'source_file': source.name, 'asset': 'asset.usda',
                  'original_meters_per_unit': original_units, 'original_up_axis': original_axis,
                  'size_m': list(hi - lo), 'mesh_count': len(meshes),
                  'mesh_paths': meshes, 'physics_ready': False,
                  'unresolved_dependencies': list(unresolved)}
        (output / 'conversion.json').write_text(json.dumps(report, indent=2) + '\n')
        print('STEP_COMPLETE', json.dumps(report), flush=True)
    finally:
        app.close()


if __name__ == '__main__':
    main()
