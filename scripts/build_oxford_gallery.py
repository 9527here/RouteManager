#!/usr/bin/env python3
"""构造可搬移的机构验证场景，不改变原有人形服务。"""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('asset_root', type=Path)
    args = parser.parse_args()
    root = args.asset_root.resolve()
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade
    path = root / 'gallery.usda'
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(world.GetPrim())
    dome = UsdLux.DomeLight.Define(stage, '/World/DomeLight')
    dome.CreateIntensityAttr(300.0)
    sun = UsdLux.DistantLight.Define(stage, '/World/Sun')
    sun.CreateIntensityAttr(1200.0)
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-35, 15, 20))
    floor = UsdGeom.Cube.Define(stage, '/World/Floor')
    floor.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(floor.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.02))
    xf.AddScaleOp().Set(Gf.Vec3f(2, 2, 0.04))
    floor.CreateDisplayColorAttr([(0.24, 0.28, 0.32)])
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    layout = {'e_stop_button': ([-0.20, 0, 0.9], [90, 0, 0]),
              'isolator_switch': ([0, 0, 0.9], [90, 0, 0]),
              'safety_switch': ([0.33, 0, 0.8], [0, 0, 0])}
    catalog = json.loads((root / 'catalog.json').read_text())
    for item in catalog:
        name = item['id']
        prim_path = f'/World/Devices/{name}'
        prim = stage.DefinePrim(prim_path, 'Xform')
        prim.GetReferences().AddReference('./' + item['usd'])
        position, rotation = layout[name]
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(*position))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
        item['gallery_prim'] = prim_path
        item['gallery_translate'] = position
        item['gallery_rotation'] = rotation
        # 6.2 导入器把 root_joint.body0 指向资产根 Xform，局部锚点保持零。
        # 根 Xform 自己会带入摆放位姿，再写世界位置会造成双重偏移。
    (root / 'catalog.json').write_text(json.dumps(catalog, indent=2) + '\n')
    stage.GetRootLayer().Save()
    print(path)


if __name__ == '__main__':
    main()
