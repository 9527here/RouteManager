#!/usr/bin/env python3
"""把已迁移机构排到展示支架；支架尺寸仅供排布，不代表实物规格。"""
import argparse
import json
import os
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--asset-root', type=Path, default=Path(__file__).resolve().parents[1] / 'assets')
    args = parser.parse_args()
    root = args.asset_root.resolve()
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    output = root / 'operation_panel.usda'
    stage = Usd.Stage.CreateNew(str(output))
    panel = UsdGeom.Xform.Define(stage, '/OperationPanel')
    stage.SetDefaultPrim(panel.GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, 'Z')

    def cube(name, position, size, color):
        geom = UsdGeom.Cube.Define(stage, '/OperationPanel/' + name)
        geom.CreateSizeAttr(1.0)
        geom.CreateDisplayColorAttr([color])
        xf = UsdGeom.Xformable(geom)
        xf.AddTranslateOp().Set(Gf.Vec3d(*position))
        xf.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())

    cube('Board', [.96, -.95, 1.1], [.04, 1., .74], [.19, .24, .28])
    for i, y in enumerate([-1.35, -.55]):
        cube(f'Leg{i}', [1., y, .36], [.05, .05, .72], [.22, .24, .26])
        cube(f'Foot{i}', [.96, y, .015], [.4, .10, .03], [.18, .20, .22])
    catalog = json.loads((root / 'oxford_devices/catalog.json').read_text())
    layout = {'e_stop_button': ([.935, -1.18, 1.25], [0, -90, 0]),
              'isolator_switch': ([.935, -.72, 1.25], [0, -90, 0]),
              'safety_switch': ([.85, -.72, .88], [0, 0, 180])}
    for item in catalog:
        prim = stage.DefinePrim('/OperationPanel/Devices/' + item['id'], 'Xform')
        source = root / 'oxford_devices' / item['usd']
        prim.GetReferences().AddReference('./' + os.path.relpath(source, output.parent))
        position, rotation = layout[item['id']]
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(*position))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
    breaker = stage.DefinePrim('/OperationPanel/BreakerCAD', 'Xform')
    geometry = stage.DefinePrim('/OperationPanel/BreakerCAD/Asset', 'Xform')
    geometry.GetReferences().AddReference('./manufacturer_cad/schneider_gv6p320h/usd/asset.usda')
    xf = UsdGeom.Xformable(breaker)
    xf.AddTranslateOp().Set(Gf.Vec3d(.85, -1.18, .88))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 90))
    stage.GetRootLayer().Save()
    print(output)


if __name__ == '__main__':
    main()
