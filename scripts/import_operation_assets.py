#!/usr/bin/env python3
"""复制并检查一批 USDZ，生成项目内相对引用的静态资产预览场景。

不推定设备名称和真实规格，不自动赋予关节、碰撞或质量。
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil
import zipfile


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inspect(path):
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils, Sdf
    with zipfile.ZipFile(path) as package:
        failed = package.testzip()
        if failed:
            raise RuntimeError(f'USDZ CRC 校验失败: {path}: {failed}')
        members = package.namelist()
    stage = Usd.Stage.Open(str(path))
    root = stage.GetDefaultPrim()
    if not root:
        raise RuntimeError(f'没有 defaultPrim: {path}')
    prims = list(stage.Traverse())
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render']).ComputeWorldBound(root).ComputeAlignedRange()
    _, dependencies, unresolved = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(str(path)))
    joints = [p for p in prims if p.IsA(UsdPhysics.Joint)]
    return {
        'id': path.stem, 'file': path.name, 'sha256': digest(path),
        'bytes': path.stat().st_size, 'crc_ok': True, 'package_members': members,
        'default_prim': str(root.GetPath()), 'meters_per_unit': UsdGeom.GetStageMetersPerUnit(stage),
        'up_axis': str(UsdGeom.GetStageUpAxis(stage)),
        'bounds_min': list(bounds.GetMin()), 'bounds_max': list(bounds.GetMax()),
        'size_m_authored': list(bounds.GetSize()),
        'types': dict(Counter(p.GetTypeName() for p in prims)),
        'meshes': [str(p.GetPath()) for p in prims if p.IsA(UsdGeom.Mesh)],
        'joint_count': len(joints),
        'rigid_body_count': sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in prims),
        'collision_count': sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in prims),
        'shader_types': sorted({str(p.GetAttribute('info:id').Get()) for p in prims if p.IsA(UsdShade.Shader)}),
        'unresolved_dependencies': list(map(str, unresolved)),
        'dependency_count': len(dependencies),
        'status': 'visual_only' if not joints else 'requires_runtime_validation',
        'semantic_name': None,
    }


def make_gallery(target, items):
    from pxr import Usd, UsdGeom, UsdLux, Gf
    stage = Usd.Stage.CreateNew(str(target/'gallery.usda'))
    root = stage.DefinePrim('/World', 'Xform')
    stage.SetDefaultPrim(root)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    for index, item in enumerate(items):
        prim = stage.DefinePrim('/World/Asset'+item['id'], 'Xform')
        prim.GetReferences().AddReference('./'+item['file'])
        center = [(a+b)/2 for a,b in zip(item['bounds_min'],item['bounds_max'])]
        # 保留源单位，预览排版只平移，不把模型强行缩放成“真实尺寸”。
        position = [-center[0], (index%4)*1.05-center[1], (2-index//4)*1.25-item['bounds_min'][2]]
        UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*position))
        # 每个源文件自带黑环境贴图，展示场景统一布光。
        env = stage.GetPrimAtPath(str(prim.GetPath())+'/env_light')
        if env:
            env.SetActive(False)
        item['gallery_prim'] = str(prim.GetPath())
        item['gallery_translate'] = position
    UsdLux.DomeLight.Define(stage, '/World/PreviewDome').CreateIntensityAttr(500)
    sun = UsdLux.DistantLight.Define(stage, '/World/PreviewSun')
    sun.CreateIntensityAttr(1200)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(0,-35,-25))
    stage.GetRootLayer().Save()


def make_normalized(target, items):
    from pxr import Usd, UsdGeom, Gf
    folder=target/'normalized'
    folder.mkdir(exist_ok=True)
    for item in items:
        path=folder/(item['id']+'.usda')
        stage=Usd.Stage.CreateNew(str(path))
        root=stage.DefinePrim('/Asset','Xform')
        stage.SetDefaultPrim(root)
        UsdGeom.SetStageMetersPerUnit(stage,1.0)
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
        root.GetReferences().AddReference('../'+item['file'])
        center=[(a+b)/2 for a,b in zip(item['bounds_min'],item['bounds_max'])]
        UsdGeom.Xformable(root).AddTranslateOp().Set(Gf.Vec3d(-center[0],-center[1],-item['bounds_min'][2]))
        root.SetCustomDataByKey('asset_status','visual_only_dimensions_unverified')
        env=stage.GetPrimAtPath('/Asset/env_light')
        if env:env.SetActive(False)
        stage.GetRootLayer().Save()
        item['normalized_file']='normalized/'+path.name


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--target', type=Path, required=True)
    args = parser.parse_args()
    source, target = args.source.resolve(), args.target.resolve()
    if source == target:
        parser.error('源目录与项目接入目录不能相同')
    paths = sorted(source.rglob('*'))
    copied = []
    for path in paths:
        if not path.is_file():
            continue
        relative = path.relative_to(source)
        destination = target/relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists() and digest(path) != digest(destination):
            raise RuntimeError(f'项目目录有不同版本，停止覆盖: {destination}')
        if not destination.exists():
            shutil.copy2(path, destination)
        if digest(path) != digest(destination):
            raise RuntimeError(f'复制后校验不一致: {destination}')
        copied.append({'file':str(relative),'sha256':digest(destination),'bytes':destination.stat().st_size})
    items = [inspect(p) for p in sorted(target.glob('*.usdz'))]
    if not items:
        raise RuntimeError('没有找到 USDZ')
    if any(i['unresolved_dependencies'] for i in items):
        raise RuntimeError('存在未解析依赖，不能标记导入成功')
    make_normalized(target,items)
    make_gallery(target,items)
    manifest = {'source':str(source),'files':copied,'assets':items,
                'warning':'尺寸为资产 authoring 数据，不等同实物标定；未添加物理参数'}
    (target/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    print(f'Imported {len(items)} USDZ; {len(copied)} files checked; gallery: {target / "gallery.usda"}')


if __name__ == '__main__':
    main()
