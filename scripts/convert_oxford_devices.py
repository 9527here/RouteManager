#!/usr/bin/env python3
"""用 Isaac Sim 6.2 官方导入器生成 Oxford 机构分层 USD。"""
import argparse
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('asset_root', type=Path)
    parser.add_argument('--usd-dir', default='usd')
    args = parser.parse_args()
    root = args.asset_root.resolve()
    items = json.loads((root / 'preparation.json').read_text())
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'width': 1280, 'height': 960,
                         'renderer': 'RayTracedLighting'})
    try:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension('isaacsim.asset.importer.urdf')
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        from pxr import Usd, UsdPhysics, UsdUtils, PhysxSchema
        import yaml
        catalog = []
        for item in items:
            name = item['id']
            out = root / args.usd_dir / name
            if out.exists() and any(out.iterdir()):
                raise RuntimeError(f'拒绝重复导入制造 _N 产物，请先检查已有目录：{out}')
            cfg = URDFImporterConfig(
                urdf_path=str(root / item['urdf']), usd_path=str(out),
                # 上游 dummy 零位 link 的惯量可比活动件大 10^5 倍。
                # 合并固定连接能保留活动关节与分件，避免 PhysX 约束链病态。
                fix_base=True, merge_fixed_joints=True, allow_self_collision=False,
                collision_from_visuals=False, collision_type='Convex Hull',
                joint_drive_type='force', joint_target_type='position',
                override_joint_stiffness=0.0, override_joint_damping=0.0)
            path = Path(URDFImporter().import_urdf(cfg))
            stage = Usd.Stage.Open(str(path))
            default = stage.GetDefaultPrim()
            default.GetVariantSet('Physics').SetVariantSelection('physx')
            controller_path = root / 'upstream/manipulation_playground_articulated_devices/devices' / name / 'controllers' / f'{name}_controllers.yaml'
            controllers = yaml.safe_load(controller_path.read_text())
            joints = []
            for prim in stage.Traverse():
                if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateEnabledSelfCollisionsAttr(False)
                if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
                    linear = prim.IsA(UsdPhysics.PrismaticJoint)
                    controller = f'{name}_controller_normal'
                    if name == 'e_stop_button' and not linear:
                        controller = 'e_stop_button_twist_controller'
                    gain = controllers[controller]['pid']
                    drive = UsdPhysics.DriveAPI.Apply(prim, 'linear' if linear else 'angular')
                    drive.CreateTypeAttr('force')
                    # USD angular Drive 以每度记增益，PhysX 运行时会乘 180/pi。
                    # Gazebo PID 是每弧度；必须转换，不能直接复制数值。
                    factor = 1.0 if linear else math.pi / 180
                    drive.CreateStiffnessAttr(float(gain['p']) * factor)
                    drive.CreateDampingAttr(float(gain['d']) * factor)
                    drive.CreateMaxForceAttr(100.0)
                    drive.CreateTargetPositionAttr(0.0)
                    joints.append({'name': prim.GetName(), 'path': str(prim.GetPath()),
                                   'type': 'prismatic' if linear else 'revolute',
                                   'drive_p': gain['p'], 'drive_d': gain['d']})
            stage.GetRootLayer().Save()
            layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(path))
            if unresolved:
                raise RuntimeError(f'未解析依赖：{unresolved}')
            row = {'id': name, 'usd': str(path.relative_to(root)),
                   'root_prim': str(default.GetPath()), 'joints': joints,
                   'source_dimensions_verified': False, 'force_calibrated': False,
                   'human_contact_verified': False}
            catalog.append(row)
            (root / 'catalog.json').write_text(json.dumps(catalog, indent=2) + '\n')
            print('CONVERTED', name, row, flush=True)
        print('CONVERT_COMPLETE', flush=True)
    finally:
        app.close()


if __name__ == '__main__':
    main()
