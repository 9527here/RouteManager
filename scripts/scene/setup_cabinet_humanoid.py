"""经 POST /python 执行：把项目人形整机加入无额外铭牌的柜体场景。

先用 cabinet_inspect.py setup 加载柜体。本脚本不加载默认扫描背景或机器人相机。
"""


def setup(executor):
    import json
    from pathlib import Path
    import numpy as np
    import omni.usd
    from pxr import Usd, UsdGeom, Gf
    import arm_kin

    if executor.articulation is not None:
        raise RuntimeError('本会话已有机器人，不能重复加载；保持当前场景')
    probe = executor._cabinet_probe
    output = probe.output
    preset = executor._config['robot']['presets']['v3_ctag2f120_rm65_l2']
    source = Path(preset['usd_path'])
    if not source.exists():
        raise FileNotFoundError(source)
    # 高度沿用 L2 的 robot_with_table.usda 已落地配置；不是 6.222 m 扫描场景。
    wrapper = output / 'humanoid_spawn.usda'
    scene = Usd.Stage.CreateNew(str(wrapper))
    root = scene.DefinePrim('/Robot', 'Xform')
    scene.SetDefaultPrim(root)
    UsdGeom.SetStageMetersPerUnit(scene, 1.0)
    UsdGeom.SetStageUpAxis(scene, UsdGeom.Tokens.z)
    root.GetReferences().AddReference(str(source))
    UsdGeom.Xformable(root).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.329))
    scene.GetRootLayer().Save()
    del scene

    # 只覆盖本会话的地面高度，避免把机器人抬到扫描背景的楼层。配置文件不写回。
    executor._base_z = 0.0
    executor._create_ground_plane()
    executor.world.stop()
    result = executor.load_robot(usd_path=str(wrapper), prim_path='/humanoid')
    urdf = executor._config['robot']['presets']['v3_ctag2f120_rm65']['urdf_path']
    arm_kin.set_urdf(urdf)
    executor._urdf_mimic = executor._parse_urdf_mimic(urdf)
    executor._build_mimic_map()
    for _ in range(180):
        executor.step(render=True)

    positions = np.asarray(executor.articulation.get_joint_positions()).reshape(-1)
    transforms = np.asarray(executor.articulation._physics_view.get_link_transforms())
    names = list(executor.articulation.dof_names)
    result.update({
        'robot_source': str(source), 'urdf': urdf, 'dof_count': len(names),
        'finite_positions': bool(np.isfinite(positions).all()),
        'finite_link_transforms': bool(np.isfinite(transforms).all()),
        'arm_joints': [n for n in names if '_arm_' in n],
        'extra_nameplates_present': bool(probe.stage.GetPrimAtPath('/World/Cabinet/Nameplates')),
        'joint_positions': dict(zip(names, map(float, positions))),
    })
    if not result['finite_positions'] or not result['finite_link_transforms']:
        raise RuntimeError('机器人反馈出现非有限数，禁止继续下发运动')
    (output / 'humanoid_validation.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=[3.4, 3.2, 2.2], target=[0.4, 0, 0.9])
    omni.usd.get_context().get_selection().set_selected_prim_paths([], False)
    for _ in range(90):
        executor.step(render=True)
    probe.capture('humanoid_cabinet.png')
    probe.stage.GetRootLayer().Export(str(output / 'session.usda'))


setup(executor)  # noqa: F821 — HTTP 服务提供仿真主线程执行器。
