#!/usr/bin/env python3
"""在已启动的RouteManager里一键打开电力柜动作场景；已有工位只重开中文面板。"""
import argparse
import inspect
import json
from pathlib import Path
import urllib.request


def setup(executor, project, cabinet_asset):
    import hashlib
    import json
    import sys
    import numpy as np
    from scipy.spatial.transform import Rotation
    project = Path(project)
    sys.path.insert(0, str(project/'scripts'))
    if not hasattr(executor, '_l3_station'):
        if executor.articulation is not None:
            raise RuntimeError('当前已有其它机器人场景，请使用独立空白服务；不能二次加载机器人')
        from cabinet_inspect import CabinetProbe
        output = project/'outputs/cabinet_humanoid'
        output.mkdir(parents=True, exist_ok=True)
        executor._cabinet_probe = CabinetProbe(executor, str(output))
        executor._cabinet_probe.setup(cabinet_asset, True, (1.1, 0., 0.), 180.)
        namespace = {'__file__': str(project/'scripts/ensure_cabinet_shell_collision.py')}
        source = project/'scripts/ensure_cabinet_shell_collision.py'
        exec(compile(source.read_text(), str(source), 'exec'), namespace)
        namespace['ensure'](executor)
        # 该入口只在空白场景导入一次机器人，沿用现有整机与软件mimic配置。
        source = project/'scripts/scene/setup_cabinet_humanoid.py'
        exec(compile(source.read_text(), str(source), 'exec'), {'executor': executor, '__file__': str(source)})
        pose = json.loads((project/'assets/l3_operable/humanoid_demo_pose.json').read_text())
        transport = json.loads((project/'assets/l3_operable/verified_transport_pose.json').read_text())
        urdf=Path(executor._config['robot']['presets']['v3_ctag2f120_rm65']['urdf_path'])
        usd=Path(executor._config['robot']['presets']['v3_ctag2f120_rm65_l2']['usd_path'])
        if (hashlib.sha256(urdf.read_bytes()).hexdigest()!=transport['source_urdf_sha256']
                or hashlib.sha256(usd.read_bytes()).hexdigest()!=transport['source_usd_sha256']):
            raise RuntimeError('机器人文件与已验证运输姿态不匹配，新URDF需重新验证，禁止套用旧关节目标')
        for item in transport['by_side'].values():
            for name, value in zip(item['joint_names'], item['goal_q']):
                pose['joint_positions'][pose['joint_names'].index(name)] = value
        robot = executor.articulation
        if set(robot.dof_names) != set(pose['joint_names']):
            raise RuntimeError('机器人关节与演示初态不匹配，请重新确认新URDF')
        q = np.asarray([[pose['joint_positions'][pose['joint_names'].index(n)] for n in robot.dof_names]])
        matrix = np.asarray(pose['base_link_matrix'])
        quat = Rotation.from_matrix(matrix[:3, :3]).as_quat(scalar_first=True)
        robot.set_world_poses(positions=matrix[None, :3, 3], orientations=quat[None, :])
        robot.set_joint_positions(q)
        robot.set_joint_velocities(np.zeros_like(q))
        robot.set_joint_position_targets(q)
        executor._arm_transit_stow_goals = transport['by_side']
        executor._mimic_targets = None
        from operation_panel_demo import install as install_panel
        install_panel(executor, project/'assets/operation_panel.usda', project/'outputs/operation_panel')
        from l3_station import install
        install(executor, project/'assets/l3_operable', project/'outputs/l3_operable/live')
    from l3_all_assets import expand
    expand(executor._l3_station, project/'assets/l3_operable')
    namespace = {'__file__': str(project/'scripts/ensure_cabinet_shell_collision.py')}
    source = project/'scripts/ensure_cabinet_shell_collision.py'
    exec(compile(source.read_text(), str(source), 'exec'), namespace)
    namespace['ensure'](executor)
    profile = json.loads((project/'assets/l3_operable/mobile_operation_profile.json').read_text())
    executor.configure_swerve_base(reference_steering_limit_deg=profile['steering_limit_deg'])
    executor._l3_station.show_ui()
    executor._l3_station.all_view()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:18765')
    parser.add_argument('--cabinet', type=Path, default=Path('/home/jy/VLA/simManager/data/environment/dongligui/dongligui.usd'))
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    code = 'from pathlib import Path\n' + inspect.getsource(setup)
    code += f'\nsetup(executor, {str(project)!r}, {str(args.cabinet.resolve())!r})\n'
    request = urllib.request.Request(args.url.rstrip('/')+'/python', data=json.dumps({'code': code}).encode(),
                                     headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=600) as response:
        print(response.read().decode())


if __name__ == '__main__':
    main()
