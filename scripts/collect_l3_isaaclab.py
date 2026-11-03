#!/usr/bin/env python3
"""Isaac Lab 3.0 并行机构采集：广义力动作、真实反馈、回合判据与可选同步 RGB。"""
import argparse
from types import SimpleNamespace
import json
import math
from pathlib import Path
import signal
import sys
import time

from l3_task_catalog import ASSET_ROOT, build_tasks


def atomic_json(path, data):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2))
    temporary.replace(path)


class L3BatchEnvironment:
    """显式 step/reset 的采集环境；InteractiveScene 克隆隔离，所有力与反馈按环境批处理。"""
    def __init__(self, args, sim, scene):
        import torch
        self.torch, self.args, self.sim, self.scene = torch, args, sim, scene
        self.catalog = {x['id']: x for x in json.loads((ASSET_ROOT / 'catalog.json').read_text())}
        self.profile = json.loads((ASSET_ROOT / 'reference_profile.json').read_text())
        self.assets = {name: scene[name] for name in self.catalog}
        self.names = {name: list(asset.joint_names) for name, asset in self.assets.items()}
        self.pressed = {name: torch.zeros(args.num_envs, dtype=torch.bool, device=args.device)
                        for name, item in self.catalog.items() if item['kind'] == 'button'}
        self.limit_keys = {}
        self.last_efforts, self.last_targets = {}, {}
        self.visual_previous = {}

    def update_lights(self):
        if not self.args.rgb:
            return
        import omni.usd
        from pxr import Gf, UsdShade, UsdLux
        stage = omni.usd.get_context().get_stage()
        for name in self.pressed:
            item = self.catalog[name]
            for env_id, on in enumerate(self.pressed[name].tolist()):
                if self.visual_previous.get((name, env_id)) == on:
                    continue
                prefix = f'/World/envs/env_{env_id}/{name}'
                color = (.1, 2., .1) if name in ('object_0', 'object_0_variant') else ((2., .1, .1) if name == 'object_1' else (1.5, 1.5, 1.5))
                shader = UsdShade.Shader(stage.GetPrimAtPath(prefix+item['indicator_shader'].removeprefix('/Asset')))
                shader.GetInput('emissiveColor').Set(Gf.Vec3f(*(color if on else (0., 0., 0.))))
                light = UsdLux.SphereLight(stage.GetPrimAtPath(prefix+item['indicator_light'].removeprefix('/Asset')))
                light.GetIntensityAttr().Set(10. if on else 0.)
                self.visual_previous[name, env_id] = on

    def array(self, value):
        return self.torch.as_tensor(value, dtype=self.torch.float32, device=self.args.device)

    def reset(self, tasks):
        # 每个回合只在 reset 写初态；采集期间只施力和更新弹簧/等效螺旋副目标。
        for name, asset in self.assets.items():
            q = asset.data.default_joint_pos.torch.clone()
            for env_id, task in enumerate(tasks):
                if task and task['asset_id'] == name:
                    for joint, position in task['initial_positions'].items():
                        q[env_id, self.names[name].index(joint)] = position
            if self.catalog[name]['kind'] == 'plate':
                limits = asset.data.joint_pos_limits.torch.clone()
                index = self.names[name].index('bridge_swing')
                limits[:, index, :] = self.array([0., math.pi/2])
                for joint in ('upper_turn', 'lower_turn'):
                    limits[:, self.names[name].index(joint), :] = self.array([0., 5*math.pi])
                asset.write_joint_position_limit_to_sim_index(limits=limits, warn_limit_violation=False)
            pose = asset.data.default_root_pose.torch.clone()
            pose[:, :3] += self.scene.env_origins
            asset.write_root_pose_to_sim_index(root_pose=pose)
            asset.write_root_velocity_to_sim_index(root_velocity=self.torch.zeros_like(asset.data.default_root_vel.torch))
            asset.write_joint_position_to_sim_index(position=q)
            asset.write_joint_velocity_to_sim_index(velocity=self.torch.zeros_like(q))
            asset.actuators.target_command.set_effort_index(value=self.torch.zeros_like(q))
            asset.actuators.target_command.set_position_index(value=q)
        self.scene.reset()
        self.sim.forward()
        self.limit_keys.clear()
        for name in self.pressed:
            self.pressed[name].zero_()
        # 不在这里 advance，释放动作必须记录到按下初态。
        self.scene.update(0.)

    def feedback(self, name):
        torch = self.torch
        q = self.assets[name].data.joint_pos.torch
        idx = {j: i for i, j in enumerate(self.names[name])}
        kind = self.catalog[name]['kind']
        if kind == 'button':
            stroke = -q[:, idx['button_slide']]
            threshold = self.profile['button']['switch_travel_m']
            pressed = stroke >= torch.where(self.pressed[name], threshold*.8, threshold)
            self.pressed[name] = pressed
            return torch.stack((stroke, pressed.float(), pressed.float()), dim=1)
        if kind == 'selector':
            angle = q[:, idx['selector_turn']]
            detents = self.array(self.catalog[name]['detents_deg']) * math.pi/180
            nearest = torch.argmin(abs(angle[:, None]-detents[None, :]), dim=1)
            settled = abs(angle-detents[nearest]) < math.radians(3.)
            return torch.stack((angle, nearest.float(), settled.float()), dim=1)
        upper, lower, swing = (q[:, idx[j]] for j in ('upper_turn', 'lower_turn', 'bridge_swing'))
        unlocked = ((upper > 4*math.pi-.10) & (lower > 4*math.pi-.10)
                    & (q[:, idx['upper_slide']] > .0019) & (q[:, idx['lower_slide']] > .0019))
        out = abs(swing) > math.radians(5.)
        connected = (abs(swing) < math.radians(3.)) & (upper < .12) & (lower < .12)
        return torch.stack((upper/(2*math.pi), lower/(2*math.pi), swing,
                            unlocked.float(), out.float(), connected.float()), dim=1)

    def apply(self, effort_by_name):
        torch = self.torch
        for name, asset in self.assets.items():
            q = asset.data.joint_pos.torch
            target = torch.zeros_like(q)
            idx = {j: i for i, j in enumerate(self.names[name])}
            kind = self.catalog[name]['kind']
            if kind == 'button':
                p = self.profile['button']
                target[:, idx['button_slide']] = p['preload_N']/((p['switch_force_N']-p['preload_N'])/p['switch_travel_m'])
            elif kind == 'selector':
                values = self.array(self.catalog[name]['detents_deg'])*math.pi/180
                target[:, idx['selector_turn']] = values[torch.argmin(abs(q[:, idx['selector_turn'], None]-values), dim=1)]
            else:
                feedback = self.feedback(name)
                unlocked, out = feedback[:, 3].bool(), feedback[:, 4].bool()
                limits = asset.data.joint_pos_limits.torch.clone()
                limits[:, idx['bridge_swing'], 0] = 0.
                limits[:, idx['bridge_swing'], 1] = torch.where(unlocked | out, math.pi/2, .0001)
                for part in ('upper', 'lower'):
                    target[:, idx[part+'_slide']] = torch.clamp(q[:, idx[part+'_turn']], min=0)*.001/(2*math.pi)
                    limits[:, idx[part+'_turn'], 0] = torch.where(out, 4*math.pi-.10, 0.)
                key = tuple(zip(unlocked.tolist(), out.tolist()))
                if key != self.limit_keys.get(name):
                    asset.write_joint_position_limit_to_sim_index(limits=limits, warn_limit_violation=False)
                    self.limit_keys[name] = key
            effort = effort_by_name.get(name, torch.zeros_like(q))
            asset.actuators.target_command.set_position_index(value=target)
            asset.actuators.target_command.set_effort_index(value=effort)
            self.last_efforts[name] = effort.clone()
            self.last_targets[name] = target.clone()
        self.scene.write_data_to_sim()

    def step(self, efforts):
        self.apply(efforts)
        self.sim.step(render=False)
        self.scene.update(1/240)
        for name in self.pressed:
            self.feedback(name)


def phases(task):
    if task['kind'] == 'button':
        return [('button_slide', None, .8 if task['action'] == 'press' else 1.)]
    if task['kind'] == 'selector':
        return [('selector_turn', math.radians(task['target_deg']), 4.), (None, None, 1.)]
    opened = [('upper_turn', 4.4*math.pi, 8.), ('lower_turn', 4.4*math.pi, 8.), ('bridge_swing', math.pi/2, 4.)]
    closed = [('bridge_swing', 0., 4.), ('upper_turn', 0., 8.), ('lower_turn', 0., 8.)]
    return [p for phase in (opened if task['action'] == 'plate_out' else closed)
            for p in (phase, (None, None, .6))]


def success(task, q, feedback, initial):
    if not all(math.isfinite(float(v)) for v in q):
        return False
    if task['action'] == 'press':
        return bool(feedback[1]) and feedback[0] >= .0026 and not bool(initial[1])
    if task['action'] == 'release':
        return bool(initial[1]) and not bool(feedback[1]) and feedback[0] < .0002
    if task['kind'] == 'selector':
        return abs(feedback[0]-math.radians(task['target_deg'])) < math.radians(3) and bool(feedback[2])
    if task['action'] == 'plate_out':
        return bool(feedback[3]) and bool(feedback[4]) and abs(feedback[2]-math.pi/2) < math.radians(3)
    return bool(feedback[5]) and not bool(feedback[4])


def collect(args, sim, scene, output, tasks):
    import h5py
    import numpy as np
    import torch
    env = L3BatchEnvironment(args, sim, scene)
    jobs = [{**task, 'repetition': repetition} for task in tasks for repetition in range(args.episodes)]
    rng = np.random.default_rng(args.seed)
    completed, passed, rows, stopped = 0, 0, [], False
    started = time.time()
    camera = scene['camera'] if args.rgb else None
    status = {'status': 'running', 'total_episodes': len(jobs), 'completed_episodes': 0,
              'successful_episodes': 0, 'num_envs': args.num_envs, 'message': '正在初始化并行机构'}
    atomic_json(output/'progress.json', status)
    with h5py.File(output/'episodes.partial.h5', 'w') as file:
        file.attrs['schema_version'] = 'l3_fixture_v1'
        file.attrs['execution_mode'] = 'fixture_effort'
        file.attrs['robot_contact_demonstration'] = False
        file.attrs['physics_dt'] = 1/240
        file.attrs['sample_dt'] = 1/30
        file.attrs['seed'] = args.seed
        for offset in range(0, len(jobs), args.num_envs):
            batch = jobs[offset:offset+args.num_envs]
            env.reset(batch)
            runs = []
            for i, task in enumerate(batch):
                name = task['asset_id']
                runs.append({'task': task, 'phases': phases(task), 'phase': 0, 'elapsed': 0.,
                    'done': False, 'timed_out': False, 'force_scale': float(rng.uniform(.9, 1.1)),
                    'initial': env.feedback(name)[i].cpu().numpy().copy(), 'records': []})
            count = 0
            while not all(run['done'] for run in runs):
                if (output/'STOP').exists():
                    stopped = True
                    break
                efforts = {name: torch.zeros_like(asset.data.joint_pos.torch) for name, asset in env.assets.items()}
                for i, run in enumerate(runs):
                    if run['done']:
                        continue
                    task, name = run['task'], run['task']['asset_id']
                    joint, target, duration = run['phases'][run['phase']]
                    if joint is not None:
                        index = env.names[name].index(joint)
                        position = float(env.assets[name].data.joint_pos.torch[i, index])
                        if target is None:
                            value = -8.*run['force_scale'] if task['action'] == 'press' else 0.
                            reached = False
                        else:
                            error = target-position
                            reached = abs(error) < .025
                            gain, maximum = (.05, .12) if joint.endswith('_turn') and joint != 'selector_turn' else (.12, .1)
                            value = (float(np.clip(gain*error, -maximum, maximum)) if joint != 'bridge_swing'
                                     else math.copysign(.03, error))
                            value *= run['force_scale']
                        efforts[name][i, index] = value
                    else:
                        reached = False
                    run['elapsed'] += 1/240
                    if run['elapsed'] >= duration or reached:
                        if target is not None and not reached:
                            run['timed_out'] = True
                            run['done'] = True
                        else:
                            run['phase'] += 1
                            run['elapsed'] = 0.
                            run['done'] = run['phase'] == len(run['phases'])
                        # 按下回合的终态要在保持力下记录，撤力属于独立的释放任务。
                        if joint is not None and task['action'] != 'press':
                            efforts[name][i, env.names[name].index(joint)] = 0.
                # action[t] 对应 pre-state[t] -> post-state[t]，只按30Hz抽样，控制保持240Hz。
                before = {name: (asset.data.joint_pos.torch.cpu().numpy().copy(),
                                  asset.data.joint_vel.torch.cpu().numpy().copy()) for name, asset in env.assets.items()}
                env.step(efforts)
                for i, run in enumerate(runs):
                    name = run['task']['asset_id']
                    if not torch.isfinite(env.assets[name].data.joint_pos.torch[i]).all():
                        raise RuntimeError(f"{run['task']['task_id']} 第{count}步反馈出现非有限数，停止本批，保留partial文件供排查")
                if count % 8 == 0 or any(run['done'] and not run.get('saved_done') for run in runs):
                    if camera:
                        env.update_lights()
                        sim.render()
                        camera.update(1/30, force_recompute=True)
                        rgb = camera.data.output['rgb'].torch.cpu().numpy()
                    for i, run in enumerate(runs):
                        if run.get('saved_done'):
                            continue
                        name = run['task']['asset_id']
                        record = {'time': count/240, 'next_time': (count+1)/240,
                                  'q': before[name][0][i], 'qd': before[name][1][i],
                                  'effort': env.last_efforts[name][i].cpu().numpy().copy(),
                                  'spring_target': env.last_targets[name][i].cpu().numpy().copy(),
                                  'next_q': env.assets[name].data.joint_pos.torch[i].cpu().numpy().copy(),
                                  'next_qd': env.assets[name].data.joint_vel.torch[i].cpu().numpy().copy(),
                                  'feedback': env.feedback(name)[i].cpu().numpy().copy(), 'phase': run['phase']}
                        if camera:
                            record['rgb'] = rgb[i, ..., :3].copy()
                            record['rgb_time'] = (count+1)/240
                        run['records'].append(record)
                        run['saved_done'] = run['done']
                count += 1
                if count % 120 == 0:
                    status.update(message=f'正在采集第 {offset+1}–{offset+len(batch)} 回合', elapsed_seconds=time.time()-started)
                    atomic_json(output/'progress.json', status)
            env.apply({})
            for i, run in enumerate(runs):
                task, records = run['task'], run['records']
                if not records:
                    continue
                finite = all(np.isfinite(record[key]).all() for record in records
                             for key in ('q', 'qd', 'effort', 'next_q', 'next_qd', 'feedback'))
                ok = not stopped and not run['timed_out'] and finite and success(task, records[-1]['next_q'], records[-1]['feedback'], run['initial'])
                group = file.create_group(f'episodes/{completed:06d}')
                metadata = {**task, 'env_id': i, 'success': bool(ok), 'interrupted': stopped,
                            'timed_out': run['timed_out'], 'finite': bool(finite), 'force_scale': run['force_scale'],
                            'joint_names': env.names[task['asset_id']], 'initial_feedback': run['initial'].tolist(),
                            'frame_count': len(records), 'state_time_alignment': 'pre_state_action_post_state_same_physics_step'}
                group.attrs['metadata_json'] = json.dumps(metadata, ensure_ascii=False)
                for key in records[0]:
                    group.create_dataset(key, data=np.asarray([x[key] for x in records]), compression='gzip')
                rows.append(metadata)
                completed += 1
                passed += int(ok)
            file.flush()
            status.update(completed_episodes=completed, successful_episodes=passed)
            atomic_json(output/'progress.json', status)
            print(f'采集进度 {completed}/{len(jobs)}，成功 {passed}', flush=True)
            if stopped:
                break
        if camera:
            file.create_dataset('camera/intrinsics', data=camera.data.intrinsic_matrices.torch.cpu().numpy())
            file.create_dataset('camera/position_world', data=camera.data.pos_w.torch.cpu().numpy())
            file.create_dataset('camera/quaternion_world_xyzw', data=camera.data.quat_w_world.torch.cpu().numpy())
            file.attrs['camera_convention'] = 'world: +X forward, +Z up; per-env fixed camera'
    (output/'episodes.partial.h5').replace(output/'episodes.h5')
    final = {**status, 'status': 'cancelled' if stopped else 'completed', 'elapsed_seconds': time.time()-started,
             'all_success': passed == len(jobs), 'message': '已停止并保存已有数据' if stopped else '采集完成，失败回合单独标记',
             'dataset': str(output/'episodes.h5'), 'episodes': rows}
    atomic_json(output/'summary.json', final)
    atomic_json(output/'progress.json', {k: v for k, v in final.items() if k != 'episodes'})
    return final


def main():
    from isaaclab.app import add_launcher_args, launch_simulation
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--num-envs', type=int, default=4)
    parser.add_argument('--episodes', type=int, default=10, help='每个独立任务的回合数')
    parser.add_argument('--tasks', choices=('all', 'button', 'selector', 'plate'), default='all')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rgb', action='store_true')
    parser.add_argument('--seed', type=int, default=42)
    add_launcher_args(parser)
    args = parser.parse_args()
    if not 1 <= args.num_envs <= 128 or not 1 <= args.episodes <= 10000:
        parser.error('环境数范围1–128，每任务回合数范围1–10000')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if (output/'episodes.h5').exists() or (output/'episodes.partial.h5').exists():
        parser.error('输出目录已有数据，请选择新的目录')
    signal.signal(signal.SIGTERM, lambda *_: (output/'STOP').touch())
    signal.signal(signal.SIGINT, lambda *_: (output/'STOP').touch())
    atomic_json(output/'progress.json', {'status': 'running', 'message': '正在启动 Isaac Lab 3.0 / Isaac Sim 6.2'})
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.assets import ArticulationCfg, AssetBaseCfg
        from isaaclab.actuators import ImplicitActuatorCfg
        from isaaclab.scene import InteractiveSceneCfg
        from isaaclab.sensors import CameraCfg
        from isaaclab.sim import SimulationContext
        from isaaclab.utils import instantiate
        from isaaclab_physx.physics import PhysxCfg
        sim_cfg = sim_utils.SimulationCfg(device=args.device, dt=1/240, render_interval=8,
                                         physics=PhysxCfg(solver_type=1), use_newton_actuators=False)
        # 固定到世界的机构需要每个环境独立的世界锚点，不能复制同一物理锚点。
        cfg = InteractiveSceneCfg(num_envs=args.num_envs, env_spacing=2., replicate_physics=False)
        cfg.light = AssetBaseCfg(prim_path='/World/Light', spawn=sim_utils.DomeLightCfg(intensity=1800.))
        cfg.ground = AssetBaseCfg(prim_path='/World/Ground', spawn=sim_utils.GroundPlaneCfg())
        cfg.panel = AssetBaseCfg(prim_path='{ENV_REGEX_NS}/Panel',
            spawn=sim_utils.CuboidCfg(size=(.015, .88, .66),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.22, .28, .32))),
            init_state=AssetBaseCfg.InitialStateCfg(pos=(-.04, 0., 1.1)))
        catalog = json.loads((ASSET_ROOT/'catalog.json').read_text())
        for index, item in enumerate(catalog):
            setattr(cfg, item['id'], ArticulationCfg(prim_path='{ENV_REGEX_NS}/'+item['id'],
                spawn=sim_utils.UsdFileCfg(usd_path=str(ASSET_ROOT/item['usd'])),
                init_state=ArticulationCfg.InitialStateCfg(pos=(0., (index%5-2)*.16, 1.3-(index//5)*.20)),
                actuators={'mechanism': ImplicitActuatorCfg(joint_names_expr=['.*'], stiffness=None, damping=None)}))
        if args.rgb:
            cfg.camera = CameraCfg(prim_path='{ENV_REGEX_NS}/Camera', update_period=1/30, width=640, height=480,
                data_types=['rgb'], spawn=sim_utils.PinholeCameraCfg(focal_length=18., horizontal_aperture=20.955, clipping_range=(.05, 10.)),
                offset=CameraCfg.OffsetCfg(pos=(.85, 0., 1.1), rot=(0., 0., 1., 0.), convention='world'))
        tasks = build_tasks(kinds=args.tasks)
        manifest = {'schema_version': 'l3_fixture_v1', 'framework': 'Isaac Lab 3.0', 'simulator': 'Isaac Sim 6.2',
            'execution_mode': 'fixture_effort', 'robot_contact_demonstration': False,
            'calibrated_to_user_hardware': False, 'physics_hz': 240, 'sample_hz': 30,
            'rgb': args.rgb, 'num_envs': args.num_envs, 'episodes_per_task': args.episodes,
            'seed': args.seed, 'tasks': tasks,
            'transition_dt': 1/240, 'sampling': '30Hz稀疏子步记录；next_q是同一240Hz子步的后态，非下一条采样帧',
            'rgb_alignment': 'rgb_time=next_time；渲染时物理不推进',
            'reference_profile': json.loads((ASSET_ROOT/'reference_profile.json').read_text())}
        atomic_json(output/'manifest.json', manifest)
        # 配置树包含 CameraCfg，启动器才能在建 Kit 前正确启用 RTX 传感器。
        with launch_simulation(SimpleNamespace(sim=sim_cfg, scene=cfg), args):
            try:
                sim = SimulationContext(sim_cfg)
                if args.rgb:
                    # Lab 的 Kit stage 绑定是异步的；让 SyntheticData 切到新 stage 后再建相机图。
                    import omni.kit.app
                    for _ in range(6):
                        omni.kit.app.get_app().update()
                scene = instantiate(cfg)
                from pxr import UsdGeom, UsdPhysics, Gf
                cache = UsdGeom.XformCache()
                for env_id in range(args.num_envs):
                    for item in catalog:
                        prefix = f'/World/envs/env_{env_id}/'+item['id']
                        joint = UsdPhysics.FixedJoint(sim.stage.GetPrimAtPath(prefix+'/Joints/Mount'))
                        transform = cache.GetLocalToWorldTransform(sim.stage.GetPrimAtPath(prefix+'/Base'))
                        joint.CreateBody0Rel().SetTargets([])
                        joint.CreateLocalPos0Attr(Gf.Vec3f(*transform.ExtractTranslation()))
                        joint.CreateLocalRot0Attr(Gf.Quatf(transform.ExtractRotationQuat()))
                        joint.CreateLocalPos1Attr(Gf.Vec3f(0))
                        joint.CreateLocalRot1Attr(Gf.Quatf(1))
                if args.rgb:
                    for _ in range(4):
                        sim.render()
                sim.reset()
                scene.update(1/240)
                import numpy as np
                origins = scene.env_origins.cpu().numpy()
                clone_errors = {}
                for index, item in enumerate(catalog):
                    actual = scene[item['id']].data.root_pos_w.torch.cpu().numpy()
                    expected = origins + np.array([0., (index%5-2)*.16, 1.3-(index//5)*.20])
                    clone_errors[item['id']] = float(np.max(abs(actual-expected)))
                atomic_json(output/'clone_validation.json', {'max_root_position_error_m': clone_errors,
                            'pass': max(clone_errors.values()) < .002})
                if max(clone_errors.values()) >= .002:
                    raise RuntimeError('并行克隆的机构固定根位置异常，停止采集')
                result = collect(args, sim, scene, output, tasks)
            except Exception as error:
                atomic_json(output/'progress.json', {'status': 'failed', 'message': str(error)})
                raise
        return 0 if result['all_success'] or result['status'] == 'cancelled' else 2
    except BaseException as error:
        atomic_json(output/'progress.json', {'status': 'failed', 'message': str(error)})
        raise


if __name__ == '__main__':
    sys.exit(main())
