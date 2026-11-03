"""在当前Kit场景附着官方SimPeople扩展；所有函数由RouteManager主线程调用。"""
import json
from pathlib import Path
import time


def attach(executor, output, simpeople_root):
    import carb
    import omni.kit.app
    import omni.timeline
    import omni.usd
    from pxr import Sdf, Usd
    output, simpeople_root = Path(output), Path(simpeople_root)
    stage = omni.usd.get_context().get_stage()
    if executor.articulation is None or not executor.articulation._physics_view.check():
        raise RuntimeError('已有机器人物理反馈无效，禁止附着')
    for name in ('scene_overlay.usda', 'semantics.json', 'patrol_cabinet.usda'):
        if not (output / name).is_file():
            raise RuntimeError('缺少已测量的场景配置：' + name)
    for name in ('Characters/Human/Default.usd', 'MotionLibrary/Human/HumanMotionLibrary.usd'):
        if not (simpeople_root / 'assets/cache/behavior' / name).is_file():
            raise RuntimeError('人物缓存不完整：' + name)
    layer = Sdf.Layer.FindOrOpen(str(output / 'scene_overlay.usda'))
    if layer.identifier not in stage.GetSessionLayer().subLayerPaths:
        stage.GetSessionLayer().subLayerPaths.append(layer.identifier)
    executor._simpeople_scene_layer = layer
    manager = omni.kit.app.get_app().get_extension_manager()
    for extension in ('omni.anim.behavior.core', 'omni.anim.navigation.core',
                      'omni.anim.graph.core', 'omni.anim.skelJoint'):
        if not manager.set_extension_enabled_immediate(extension, True):
            raise RuntimeError('扩展加载失败：' + extension)
    # 旧台架USD只有-1~0帧。先拓展时间范围，避免动画时间回绕销毁PhysX视图。
    timeline = omni.timeline.get_timeline_interface()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        timeline.set_end_time(max(timeline.get_end_time(), 100000.))
    timeline.set_looping(False)
    timeline.set_auto_update(True)
    import omni.anim.navigation.core as nav
    if not nav.acquire_interface().get_navmesh():
        if not nav.acquire_interface().start_navmesh_baking_and_wait():
            raise RuntimeError('当前场景NavMesh烘焙失败')
    manager.add_path(str(simpeople_root / 'extensions'))
    settings = carb.settings.get_settings()
    settings.set('/exts/rosclaw.simpeople/semantics_file', str(output / 'semantics.json'))
    settings.set('/exts/rosclaw.simpeople/port', 8768)
    settings.set('/exts/rosclaw.simpeople/allow_unverified', False)
    if not manager.set_extension_enabled_immediate('rosclaw.simpeople', True):
        raise RuntimeError('官方SimPeople附着扩展加载失败')
    executor._simpeople_config = {'output': str(output), 'root': str(simpeople_root)}
    show_ui(executor, output, simpeople_root)
    return {'attached': True, 'independent_simulation': False}


def overview(executor, output):
    from isaacsim.core.rendering_manager import ViewportManager
    # 用现有视口覆盖阀门、机器人、电柜与外侧巡检路线，不新建相机传感器。
    ViewportManager.set_camera_view('/OmniverseKit_Persp',
        eye=[4.5, -5.5, 4.8], target=[.9, -.8, .6])


def valve_demo(executor, output):
    import numpy as np
    from valve_feedback import capture
    from valve_runtime import run
    output = Path(output) / ('valve_demo_' + time.strftime('%Y%m%d_%H%M%S'))
    output.mkdir(parents=True, exist_ok=False)
    active_file = output.parent / 'active_demo.json'
    # 本次已验证布局中阀门2在右手操作位，其它工位原位保留。
    root = '/World/ValveStation2/Valve'
    initial = capture(executor, root)
    native = executor.articulation._physics_view
    if not native.shared_metatype.fixed_base:
        raise RuntimeError('此演示要求已有世界固定底盘，不能自动加固定约束')
    if np.linalg.norm(np.asarray(initial['fixed_anchor']['pose']['position'])[:2] - [-.35, -1.3]) > .01:
        raise RuntimeError('阀门2已离开已验证工位，须重新规划')
    # 与末次+90度反向。轮轴无限转动，抓取朝向仍从本次实测轴角计算。
    previous = output.parent / 'latest_demo.json'
    angle = -90.
    if previous.exists():
        prior = Path(json.loads(previous.read_text())['output']) / 'combined_postcheck.json'
        if prior.exists():
            data = json.loads(prior.read_text())['result']
            if data.get('success'):
                angle = -float(data['target_angle_deg'])
    # 早期失败记录可能曾覆盖latest；只从实际完成记录决定交替方向。
    for prior in sorted(output.parent.glob('valve_demo_*/combined_postcheck.json'), reverse=True):
        data = json.loads(prior.read_text())['result']
        if data.get('success'):
            angle = -float(data['target_angle_deg'])
            break
    roll = (270. + initial['wheel_angle_deg_wrapped']) % 360.
    targets = np.asarray(native.get_dof_velocity_targets()).copy()
    uri = executor._config['robot']['presets']['v3_ctag2f120_rm65']['urdf_path']
    # 对称轮盘允许两个相差180度的夹持朝向；只用实测几何做只读IK候选，执行仍走完整预检。
    from valve_plan import ValvePlanner, PlanningError
    attempts = []
    feasible = []
    planner = ValvePlanner(uri)
    state_names = initial['state']['joint_names']
    current_q = np.asarray([initial['state']['positions'][state_names.index(name)] for name in planner.names])
    for offset in (0., 180.):
        candidate = (roll + offset) % 360.
        try:
            plan = planner.plan(initial, roll_deg=candidate, angle=angle, approach_open_margin_mm=8.)
            first_q = np.asarray(plan['stages']['pre'][0]['q'])
            change = float(np.max(np.abs(first_q - current_q)))
            attempts.append({'roll_deg': candidate, 'ik_passed': True, 'metrics': plan['metrics'],
                             'initial_max_joint_change_rad': change})
            feasible.append((change, candidate))
        except PlanningError as error:
            attempts.append({'roll_deg': candidate, 'ik_passed': False, 'error': str(error)})
    if not feasible:
        (output / 'orientation_candidates.json').write_text(json.dumps(attempts, ensure_ascii=False, indent=2))
        raise RuntimeError('两个夹持朝向均未通过IK，未下发动作')
    # 几何对称不代表机械臂位形相同；优先保留当前肘部/腕部连续性，避免选到大幅翻肘分支。
    roll = min(feasible)[1]
    (output / 'orientation_candidates.json').write_text(json.dumps(attempts, ensure_ascii=False, indent=2))
    overview(executor, output)
    active_file.write_text(json.dumps({'output': str(output), 'running': True}))
    executor.world.play()
    try:
        result = run(executor, valve_root=root, output=output, roll_deg=roll,
            angle=angle, urdf_path=uri, policy={'joint_path_speed_rad_s': .8,
            'turn_joint_speed_rad_s': .8, 'grip_speed_position_s': 180.,
            'grip_open_speed_position_s': 400., 'phase_min_duration_s': .25})
    finally:
        executor.world.pause()
        active_file.write_text(json.dumps({'output': str(output), 'running': False}))
    final = capture(executor, root)
    from scipy.spatial.transform import Rotation
    before_rotation = Rotation.from_quat(initial['base_pose']['orientation'], scalar_first=True)
    after_rotation = Rotation.from_quat(final['base_pose']['orientation'], scalar_first=True)
    report = {'result': result, 'base_translation_m': float(np.linalg.norm(
        np.asarray(final['base_pose']['position']) - initial['base_pose']['position'])),
        'base_rotation_deg': float(np.degrees((after_rotation * before_rotation.inv()).magnitude())),
        'view_valid': bool(native.check()), 'robot_shapes': executor.articulation.num_shapes,
        'velocity_targets_preserved': bool(np.array_equal(targets, native.get_dof_velocity_targets()))}
    (output / 'combined_postcheck.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    if not result['success']:
        raise RuntimeError('阀门动作未完成，保持暂停，详见 ' + str(output))
    (output.parent / 'latest_demo.json').write_text(json.dumps({'output': str(output)}, indent=2))
    return report


def show_ui(executor, output, root):
    import subprocess
    import omni.ui as ui
    output, root = Path(output), Path(root)
    script = Path(__file__).with_name('simpeople_patrol.py')
    python = root / '.controller-venv/bin/python'
    def launch(command):
        # 回调只启动本机HTTP客户端；长动作由已有仿真主线程排队执行。
        with (output / 'panel_commands.log').open('ab') as log:
            log.write((json.dumps({'command': command, 'wall_time': time.time()}) + '\n').encode())
            subprocess.Popen([str(python), str(script), command, '--output', str(output)],
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
    window = ui.Window('人物巡检与阀门演示', width=400, height=290)
    with window.frame:
        with ui.VStack(spacing=8):
            ui.Label('单个工人：电柜外侧两点巡检', height=25)
            ui.Label('机器人：固定底盘、右臂操作阀门2', height=25)
            with ui.HStack(height=32):
                ui.Button('开始巡检', clicked_fn=lambda: launch('start'))
                ui.Button('停止巡检', clicked_fn=lambda: launch('stop'))
            ui.Button('机器人原地拧阀门', height=32, clicked_fn=lambda: launch('demo'))
            with ui.HStack(height=32):
                ui.Button('总览视角', clicked_fn=lambda: launch('view'))
                ui.Button('暂停场景', clicked_fn=lambda: launch('pause'))
            status = ui.Label('正在读取场景状态', height=40, word_wrap=True)
            ui.Label('人物由原生动画驱动；未认证物理接触/避人', height=24)
    executor._simpeople_window = window
    import omni.kit.app
    last_update = [0.]
    def update_status(event):
        if time.monotonic() - last_update[0] < .5:
            return
        last_update[0] = time.monotonic()
        message = '场景播放中' if executor.world.is_playing() else '场景已暂停'
        active = output / 'active_demo.json'
        try:
            if active.exists():
                data = json.loads(active.read_text())
                progress = Path(data['output']) / 'progress.json'
                if progress.exists():
                    row = json.loads(progress.read_text())
                    phase = {'initial_stability': '检查停稳', 'pre': '接近', 'grip': '夹持',
                             'turn': '转动', 'release': '松爪', 'retreat': '退出',
                             'complete': '完成'}.get(row.get('phase'), '预检')
                    if data.get('running'):
                        message += '；阀门：' + phase
                    elif row.get('error'):
                        message += '；动作已停止：' + row['error']
                    else:
                        message += '；阀门动作完成'
        except (OSError, ValueError):
            message += '；状态文件更新中'
        status.text = message
    executor._simpeople_ui_subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        update_status, name='中文巡检面板状态')


def record_viewport(executor, output, duration_s=120):
    """低频截取已有视口并保存时间戳；不建相机或render product。"""
    import omni.kit.app
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    import omni.timeline
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    clock = time.monotonic()
    holder = {'next': 0., 'count': 0}
    def update(event):
        elapsed = time.monotonic() - clock
        if elapsed > duration_s:
            executor._simpeople_record_subscription = None
            return
        if elapsed < holder['next']:
            return
        holder['next'] = elapsed + .1
        filename = f'{holder["count"]:06d}.png'
        capture_viewport_to_file(get_active_viewport(), str(output / filename))
        with (output / 'timestamps.jsonl').open('a') as stream:
            stream.write(json.dumps({'file': filename, 'wall_elapsed_s': elapsed,
                'timeline_s': omni.timeline.get_timeline_interface().get_current_time(),
                'physics_s': float(executor.world.current_time)}) + '\n')
        holder['count'] += 1
    executor._simpeople_record_subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        update, name='现有视口演示录像')
