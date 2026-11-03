#!/usr/bin/env python3
"""阀门操作入口：现场只读采样、离线规划、避碰预检、物理执行及合作停止。"""
import argparse
import json
from pathlib import Path
import sys
import time
import urllib.error
import urllib.request

REPO = Path(__file__).resolve().parents[1]
OUTPUT = REPO / 'outputs/valve_operation_20261102'


def remote(code, url='http://127.0.0.1:18765', timeout=120):
    body = json.dumps({'code': code}).encode()
    for attempt in range(30):
        request = urllib.request.Request(url.rstrip('/') + '/python', data=body,
                                         headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                result = json.loads(response.read().decode())
            if result.get('status') != 'success':
                raise RuntimeError('仿真主线程执行失败: ' + str(result))
            return result
        except urllib.error.HTTPError as error:
            if error.code != 409 or attempt == 29:
                raise RuntimeError(f'HTTP {error.code}: ' + error.read().decode()) from error
            time.sleep(.2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['setup', 'capture', 'plan', 'preflight', 'run', 'status', 'stop'])
    parser.add_argument('--url', default='http://127.0.0.1:18765')
    parser.add_argument('--output', type=Path, default=OUTPUT)
    parser.add_argument('--snapshot', type=Path)
    parser.add_argument('--valve-root', default='/World/ValveStation/Valve')
    parser.add_argument('--position', type=float, nargs=3,
                        help='setup 必须显式给出已规划的工位根世界坐标(m)')
    parser.add_argument('--source', default='/home/jy/datasets/vla_data/L2_仿真运行/场景/操作台_piper_table_v2/asserts/valve/valve_1/valve_asset.usd')
    parser.add_argument('--body-xy', type=float, nargs=2, default=[0., 0.])
    parser.add_argument('--lift-delta', type=float, default=0.)
    parser.add_argument('--roll', type=float, default=0.)
    parser.add_argument('--angle', type=float, default=90.)
    parser.add_argument('--joint-speed', type=float, help='机械臂参考关节速度(rad/s)，仍受真实限位与跟踪门限制')
    parser.add_argument('--turn-speed', type=float, help='夹持后转动的参考关节速度(rad/s)')
    parser.add_argument('--grip-speed', type=float, help='夹爪接触前慢段/预紧速度(position单位/s)')
    parser.add_argument('--grip-fast-speed', type=float, help='夹爪无接触空行程速度(position单位/s)')
    parser.add_argument('--approach-open-margin', type=float, help='按实测夹宽预张开的总余量(mm)')
    parser.add_argument('--grip-slow-margin', type=float, help='预计接触前切换慢速的总间隙(mm)')
    parser.add_argument('--open-speed', type=float, help='夹爪张开速度(position单位/s)')
    parser.add_argument('--phase-duration', type=float, help='单阶段最短仿真时间(s)')
    parser.add_argument('--grasp-axial-offset', type=float, default=0.,
                        help='抓取 TCP 相对实测轮盘中心沿轮轴的偏移(m)，默认0')
    parser.add_argument('--audit-static-assembly', action='store_true',
                        help='显式审计不随右臂运动改变的既存底盘装配重叠；保留全部碰撞记录')
    parser.add_argument('--legacy-grasp-reference', action='store_true',
                        help='仅离线plan：历史快照缺网格支持点时，显式使用轴对齐bbox参考；不可作为执行依据')
    args = parser.parse_args()
    if args.legacy_grasp_reference and args.command != 'plan':
        parser.error('--legacy-grasp-reference 仅用于离线plan，现场必须重新采真实几何')
    if args.command == 'setup' and args.position is None:
        parser.error('setup 必须显式提供 --position X Y Z，不能假设现场布局')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.command == 'status':
        path = output / 'progress.json'
        print(path.read_text() if path.exists() else '尚无运行记录')
        return
    if args.command == 'stop':
        (output / 'STOP').write_text('用户请求停止\n')
        print('已请求合作停止，运行时会停轮并保持手臂当前反馈位置')
        return
    if args.command == 'plan':
        if args.snapshot is None:
            parser.error('离线 plan 必须显式提供 --snapshot')
        sys.path.insert(0, str(REPO))
        from valve_plan import ValvePlanner
        config = json.loads((REPO / 'config.json').read_text())
        preset = config['robot']['presets'][config['robot']['name']]
        snapshot = json.loads(args.snapshot.read_text())
        plan = ValvePlanner(preset['urdf_path']).plan(
            snapshot, body_xy=args.body_xy, lift_delta=args.lift_delta,
            roll_deg=args.roll, angle=args.angle, grasp_axial_offset_m=args.grasp_axial_offset,
            allow_legacy_grasp_reference=args.legacy_grasp_reference,
            approach_open_margin_mm=args.approach_open_margin)
        (output / 'plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2))
        print(json.dumps({**plan['metrics'], 'grasp_support_width_mm': plan['grasp_support_width_mm'],
                          'legacy_grasp_reference_only': plan['legacy_grasp_reference_only']}, ensure_ascii=False, indent=2))
        return
    prefix = f'import sys, json, importlib\nfrom pathlib import Path\nsys.path.insert(0, {str(REPO / "scripts")!r})\nimportlib.invalidate_caches()\n'
    # 长会话可能同时由脚本入口与package入口导入；先刷新几何/数学依赖，再刷新持有from-import的调用方。
    # 不重载RobotManager，避免部署其他聊天尚未验收的执行器改动。
    prefix += "for name in ('scripts.robot_collision_guard','robot_collision_guard','scripts.valve_ik','valve_ik','scripts.valve_feedback','valve_feedback','scripts.valve_feedback_cache','valve_feedback_cache','scripts.valve_plan','valve_plan','scripts.valve_safety','valve_safety','scripts.valve_transit','valve_transit','scripts.valve_grip_stability','valve_grip_stability','scripts.valve_runtime','valve_runtime','scripts.valve_station','valve_station'):\n    if name in sys.modules: importlib.reload(sys.modules[name])\n"
    if args.command == 'setup':
        code = prefix + 'from valve_station import setup\n'
        code += f'result=setup(executor, {args.position!r}, {str(output)!r}, {args.source!r})\n'
    elif args.command == 'capture':
        code = prefix + 'from valve_feedback import capture\n'
        code += f'result=capture(executor, {args.valve_root!r})\n'
        code += f'Path({str(output / "snapshot.json")!r}).write_text(json.dumps(result,ensure_ascii=False,indent=2))\n'
    else:
        code = prefix + 'from valve_runtime import preflight, run\n'
        function = 'run' if args.command == 'run' else 'preflight'
        policy = {key:value for key,value in (('joint_path_speed_rad_s',args.joint_speed),
                  ('turn_joint_speed_rad_s',args.turn_speed),
                  ('grip_speed_position_s',args.grip_speed),('grip_open_speed_position_s',args.open_speed),
                  ('grip_fast_speed_position_s',args.grip_fast_speed),
                  ('grip_approach_margin_mm',args.approach_open_margin),
                  ('grip_slow_margin_mm',args.grip_slow_margin),
                  ('phase_min_duration_s',args.phase_duration)) if value is not None}
        code += f'result={function}(executor, valve_root={args.valve_root!r}, output={str(output)!r}, body_xy={args.body_xy!r}, lift_delta={args.lift_delta!r}, roll_deg={args.roll!r}, angle={args.angle!r}, allow_invariant_assembly={args.audit_static_assembly!r}, grasp_axial_offset_m={args.grasp_axial_offset!r}, policy={policy!r})\n'
    print(json.dumps(remote(code, args.url, timeout=1200 if args.command == 'run' else 300), ensure_ascii=False))
    for name in ({'setup': 'station_setup.json', 'capture': 'snapshot.json',
                  'preflight': 'preflight.json', 'run': 'result.json'}.get(args.command),):
        if name and (output / name).exists():
            print('结果文件: ' + str(output / name))
            if args.command in ('preflight', 'run'):
                report = json.loads((output / name).read_text())
                passed = bool(report.get('safe' if args.command == 'preflight' else 'success', False))
                print(json.dumps({key: report[key] for key in ('safe', 'success', 'phase', 'reason', 'error',
                    'motion_started', 'target_angle_deg', 'actual_angle_deg', 'plan_metrics', 'metrics') if key in report}, ensure_ascii=False, indent=2))
                if not passed:
                    raise SystemExit(1)


if __name__ == '__main__':
    main()
