#!/usr/bin/env python3
"""实测中文操作：物理走位、停稳、首次动作门控、实际机构反馈和动作期间底盘漂移。"""
import argparse
import inspect
import json
import urllib.request


def validate(executor):
    import json
    import math
    import numpy as np
    station = executor._l3_station
    report = {'method': '真实四轮八驱走位后执行中文动作，无机器人位姿瞬移', 'checks': []}
    output = station.output.parent.parent/'mobile_operations'/'live_actions_validation.json'
    def save():
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    save()
    for name, title in [('object_0_variant', '按压并复位'), ('object_19', '切至 45°')]:
        station.select(name)
        approach = station._get_approach()
        before = approach.pose()[0].copy()
        count = len(station._action_history)
        first = None
        first_pose = None
        drift_xy = drift_yaw = 0.
        early = False
        station.ui_callbacks[title]()
        initial = {'pending': bool(station._pending_action), 'fixture_queue_empty': not bool(station.queue)}
        for frame in range(40000):
            executor.step(render=(frame%8 == 0))
            pose = approach.pose()[0]
            if station.queue:
                if first is None:
                    first_pose = pose.copy()
                    first = {'at_target': approach.at_target(name), 'phase': approach.state['phase'],
                             'stationary': approach.state.get('stationary_verified', False), 'pose': pose.tolist()}
                    early = not (first['at_target'] and first['stationary'] and first['phase'] == 'arrived')
                drift_xy = max(drift_xy, float(np.linalg.norm(pose[:2]-first_pose[:2])))
                drift_yaw = max(drift_yaw, abs((pose[2]-first_pose[2]+math.pi)%(2*math.pi)-math.pi))
            if frame%240 == 0:
                report['in_progress'] = {'asset': name, 'state': dict(approach.state), 'frame': frame}
                save()
            if not station._approach_busy() and not station.queue and station._action_job is None:
                break
        row = station._action_history[-1] if len(station._action_history)>count else {}
        report['checks'].append({'asset': name, 'asset_name': station._asset_name('l3', name),
            'pass': bool(row.get('pass')) and first is not None and not early,
            'initial': initial, 'first_operation': first, 'premature_operation': early,
            'movement_xy_m': float(np.linalg.norm(pose[:2]-before[:2])),
            'yaw_change_deg': math.degrees((pose[2]-before[2]+math.pi)%(2*math.pi)-math.pi),
            'operation_max_base_drift_m': drift_xy, 'operation_max_base_yaw_drift_deg': math.degrees(drift_yaw),
            'navigation': dict(approach.state), 'action_result': row})
        save()
        if not report['checks'][-1]['pass']:
            break
    report.pop('in_progress', None)
    report['pass'] = len(report['checks']) == 2 and all(x['pass'] for x in report['checks'])
    report['robot_finite'] = bool(np.isfinite(executor.articulation.get_joint_positions()).all())
    save()
    station.write()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:18765')
    args = parser.parse_args()
    code = inspect.getsource(validate)+'\nvalidate(executor)\n'
    req = urllib.request.Request(args.url.rstrip('/')+'/python', data=json.dumps({'code': code}).encode(),
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=600) as response:
        print(response.read().decode())


if __name__ == '__main__':
    main()
