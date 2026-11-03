#!/usr/bin/env python3
"""L3 机构任务清单；只读资产元数据，普通 Python 即可查看，不依赖 Isaac。"""
import hashlib
import json
import math
from pathlib import Path

from l3_asset_labels import asset_metadata


ASSET_ROOT = Path(__file__).resolve().parents[1] / 'assets' / 'l3_operable'


def build_tasks(asset_root=None, kinds='all'):
    """返回独立功能动作；初态与目标不同，避免把无需运动的样本算作成功。"""
    root = Path(asset_root or ASSET_ROOT)
    catalog = json.loads((root / 'catalog.json').read_text())
    profile = json.loads((root / 'reference_profile.json').read_text())
    selected = {'button', 'selector', 'plate'} if kinds == 'all' else set(
        kinds.split(',') if isinstance(kinds, str) else kinds)
    if not selected or not selected <= {'button', 'selector', 'plate'}:
        raise ValueError('任务类型必须为 all、button、selector 或 plate')
    tasks = []
    for item in catalog:
        kind, asset_id = item['kind'], item['id']
        if kind not in selected:
            continue
        naming = asset_metadata(asset_id, item)
        title = naming['asset_name']
        base = {
            'asset_id': asset_id, 'kind': kind,
            **naming,
            'execution_mode': 'fixture_effort',
            'robot_contact_demonstration': False,
            'calibrated_to_user_hardware': False,
            'source_file': item['source_file'],
            'source_sha256': item['source_sha256'],
            'usd': item['usd'],
            'usd_sha256': hashlib.sha256((root / item['usd']).read_bytes()).hexdigest(),
            'units': {j['name']: 'm' if j['type'] == 'prismatic' else 'rad' for j in item['joints']},
        }

        def add(action, initial, goal, prompt, **extra):
            suffix = ('_' + f"{extra['target_deg']:g}".replace('-', 'minus')) if 'target_deg' in extra else ''
            task_name = {'press': '按下', 'release': '释放', 'plate_out': '投出',
                         'plate_in': '投入'}.get(action, '') + title
            if action == 'selector_to':
                task_name = f"{title}切至{extra['target_deg']:g}°档"
            tasks.append({**base, 'task_id': f'{asset_id}/{action}{suffix}', 'action': action,
                          'task_name': task_name,
                          'initial_positions': initial, 'goal_positions': goal,
                          'prompt_zh': prompt, **extra})

        if kind == 'button':
            # 初始按下只用于复位任务的 reset；采集动作本身仍由外力撤除产生。
            down = min(profile['button']['stroke_m'] * .77,
                       profile['button']['switch_travel_m'] * 1.25)
            add('press', {'button_slide': 0.}, {'button_slide': -down},
                f'按下{title}，确认指示灯亮起。')
            add('release', {'button_slide': -down}, {'button_slide': 0.},
                f'松开已按下的{title}，确认回位且指示灯熄灭。')
        elif kind == 'selector':
            angles = item['detents_deg']
            for index, target in enumerate(angles):
                initial = angles[(index + 1) % len(angles)]
                add('selector_to', {'selector_turn': math.radians(initial)},
                    {'selector_turn': math.radians(target)},
                    f'将{title}从{initial:g}度切换到{target:g}度档并确认保持。',
                    initial_deg=initial, target_deg=target)
        else:
            p = profile['plate']
            # 超过参考解锁点0.2圈，保证投入初态已完整解锁且轴移到位。
            turns = min(p['release_turns'] + .2, p['max_turns'])
            opened = {'upper_slide': turns * p['screw_pitch_m'],
                      'upper_turn': turns * 2 * math.pi,
                      'lower_slide': turns * p['screw_pitch_m'],
                      'lower_turn': turns * 2 * math.pi,
                      'bridge_swing': math.radians(p['swing_deg'])}
            closed = {j['name']: 0. for j in item['joints']}
            add('plate_out', closed, opened,
                f'将{title}的上下螺母旋松，再拨开压板至投出位置。')
            add('plate_in', opened, closed,
                f'将已投出的{title}拨回，再旋紧上下螺母至投入状态。')
    return tasks


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='查看 L3 参考机构采集任务清单')
    parser.add_argument('--tasks', default='all', choices=('all', 'button', 'selector', 'plate'))
    parser.add_argument('--asset-root', type=Path, default=ASSET_ROOT)
    args = parser.parse_args()
    print(json.dumps(build_tasks(args.asset_root, args.tasks), ensure_ascii=False, indent=2))
