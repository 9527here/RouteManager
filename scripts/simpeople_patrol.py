#!/usr/bin/env python3
"""控制已附着当前RouteManager场景的单人巡检，不启动另一套仿真。"""
import argparse
import json
import os
from pathlib import Path
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid

REPO = Path(__file__).resolve().parents[1]
ACTOR = 'inspector_001'
CONTROLLER = 'routemanager-patrol'


def rpc(method, params=None, endpoint='http://127.0.0.1:8768'):
    parsed = urllib.parse.urlparse(endpoint)
    if parsed.scheme != 'http' or parsed.hostname not in ('localhost', '127.0.0.1', '::1'):
        raise ValueError('人物控制端点必须为本机HTTP')
    request = urllib.request.Request(endpoint.rstrip('/') + '/rpc',
        data=json.dumps({'method': 'people.' + method, 'params': params or {}}).encode(),
        headers={'Content-Type': 'application/json'})
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(request, timeout=35) as response:
        result = json.load(response)
    if 'error' in result:
        raise RuntimeError(json.dumps(result['error'], ensure_ascii=False))
    return result['result']


def write_call(method, endpoint, **params):
    return rpc(method, {'request_id': uuid.uuid4().hex,
                       'controller_id': CONTROLLER, **params}, endpoint)


def remote(code, url):
    from valve_operation import remote as execute
    return execute(code, url=url, timeout=1200)


def state(endpoint):
    return rpc('get_state', {'actor_id': ACTOR}, endpoint)


def start(endpoint, url, cycles):
    deadline = time.monotonic() + 60
    while True:
        try:
            ready = rpc('preflight', endpoint=endpoint)['ready']
            break
        except urllib.error.URLError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(.25)
    if not ready:
        raise RuntimeError('人物预检未通过')
    write_call('control_session', endpoint, action='acquire')
    info = rpc('inspect_scene', endpoint=endpoint)
    if ACTOR not in info['actors']:
        write_call('spawn', endpoint, actor_id=ACTOR, asset_id='default',
                   spawn_anchor='patrol_a', role='inspector', group='cabinet_patrol')
    remote('executor.world.play()', url)
    deadline = time.monotonic() + 60
    while not state(endpoint)['actors'][0]['observation'].get('ready'):
        if time.monotonic() >= deadline:
            raise RuntimeError('原生人物未就绪，不能把accepted当成功')
        time.sleep(.25)
    # 每个动作由原生运行时推进；控制端退出后活动仍保留。
    steps = []
    for _ in range(cycles):
        for target in ('patrol_b', 'patrol_a'):
            steps.extend([{'action': 'move_to', 'target': target, 'gait': 'walk',
                           'speed_mps': 1.2, 'tolerance_m': .12, 'timeout_s': 40},
                          {'action': 'wait', 'duration_s': 1.5}])
    return write_call('run_activity', endpoint, actor_id=ACTOR, steps=steps)


def stop(endpoint, url):
    initial = state(endpoint)
    before = initial['actors'][0]
    task = before.get('task')
    if not task or task['status'] in ('succeeded', 'cancelled', 'failed'):
        return before
    write_call('cancel', endpoint, actor_id=ACTOR)
    # 取消需要真实仿真帧让原生人物刹停，不能暂停后报告零速度。
    if initial['paused']:
        remote('executor.world.play()', url)
    deadline = time.monotonic() + 60
    while True:
        actor = state(endpoint)['actors'][0]
        if actor['task']['status'] == 'cancelled' and actor['observation'].get('speed_mps', 1) < .05:
            return actor
        if actor['task']['status'] == 'failed' or time.monotonic() >= deadline:
            raise RuntimeError('人物取消未完成停稳: ' + json.dumps(actor, ensure_ascii=False))
        time.sleep(.25)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['start', 'stop', 'status', 'view', 'demo', 'attach', 'pause'])
    parser.add_argument('--url', default=os.environ.get('ROUTE_MANAGER_URL', 'http://127.0.0.1:18765'))
    parser.add_argument('--endpoint', default=os.environ.get('SIMPEOPLE_ENDPOINT', 'http://127.0.0.1:8768'))
    parser.add_argument('--cycles', type=int, default=100)
    parser.add_argument('--output', type=Path, default=REPO / 'outputs/simpeople_patrol_20261103')
    parser.add_argument('--simpeople-root', type=Path, default=Path(os.environ.get('SIMPEOPLE_ROOT', Path.home() / 'simpeople')))
    args = parser.parse_args()
    if not 1 <= args.cycles <= 250:
        parser.error('cycles必须在1~250之间')
    if args.command == 'start':
        result = start(args.endpoint, args.url, args.cycles)
    elif args.command == 'stop':
        result = stop(args.endpoint, args.url)
    elif args.command == 'status':
        result = state(args.endpoint)
    elif args.command == 'pause':
        active = args.output / 'active_demo.json'
        if active.exists():
            task = json.loads(active.read_text())
            if task.get('running'):
                (Path(task['output']) / 'STOP').write_text('用户请求暂停演示\n')
        result = stop(args.endpoint, args.url)
        remote('executor.world.pause()', args.url)
    else:
        prefix = f'import sys\nsys.path.insert(0,{str(REPO / "scripts")!r})\n'
        prefix += 'import simpeople_scene\n'
        # Kit fast importer会缓存旧模块，直接编译磁盘源码才能可靠热更新这些无状态入口。
        source = str(REPO / 'scripts/simpeople_scene.py')
        prefix += f'from pathlib import Path\nexec(compile(Path({source!r}).read_text(),{source!r},"exec"),simpeople_scene.__dict__)\n'
        function = {'view': 'overview', 'demo': 'valve_demo', 'attach': 'attach'}[args.command]
        code = prefix + f'simpeople_scene.{function}(executor, {str(args.output.resolve())!r}'
        if args.command == 'attach':
            code += f', {str(args.simpeople_root.resolve())!r}'
        result = remote(code + ')', args.url)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
