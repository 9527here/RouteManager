#!/usr/bin/env python3
"""在 RouteManager 空白服务加载机构，或读取主线程状态机反馈。"""
import argparse
import json
import os
import urllib.request
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['setup', 'attach', 'status'])
    parser.add_argument('--url', default=os.environ.get('ROUTE_MANAGER_URL', 'http://127.0.0.1:18765'))
    parser.add_argument('--asset-root', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'assets/oxford_devices')
    parser.add_argument('--output', type=Path, default=Path('outputs/oxford_migration'))
    parser.add_argument('--prefix', default='/World/Devices')
    args = parser.parse_args()
    root, output = args.asset_root.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    result = output / 'runtime_status.json'
    module_dir = str(Path(__file__).resolve().parent)
    common = f'import sys, json\nfrom pathlib import Path\nsys.path.insert(0, {module_dir!r})\nimport omni.usd\n'
    if args.action == 'setup':
        code = common + f'''
if executor.articulation is not None:
    raise RuntimeError('人形已加载；请保留当前场景，在空白服务使用 setup')
stage = omni.usd.get_context().get_stage()
if stage.GetPrimAtPath({args.prefix!r}):
    raise RuntimeError('机构已加载，请改用 attach 或 status')
executor.world.stop()
executor.load_scene({str(root / 'gallery.usda')!r}, '/World')
executor.world.reset()
from oxford_device_runtime import attach_runtime
executor._oxford_devices = attach_runtime(executor.world, stage, {args.prefix!r})
for _ in range(180):
    executor.step(render=True)
from isaacsim.core.utils.viewports import set_camera_view
set_camera_view(eye=[0.8,-1.2,1.25], target=[0.09,0,0.87])
'''
    elif args.action == 'attach':
        code = common + f'''
if hasattr(executor, '_oxford_devices'):
    raise RuntimeError('已有状态机；重复 attach 会留下重复回调')
from oxford_device_runtime import attach_runtime
executor._oxford_devices = attach_runtime(executor.world, omni.usd.get_context().get_stage(), {args.prefix!r})
'''
    else:
        code = common
    code += f"\nexecutor._oxford_devices.step(0)\nPath({str(result)!r}).write_text(json.dumps(executor._oxford_devices.snapshot(), indent=2))\n"
    request = urllib.request.Request(args.url.rstrip('/') + '/python',
                                     data=json.dumps({'code': code}).encode(),
                                     headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=120) as response:
        status = json.load(response)
    if status.get('status') not in ('success', 'ok'):
        raise RuntimeError(status)
    print(result.read_text())


if __name__ == '__main__':
    main()
