#!/usr/bin/env python3
"""SimScene资产/编辑MCP，加上当前RouteManager会话的静态覆盖层入口。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time
import urllib.error
import urllib.request
import uuid

REPO = Path(__file__).resolve().parents[1]
OUTPUT = Path(os.environ.get('SIMSCENE_ROUTE_OUTPUT', REPO / 'outputs/simscene_integration_20261103'))
URL = os.environ.get('ROUTE_MANAGER_URL', 'http://127.0.0.1:18765')


def live(operation, **arguments):
    import urllib.parse
    if urllib.parse.urlparse(URL).hostname not in ('127.0.0.1', 'localhost', '::1'):
        raise ValueError('当前场景适配只接受本机RouteManager')
    OUTPUT.mkdir(parents=True, exist_ok=True)
    result_path = OUTPUT / ('rpc_' + uuid.uuid4().hex + '.json')
    source = REPO / 'scripts/simscene_live.py'
    code = f'import sys,json,types\nfrom pathlib import Path\nsys.path.insert(0,{str(source.parent)!r})\n'
    code += f'simscene_live=sys.modules.get("simscene_live") or types.ModuleType("simscene_live")\nsimscene_live.__file__={str(source)!r}\nsys.modules["simscene_live"]=simscene_live\n'
    code += f'exec(compile(Path({str(source)!r}).read_text(),{str(source)!r},"exec"),simscene_live.__dict__)\n'
    code += f'result=simscene_live.dispatch(executor,{operation!r},json.loads({json.dumps(arguments)!r}))\n'
    code += f'Path({str(result_path)!r}).write_text(json.dumps(result,ensure_ascii=False,allow_nan=False))\n'
    request = urllib.request.Request(URL.rstrip('/') + '/python',
        data=json.dumps({'code': code}).encode(), headers={'Content-Type': 'application/json'})
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    for attempt in range(20):
        try:
            with opener.open(request, timeout=180) as response:
                result = json.load(response)
            break
        except urllib.error.HTTPError as error:
            if error.code != 409 or attempt == 19:
                raise RuntimeError(error.read().decode()) from error
            time.sleep(.2)
    if result.get('status') != 'success' or not result_path.is_file():
        raise RuntimeError('当前仿真主线程没有返回有效结果')
    return json.loads(result_path.read_text())


def preview(scene_id):
    from simscene.scene import SceneStore, validate
    from simscene.usd_export import export_scene
    from simscene.placement import measure
    scene = SceneStore().get(scene_id)
    issues = validate(scene)
    if issues:
        raise ValueError(str(issues))
    for name, obj in scene['objects'].items():
        if obj['kind'] not in ('asset', 'box', 'sphere'):
            raise ValueError('当前附着适配只支持静态视觉资产：' + name)
        if any(obj.get(k) for k in ('dynamic', 'collision', 'pose_follower', 'conveyor', 'spawn_step')):
            raise ValueError('物理/控制器/动态投料不能通过视觉附着入口启用：' + name)
        if obj['kind'] == 'asset' and (obj.get('asset_physics') != 'visual_only' or obj.get('asset_lights') != 'disabled'):
            raise ValueError('导入资产须显式asset_physics=visual_only、asset_lights=disabled：' + name)
    roots = {obj['asset_path']: hashlib.sha256(Path(obj['asset_path']).read_bytes()).hexdigest()
             for obj in scene['objects'].values() if obj['kind'] == 'asset'}
    digest = hashlib.sha256(json.dumps({'scene': scene, 'asset_roots': roots}, sort_keys=True).encode()).hexdigest()
    folder = OUTPUT / 'exports' / scene_id / digest[:16]
    folder.mkdir(parents=True, exist_ok=True)
    destination = folder / 'scene.usda'
    exported = export_scene(scene, destination) if not destination.exists() else {'path': str(destination), 'reused': True}
    return {'scene_id': scene_id, 'revision': scene['revision'], 'spec_sha256': digest,
            'export': exported, 'measurement': measure(scene, list(scene['objects'])),
            'asset_root_hashes': roots, 'dependency_policy': 'external files, not snapshotted',
            'applied_to_live_scene': False, 'physics_validation': 'not run'}


def install_tools():
    from simscene import mcp_server as official
    # 用户要求附着既有会话；从实际工具表移除会启动独立worker或控制其job的入口。
    for name in ('simulation_start', 'simulation_pause', 'simulation_resume', 'robot_prepare_profile',
                 'run_observe', 'run_contacts', 'run_verify_lock', 'run_verify', 'run_verify_evidence',
                 'run_video_chunk', 'scene_observe', 'job_cancel', 'run_presentation_events', 'scene_plan_synchronize'):
        official.server.remove_tool(name)

    @official.threaded_tool(annotations=official._read_local)
    def attached_scene_info() -> dict:
        """读取当前RouteManager/Isaac会话、机器人反馈与场景对象，不启动仿真。"""
        return live('inspect')

    @official.threaded_tool(annotations=official._read_local)
    def attached_scene_preview(scene_id: str) -> dict:
        """测量并导出SimScene保存的静态视觉场景；不改当前运行场景。"""
        return preview(scene_id)

    @official.threaded_tool()
    def attached_scene_import(scene_id: str, expected_stage: str, expected_revision: int) -> dict:
        """把已预览的静态视觉层附着当前暂停会话。要求场景身份/版本一致，保护现有机器人与巡检区域。"""
        plan = preview(scene_id)
        if plan['revision'] != expected_revision:
            raise ValueError('SimScene版本已改变，请重新预览')
        return live('attach', scene_id=scene_id, source=plan['export']['path'],
                    expected_stage=expected_stage, output=str(OUTPUT))

    @official.threaded_tool()
    def attached_scene_remove(scene_id: str, expected_stage: str) -> dict:
        """移除本适配器拥有的静态覆盖层；须暂停，会验证原生反馈和目标保留，不reset。"""
        return live('remove', scene_id=scene_id, expected_stage=expected_stage)

    return official.server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inspect', action='store_true')
    parser.add_argument('--preview')
    args = parser.parse_args()
    if args.inspect:
        print(json.dumps(live('inspect'), ensure_ascii=False, indent=2))
    elif args.preview:
        print(json.dumps(preview(args.preview), ensure_ascii=False, indent=2))
    else:
        install_tools().run(transport='stdio')


if __name__ == '__main__':
    main()
