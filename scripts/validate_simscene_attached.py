#!/usr/bin/env python3
"""真实stdio MCP与现有暂停Kit会话的附着烟测，不启动另一套仿真。"""
import asyncio
import json
import os
from pathlib import Path
import sys
import tempfile


async def check(python, output):
    from mcp import ClientSession, StdioServerParameters
    from mcp.client.stdio import stdio_client
    script = Path(__file__).with_name('simscene_mcp.py')
    with tempfile.TemporaryDirectory(prefix='simscene_mcp_case_') as temporary:
        env = dict(os.environ, SIMSCENE_HOME=temporary, SIMSCENE_ROUTE_OUTPUT=str(output))
        params = StdioServerParameters(command='/usr/bin/env',
            args=['-u', 'PYTHONPATH', '-u', 'LD_LIBRARY_PATH', python, str(script)], env=env)
        async with stdio_client(params) as (reader, writer):
            async with ClientSession(reader, writer) as session:
                await session.initialize()
                async def call(name, args=None):
                    r = await session.call_tool(name, args or {})
                    if r.isError:
                        raise RuntimeError(str(r.content))
                    return r.structuredContent or json.loads(r.content[0].text)
                names = [t.name for t in (await session.list_tools()).tools]
                assert 'asset_search' in names and 'attached_scene_import' in names
                assert 'simulation_start' not in names and 'robot_prepare_profile' not in names
                before = await call('attached_scene_info')
                assert not before['playing'] and before['robot_view_valid']
                scene = await call('scene_open', {'scene_id': 'integration_probe'})
                edited = await call('scene_apply', {'scene_id': 'integration_probe', 'base_revision': scene['revision'],
                    'request_id': 'create_probe', 'operations': [{'op': 'add', 'id': 'probe',
                        'object': {'kind': 'box', 'size': [.02, .02, .02], 'position': [-4., -4., 1.],
                                   'color': [.2, .5, .8], 'dynamic': False, 'collision': False}}]})
                plan = await call('attached_scene_preview', {'scene_id': 'integration_probe'})
                imported = await call('attached_scene_import', {'scene_id': 'integration_probe',
                    'expected_stage': before['stage_id'], 'expected_revision': edited['revision']})
                assert imported['attached'] and all(imported['state_preserved'].values())
                removed = await call('attached_scene_remove', {'scene_id': 'integration_probe',
                    'expected_stage': before['stage_id']})
                assert all(removed['state_preserved'].values())
                # 侵入机器人作业区的可视物也必须拒绝，且不能先写入场景再检查。
                await call('scene_apply', {'scene_id': 'integration_probe', 'base_revision': edited['revision'],
                    'request_id': 'overlap_probe', 'operations': [{'op': 'update', 'id': 'probe',
                        'changes': {'position': [0., -.8, .8], 'size': [1., 1., 1.]}}]})
                blocked = await session.call_tool('attached_scene_import', {'scene_id': 'integration_probe',
                    'expected_stage': before['stage_id'], 'expected_revision': edited['revision'] + 1})
                assert blocked.isError
                after = await call('attached_scene_info')
                assert after['pid'] == before['pid'] and after['stage_id'] == before['stage_id']
                assert after['robot_shapes'] == before['robot_shapes'] == 96
                assert after['robot_view_valid'] and after['finite_q'] and not after['playing']
                assert not after['owned']
                result = {'passed': True, 'tools': names, 'before': before, 'preview': plan,
                    'import': imported, 'remove': removed, 'overlap_rejected': blocked.isError,
                    'after': after, 'independent_simulation_started': False}
                output.mkdir(parents=True, exist_ok=True)
                (output / 'mcp_acceptance.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
                print(json.dumps({'passed': True, 'tools': len(names), 'same_pid': before['pid'],
                    'robot_shapes': after['robot_shapes'], 'import_remove_preserved': True,
                    'overlap_rejected': True, 'scene_paused': True}, ensure_ascii=False))


if __name__ == '__main__':
    output = Path(__file__).resolve().parents[1] / 'outputs/simscene_integration_20261103'
    asyncio.run(check(sys.executable, output))
