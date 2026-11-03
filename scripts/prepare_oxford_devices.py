#!/usr/bin/env python3
"""展开上游 Xacro 为可移植 URDF；原件始终保留在 upstream。"""
import argparse
import hashlib
import json
import os
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from fetch_oxford_devices import DEVICES, PACKAGE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('asset_root', type=Path)
    parser.add_argument('--ros-python-path', type=Path,
                        help='系统 xacro 模块目录，不传入 Isaac 进程')
    args = parser.parse_args()
    root = args.asset_root.resolve()
    provenance = json.loads((root / 'provenance.json').read_text())
    for item in provenance['files']:
        assert hashlib.sha256((root / item['path']).read_bytes()).hexdigest() == item['sha256']
    upstream = root / 'upstream'
    destination = root / 'urdf'
    destination.mkdir(exist_ok=True)
    env = os.environ.copy()
    if args.ros_python_path:
        env['PYTHONPATH'] = str(args.ros_python_path)
    audit = []
    with tempfile.TemporaryDirectory(prefix='oxford_xacro_') as temporary:
        temporary = Path(temporary)
        # $(find ...) 只用于 include，替换到隔离副本，免装 ROS1/catkin。
        for source in upstream.rglob('*'):
            if source.suffix not in ('.xacro', '.gazebo'):
                continue
            target = temporary / source.relative_to(upstream)
            target.parent.mkdir(parents=True, exist_ok=True)
            text = source.read_text().replace(f'$(find {PACKAGE})', str(temporary / PACKAGE))
            target.write_text(text)
        for name in DEVICES:
            source = temporary / PACKAGE / 'devices' / name / 'model' / f'{name}.urdf.xacro'
            command = ['/usr/bin/python3', '-c',
                       'import xacro,sys; print(xacro.process_file(sys.argv[1]).toxml())', str(source)]
            result = subprocess.run(command, env=env, text=True, capture_output=True, check=True)
            robot = ET.fromstring(result.stdout)
            # Gazebo 插件和传动不会在 Isaac 生效；由项目运行时实现状态机。
            removed = []
            for child in list(robot):
                if child.tag in ('gazebo', 'transmission') or (
                        child.tag == 'link' and child.get('name') == 'world') or (
                        child.tag == 'joint' and child.find('parent').get('link') == 'world'):
                    removed.append(child.tag + ':' + child.get('name', ''))
                    robot.remove(child)
            for mesh in robot.iter('mesh'):
                filename = mesh.get('filename')
                if filename.startswith('package://'):
                    mesh.set('filename', '../upstream/' + filename[len('package://'):])
                    assert (destination / mesh.get('filename')).resolve().exists()
            # 上游 grip visual 原点高度用了 height1(18mm)，collision 用 height3(30mm)。
            # 以已有 collision 为准对齐外观，避免机器人看见与接触相差 6mm。
            fixes = []
            if name == 'isolator_switch':
                grip = robot.find("link[@name='grip']")
                grip.find('visual/origin').set('xyz', grip.find('collision/origin').get('xyz'))
                fixes.append('grip visual origin aligned to collision: z 0.009 -> 0.015 m')
            ET.indent(robot)
            path = destination / f'{name}.urdf'
            ET.ElementTree(robot).write(path, encoding='utf-8', xml_declaration=True)
            joints = [{'name': j.get('name'), 'type': j.get('type'),
                       'axis': j.find('axis').attrib if j.find('axis') is not None else None,
                       'limit': j.find('limit').attrib if j.find('limit') is not None else None}
                      for j in robot.findall('joint')]
            audit.append({'id': name, 'urdf': str(path.relative_to(root)),
                          'removed': removed, 'fixes': fixes, 'joints': joints,
                          'collision_count': len(robot.findall('.//collision')),
                          'parameters_calibrated': False})
    (root / 'preparation.json').write_text(json.dumps(audit, indent=2) + '\n')
    print('PREPARED', len(audit))


if __name__ == '__main__':
    main()
