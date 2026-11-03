#!/usr/bin/env python3
"""下载固定版本的 Oxford 电气机构子集，保留原文件与 SHA256。"""
import argparse
import concurrent.futures
import hashlib
import json
import re
from pathlib import Path, PurePosixPath

import requests

REVISION = '3a5b26c785709ffb88048c917d6de84d234bce72'
REPOSITORY = 'https://github.com/ori-arg/manipulation_playground'
PACKAGE = 'manipulation_playground_articulated_devices'
DEVICES = ('e_stop_button', 'isolator_switch', 'safety_switch')


def fetch(url):
    response = requests.get(url, timeout=(10, 45))
    response.raise_for_status()
    return response.content


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', type=Path, required=True)
    args = parser.parse_args()
    root = args.target.resolve()
    upstream = root / 'upstream'
    paths = {'LICENSE', 'README.md', f'{PACKAGE}/package.xml'}
    pending = [f'{PACKAGE}/devices/{name}' for name in DEVICES]
    seen = set()
    # 只抓取目标机构目录，避免阀门资源和图片让整库下载过大。
    while pending:
        directory = pending.pop()
        if directory in seen:
            continue
        seen.add(directory)
        html = fetch(f'{REPOSITORY}/tree/{REVISION}/{directory}').decode()
        entries = re.findall(r'"name":"[^"]+","path":"([^"]+)","contentType":"([^"]+)"', html)
        children = [(p, kind) for p, kind in entries
                    if str(PurePosixPath(p).parent) == directory]
        if not children:
            raise RuntimeError(f'无法解析目录：{directory}')
        for path, kind in children:
            if kind == 'directory':
                pending.append(path)
            else:
                paths.add(path)
        print('INDEX', directory, len(children), flush=True)

    def download(path):
        url = f'https://raw.githubusercontent.com/ori-arg/manipulation_playground/{REVISION}/{path}'
        data = fetch(url)
        target = upstream / path
        target.parent.mkdir(parents=True, exist_ok=True)
        # 同版本重复运行允许恢复；不覆盖意外改过的源文件。
        if target.exists() and target.read_bytes() != data:
            raise RuntimeError(f'源文件已改变，拒绝覆盖：{target}')
        target.write_bytes(data)
        return {'path': str(target.relative_to(root)), 'url': url,
                'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}

    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        files = list(pool.map(download, sorted(paths)))
    result = {'repository': REPOSITORY, 'revision': REVISION,
              'license': 'BSD-3-Clause', 'devices': list(DEVICES), 'files': files}
    root.mkdir(parents=True, exist_ok=True)
    (root / 'provenance.json').write_text(json.dumps(result, indent=2) + '\n')
    (root / 'LICENSE').write_bytes((upstream / 'LICENSE').read_bytes())
    print('FETCH_COMPLETE', len(files), sum(f['bytes'] for f in files), flush=True)


if __name__ == '__main__':
    main()
