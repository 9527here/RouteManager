#!/usr/bin/env python3
"""把外部资产库链接到源码入口；不复制资产，不改预设或运行中的仿真。"""
import argparse
import json
import os
from pathlib import Path


def main():
    repo = Path(__file__).resolve().parents[1]
    registry = json.loads((repo / 'third_party/libraries.json').read_text())
    library = next(item for item in registry['libraries'] if item['id'] == 'vla_assets')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['link', 'status'])
    parser.add_argument('--root', type=Path, help='外部资产库目录，优先于环境变量')
    args = parser.parse_args()
    configured = args.root or os.environ.get(library['root_env']) or library['default_root']
    root = Path(configured).expanduser().resolve()
    link = repo / library['link_path']
    if args.command == 'link':
        if not root.is_dir():
            parser.error(f'资产库目录不存在：{root}')
        # 禁止把源码本身或其祖先/子目录接回来，避免递归遍历和误归档。
        if root == repo or root in repo.parents or repo in root.parents:
            parser.error('资产库须位于源码仓库外，且不能是源码的父目录')
        if link.is_symlink():
            if link.resolve() != root:
                parser.error(f'已有链接指向其它位置，保留原链接：{link.readlink()}')
        elif link.exists():
            parser.error(f'入口已有实体文件或目录，拒绝覆盖：{link}')
        else:
            link.parent.mkdir(parents=True, exist_ok=True)
            link.symlink_to(root, target_is_directory=True)
    # status只读现有入口；不要求每次记住建链接时的--root或临时环境变量。
    target = link.resolve() if link.is_symlink() else None
    result = {
        'library': library['id'], 'link': str(link),
        'configured_root': str(root),
        'linked_root': str(target) if target else None,
        'ready': bool(target and target.is_dir()),
        'layers': {name: (target / name).is_dir() if target else False
                   for name in library['layers']},
        'note': '只检查链接和层级目录；不代表全部资产、USD依赖或许可证已验收',
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if not result['ready']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
