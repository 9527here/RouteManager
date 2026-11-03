#!/usr/bin/env python3
"""从保留的Git历史恢复旧机器人包的已跟踪部分；不覆盖不同内容，不恢复私有大件。"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def restore(destination, dry_run=False, link=False):
    repo = Path(__file__).resolve().parents[1]
    manifest = json.loads((repo / 'assets/legacy_humanoid_manifest.json').read_text())
    revision = manifest['source_commit']
    if subprocess.run(['git', 'cat-file', '-e', revision + '^{commit}'], cwd=repo,
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode:
        raise RuntimeError('本地缺少旧提交；浅克隆请先获取完整历史，再重试')
    destination = Path(destination).expanduser().resolve()
    missing = []
    for entry in manifest['files']:
        relative = Path(entry['path']).relative_to(manifest['source_root'])
        target = destination / relative
        if target.exists():
            if hashlib.sha256(target.read_bytes()).hexdigest() != entry['sha256']:
                raise RuntimeError('已有文件与记录不同，拒绝覆盖：' + str(target))
        else:
            missing.append((entry, target))
    if not dry_run:
        for entry, target in missing:
            data = subprocess.check_output(['git', 'show', revision + ':' + entry['path']], cwd=repo)
            if hashlib.sha256(data).hexdigest() != entry['sha256']:
                raise RuntimeError('历史资产hash不符：' + entry['path'])
            target.parent.mkdir(parents=True, exist_ok=True)
            temporary = target.with_name(target.name + '.restore-part')
            temporary.write_bytes(data)
            temporary.chmod(0o755 if entry.get('executable') else 0o644)
            temporary.replace(target)
        if link:
            alias = repo / manifest['source_root']
            if alias.is_symlink() and alias.resolve() == destination:
                pass
            elif alias.exists() or alias.is_symlink():
                raise RuntimeError('原路径已有对象，未替换：' + str(alias))
            else:
                alias.symlink_to(destination, target_is_directory=True)
    return {'destination': str(destination), 'missing_files': len(missing),
            'missing_bytes': sum(entry['bytes'] for entry, _ in missing), 'dry_run': dry_run,
            'scope': '仅恢复历史已跟踪文件；base_link.STL、导入USD等本地额外文件须另行准备'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination', type=Path,
                        default=Path.home() / 'RouteManager_assets/legacy/humanoid_upper_body_v3')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--link', action='store_true')
    args = parser.parse_args()
    print(json.dumps(restore(args.destination, args.dry_run, args.link), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
