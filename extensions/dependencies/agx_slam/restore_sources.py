#!/usr/bin/env python3
"""Restore pinned sources and local overlays; preserve different existing files."""
from pathlib import Path
import shutil
import subprocess
import yaml

HERE = Path(__file__).resolve().parent
EXTENSIONS = HERE.parents[1]


def git(path, *args, capture=False):
    return subprocess.run(['git', '-C', str(path), *args], check=True,
                          text=True, capture_output=capture)


def main():
    repos = yaml.safe_load((HERE / 'sources.repos').read_text())['repositories']
    for relative, spec in repos.items():
        target = EXTENSIONS / relative
        if not target.exists():
            target.mkdir(parents=True)
            git(target, 'init')
            git(target, 'remote', 'add', 'origin', spec['url'])
            git(target, 'fetch', '--depth=1', '--filter=blob:none', 'origin', spec['version'])
            patterns = None
            if relative.endswith('/rtabmap'):
                patterns = ['app', 'cmake_modules', 'corelib', 'guilib', 'tools', 'utilite', 'data/presets']
            elif relative.endswith('/fast_lio'):
                patterns = ['include', 'src', 'config', 'launch', 'msg', 'rviz']
            if patterns:
                git(target, 'config', 'core.sparseCheckout', 'true')
                (target / '.git/info/sparse-checkout').write_text(
                    '/*\n!/*/\n' + ''.join('/' + p + '/\n' for p in patterns))
            git(target, 'checkout', '--detach', spec['version'])
        actual = git(target, 'rev-parse', 'HEAD', capture=True).stdout.strip()
        if actual != spec['version']:
            raise SystemExit(f'{target}: expected {spec["version"]}, found {actual}; preserved')
        if relative.endswith('/fast_lio'):
            git(target, 'submodule', 'update', '--init', '--recursive')
    driver = EXTENSIONS / 'ros2_ws/src/livox_ros_driver2'
    patch = str(HERE / 'patches/livox-install-launch.patch')
    applied = subprocess.run(['git', '-C', str(driver), 'apply', '--reverse', '--check', patch],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
    if not applied:
        git(driver, 'apply', '--check', patch)
        git(driver, 'apply', patch)
    for source in (HERE / 'overlays').rglob('*'):
        if source.is_file():
            target = EXTENSIONS / source.relative_to(HERE / 'overlays')
            if target.exists() and target.read_bytes() != source.read_bytes():
                raise SystemExit(f'{target}: different existing file preserved')
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
    print('Pinned sources and overlays are ready; follow the installation documents to build.')


if __name__ == '__main__':
    main()
