#!/usr/bin/env python3
"""Fetch immutable simulator assets and SDK; application source is in src/."""
import argparse
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
from .fetch_hardware_dependencies import REV, fetch, tree_hash

URL = 'https://github.com/engineai-robotics/engineai_navigation.git'
ASSETS = {
    'simulator/config/robot/pm01_lidar_v1/meshes': 'c7bd5659abcc60ad9dcb4aeff25fafa804ff290d',
    'rl_walking/config/policy': '01d6df260ae9bc6a9b8144f88cca91308a2fee49',
    'rl_walking/third_party/mnn': '43471b5bb783ec03642675cef9e4842ed24a77f0',
}


def prepare(workspace, cache=None):
    workspace = Path(workspace)
    destination = workspace / '.deps/sim_assets'
    destination.mkdir(parents=True, exist_ok=True)
    (destination.parent / 'COLCON_IGNORE').touch()
    missing = []
    for name, expected in ASSETS.items():
        path = destination / name
        if path.exists():
            if tree_hash(path) != expected:
                raise RuntimeError('Simulator asset differs from the pinned tree: ' + str(path))
        else:
            missing.append(name)
    # A prior checkout is only an object cache; source and local edits stay intact.
    prior = workspace / '.deps/engineai_navigation'
    if cache is None and (prior / '.git').exists():
        cache = prior
    if not missing and (destination / 'livox_sdk2').is_dir():
        fetch(destination / 'livox_sdk2')
        print('Pinned simulator assets verified: ' + str(destination))
        return destination
    with tempfile.TemporaryDirectory(prefix='pm01-sim-assets-') as temporary:
        temp = Path(temporary)
        repository = Path(cache).resolve(strict=True) if cache else temp / 'upstream'
        if cache is None:
            subprocess.run(['git', 'clone', '--filter=blob:none', '--no-checkout', '--depth=1',
                            URL, str(repository)], check=True, timeout=180)
            subprocess.run(['git', '-C', str(repository), 'fetch', '--filter=blob:none',
                            '--depth=1', 'origin', REV], check=True, timeout=180)
        for name in missing:
            expected = ASSETS[name]
            actual = subprocess.check_output(['git', '-C', str(repository), 'rev-parse',
                REV + ':ros/src/simulation/' + name], text=True).strip()
            if actual != expected:
                raise RuntimeError('Upstream simulator asset identity mismatch: ' + name)
            archive = temp / 'asset.tar'
            with archive.open('wb') as output:
                subprocess.run(['git', '-C', str(repository), 'archive', expected],
                               stdout=output, check=True, timeout=300)
            stage = temp / 'asset'
            stage.mkdir()
            with tarfile.open(archive) as stream:
                for member in stream.getmembers():
                    if member.name.startswith('/') or '..' in Path(member.name).parts or member.issym() or member.islnk():
                        raise RuntimeError('Unexpected simulator asset archive member')
                stream.extractall(stage)
            if tree_hash(stage) != expected:
                raise RuntimeError('Extracted simulator asset identity mismatch: ' + name)
            target = destination / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(stage), str(target))
        fetch(destination / 'livox_sdk2', repository)
    print('Pinned simulator assets ready: ' + str(destination))
    return destination


def main(workspace):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, help='Existing public upstream Git object cache')
    args = parser.parse_args()
    prepare(workspace, args.cache)
