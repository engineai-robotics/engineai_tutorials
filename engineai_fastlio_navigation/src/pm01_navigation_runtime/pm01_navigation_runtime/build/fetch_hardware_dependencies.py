#!/usr/bin/env python3
"""Fetch only the pinned Livox SDK source; hardware needs no simulator/models."""
import argparse
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

REV = '7ee29a3ede5ecd7d884a1a53b258cd5a0147bd81'
SDK_PATH = 'ros/src/deployment/livox/livox_sdk2'
SDK_TREE = 'bc4597a34a4d0220f6112cb3e7156281499487e4'


def object_hash(kind, content):
    return hashlib.sha1(kind.encode()+b' '+str(len(content)).encode()+b'\0'+content).digest()


def tree_hash(directory):
    entries = []
    paths = sorted(Path(directory).iterdir(), key=lambda p: (p.name+('/' if p.is_dir() and not p.is_symlink() else '')).encode())
    for path in paths:
        if path.is_symlink():
            mode, digest = b'120000', object_hash('blob', os.readlink(path).encode())
        elif path.is_dir():
            mode, digest = b'40000', bytes.fromhex(tree_hash(path))
        else:
            mode = b'100755' if path.stat().st_mode & 0o111 else b'100644'
            digest = object_hash('blob', path.read_bytes())
        entries.append(mode+b' '+path.name.encode()+b'\0'+digest)
    return object_hash('tree', b''.join(entries)).hex()


def fetch(destination, cache=None):
    destination = Path(destination)
    if destination.exists():
        if tree_hash(destination) != SDK_TREE:
            raise RuntimeError('SDK source differs from the pinned tree; preserve changes and use a new directory')
        print('Pinned hardware SDK verified: '+SDK_TREE)
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='pm01-sdk-') as temporary:
        temp = Path(temporary)
        if cache:
            repository = Path(cache).resolve(strict=True)
        else:
            repository = temp/'upstream'
            subprocess.run(['git', 'clone', '--filter=blob:none', '--no-checkout', '--depth=1',
                'https://github.com/engineai-robotics/engineai_navigation.git', str(repository)], check=True, timeout=180)
            subprocess.run(['git', '-C', str(repository), 'fetch', '--filter=blob:none', '--depth=1', 'origin', REV], check=True, timeout=180)
            subprocess.run(['git', '-C', str(repository), 'sparse-checkout', 'set', SDK_PATH], check=True)
            subprocess.run(['git', '-C', str(repository), 'checkout', '--detach', REV], check=True, timeout=180)
        actual = subprocess.check_output(['git', '-C', str(repository), 'rev-parse', REV+':'+SDK_PATH], text=True).strip()
        if actual != SDK_TREE:
            raise RuntimeError('Upstream SDK tree identity mismatch')
        archive = temp/'sdk.tar'
        with archive.open('wb') as output:
            subprocess.run(['git', '-C', str(repository), 'archive', SDK_TREE], stdout=output, check=True)
        stage = temp/'source'; stage.mkdir()
        with tarfile.open(archive) as tar:
            # Archive is selected by the fixed Git tree above, not a branch or
            # user-provided download. Reject path escapes regardless.
            for member in tar.getmembers():
                if member.name.startswith('/') or '..' in Path(member.name).parts or member.issym() or member.islnk():
                    raise RuntimeError('Unexpected SDK archive member')
            tar.extractall(stage)
        if tree_hash(stage) != SDK_TREE:
            raise RuntimeError('Extracted SDK identity mismatch')
        shutil.copytree(stage, destination)
    print('Hardware SDK source ready: '+SDK_TREE)


def main(workspace):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, help='Optional local checkout of the pinned PUBLIC upstream')
    parser.add_argument('--output', type=Path, default=workspace/'.deps/hardware/livox_sdk2')
    parser.add_argument('--verify-only', action='store_true', help='Offline robot: never download missing source')
    args = parser.parse_args()
    if args.verify_only and not args.output.is_dir():
        parser.error('Offline SDK missing. Prepare a hardware bundle on the developer computer and transfer it first.')
    (workspace/'.deps').mkdir(exist_ok=True)
    (workspace/'.deps/COLCON_IGNORE').touch()
    fetch(args.output, args.cache)


