#!/usr/bin/env python3
"""Build the complete CPU-only navigation image on an online developer PC."""
import argparse
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile
import uuid



def run(args, **kwargs):
    subprocess.run(args, check=True, **kwargs)


def source_hash(workspace):
    result = hashlib.sha256()
    for directory in ('src', 'config', 'scripts', 'docker', '.deps/hardware/livox_sdk2'):
        for path in sorted((workspace/directory).rglob('*')):
            if (not path.is_file() or '.git' in path.parts or '__pycache__' in path.parts or path.suffix == '.pyc'):
                continue
            result.update(str(path.relative_to(workspace)).encode()+b'\0')
            result.update(str(path.stat().st_mode & 0o777).encode()+b'\0')
            result.update(path.read_bytes())
    for name in ('LICENSE', 'THIRD_PARTY.md', 'dependencies.repos'):
        path = workspace/name
        if path.is_file():
            result.update(name.encode()+b'\0'+path.read_bytes())
    return result.hexdigest()


def verify_runtime_base(reference, platform, workspace):
    """No recompilation shortcut if system, SDK, driver or LIO inputs changed."""
    info = __import__('json').loads(subprocess.check_output(['docker', 'image', 'inspect', reference], text=True))[0]
    labels = info.get('Config', {}).get('Labels', {}) or {}
    if (labels.get('cn.engineai.pm01.project') != 'fastlio_navigation' or
            labels.get('cn.engineai.pm01.profile') != 'hardware' or
            info['Architecture'] != platform.split('/')[1]):
        raise ValueError('Runtime base must be an existing navigation image for the selected architecture')
    name = 'pm01-release-check-'+uuid.uuid4().hex[:12]
    run(['docker', 'container', 'create', '--pull=never', '--platform', platform,
         '--name', name, reference], stdout=subprocess.DEVNULL)
    try:
        with tempfile.TemporaryDirectory(prefix='pm01-base-check-') as temporary:
            temp = Path(temporary)
            inputs = ['src/FAST_LIO_ROS2', 'src/livox_ros_driver2', 'src/interface_protocol',
                      '.deps/hardware/livox_sdk2', 'docker/hardware/packages.txt',
                      'src/slam_toolbox',
                      'src/pm01_navigation_runtime/pm01_navigation_runtime/build/profiles.json',
                      'scripts/config/nav2_packages.txt',
                      'scripts/build.sh', 'scripts/build_sdk.sh', 'scripts/fetch_hardware_dependencies.py']
            for index, relative in enumerate(inputs):
                old = temp/str(index)
                run(['docker', 'cp', name+':/opt/pm01_navigation/'+relative, str(old)])
                current = workspace/relative
                files = sorted(p.relative_to(current) for p in current.rglob('*') if p.is_file()) if current.is_dir() else [Path('.')]
                old_files = sorted(p.relative_to(old) for p in old.rglob('*') if p.is_file()) if old.is_dir() else [Path('.')]
                keep = lambda p: ('__pycache__' not in p.parts and p.suffix != '.pyc')
                files, old_files = list(filter(keep, files)), list(filter(keep, old_files))
                if relative == 'src/livox_ros_driver2' and Path('package.xml') not in files:
                    files = sorted([*files, Path('package.xml')])
                if files != old_files:
                    raise ValueError('Dependency inputs changed: '+relative+'; use a full build')
                for file in files:
                    actual = current if file == Path('.') else current/file
                    prior = old if file == Path('.') else old/file
                    # Driver build installs its ROS2 package manifest in source.
                    if relative == 'src/livox_ros_driver2' and str(file) == 'package.xml':
                        actual = current/'package_ROS2.xml'
                    if actual.read_bytes() != prior.read_bytes():
                        raise ValueError('Dependency input changed: '+relative+'/'+str(file)+'; use a full build')
    finally:
        run(['docker', 'container', 'rm', name], stdout=subprocess.DEVNULL)


def main(workspace):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', default='pm01-navigation:v1.0.0-arm64')
    parser.add_argument('--platform', choices=['linux/arm64', 'linux/amd64'], default='linux/arm64')
    parser.add_argument('--sdk-cache', help='Existing pinned public engineai_navigation Git checkout')
    parser.add_argument('--base-image', help='Override the pinned ROS base, e.g. for a maintained registry mirror')
    parser.add_argument('--runtime-base', help='Reuse a loaded navigation release when dependency inputs are unchanged')
    parser.add_argument('--jobs', type=int, default=2, help='Compiler parallelism; lower for limited RAM')
    args = parser.parse_args()
    if not 1 <= args.jobs <= 32:
        parser.error('jobs must be 1..32')
    run(['docker', 'info'], stdout=subprocess.DEVNULL)
    run(['docker', 'buildx', 'version'], stdout=subprocess.DEVNULL)
    fetch = [sys.executable, str(workspace/'scripts/fetch_hardware_dependencies.py')]
    if args.sdk_cache:
        fetch += ['--cache', args.sdk_cache]
    run(fetch)
    git = ['git', '-C', str(workspace)]
    tracked = subprocess.run([*git, 'ls-files', '--error-unmatch', '--', 'scripts/docker_build.py'],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    if tracked.returncode == 0:
        revision = subprocess.check_output([*git, 'rev-parse', 'HEAD'], text=True).strip()
        if subprocess.check_output([*git, 'status', '--porcelain', '--', '.'], text=True):
            revision += '-dirty'
    else:
        version_file = workspace/'SOURCE_VERSION'
        revision = version_file.read_text().strip() if version_file.is_file() else 'source-export'
    if args.runtime_base:
        if args.base_image:
            parser.error('Choose --runtime-base or --base-image, not both')
        verify_runtime_base(args.runtime_base, args.platform, workspace)
    command = ['docker', 'buildx', 'build', '--load', '--provenance=false', '--platform', args.platform,
               '--file', str(workspace/('docker/hardware/Dockerfile.update' if args.runtime_base else 'docker/hardware/Dockerfile')), '--tag', args.image,
               '--build-arg', 'SOURCE_REVISION='+revision,
               '--build-arg', 'SOURCE_SHA256='+source_hash(workspace),
               '--build-arg', 'BUILD_JOBS='+str(args.jobs)]
    if args.base_image:
        command += ['--build-arg', 'ROS_BASE='+args.base_image]
    if args.runtime_base:
        command += ['--build-arg', 'RUNTIME_BASE='+args.runtime_base]
    command.append(str(workspace))
    run(command)
    print('Built '+args.image+' for '+args.platform)
    print('Next: python3 scripts/container_bundle.py prepare --image '+args.image)
