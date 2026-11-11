#!/usr/bin/env python3
"""Export a built navigation image; optionally prepare offline Docker Engine debs."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import pwd
import shutil
import subprocess
import sys
import tempfile
import time
import uuid

from pm01_navigation_runtime.containers import manager as runtime


def standalone_program(module):
    """Generate a stdlib-only remote launcher from the installed runtime source.

    The payload is temporary transport, not another maintained implementation.
    It also lets preparation inspect older deployments before upgrading tools.
    """
    import base64
    import io
    import zipfile
    import pm01_navigation_runtime
    payload = io.BytesIO()
    root = Path(pm01_navigation_runtime.__file__).parent
    with zipfile.ZipFile(payload, 'w', zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(root.rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix in ('.py', '.json'):
                archive.writestr('pm01_navigation_runtime/'+str(path.relative_to(root)), path.read_bytes())
    return ('import base64, pathlib, runpy, sys, tempfile\n'
            'with tempfile.TemporaryDirectory(prefix="pm01-helper-") as folder:\n'
            '    path = pathlib.Path(folder)/"runtime.zip"\n'
            '    path.write_bytes(base64.b64decode('+repr(base64.b64encode(payload.getvalue()).decode())+'))\n'
            '    sys.path.insert(0, str(path))\n'
            '    runpy.run_module('+repr('pm01_navigation_runtime.'+module)+', run_name="__main__")\n')


def export_runtime(workspace, destination):
    """Copy the sole implementation, including its build manifest, into a payload."""
    import pm01_navigation_runtime
    destination = Path(destination)
    module_root = Path(pm01_navigation_runtime.__file__).parent
    shutil.copytree(module_root, destination/'pm01_navigation_runtime',
                    ignore=shutil.ignore_patterns('__pycache__', '*.pyc'), dirs_exist_ok=True)
    for name in ('_bootstrap.py', 'pm01_container.py', 'container_bundle.py'):
        shutil.copy2(workspace/'scripts'/name, destination/name)


def prepare(image, output, workspace):
    path = Path(output).expanduser().resolve()
    if path.exists() or path.is_relative_to(workspace):
        raise ValueError('Use a new bundle directory outside the source workspace')
    info = json.loads(runtime.run(['docker', 'image', 'inspect', image]))[0]
    labels = info['Config'].get('Labels', {}) or {}
    if labels.get('cn.engineai.pm01.project') != runtime.PROJECT or info['Os'] != 'linux':
        raise ValueError('Not a navigation image')
    tags = info.get('RepoTags') or []
    if not tags:
        raise ValueError('Tag the image before exporting it for offline deployment')
    image = image if image in tags else image+':latest' if image+':latest' in tags else tags[0]
    path.mkdir(parents=True)
    runtime.run(['docker', 'image', 'save', '--output', str(path/'image.tar'), image], timeout=1800)
    export_runtime(workspace, path)
    # Metadata extraction does not start an ARM64 process on the build host.
    temporary = 'pm01-export-'+uuid.uuid4().hex[:12]
    runtime.run(['docker', 'container', 'create', '--pull=never', '--platform',
                 'linux/'+info['Architecture'], '--name', temporary, image])
    try:
        for name in ('container-packages.tsv', 'container-revision.txt'):
            runtime.run(['docker', 'cp', temporary+':/opt/pm01_navigation/'+name, str(path/name)])
    finally:
        runtime.run(['docker', 'container', 'rm', temporary])
    manifest = dict(schema=1, project=runtime.PROJECT, image=image, image_id=info['Id'],
                    image_fingerprint=runtime.image_fingerprint(info),
                    architecture=info['Architecture'], source_revision=labels.get('org.opencontainers.image.revision'),
                    source_sha256=labels.get('cn.engineai.pm01.source-sha256'),
                    files={str(file.relative_to(path)): runtime.digest(file)
                           for file in path.rglob('*') if file.is_file()})
    (path/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(str(path))


def prepare_engine(inventory, bundle):
    """Resolve only Docker Engine host packages against the actual robot status."""
    path = Path(bundle).expanduser().resolve(strict=True)
    manifest = runtime.verify_bundle(path)
    if manifest['architecture'] != 'arm64':
        raise ValueError('Engine preparation is for Ubuntu 22.04 ARM64 PM01')
    status = Path(inventory).expanduser().read_text()
    if 'Architecture: arm64' not in status or 'Architecture: amd64' in status:
        raise ValueError('Use the ARM64 robot /var/lib/dpkg/status inventory')
    debs = path/'engine-debs'
    if debs.exists():
        raise ValueError('Engine archives already exist; use a new export directory')
    debs.mkdir(); (debs/'partial').mkdir()
    with tempfile.TemporaryDirectory(prefix='pm01-engine-apt-') as temporary:
        work = Path(temporary)
        (work/'status').write_text(status)
        (work/'lists/partial').mkdir(parents=True); (work/'sources.list.d').mkdir()
        (work/'sources.list').write_text('\n'.join(
            'deb [arch=arm64 signed-by=/usr/share/keyrings/ubuntu-archive-keyring.gpg] '
            'http://ports.ubuntu.com/ubuntu-ports '+suite+' main universe restricted multiverse'
            for suite in ('jammy', 'jammy-updates', 'jammy-security'))+'\n')
        options = ['-o', 'APT::Architecture=arm64', '-o', 'APT::Architectures::=arm64',
                   '-o', 'APT::Sandbox::User='+pwd.getpwuid(os.getuid()).pw_name,
                   '-o', 'Dir::Etc::sourcelist='+str(work/'sources.list'),
                   '-o', 'Dir::Etc::sourceparts='+str(work/'sources.list.d'),
                   '-o', 'Dir::State::status='+str(work/'status'), '-o', 'Dir::State::lists='+str(work/'lists'),
                   '-o', 'Dir::Cache::archives='+str(debs), '-o', 'Dir::Cache::pkgcache='+str(work/'pkgcache'),
                   '-o', 'Dir::Cache::srcpkgcache='+str(work/'srccache'), '-o', 'Acquire::Languages=none']
        for command in (['apt-get', *options, 'update'],
                        ['apt-get', *options, '--download-only', '--no-install-recommends', '--no-remove',
                         '-y', 'install', 'docker.io']):
            subprocess.run(command, check=True)
    shutil.rmtree(debs/'partial'); (debs/'lock').unlink(missing_ok=True)
    manifest['engine_inventory_sha256'] = runtime.digest(Path(inventory).expanduser())
    manifest['files'].update({str(file.relative_to(path)): runtime.digest(file) for file in debs.glob('*.deb')})
    (path/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print('Offline Docker Engine archives ready: '+str(debs))


def install_engine(bundle):
    path = Path(bundle).expanduser().resolve(strict=True)
    manifest = runtime.verify_bundle(path)
    if architecture := manifest.get('architecture'):
        if architecture != runtime.architecture():
            raise ValueError('Wrong host architecture')
    if shutil.which('docker'):
        print('Docker already installed; reusing it without replacing packages')
        return
    if os.geteuid() != 0:
        raise PermissionError('Run install-engine with sudo')
    names = [name for name in manifest['files'] if name.startswith('engine-debs/') and name.endswith('.deb')]
    if not names:
        raise ValueError('No engine archives; run engine preparation on the developer PC first')
    # With --no-download APT expects even explicit local debs in its archive
    # cache. Use a private writable cache so a read-only deployment bundle works
    # and no package is fetched or left in the system cache.
    with tempfile.TemporaryDirectory(prefix='pm01-engine-install-') as temporary:
        cache = Path(temporary)
        (cache/'partial').mkdir()
        for name in names:
            shutil.copy2(path/name, cache/Path(name).name)
        subprocess.run(['apt-get', '-o', 'Dir::Cache::archives='+str(cache),
                        '--no-download', '--no-remove', '--no-install-recommends', '-y',
                        'install', *[str(path/name) for name in names]], check=True)
    print('Docker installed. Enable its service and grant the deployment account Docker access.')


def main(workspace):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    export = sub.add_parser('prepare'); export.add_argument('--image', default='pm01-navigation:v1.0.0-arm64')
    export.add_argument('--output', default='../pm01-navigation-bundle-'+time.strftime('%Y%m%d-%H%M%S'))
    engine = sub.add_parser('engine'); engine.add_argument('--inventory', required=True); engine.add_argument('--bundle', required=True)
    install = sub.add_parser('install-engine'); install.add_argument('--bundle', required=True)
    args = parser.parse_args()
    if args.action == 'prepare':
        prepare(args.image, args.output, workspace)
    elif args.action == 'engine':
        prepare_engine(args.inventory, args.bundle)
    else:
        install_engine(args.bundle)


