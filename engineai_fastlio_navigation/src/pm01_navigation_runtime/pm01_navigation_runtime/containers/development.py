"""Optional offline source workspace using the installed navigation image."""
import json
import hashlib
import os
from pathlib import Path
import re
import shutil
import time
import uuid
import xml.etree.ElementTree as ET

from pm01_navigation_runtime.build.manifest import packages as profile_packages

DEFAULT_PACKAGES = [p for p in profile_packages('hardware') if p.startswith('pm01_navigation_')]
TARGET = '/opt/pm01_dev'


class Development:
    def __init__(self, runtime, run, write_config, image_info):
        self.runtime, self.run = runtime, run
        self.write_config, self.image_info = write_config, image_info

    def stopped(self):
        info = self.runtime.owned()
        if info and info['State']['Running']:
            raise RuntimeError('Stop this project session with session stop before development changes')

    def workspace(self, value=None):
        value = value or self.runtime.config.get('development', {}).get('workspace')
        if not value:
            raise ValueError('Run dev init first')
        root = Path(value).expanduser().resolve()
        if root == Path('/') or root == Path.home() or '\n' in str(root) or ',' in str(root):
            raise ValueError('Use a dedicated development workspace directory')
        return root

    def metadata(self, root):
        path = root/'development.json'
        if not path.is_file():
            raise ValueError('Not an initialized navigation development workspace')
        return json.loads(path.read_text())

    def task(self, root, command, timeout=1800, source=None):
        # A separate short-lived build container: no robot network, host PID
        # namespace, ROS daemon, hardware access or persistent rootfs writes.
        args = ['docker', 'run', '--rm', '--pull=never', '--network=none',
            '--read-only', '--cap-drop=ALL', '--security-opt=no-new-privileges',
            '--user', str(os.getuid())+':'+str(os.getgid()),
            '--env', 'ROS_LOG_DIR=/tmp/ros', '--env', 'ROS_HOME=/tmp/ros',
            '--env', 'PYTHONDONTWRITEBYTECODE=1',
            '--tmpfs', '/tmp:rw,nosuid,nodev,size=512m',
            '--mount', 'type=bind,src='+str(root)+',dst='+TARGET]
        if source is not None:
            args += ['--mount', 'type=bind,src='+str(source)+',dst='+TARGET+'/src,readonly']
        args += ['--workdir', TARGET, '--entrypoint', '/bin/bash',
                 self.runtime.config['image_id'], '-c', command]
        return self.run(args, timeout=timeout)

    def initialize(self, value):
        root = self.workspace(value or '~/.local/share/pm01_navigation/development')
        if root.exists() and any(root.iterdir()):
            self.metadata(root)  # Repeated init retains all local source edits.
            self.require_configuration(root)
        else:
            root.mkdir(parents=True, exist_ok=True)
            self.image_info(self.runtime.config['image_id'])
            self.task(root, 'test -d /opt/pm01_navigation/config && '
                      'cp -a /opt/pm01_navigation/src /opt/pm01_navigation/config .')
            self.write_config(root/'development.json', {
                'schema': 1, 'initialized_image_id': self.runtime.config['image_id'],
                'built_image_id': None, 'built_packages': [], 'generation': None})
        config = self.runtime.config.setdefault('development', {})
        if config.get('enabled') and config.get('workspace') != str(root):
            raise RuntimeError('Disable the current development workspace before changing it')
        config.update(workspace=str(root), enabled=config.get('enabled', False))
        self.write_config(self.runtime.path, self.runtime.config)
        return dict(ok=True, workspace=str(root), enabled=config['enabled'],
                    message='Source workspace prepared; build before enabling')

    @staticmethod
    def require_configuration(root):
        if (not (root/'config').is_dir() or
                (root/'src/pm01_navigation_bringup/config').exists() or
                (root/'src/pm01_navigation_console/config').exists()):
            raise RuntimeError('Workspace uses the old configuration layout. Preserve local edits, '
                'then sync config/, pm01_navigation_bringup and pm01_navigation_console from the same source revision; '
                'the previous install has not been changed. See docs/development.md.')

    @staticmethod
    def packages(root, requested):
        packages = {}
        for path in (root/'src').glob('*/package.xml'):
            tree = ET.parse(path).getroot()
            name = tree.findtext('name')
            if name and re.fullmatch(r'[a-zA-Z0-9_]+', name):
                deps = {e.text.strip() for e in tree if e.text and
                        (e.tag.endswith('depend') or e.tag == 'depend')}
                packages[name] = (path.parent, deps)
        selected = set(requested or DEFAULT_PACKAGES)
        missing = selected-set(packages)
        if missing:
            raise ValueError('Source package not found: '+', '.join(sorted(missing)))
        # Only the selected hardware profile is supported by this image.
        supported = set(profile_packages('hardware'))
        while True:
            expanded = selected | {p for p, (_, deps) in packages.items()
                                   if p in supported and deps & selected}
            if expanded == selected:
                break
            selected = expanded
        return sorted(selected), [str(packages[p][0].relative_to(root)) for p in sorted(selected)]

    def build(self, requested=None, jobs=2):
        import shlex
        self.stopped()
        root = self.workspace()
        metadata = self.metadata(root)
        self.require_configuration(root)
        selected, sources = self.packages(root, sorted(set(requested or DEFAULT_PACKAGES) |
                                                     set(metadata.get('built_packages', []))))
        if not 1 <= jobs <= 32:
            raise ValueError('jobs must be 1..32')
        generation = uuid.uuid4().hex[:12]
        target = root/'generations'/generation
        target.mkdir(parents=True)
        # Symlink installs must point at this generation's immutable source,
        # never the editable workspace. Otherwise a failed Python/config build
        # silently changes the previously working installation too.
        snapshot = target/'src'
        snapshot.mkdir()
        for name in selected:
            shutil.copytree(root/'src'/name, snapshot/name,
                            ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        shutil.copytree(root/'config', target/'config',
                        ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '.DS_Store'))
        (target/'config.meta').write_text(json.dumps({'names': {
            'pm01_navigation_bringup': {'cmake-args': ['-DPM01_CONFIG_ROOT='+TARGET+'/config']},
            'slam_toolbox': {'cmake-args': ['-DPYTHON_EXECUTABLE=/usr/bin/python3']},
            'fast_lio': {'cmake-args': ['-DPYTHON_EXECUTABLE=/usr/bin/python3']},
            'livox_ros_driver2': {'cmake-args': ['-DROS_EDITION=ROS2', '-DDISTRO_ROS=humble',
                '-DPYTHON_EXECUTABLE=/usr/bin/python3']},
            'interface_protocol': {'cmake-args': ['-DPYTHON_EXECUTABLE=/usr/bin/python3']}}}))
        if metadata.get('generation'):
            old = root/'generations'/metadata['generation']/'build'
            if old.is_dir():
                # Never hardlink object files: a downstream failure must not
                # mutate binaries referenced by the last successful install.
                shutil.copytree(old, target/'build', symlinks=True)
        # Both generations are mounted at the same container prefix. CMake
        # caches remain valid without sharing writable binaries across them.
        command = ['colcon', 'build', '--symlink-install', '--executor', 'sequential',
                   '--metas', 'config.meta',
                   '--base-paths', *sources, '--packages-select', *selected,
                   '--build-base', 'build', '--install-base', 'install',
                   '--cmake-args', '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_TESTING=OFF',
                   '-DPython3_EXECUTABLE=/usr/bin/python3']
        setup = ('set -eo pipefail; source /opt/ros/humble/setup.bash; '
                 'source /opt/pm01_navigation/install/local_setup.bash; '
                 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; unset CYCLONEDDS_URI; '
                 'export CMAKE_PREFIX_PATH=/opt/pm01_navigation/.deps/prefix:$CMAKE_PREFIX_PATH; '
                 'export LD_LIBRARY_PATH=/opt/pm01_navigation/.deps/prefix/lib:$LD_LIBRARY_PATH; '
                 'export CMAKE_BUILD_PARALLEL_LEVEL='+str(jobs)+' MAKEFLAGS=-j'+str(jobs)+'; ')
        try:
            self.task(target, setup+shlex.join(command)+' 2>&1 | tee build.log',
                      timeout=3600, source=snapshot)
        except RuntimeError as error:
            raise RuntimeError('Build failed; previous install retained. Log: '+str(target/'build.log')+
                               '\n'+str(error)) from error
        install = target/'install'
        if not (install/'local_setup.bash').is_file():
            raise RuntimeError('Build returned without a complete install; previous version retained')
        temporary = root/('.install-'+generation)
        temporary.symlink_to(Path('generations')/generation/'install', target_is_directory=True)
        temporary.replace(root/'install')
        metadata.update(built_image_id=self.runtime.config['image_id'], generation=generation,
                        built_packages=selected, built_at=time.time(), source_sha256=self.source_hash(target, selected))
        self.write_config(root/'development.json', metadata)
        return dict(ok=True, workspace=str(root), packages=selected, generation=generation,
                    log=str(target/'build.log'), message='Build complete; restart the session to use it')

    def enable(self, enabled):
        self.stopped()
        config = self.runtime.config.setdefault('development', {})
        if enabled:
            root = self.workspace()
            metadata = self.metadata(root)
            if not (root/'install/local_setup.bash').is_file():
                raise RuntimeError('Run dev build before enabling this workspace')
            if metadata.get('built_image_id') != self.runtime.config['image_id']:
                raise RuntimeError('Workspace was built against another image; run dev build')
        info = self.runtime.owned()
        if info:
            self.run(['docker', 'container', 'rm', self.runtime.name])
        config['enabled'] = bool(enabled)
        self.write_config(self.runtime.path, self.runtime.config)
        return dict(ok=True, enabled=bool(enabled), message='Selection saved; motion output remains disabled')

    def status(self):
        config = self.runtime.config.get('development', {})
        metadata = self.metadata(self.workspace()) if config.get('workspace') else {}
        image = self.image_info(self.runtime.config['image_id'])
        revision = (image.get('Config', {}).get('Labels', {}) or {}).get('org.opencontainers.image.revision')
        current = self.runtime.owned()
        mounted = next((m['Source'] for m in (current or {}).get('Mounts', [])
                        if m['Destination'] == TARGET), None)
        return dict(ok=True, enabled=bool(config.get('enabled')), workspace=config.get('workspace'),
                    image=self.runtime.config['image'], image_id=self.runtime.config['image_id'],
                    image_source_revision=revision, build=metadata,
                    selected_generation=metadata.get('generation'),
                    container_generation=Path(mounted).name if mounted else None,
                    container_running=bool(current and current['State']['Running']),
                    source_modified_since_build=(self.source_hash(self.workspace(), metadata.get('built_packages', []))
                        != metadata.get('source_sha256') if config.get('workspace') and metadata.get('generation') else None))

    @staticmethod
    def source_hash(root, packages):
        digest = hashlib.sha256()
        directories = [root/'src'/name for name in sorted(packages)] + [root/'config']
        for directory in directories:
            for path in sorted(directory.rglob('*')):
                if path.is_file() and '__pycache__' not in path.parts and path.suffix != '.pyc':
                    digest.update(str(path.relative_to(root)).encode()+b'\0'+path.read_bytes())
        return digest.hexdigest()
