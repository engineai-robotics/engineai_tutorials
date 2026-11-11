#!/usr/bin/env python3
"""Offline PM01 container lifecycle. Host requires Python 3 and Docker, not ROS."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

PROJECT = 'fastlio_navigation'
DEFAULT_CONFIG = '~/.config/pm01_navigation/container.json'


def run(argv, timeout=30):
    result = subprocess.run(argv, text=True, capture_output=True, timeout=timeout)
    if result.returncode:
        raise RuntimeError((result.stderr or result.stdout)[-2000:] or 'Command failed')
    return result.stdout


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def machine():
    return Path('/etc/machine-id').read_text().strip()


def architecture():
    value = platform.machine()
    return {'aarch64': 'arm64', 'arm64': 'arm64', 'x86_64': 'amd64'}.get(value, value)


def image_info(reference):
    info = json.loads(run(['docker', 'image', 'inspect', reference]))[0]
    labels = info.get('Config', {}).get('Labels', {}) or {}
    if labels.get('cn.engineai.pm01.project') != PROJECT or labels.get('cn.engineai.pm01.profile') != 'hardware':
        raise RuntimeError('Image is not a PM01 hardware navigation image')
    if info['Architecture'] != architecture() or info['Os'] != 'linux':
        raise RuntimeError('Image platform does not match this Linux host')
    return info


def image_fingerprint(info):
    """Portable identity: layer content and runtime config, not a daemon-local ID.

    Docker's classic and containerd stores can expose different IDs for the same
    saved image. Diff IDs and the executable configuration survive save/load.
    """
    config = info['Config']
    value = dict(architecture=info['Architecture'], os=info['Os'], layers=info['RootFS']['Layers'],
                 env=config.get('Env') or [], entrypoint=config.get('Entrypoint') or [],
                 cmd=config.get('Cmd') or [], user=config.get('User') or '',
                 working_dir=config.get('WorkingDir') or '', labels=config.get('Labels') or {},
                 volumes=sorted((config.get('Volumes') or {}).keys()),
                 ports=sorted((config.get('ExposedPorts') or {}).keys()),
                 stop_signal=config.get('StopSignal') or '', healthcheck=config.get('Healthcheck') or {})
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def verify_bundle(directory):
    directory = Path(directory).expanduser().resolve(strict=True)
    manifest = json.loads((directory/'manifest.json').read_text())
    if manifest.get('schema') != 1 or manifest.get('project') != PROJECT:
        raise RuntimeError('Unsupported navigation bundle')
    for name, checksum in manifest['files'].items():
        path = directory/name
        if (Path(name).is_absolute() or '..' in Path(name).parts or path.is_symlink()
                or not path.resolve().is_relative_to(directory) or not path.is_file()
                or digest(path) != checksum):
            raise RuntimeError('Bundle file missing or changed: '+name)
    return manifest


def load_bundle(directory):
    manifest = verify_bundle(directory)
    if manifest['architecture'] != architecture():
        raise RuntimeError('Bundle architecture does not match this host')
    run(['docker', 'image', 'load', '--input', str(Path(directory).expanduser().resolve()/'image.tar')], timeout=1800)
    image = image_info(manifest['image'])
    if image_fingerprint(image) != manifest['image_fingerprint']:
        raise RuntimeError('Loaded image content/configuration mismatch')
    return dict(ok=True, image=manifest['image'], image_id=image['Id'])


def migrate_discovery_profile(filename):
    """Repair only the generated legacy placeholder; retain all other XML bytes."""
    path = Path(filename).expanduser().resolve()
    if not path.is_file():
        return dict(ok=True, changed=False, reason='profile_missing', profile=str(path))
    original = path.read_bytes()
    ns = '{http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles}'
    try:
        root = ET.fromstring(original)
    except ET.ParseError:
        return dict(ok=True, changed=False, reason='invalid_xml_unchanged', profile=str(path))
    participants = [p for p in root.findall(ns+'participant')
                    if p.get('is_default_profile', '').lower() == 'true']
    if (len(participants) != 1 or len(root.findall(ns+'participant')) != 1 or
            participants[0].get('profile_name') != 'pm01'):
        return dict(ok=True, changed=False, reason='custom_profile_unchanged', profile=str(path))
    rtps = participants[0].find(ns+'rtps')
    if rtps is None or not any(p.text == 'pm01_udp' for p in rtps.findall(ns+'userTransports/'+ns+'transport_id')):
        return dict(ok=True, changed=False, reason='custom_profile_unchanged', profile=str(path))
    builtin = rtps.find(ns+'builtin')
    if builtin is None or builtin.find(ns+'metatrafficUnicastLocatorList') is None:
        return dict(ok=True, changed=False, reason='not_needed', profile=str(path))
    # Explicit multicast settings or concrete unicast locators belong to the user.
    pattern = rb'<metatrafficUnicastLocatorList>\s*<locator\s*/>\s*</metatrafficUnicastLocatorList>'
    matches = list(re.finditer(pattern, original))
    if builtin.find(ns+'metatrafficMulticastLocatorList') is not None or len(matches) != 1:
        return dict(ok=True, changed=False, reason='custom_profile_unchanged', profile=str(path))
    match = matches[0]
    updated = original[:match.start()] + original[match.end():]
    if ET.fromstring(updated).find('.//'+ns+'metatrafficUnicastLocatorList') is not None:
        return dict(ok=True, changed=False, reason='custom_profile_unchanged', profile=str(path))
    backup = path.with_name(path.name+'.before-dds-discovery-'+hashlib.sha256(original).hexdigest()[:12]+'.bak')
    try:
        fd = os.open(backup, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        if backup.read_bytes() != original:
            raise RuntimeError('DDS backup content differs; existing backup left untouched')
    else:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(original)
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(updated)
    try:
        temporary.chmod(path.stat().st_mode & 0o777)
        if path.read_bytes() != original:
            raise RuntimeError('DDS profile changed during migration; retry after the editor is closed')
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
    return dict(ok=True, changed=True, reason='legacy_placeholder_removed',
                profile=str(path), backup=str(backup))


class Runtime:
    def __init__(self, filename, expected=''):
        self.path = Path(filename).expanduser().resolve()
        self.config = json.loads(self.path.read_text())
        if self.config.get('schema') != 1:
            raise RuntimeError('Unsupported container configuration')
        self.identity = machine()
        if self.config['machine_id'] != self.identity or (expected and expected != self.identity):
            raise RuntimeError('Robot identity mismatch; no action performed')
        self.name = self.config['container']
        if not re.fullmatch(r'pm01-navigation-[a-zA-Z0-9_.-]+', self.name):
            raise ValueError('Invalid container name')
        self.owner = self.identity+':'+str(os.getuid())+':'+hashlib.sha256(str(self.path).encode()).hexdigest()[:16]
        self.session_config = str(Path(self.config['session_config']).expanduser().resolve())
        if not Path(self.session_config).is_relative_to(Path.home()/'.config/pm01_navigation'):
            raise ValueError('Session configuration must be inside the mounted user configuration directory')

    def owned(self):
        # Listing first distinguishes a missing container from a daemon/permission error.
        names = run(['docker', 'container', 'ls', '--all', '--format', '{{.Names}}']).splitlines()
        if self.name not in names:
            return None
        info = json.loads(run(['docker', 'container', 'inspect', self.name]))[0]
        labels = info['Config'].get('Labels', {}) or {}
        if labels.get('cn.engineai.pm01.owner') != self.owner or labels.get('cn.engineai.pm01.project') != PROJECT:
            raise RuntimeError('Container ownership mismatch; left untouched')
        return info

    def ensure(self):
        info = self.owned()
        active = None
        development = self.config.get('development', {})
        if development.get('enabled'):
            root = Path(development['workspace']).expanduser().resolve(strict=True)
            metadata = json.loads((root/'development.json').read_text())
            generation = metadata.get('generation', '')
            if not re.fullmatch('[a-f0-9]{12}', generation):
                raise RuntimeError('Development build generation is missing; run dev build')
            active = root/'generations'/generation
            if not (active/'install/local_setup.bash').is_file():
                raise RuntimeError('Development install is missing; build it or run dev disable')
        if info:
            mounted = next((m['Source'] for m in info.get('Mounts', [])
                            if m['Destination'] == '/opt/pm01_dev'), None)
            if mounted != (str(active) if active is not None else None):
                if info['State']['Running']:
                    raise RuntimeError('Development selection changed; stop this project session first')
                run(['docker', 'container', 'rm', self.name])
                info = None
        if info:
            if info['Image'] != self.config['image_id']:
                raise RuntimeError('Container uses another image; stop it and run use-image')
            if not info['State']['Running']:
                run(['docker', 'container', 'start', self.name])
            return
        image_info(self.config['image_id'])  # Never pull implicitly.
        home = Path.home()
        args = ['docker', 'container', 'create', '--pull=never', '--name', self.name,
                '--label', 'cn.engineai.pm01.project='+PROJECT,
                '--label', 'cn.engineai.pm01.owner='+self.owner,
                '--network', 'host', '--pid', 'host', '--read-only',
                '--cap-drop', 'ALL', '--security-opt', 'no-new-privileges',
                '--user', str(os.getuid())+':'+str(os.getgid()), '--env', 'HOME='+str(home),
                '--env', 'ROS_LOG_DIR='+str(home/'.local/share/pm01_navigation/logs'),
                '--env', 'ROS_HOME='+str(home/'.local/share/pm01_navigation/ros'),
                '--tmpfs', '/tmp:rw,nosuid,nodev,size=512m',
                '--mount', 'type=bind,src=/etc/machine-id,dst=/etc/machine-id,readonly']
        for folder in (home/'.config/pm01_navigation', home/'.local/share/pm01_navigation',
                       home/'.local/state/pm01_navigation'):
            folder.mkdir(parents=True, exist_ok=True)
            args += ['--mount', 'type=bind,src='+str(folder)+',dst='+str(folder)]
        if active is not None:
            # New generations own the sources referenced by symlink installs.
            # Existing generations keep their original mount until rebuilt.
            source = active/'src' if (active/'src').is_dir() else root/'src'
            args += ['--mount', 'type=bind,src='+str(active)+',dst=/opt/pm01_dev,readonly',
                     '--mount', 'type=bind,src='+str(source)+',dst=/opt/pm01_dev/src,readonly']
        args += [self.config['image_id']]
        run(args)
        run(['docker', 'container', 'start', self.name])

    def execute(self, args, timeout=30, expected_container_id=None):
        info = self.owned()
        if not info or not info['State']['Running']:
            raise RuntimeError('Navigation container is stopped')
        if expected_container_id and info.get('Id') != expected_container_id:
            raise RuntimeError('Container changed during request; replacement left untouched')
        command = ['/usr/local/bin/pm01-env']
        if Path(self.session_config).is_file():
            # Reuse the shared environment loader for debug commands as well as
            # sessions. No host ROS installation or duplicated DDS logic.
            loader = 'source /opt/pm01_navigation/scripts/env.sh --config "$1"'
            if self.config.get('development', {}).get('enabled'):
                # env.sh owns the final DDS selection. Reapply it after the
                # overlay, which must not replace deployment communication.
                loader += (' && source /opt/pm01_dev/install/local_setup.bash'
                           ' && pm01_settings=$(python3 -m pm01_navigation_runtime.system.environment "$1")'
                           ' && while IFS= read -r pm01_setting; do export "$pm01_setting"; done <<< "$pm01_settings"'
                           ' && unset CYCLONEDDS_URI FASTDDS_DEFAULT_PROFILES_FILE')
            command += ['bash', '-c', loader+' && shift && exec "$@"',
                        'pm01-session-exec', self.session_config]
        return run(['docker', 'container', 'exec', expected_container_id or self.name, *command, *args], timeout)

    @staticmethod
    def stopped(info):
        return info is None or (info['State'].get('Running') is False
            and info['State'].get('Pid') == 0
            and info['State'].get('Status') in ('created', 'exited'))

    def stop_session_container(self, info):
        """Stop our container even when its internal ROS shutdown RPC fails.

        Docker ownership and immutable ID, not ROS discovery or a stale status
        topic, define the process boundary. No external publisher or controller
        is stopped or claimed to have taken physical control.
        """
        if self.stopped(info):
            return dict(ok=True, running=False, container_running=False,
                        output_process_exit_confirmed=True,
                        message='本项目容器已停止', physical_controller_owner='unknown')
        target = info.get('Id', '')
        if not re.fullmatch('[a-f0-9]{64}', target):
            raise RuntimeError('Container ID unavailable; stop result unconfirmed')
        session_error = None
        request_state = 'not_required'
        if Path(self.session_config).is_file():
            args = ['ros2', 'run', 'pm01_navigation_bringup', 'pm01_session',
                    '--config', self.session_config, '--expected-machine-id', self.identity, 'stop']
            try:
                response = json.loads(self.execute(args, timeout=28, expected_container_id=target).strip().splitlines()[-1])
                if not isinstance(response, dict) or not response.get('ok'):
                    raise RuntimeError(response.get('error', 'Session stop unconfirmed')
                                       if isinstance(response, dict) else 'Invalid session response')
                request_state = 'confirmed'
            except (RuntimeError, subprocess.TimeoutExpired, ValueError, IndexError) as error:
                session_error = str(error)[-2000:]
                request_state = 'unconfirmed'
        # Recheck ownership immediately before the mutation, and address Docker
        # by immutable ID so a same-name replacement cannot be stopped by race.
        current = self.owned()
        if current is not None and current.get('Id') != target:
            raise RuntimeError('Container changed during stop; replacement left untouched')
        stop_error = None
        if current is not None and not self.stopped(current):
            try:
                run(['docker', 'container', 'stop', '--time', '2', target], timeout=8)
            except (RuntimeError, subprocess.TimeoutExpired) as error:
                stop_error = str(error)[-1000:]
        final = self.owned()
        if final is not None and final.get('Id') != target:
            raise RuntimeError('Container changed during stop; replacement state unconfirmed')
        if not self.stopped(final):
            raise RuntimeError('Own container stop unconfirmed: '+(stop_error or 'processes still running'))
        result = dict(ok=True, running=False, container_running=False,
                      output_process_exit_confirmed=True,
                      session_request_confirmed=request_state == 'confirmed',
                      session_request_state=request_state,
                      goal_cancel='unknown', physical_controller_owner='unknown',
                      stop_method='owned_container_fallback' if session_error else (
                          'container_only' if request_state == 'not_required' else 'session_then_container'),
                      message='本项目容器、输出进程和导航后台已停止')
        if session_error:
            result.update(session_error=session_error,
                message=result['message']+'；内部 ROS 停止请求未确认，已按所属容器 ID 完成停止')
        if stop_error:
            result['docker_stop_warning'] = stop_error
        return result

    def session(self, action, mode=None, map_yaml=''):
        if action == 'stop':
            return self.stop_session_container(self.owned())
        if action in ('identity', 'status'):
            info = self.owned()
            if not info or not info['State']['Running'] or not Path(self.session_config).is_file():
                return dict(ok=True, identity={'machine_id': self.identity, 'architecture': architecture()},
                            running=False, container_running=bool(info and info['State']['Running']))
        if action in ('start', 'restart', 'maps'):
            if not Path(self.session_config).is_file():
                raise RuntimeError('Run configure before starting a session')
            self.ensure()
        args = ['ros2', 'run', 'pm01_navigation_bringup', 'pm01_session',
                '--config', self.session_config, '--expected-machine-id', self.identity, action]
        if mode is not None:
            args += ['--mode', mode]
        if map_yaml:
            args += ['--map', map_yaml]
        result = json.loads(self.execute(args, timeout=28 if action == 'restart' else 15).strip().splitlines()[-1])
        if not result.get('ok'):
            raise RuntimeError(result.get('error', 'Session result unknown'))
        result['container_running'] = True
        return result

    def configure(self, args):
        self.ensure()
        command = ['python3', '/opt/pm01_navigation/scripts/configure_hardware.py', 'robot',
                   '--workspace', '/opt/pm01_navigation', '--machine-id', self.identity,
                   '--output-dir', str(Path(self.session_config).parent),
                   '--robot-address', args.robot_address, '--developer-address', args.developer_address,
                   '--robot-lan-address', args.robot_lan_address, '--motion-peer', args.motion_peer,
                   '--domain', str(args.domain)]
        if args.lidar_config:
            command += ['--lidar-config', args.lidar_config]
        output = self.execute(command)
        # The image path is identical across containers. Give each configuration
        # its own native Session state root, so one cannot stop another's session.
        session = json.loads(Path(self.session_config).read_text())
        state_root = str(Path.home()/'.local/state/pm01_navigation'/
                         ('docker-'+hashlib.sha256(str(self.path).encode()).hexdigest()[:16]))
        if session.get('state_root') != state_root:
            session['state_root'] = state_root
            write_config(Path(self.session_config), session)
        return dict(ok=True, session_config=self.session_config, message=output.strip())

    def use_image(self, reference):
        image = image_info(reference)
        info = self.owned()
        if info:
            if info['State']['Running']:
                raise RuntimeError('Stop the navigation session before changing its image')
            run(['docker', 'container', 'rm', self.name])
        self.config.update(image=reference, image_id=image['Id'])
        # Retain editable sources, but a new image must not silently load a
        # previously built ABI overlay. Rebuild/enable remains explicit.
        if self.config.get('development'):
            self.config['development']['enabled'] = False
        write_config(self.path, self.config)
        return dict(ok=True, image=reference, message='镜像已切换；现场配置、地图和日志保留')

    def migrate_discovery(self):
        info = self.owned()
        if info and info['State']['Running']:
            raise RuntimeError('Stop this project session before migrating its DDS profile')
        path = Path(self.session_config)
        if not path.is_file():
            return dict(ok=True, changed=False, reason='session_not_configured')
        profile = json.loads(path.read_text()).get('dds_profile')
        if not profile:
            return dict(ok=True, changed=False, reason='profile_not_configured')
        return migrate_discovery_profile(profile)


def write_config(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, indent=2)+'\n')
    temporary.chmod(0o600)
    temporary.replace(path)


def initialize(filename, reference, expected):
    if not re.fullmatch('[0-9a-f]{32}', expected) or machine() != expected:
        raise RuntimeError('Robot identity mismatch; no action performed')
    path = Path(filename).expanduser().resolve()
    # Persistent paths are deliberately the same on the host and in the container.
    config_root = Path.home()/'.config/pm01_navigation'
    if not path.is_relative_to(config_root):
        raise ValueError('Keep container configuration under ~/.config/pm01_navigation/')
    if path.exists():
        current = Runtime(path, expected)
        return dict(ok=True, config=str(path), reused=True, image=current.config['image'],
                    message='Existing configuration retained; image unchanged. Use use-image to change it.')
    image = image_info(reference)
    suffix = 'docker' if path == config_root/'container.json' else 'docker-'+hashlib.sha256(str(path).encode()).hexdigest()[:8]
    data = dict(schema=1, machine_id=expected, image=reference, image_id=image['Id'],
                container='pm01-navigation-'+str(os.getuid())+'-'+hashlib.sha256(str(path).encode()).hexdigest()[:8],
                session_config=str(config_root/suffix/'session.json'))
    write_config(path, data)
    return dict(ok=True, config=str(path), message='配置已创建；尚未启动导航')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=DEFAULT_CONFIG)
    parser.add_argument('--expected-machine-id', default='')
    sub = parser.add_subparsers(dest='action', required=True)
    load = sub.add_parser('load'); load.add_argument('--bundle', required=True)
    init = sub.add_parser('init'); init.add_argument('--image', default='pm01-navigation:v1.0.0-arm64')
    init.add_argument('--machine-id', required=True)
    configure = sub.add_parser('configure')
    configure.add_argument('--robot-address', required=True); configure.add_argument('--developer-address', required=True)
    configure.add_argument('--robot-lan-address', default='192.168.0.162')
    configure.add_argument('--motion-peer', default='192.168.0.163')
    configure.add_argument('--domain', type=int, default=69); configure.add_argument('--lidar-config', default='')
    session = sub.add_parser('session')
    session.add_argument('operation', choices=['identity', 'status', 'maps', 'start', 'stop', 'restart'])
    session.add_argument('--mode', choices=['fresh', 'mapping', 'saved']); session.add_argument('--map', default='')
    change = sub.add_parser('use-image'); change.add_argument('--image', required=True)
    sub.add_parser('migrate-dds', help='Back up and repair the legacy project discovery XML; session must be stopped')
    execute = sub.add_parser('exec'); execute.add_argument('command', nargs=argparse.REMAINDER)
    development = sub.add_parser('dev')
    operations = development.add_subparsers(dest='operation', required=True)
    prepare = operations.add_parser('init'); prepare.add_argument('--workspace')
    build = operations.add_parser('build'); build.add_argument('--packages', nargs='+')
    build.add_argument('--jobs', type=int, default=2)
    for name in ('enable', 'disable', 'status'):
        operations.add_parser(name)
    args = parser.parse_args()
    try:
        if args.action == 'load':
            result = load_bundle(args.bundle)
        elif args.action == 'init':
            result = initialize(args.config, args.image, args.machine_id)
        else:
            runtime = Runtime(args.config, args.expected_machine_id)
            # Cross-process lock guards our lifecycle only, never ROS release requests.
            with runtime.path.with_suffix('.lock').open('a') as lock:
                if not (args.action == 'session' and args.operation in ('identity', 'status')):
                    try:
                        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    except BlockingIOError as error:
                        raise RuntimeError('Another container operation is in progress; refresh status') from error
                if args.action == 'session':
                    result = runtime.session(args.operation, args.mode, args.map)
                elif args.action == 'configure':
                    result = runtime.configure(args)
                elif args.action == 'use-image':
                    result = runtime.use_image(args.image)
                elif args.action == 'migrate-dds':
                    result = runtime.migrate_discovery()
                elif args.action == 'dev':
                    from pm01_navigation_runtime.containers.development import Development
                    dev = Development(runtime, run, write_config, image_info)
                    if args.operation == 'init':
                        result = dev.initialize(args.workspace)
                    elif args.operation == 'build':
                        result = dev.build(args.packages, args.jobs)
                    elif args.operation == 'status':
                        result = dev.status()
                    else:
                        result = dev.enable(args.operation == 'enable')
                else:
                    result = dict(ok=True, output=runtime.execute(args.command, timeout=120))
        print(json.dumps(result, ensure_ascii=False))
    except Exception as error:
        print(json.dumps(dict(ok=False, error=str(error)), ensure_ascii=False))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
