#!/usr/bin/env python3
"""Standard-library preparation helper. Never starts a navigation session.

Sent by the developer computer to the verified robot; it does not need ROS on
the host. Only the explicit admin action requires sudo. All other actions must
run as the ordinary login user.
"""
import base64
import grp
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import pwd
import re
import shutil
import subprocess
import sys
import tempfile
import time


def command(args, timeout=30, check=True):
    result = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    if check and result.returncode:
        raise RuntimeError((result.stderr or result.stdout)[-1500:] or 'Command failed')
    return result


def system_info():
    release = {}
    for line in Path('/etc/os-release').read_text().splitlines():
        if '=' in line:
            key, value = line.split('=', 1)
            release[key] = value.strip('"')
    return dict(machine_id=Path('/etc/machine-id').read_text().strip(),
                hostname=platform.node(), architecture=platform.machine(),
                os=release.get('ID'), version=release.get('VERSION_ID'),
                home=str(Path.home()), user=pwd.getpwuid(os.getuid()).pw_name,
                uid=os.getuid(), gid=os.getgid(), time=time.time())


def verify_identity(request):
    if system_info()['machine_id'] != request['machine_id']:
        raise RuntimeError('Robot identity changed; no operation performed')


def load_runtime(tool):
    from pm01_navigation_runtime.containers import manager
    return manager


def busy_processes():
    names = {'fastlio_mapping', 'livox_ros_driver2_node', 'online_navigation_map.py',
             'rosbag2', 'realsense2_camera_node', 'realsense2_camera'}
    busy = []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            words = (path/'cmdline').read_bytes().split(b'\0')
            found = names & {Path(word.decode(errors='replace')).name for word in words if word}
            if found:
                busy.append(dict(pid=int(path.name), names=sorted(found)))
        except OSError:
            pass
    return busy


def inspect(request):
    value = system_info()
    value.update(ok=True, docker_installed=bool(shutil.which('docker')),
                 docker_access=False, docker_error='', busy=busy_processes(),
                 runtime=None, navigation_running=False, selected_image=None,
                 image_available=False, image_matches=False, selected_image_available=False, selected_image_matches=False)
    if not value['docker_installed']:
        return value
    info = command(['docker', 'info', '--format', '{{.ServerVersion}}'], check=False)
    # Some vendor Docker CLIs exit 0 with empty formatted stdout even when the
    # daemon reports permission denied. Require an actual server version.
    value['docker_access'] = info.returncode == 0 and bool(re.match(r'^\d+\.', info.stdout.strip()))
    value['docker_error'] = (info.stderr or info.stdout)[-500:] if not value['docker_access'] else ''
    config = Path(request.get('container_config', '~/.config/pm01_navigation/container.json')).expanduser()
    if config.exists():
        current = json.loads(config.read_text())
        if current.get('machine_id') != value['machine_id']:
            raise RuntimeError('Existing container configuration belongs to another robot; left unchanged')
        value['runtime'] = current
        value['selected_image'] = current['image']
    if not value['docker_access']:
        return value
    tool = Path(request.get('container_tool', '~/pm01_navigation_deploy/pm01_container.py')).expanduser()
    if tool.is_file():
        if not (tool.parent/'pm01_navigation_runtime').is_dir():
            # Read an existing pre-refactor deployment through its public CLI.
            # This preserves device/busy checks before the new runtime is staged.
            if value['runtime']:
                result = command([sys.executable, str(tool), '--config', str(config),
                                  '--expected-machine-id', value['machine_id'], 'session', 'status'])
                status = json.loads(result.stdout.strip().splitlines()[-1])
                value['navigation_running'] = bool(status.get('running'))
                value['container_running'] = bool(status.get('container_running'))
            return value
        module = load_runtime(tool)
        if value['runtime']:
            runtime = module.Runtime(str(config), value['machine_id'])
            status = runtime.session('status')
            value['navigation_running'] = bool(status.get('running'))
            value['container_running'] = bool(status.get('container_running'))
            try:
                selected = module.image_info(value['runtime'].get('image_id') or value['selected_image'])
                value['selected_image_available'] = True
                value['selected_image_matches'] = module.image_fingerprint(selected) == request.get('image_fingerprint')
            except RuntimeError:
                pass
        if request.get('image'):
            try:
                image = module.image_info(request['image'])
            except RuntimeError:
                pass
            else:
                value['image_available'] = True
                value['image_matches'] = module.image_fingerprint(image) == request.get('image_fingerprint')
    return value


def admin(request):
    verify_identity(request)
    if os.geteuid() != 0:
        raise PermissionError('The admin action needs sudo')
    user = pwd.getpwuid(int(request['uid']))
    if user.pw_uid == 0 or user.pw_name != request['user']:
        raise RuntimeError('Invalid target login user')
    bundle = Path(request['bundle'])
    if not shutil.which('docker'):
        command([sys.executable, str(bundle/'container_bundle.py'), 'install-engine',
                 '--bundle', str(bundle)], timeout=600)
    state = command(['systemctl', 'is-active', 'docker'], check=False)
    if state.returncode:
        command(['systemctl', 'enable', '--now', 'docker'], timeout=60)
    try:
        group = grp.getgrnam('docker')
    except KeyError:
        command(['groupadd', 'docker'])
        group = grp.getgrnam('docker')
    changed = group.gr_gid not in os.getgrouplist(user.pw_name, user.pw_gid)
    if changed:
        command(['usermod', '-aG', 'docker', user.pw_name])
    return dict(ok=True, group_changed=changed, navigation_started=False)


def install(request):
    verify_identity(request)
    if os.getuid() == 0:
        raise RuntimeError('Run installation as the ordinary robot login user')
    if busy_processes():
        raise RuntimeError('Sensor acquisition is running; deployment staged, running files unchanged')
    bundle = Path(request['bundle'])
    module = load_runtime(bundle/'pm01_container.py')
    manifest = module.verify_bundle(bundle)
    if manifest['architecture'] != module.architecture():
        raise RuntimeError('Deployment bundle architecture mismatch')
    existing = inspect(request)
    if existing['navigation_running'] or existing.get('container_running'):
        raise RuntimeError('Stop this project session before applying deployment changes; nothing was stopped')
    try:
        image = module.image_info(manifest['image'])
        loaded = module.image_fingerprint(image) == manifest['image_fingerprint']
    except RuntimeError:
        loaded = False
    if not loaded:
        module.load_bundle(bundle)
    # Keep old tools until a complete verified bundle and image are available.
    target = Path(request['container_tool']).expanduser().parent
    target.mkdir(parents=True, exist_ok=True)
    module_source = bundle/'pm01_navigation_runtime'
    module_target = target/'pm01_navigation_runtime'
    stage = target/('.runtime-stage-'+str(time.time_ns()))
    if module_source.is_dir():
        shutil.copytree(module_source, stage)
        if module_target.exists():
            module_target.rename(target/('.runtime-before-setup-'+str(time.time_ns())))
        stage.rename(module_target)
    for name in ('pm01_container.py', '_bootstrap.py', 'container_bundle.py', 'pm01_dev.py'):
        source = bundle/name
        if not source.is_file():
            continue
        destination = target/name
        payload = source.read_bytes()
        if destination.exists() and destination.read_bytes() == payload:
            continue
        if destination.exists():
            digest = hashlib.sha256(destination.read_bytes()).hexdigest()[:12]
            backup = destination.with_name(name+'.before-setup-'+digest+'.bak')
            if not backup.exists():
                shutil.copy2(destination, backup)
        with tempfile.NamedTemporaryFile(dir=target, delete=False) as stream:
            stream.write(payload)
            temporary = Path(stream.name)
        try:
            temporary.chmod(0o755)
            temporary.replace(destination)
        finally:
            temporary.unlink(missing_ok=True)
    config = str(Path(request['container_config']).expanduser())
    result = module.initialize(config, manifest['image'], request['machine_id'])
    if request.get('switch_image') and result.get('image') != manifest['image']:
        runtime = module.Runtime(config, request['machine_id'])
        result = runtime.use_image(manifest['image'])
    actual = module.Runtime(config, request['machine_id'])
    return dict(ok=True, image=actual.config['image'], config=config,
                development=actual.config.get('development', {}),
                image_loaded=not loaded, navigation_started=False, motion_started=False)


def probe(request, stop=False):
    verify_identity(request)
    nonce = request['nonce']
    if not __import__('re').fullmatch('[a-f0-9]{12}', nonce):
        raise ValueError('Invalid probe identifier')
    name = 'pm01-setup-probe-'+nonce
    if stop:
        check = command(['docker', 'container', 'inspect', name], check=False)
        if check.returncode == 0:
            info = json.loads(check.stdout)[0]
            if (info.get('Config', {}).get('Labels') or {}).get('cn.engineai.pm01.setup-probe') != nonce:
                raise RuntimeError('Probe ownership mismatch; container unchanged')
            command(['docker', 'stop', '--time', '2', name], timeout=8)
        return dict(ok=True, stopped=True)
    module = load_runtime(Path(request['container_tool']).expanduser())
    runtime = module.Runtime(request['container_config'], request['machine_id'])
    session = json.loads(Path(runtime.session_config).read_text())
    script = Path(request['probe_script'])
    # A separate bounded ROS process; never runs the image's navigation entrypoint.
    args = ['docker', 'run', '--detach', '--rm', '--name', name,
            '--label', 'cn.engineai.pm01.setup-probe='+nonce,
            '--network', 'host', '--user', str(os.getuid())+':'+str(os.getgid()),
            '--memory', '256m', '--cpus', '.5',
            '-e', 'ROS_DOMAIN_ID='+str(session['ros_domain_id']),
            '-e', 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
            '-e', 'FASTRTPS_DEFAULT_PROFILES_FILE=/probe-dds.xml',
            '-e', 'ROS_LOG_DIR=/tmp/pm01-probe-logs', '-e', 'ROS_HOME=/tmp/pm01-probe',
            '-v', str(script)+':/probe.py:ro',
            '-v', session['dds_profile']+':/probe-dds.xml:ro',
            '--entrypoint', '/bin/bash', runtime.config['image'], '-lc',
            'source /opt/ros/humble/setup.bash && exec python3 /probe.py server '+nonce]
    command(args)
    return dict(ok=True, name=name, max_duration_sec=35)


def dispatch(action, request):
    if action == 'identity':
        return dict(ok=True, **system_info())
    verify_identity(request)
    if action == 'inspect':
        return inspect(request)
    if action == 'inventory':
        return dict(ok=True, status=base64.b64encode(Path('/var/lib/dpkg/status').read_bytes()).decode())
    if action == 'admin':
        return admin(request)
    if action == 'install':
        return install(request)
    if action in ('probe-start', 'probe-stop'):
        return probe(request, action == 'probe-stop')
    raise ValueError('Unknown preparation action')


if __name__ == '__main__':
    try:
        if sys.argv[1] == 'identity':
            request = {}
        elif len(sys.argv) == 3:
            request = json.loads(Path(sys.argv[2]).read_text())
        else:
            request = json.load(sys.stdin)
        print(json.dumps(dispatch(sys.argv[1], request), ensure_ascii=False))
    except Exception as error:
        print(json.dumps(dict(ok=False, error=str(error)), ensure_ascii=False))
        raise SystemExit(1)
