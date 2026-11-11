"""Read one developer-owned deployment YAML; no ROS or network side effects."""
import ipaddress
from pathlib import Path
import re

import yaml


def load_deployment(filename):
    path = Path(filename).expanduser().resolve(strict=True)
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or data.get('schema') != 1:
        raise ValueError('deployment.yaml requires schema: 1')
    allowed = {'schema', 'robot', 'developer', 'network'}
    if set(data) - allowed:
        raise ValueError('Unknown deployment fields: '+', '.join(sorted(set(data)-allowed)))
    fields = {
        'robot': {'ssh_host', 'address', 'machine_id', 'container_tool', 'container_config'},
        'developer': {'address'},
        'network': {'ros_domain_id', 'robot_lan_address', 'motion_peer'},
    }
    for group, names in fields.items():
        value = data.get(group, {})
        if not isinstance(value, dict) or set(value) - names:
            raise ValueError('Invalid deployment section: '+group)
    robot, developer, network = (dict(data.get(k, {})) for k in fields)
    host = robot.get('ssh_host', 'pm01-robot')
    if not isinstance(host, str) or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]*', host):
        raise ValueError('robot.ssh_host must be a configured SSH alias')
    identity = robot.get('machine_id', '')
    if not isinstance(identity, str) or not re.fullmatch('[0-9a-f]{32}', identity):
        raise ValueError('robot.machine_id must be the verified 32-character machine ID')
    robot.update(ssh_host=host, machine_id=identity)
    robot.setdefault('container_tool', '~/pm01_navigation_deploy/pm01_container.py')
    robot.setdefault('container_config', '~/.config/pm01_navigation/container.json')
    for key in ('container_tool', 'container_config'):
        value = robot[key]
        if not isinstance(value, str) or not value.startswith(('~/', '/')) or any(c in value for c in '\n\r\0'):
            raise ValueError('robot.'+key+' must be an absolute path or start with ~/')
    network.setdefault('ros_domain_id', 69)
    network.setdefault('robot_lan_address', '192.168.0.162')
    network.setdefault('motion_peer', '192.168.0.163')
    domain = network['ros_domain_id']
    if type(domain) is not int or not 0 <= domain <= 232:
        raise ValueError('network.ros_domain_id must be an integer in 0..232')
    for group, key in ((robot, 'address'), (developer, 'address'),
                       (network, 'robot_lan_address'), (network, 'motion_peer')):
        try:
            group[key] = str(ipaddress.IPv4Address(group.get(key, '')))
        except ipaddress.AddressValueError as error:
            raise ValueError('Fill a valid IPv4 address for '+key) from error
    if robot['address'] == developer['address']:
        raise ValueError('Robot and developer addresses must be different')
    return dict(schema=1, robot=robot, developer=developer, network=network,
                console_config=str(path.parent/'console.yaml'))


def clock_connection(config, host=None, machine_id=None):
    """Explicit legacy flags are allowed when they agree with the selected file."""
    if config:
        robot = load_deployment(config)['robot']
        if (host and host != robot['ssh_host']) or (machine_id and machine_id != robot['machine_id']):
            raise ValueError('Clock arguments disagree with deployment configuration')
        return robot['ssh_host'], robot['machine_id']
    if not host:
        raise ValueError('Supply --config deployment.yaml or --ssh-host')
    return host, machine_id or ''


#!/usr/bin/env python3
"""Generate user-owned deployment files; apply updates both ends over SSH.

Legacy robot/desktop rendering stays local. No operation starts navigation.
"""
import argparse
import ipaddress
import json
import shlex
import subprocess
from pathlib import Path
import sys

from pm01_navigation_runtime.system.network import fastdds_xml, write_profile


def apply_deployment(filename):
    from .hardware_apply import replace_file
    import yaml
    deployment = load_deployment(filename)
    robot, developer, network = (deployment[k] for k in ('robot', 'developer', 'network'))
    path = Path(deployment['console_config'])
    existing = yaml.safe_load(path.read_text()) if path.exists() else {}
    if existing and (not isinstance(existing, dict) or existing.get('expected_machine_id') != robot['machine_id']):
        raise ValueError('Existing desktop configuration belongs to another robot; use its own directory')
    desired = dict(existing or {})
    desired.update(profile='hardware', runtime='docker', ssh_host=robot['ssh_host'],
                   expected_machine_id=robot['machine_id'], container_tool=robot['container_tool'],
                   container_config=robot['container_config'], ros_domain_id=network['ros_domain_id'],
                   local_addresses=list(dict.fromkeys(['127.0.0.1', developer['address']])),
                   peers=[robot['address']])
    desired.setdefault('workspace', '~/pm01_navigation_ws')
    desired.setdefault('rviz_config', '')
    desired.setdefault('request_timeout', 12.)
    # Generate the robot XML here; the robot never imports PyYAML or ROS.
    deployment['robot_dds'] = fastdds_xml(
        ['127.0.0.1', robot['address'], network['robot_lan_address']],
        [developer['address'], network['motion_peer']])
    from pm01_navigation_runtime.containers.bundle import standalone_program
    program = standalone_program('deployment.hardware_apply')
    try:
        result = subprocess.run(['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
                                 '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=5',
                                 '-o', 'ServerAliveCountMax=2', robot['ssh_host'],
                                 'python3 -c '+shlex.quote(program)],
                                input=json.dumps(deployment), capture_output=True, text=True, timeout=45)
    except subprocess.TimeoutExpired as error:
        raise RuntimeError('Configuration apply result unconfirmed; reconnect and repeat the same command') from error
    try:
        response = json.loads(result.stdout.strip().splitlines()[-1])
    except (ValueError, IndexError) as error:
        raise RuntimeError((result.stderr or result.stdout)[-1500:] or 'No robot configuration response') from error
    if result.returncode or not response.get('ok'):
        raise RuntimeError(response.get('error') or (result.stderr[-1500:] or 'Robot configuration failed'))
    desired['session_config'] = response['session_config']
    console = replace_file(path, yaml.safe_dump(desired, allow_unicode=True, sort_keys=False))
    profile = replace_file(path.parent/'fastdds-desktop.xml', fastdds_xml(desired['local_addresses'], desired['peers']))
    return dict(ok=True, desktop=dict(console_config=str(path), files=[console, profile]), robot=response,
                message='两端配置已应用；未启动导航。重新加载开发电脑环境后使用。')


def apply_main():
    parser = argparse.ArgumentParser(description='Read one deployment YAML and apply both runtime configurations over SSH')
    parser.add_argument('operation', choices=['apply'])
    parser.add_argument('--config', required=True, help='Developer-owned deployment.yaml')
    args = parser.parse_args()
    try:
        print(json.dumps(apply_deployment(args.config), ensure_ascii=False))
        return 0
    except Exception as error:
        print(json.dumps(dict(ok=False, error=str(error)), ensure_ascii=False))
        return 1


def configure_main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role', choices=['robot', 'desktop'])
    parser.add_argument('--robot-address', required=True)
    parser.add_argument('--developer-address', required=True)
    parser.add_argument('--robot-lan-address', default='192.168.0.162')
    parser.add_argument('--motion-peer', default='192.168.0.163')
    parser.add_argument('--domain', type=int, default=69)
    parser.add_argument('--workspace', default='~/pm01_navigation_ws')
    parser.add_argument('--ssh-host', default='pm01-robot')
    parser.add_argument('--machine-id', default='')
    parser.add_argument('--output-dir', default='~/.config/pm01_navigation')
    parser.add_argument('--lidar-config', default='')
    parser.add_argument('--runtime', choices=['docker'], default='docker',
                        help='Desktop session management backend; Docker is recommended for hardware')
    parser.add_argument('--container-tool', default='~/pm01_navigation_deploy/pm01_container.py')
    parser.add_argument('--container-config', default='~/.config/pm01_navigation/container.json')
    args = parser.parse_args()
    try:
        for value in (args.robot_address, args.developer_address, args.robot_lan_address, args.motion_peer):
            ipaddress.IPv4Address(value)
    except ValueError as error:
        parser.error(str(error))
    if not 0 <= args.domain <= 232:
        parser.error('domain must be 0..232')
    out = Path(args.output_dir).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    path = out/('session.json' if args.role == 'robot' else 'console.yaml')
    if path.exists():
        # Repeating an installation step must not destroy tuned site settings
        # or fail merely because the configuration was already generated.
        import yaml
        try:
            existing = yaml.safe_load(path.read_text())
            expected = (Path('/etc/machine-id').read_text().strip()
                        if args.role == 'robot' else args.machine_id)
            if not expected or (args.machine_id and args.machine_id != expected):
                raise ValueError('robot machine ID mismatch')
            if not isinstance(existing, dict) or existing.get('expected_machine_id') != expected:
                raise ValueError('existing configuration belongs to a different robot')
            required = ('workspace', 'ros_domain_id', 'dds_profile') if args.role == 'robot' else ('ssh_host', 'ros_domain_id', 'local_addresses', 'peers')
            if any(key not in existing for key in required):
                raise ValueError('existing configuration is incomplete')
        except (OSError, ValueError, yaml.YAMLError) as error:
            parser.error(str(path)+': '+str(error))
        print(str(path))
        print('Using existing configuration unchanged; command-line settings were not reapplied. '
              'Edit this file to change addresses or parameters.', file=sys.stderr)
        return
    if args.role == 'robot':
        machine_id = Path('/etc/machine-id').read_text().strip()
        if args.machine_id and args.machine_id != machine_id:
            parser.error('robot machine ID mismatch')
        dds = write_profile(out/'fastdds-robot.xml',
            ['127.0.0.1', args.robot_address, args.robot_lan_address],
            [args.developer_address, args.motion_peer])
        data = {'workspace': str(Path(args.workspace).expanduser()), 'profile': 'hardware',
                'expected_machine_id': machine_id, 'ros_domain_id': args.domain,
                'dds_profile': dds, 'map_root': '~/.local/share/pm01_navigation/maps', 'launch_args': {}}
        if args.lidar_config:
            data['launch_args']['lidar_config'] = str(Path(args.lidar_config).expanduser().resolve(strict=True))
        path.write_text(json.dumps(data, indent=2)+'\n')
    else:
        if not args.machine_id:
            parser.error('desktop requires the verified robot --machine-id')
        import yaml
        data = {'profile': 'hardware', 'ssh_host': args.ssh_host, 'expected_machine_id': args.machine_id,
                'workspace': args.workspace, 'session_config': '~/.config/pm01_navigation/session.json',
                'ros_domain_id': args.domain, 'local_addresses': ['127.0.0.1', args.developer_address],
                'peers': [args.robot_address], 'rviz_config': '', 'request_timeout': 12.}
        if args.runtime == 'docker':
            data.update(runtime='docker', container_tool=args.container_tool,
                        container_config=args.container_config,
                        session_config='~/.config/pm01_navigation/docker/session.json')
        path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
    path.chmod(0o600)
    print(str(path))



def main():
    return apply_main() if len(sys.argv) > 1 and sys.argv[1] == "apply" else configure_main()
