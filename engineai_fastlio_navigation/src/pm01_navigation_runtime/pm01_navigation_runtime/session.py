#!/usr/bin/env python3
"""Own one launch process group. Never stop a process discovered by name."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
import yaml

LAUNCH_ARGS = {'start_sensors', 'driver_params', 'lidar_config', 'lio_params',
               'extrinsics', 'nav2_params', 'map_params', 'obstacle_params', 'gui',
               'localization_backend', 'slam_params', 'amcl_params', 'scan_params'}


def identity():
    import platform
    return {'machine_id': Path('/etc/machine-id').read_text().strip(),
            'hostname': platform.node(), 'architecture': platform.machine()}


def start_ticks(pid):
    try:
        # comm may contain spaces/parentheses; fields after its closing ')'
        # start with state (field 3); starttime is field 22.
        return Path(f'/proc/{int(pid)}/stat').read_text().rsplit(')', 1)[1].split()[19]
    except (OSError, ValueError, TypeError, IndexError):
        return None


def alive(pid, ticks):
    if not ticks or start_ticks(pid) != str(ticks):
        return False
    try:
        return Path(f'/proc/{int(pid)}/stat').read_text().rsplit(')', 1)[1].split()[0] != 'Z'
    except OSError:
        return False


def validate_map(filename):
    path = Path(filename).expanduser().resolve(strict=True)
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or not isinstance(data.get('image'), str):
        raise ValueError('Invalid map YAML')
    import math
    resolution = float(data['resolution'])
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Invalid map resolution')
    origin = data['origin']
    if len(origin) != 3 or not all(math.isfinite(float(v)) for v in origin):
        raise ValueError('Invalid map origin')
    if not 0 <= float(data['free_thresh']) < float(data['occupied_thresh']) <= 1:
        raise ValueError('Invalid map thresholds')
    image = path.parent / data['image']
    if not image.is_file() or not image.stat().st_size:
        raise ValueError('Map image missing/empty')
    return str(path)


class Session:
    def __init__(self, filename, expected=''):
        self.filename = str(Path(filename).expanduser().resolve())
        self.config = json.loads(Path(self.filename).read_text())
        expected = expected or self.config.get('expected_machine_id', '')
        self.identity = identity()
        if not expected or expected != self.identity['machine_id']:
            raise RuntimeError('Robot identity mismatch; no action performed')
        self.workspace = Path(self.config['workspace']).expanduser().resolve(strict=True)
        self.profile = self.config.get('profile', 'hardware')
        if self.profile not in ('hardware', 'sim'):
            raise ValueError('Invalid profile')
        if not 0 <= int(self.config['ros_domain_id']) <= 232:
            raise ValueError('Invalid ROS domain')
        arguments = self.config.get('launch_args', {})
        if not isinstance(arguments, dict) or set(arguments) - LAUNCH_ARGS:
            raise ValueError('Unsupported launch override')
        self.root = Path(self.config.get('state_root', '~/.local/state/pm01_navigation')).expanduser()
        self.root /= hashlib.sha256(str(self.workspace).encode()).hexdigest()[:12]
        self.root.mkdir(parents=True, exist_ok=True)
        self.state_path = self.root/'session.json'

    def env(self):
        env = os.environ.copy()
        env.update(RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                   ROS_DOMAIN_ID=str(self.config['ros_domain_id']), ROS_LOCALHOST_ONLY='0',
                   PM01_USE_SIM_TIME=str(self.profile == 'sim').lower())
        env.pop('CYCLONEDDS_URI', None)
        if self.config.get('gazebo_master_uri'):
            env['GAZEBO_MASTER_URI'] = self.config['gazebo_master_uri']
        for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
            env.pop(key, None)
        if self.config.get('dds_profile'):
            path = str(Path(self.config['dds_profile']).expanduser().resolve(strict=True))
            env['FASTRTPS_DEFAULT_PROFILES_FILE'] = path
        elif self.profile == 'sim':
            from pm01_navigation_runtime.system.network import write_profile
            env['FASTRTPS_DEFAULT_PROFILES_FILE'] = write_profile(
                self.root/'fastdds-loopback.xml', ['127.0.0.1'], [])
        return env

    def read(self):
        return json.loads(self.state_path.read_text()) if self.state_path.exists() else {}

    def write(self, state):
        temp = self.state_path.with_suffix('.tmp')
        temp.write_text(json.dumps(state)+'\n')
        temp.replace(self.state_path)

    def status(self):
        state = self.read()
        running = alive(state.get('supervisor_pid'), state.get('supervisor_ticks'))
        return dict(ok=True, identity=self.identity, running=running, session=state,
                    log=str(self.root/'launch.log'))

    def conflicts(self):
        names = {'online_navigation_map.py', 'manual_map_alignment.py', 'motion_manager.py',
                 'async_slam_toolbox_node', 'amcl', 'planar_scan.py', 'map_matching_adapter.py'}
        if str(self.config.get('launch_args', {}).get('start_sensors', 'true')).lower() == 'true':
            names |= {'fastlio_mapping', 'livox_ros_driver2_node'}
        conflicts = []
        for p in Path('/proc').iterdir():
            if not p.name.isdigit():
                continue
            try:
                args = (p/'cmdline').read_bytes().split(b'\0')
                executable_names = {Path(a.decode(errors='replace')).name for a in args if a}
                if not names.intersection(executable_names):
                    continue
                environment = (p/'environ').read_bytes().split(b'\0')
                domain = next((s.split(b'=', 1)[1].decode() for s in environment if s.startswith(b'ROS_DOMAIN_ID=')), '0')
                sensor = bool({'fastlio_mapping', 'livox_ros_driver2_node'} & executable_names)
                if domain == str(self.config['ros_domain_id']) or (sensor and self.profile == 'hardware'):
                    conflicts.append({'pid': int(p.name), 'names': sorted(names & executable_names), 'domain': domain})
            except (OSError, ValueError):
                continue
        return conflicts

    def start(self, mode, map_yaml):
        if mode not in ('mapping', 'fresh', 'saved'):
            raise ValueError('Invalid mode')
        if mode == 'saved':
            map_yaml = validate_map(map_yaml)
        else:
            map_yaml = ''
        state = self.read()
        if self.status()['running'] or (state.get('phase') == 'starting' and time.time()-state.get('requested_at', 0) < 10):
            if state.get('mode') == mode and state.get('map', '') == map_yaml:
                return {'ok': True, 'already_running': True, 'session_id': state.get('id'),
                        'mode': mode, 'log': str(self.root/'launch.log'),
                        'message': 'Existing session retained; no restart or output change'}
            raise RuntimeError('Session already running with another mode/map; use restart to switch')
        conflicts = self.conflicts()
        if conflicts:
            raise RuntimeError('Existing sensor/navigation processes; left untouched: '+json.dumps(conflicts))
        token = uuid.uuid4().hex
        self.write({'id': token, 'mode': mode, 'map': map_yaml, 'phase': 'starting', 'requested_at': time.time()})
        with (self.root/'supervisor.log').open('a') as log:
            subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--config', self.filename,
                '--expected-machine-id', self.identity['machine_id'], 'supervise', '--id', token],
                env=self.env(), cwd=self.workspace, stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        return {'ok': True, 'message': '启动请求已提交；请查看状态及地图', 'session_id': token}

    def supervise(self, token):
        state = self.read()
        if state.get('id') != token:
            raise RuntimeError('Superseded session')
        state.update(supervisor_pid=os.getpid(), supervisor_ticks=start_ticks(os.getpid()))
        self.write(state)
        arguments = ['ros2', 'launch', 'pm01_navigation_bringup', 'navigation.launch.py',
            'profile:='+self.profile, 'mode:='+state['mode'], 'rviz:=false',
            'save_root:='+str(Path(self.config['map_root']).expanduser())]
        if state['map']:
            arguments += ['map:='+state['map']]
        arguments += [str(k)+':='+str(v).lower() if isinstance(v, bool) else str(k)+':='+str(v)
                      for k, v in self.config.get('launch_args', {}).items()]
        with (self.root/'launch.log').open('a', buffering=1) as log:
            child = subprocess.Popen(arguments, env=self.env(), cwd=self.workspace,
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            state.update(launch_pid=child.pid, launch_ticks=start_ticks(child.pid), phase='running')
            self.write(state)
            stopping = False
            def requested(signum, frame):
                nonlocal stopping
                stopping = True
            def terminate():
                for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
                    if child.poll() is not None:
                        break
                    if alive(child.pid, state['launch_ticks']):
                        os.killpg(child.pid, sig)
                    try:
                        child.wait(timeout=timeout)
                    except subprocess.TimeoutExpired:
                        pass
            signal.signal(signal.SIGINT, requested)
            signal.signal(signal.SIGTERM, requested)
            while child.poll() is None:
                if stopping:
                    terminate()
                time.sleep(.1)
            code = child.returncode
            state.update(phase='stopped', exit_code=code, stopped_at=time.time())
            self.write(state)
        return self.status()

    def stop(self):
        state = self.read()
        if not self.status()['running']:
            if alive(state.get('launch_pid'), state.get('launch_ticks')):
                raise RuntimeError('Supervisor missing but launch remains; inspect owned session before stopping')
            return {'ok': True, 'message': '本项目没有运行中的会话'}
        # Output withdrawal must be confirmed before stopping its manager.
        result = subprocess.run(['ros2', 'run', 'pm01_navigation_client', 'pm01_nav',
            '--timeout', '5', 'release-motion'], env=self.env(), capture_output=True, text=True, timeout=7)
        if result.returncode:
            raise RuntimeError('Output release unconfirmed; session retained: '+(result.stderr+result.stdout)[-800:])
        if alive(state['supervisor_pid'], state['supervisor_ticks']):
            os.kill(state['supervisor_pid'], signal.SIGTERM)
        deadline = time.monotonic()+13
        while self.status()['running'] and time.monotonic()<deadline:
            time.sleep(.1)
        if self.status()['running'] or alive(state.get('launch_pid'), state.get('launch_ticks')):
            raise RuntimeError('Session stop unconfirmed')
        return {'ok': True, 'message': '本项目输出已撤下，导航会话已停止'}

    def maps(self):
        root = Path(self.config['map_root']).expanduser()
        maps = []
        if root.exists():
            for path in sorted(root.rglob('*.yaml'), key=lambda p: p.stat().st_mtime, reverse=True)[:100]:
                try:
                    maps.append(validate_map(str(path)))
                except (OSError, ValueError, KeyError, TypeError):
                    continue
        return {'ok': True, 'maps': maps}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--expected-machine-id', default='')
    parser.add_argument('action', choices=['identity', 'status', 'maps', 'start', 'stop', 'restart', 'supervise'])
    parser.add_argument('--mode', choices=['mapping', 'fresh', 'saved'])
    parser.add_argument('--map', default='')
    parser.add_argument('--id', default='')
    args = parser.parse_args()
    try:
        session = Session(args.config, args.expected_machine_id)
        if args.action == 'restart' and args.mode is None:
            previous = session.read()
            args.mode, args.map = previous.get('mode', 'fresh'), previous.get('map', '')
        args.mode = args.mode or 'fresh'
        if args.action in ('identity', 'status'):
            result = session.status()
        elif args.action == 'maps':
            result = session.maps()
        elif args.action == 'supervise':
            result = session.supervise(args.id)
        else:
            with (session.root/'operation.lock').open('a') as lock:
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    raise RuntimeError('Another session operation is in progress; refresh status')
                if args.action in ('start', 'restart'):
                    if args.mode == 'saved':
                        validate_map(args.map)  # Validate before disturbing the current session.
                if args.action in ('stop', 'restart'):
                    result = session.stop()
                if args.action in ('start', 'restart'):
                    result = session.start(args.mode, args.map)
        print(json.dumps(result, ensure_ascii=False))
    except Exception as error:
        print(json.dumps({'ok': False, 'error': str(error)}, ensure_ascii=False))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
