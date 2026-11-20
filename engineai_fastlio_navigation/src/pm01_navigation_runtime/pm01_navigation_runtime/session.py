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

LAUNCH_ARGS = {'start_sensors', 'driver_params', 'lidar_config', 'lio_params', 'lio_cpu',
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


def descendants(parent, ticks):
    if not alive(parent, ticks):
        return {}
    parents, identities = {}, {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry/'stat').read_text().rsplit(')', 1)[1].split()
            parents[int(entry.name)], identities[int(entry.name)] = int(fields[1]), fields[19]
        except (OSError, ValueError, IndexError):
            continue
    owned = {int(parent): str(ticks)}
    while True:
        added = {pid: identities[pid] for pid, ppid in parents.items() if ppid in owned and pid not in owned}
        if not added:
            return owned
        owned.update(added)


def stop_descendants(owned):
    for sig, timeout in ((signal.SIGINT, 1.), (signal.SIGTERM, 1.), (signal.SIGKILL, .5)):
        pending = [pid for pid, ticks in owned.items() if alive(pid, ticks)]
        if not pending:
            return
        for pid in pending:
            try:
                if alive(pid, owned[pid]):
                    os.kill(pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic()+timeout
        while any(alive(pid, owned[pid]) for pid in pending) and time.monotonic() < deadline:
            time.sleep(.05)


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
            from pm01_navigation_runtime.system.network import write_sim_profile
            env['FASTRTPS_DEFAULT_PROFILES_FILE'] = write_sim_profile(self.root/'fastdds-sim-local.xml')
        return env

    def read(self):
        return json.loads(self.state_path.read_text()) if self.state_path.exists() else {}

    def write(self, state):
        temp = self.state_path.with_suffix('.tmp')
        temp.write_text(json.dumps(state)+'\n')
        temp.replace(self.state_path)

    def startup_state(self, token, mode, phase, reason):
        target = self.root/'startup.json'
        temporary = target.with_suffix('.tmp')
        temporary.write_text(json.dumps(dict(session_id=token, mode=mode, ready=False,
            phase=phase, reason=reason, heartbeat_wall_sec=time.time())))
        temporary.replace(target)

    def status(self):
        state = self.read()
        running = alive(state.get('supervisor_pid'), state.get('supervisor_ticks'))
        startup = {}
        try:
            candidate = json.loads((self.root/'startup.json').read_text())
            if candidate.get('session_id') == state.get('id'):
                startup = candidate
                startup['heartbeat_current'] = running and 0 <= time.time()-candidate['heartbeat_wall_sec'] <= 2.5
                if not startup['heartbeat_current'] and startup.get('phase') not in ('failed', 'stopped'):
                    startup.update(ready=False, reason='Startup status stale or session stopped')
        except (OSError, ValueError, KeyError, TypeError):
            pass
        return dict(ok=True, identity=self.identity, running=running, session=state, startup=startup,
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
                        'startup_status_file': str(self.root/'startup.json'),
                        'message': 'Existing session retained; no restart or output change'}
            raise RuntimeError('Session already running with another mode/map; use restart to switch')
        conflicts = self.conflicts()
        if conflicts:
            raise RuntimeError('Existing sensor/navigation processes; left untouched: '+json.dumps(conflicts))
        token = uuid.uuid4().hex
        self.write({'id': token, 'mode': mode, 'map': map_yaml, 'phase': 'starting', 'requested_at': time.time()})
        self.startup_state(token, mode, 'starting', '正在启动传感器与定位 / Starting sensors and localization')
        with (self.root/'supervisor.log').open('a') as log:
            subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--config', self.filename,
                '--expected-machine-id', self.identity['machine_id'], 'supervise', '--id', token],
                env=self.env(), cwd=self.workspace, stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        return {'ok': True, 'message': '启动请求已提交；请查看状态及地图', 'session_id': token,
                'startup_status_file': str(self.root/'startup.json')}

    def supervise(self, token):
        with (self.root/'operation.lock').open('a') as lock, (self.root/'launch.log').open('a', buffering=1) as log:
            fcntl.flock(lock, fcntl.LOCK_EX)
            state = self.read()
            if state.get('id') != token or state.get('phase') != 'starting':
                return {'ok': True, 'message': 'Pending startup was canceled or superseded'}
            state.update(supervisor_pid=os.getpid(), supervisor_ticks=start_ticks(os.getpid()))
            self.write(state)
            arguments = ['ros2', 'launch', 'pm01_navigation_bringup', 'navigation.launch.py',
                'profile:='+self.profile, 'mode:='+state['mode'], 'rviz:=false', 'session_id:='+token,
                'save_root:='+str(Path(self.config['map_root']).expanduser()),
                'startup_status_file:='+str(self.root/'startup.json')]
            if state['map']:
                arguments += ['map:='+state['map']]
            arguments += [str(k)+':='+str(v).lower() if isinstance(v, bool) else str(k)+':='+str(v)
                          for k, v in self.config.get('launch_args', {}).items()]
            log_start = log.tell()
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
            fcntl.flock(lock, fcntl.LOCK_UN)
            while child.poll() is None:
                if stopping:
                    terminate()
                time.sleep(.1)
            code = child.returncode
            state.update(phase='stopped', exit_code=code, stopped_at=time.time())
            if self.read().get('id') == token:
                self.write(state)
                reason = '会话已停止 / Session stopped'
                if not stopping:
                    with (self.root/'launch.log').open() as stream:
                        stream.seek(log_start)
                        errors = [line.strip() for line in stream if '[ERROR]' in line]
                    reason = '启动进程退出 / Launch exited: '+(errors[0][-1000:] if errors else str(code))
                self.startup_state(token, state['mode'], 'stopped' if stopping else 'failed', reason)
        return self.status()

    def stop(self):
        state = self.read()
        if not self.status()['running']:
            if alive(state.get('launch_pid'), state.get('launch_ticks')):
                raise RuntimeError('Supervisor missing but launch remains; inspect owned session before stopping')
            if state.get('phase') == 'starting':
                state.update(phase='stopped', stopped_at=time.time())
                self.write(state)
                self.startup_state(state['id'], state['mode'], 'stopped', '已取消待启动会话 / Pending startup canceled')
                return {'ok': True, 'message': '已取消待启动会话'}
            return {'ok': True, 'message': '本项目没有运行中的会话'}
        # Ask the manager to withdraw output before stopping this session.
        owned = descendants(state.get('launch_pid'), state.get('launch_ticks'))
        try:
            result = subprocess.run(['ros2', 'run', 'pm01_navigation_client', 'pm01_nav',
                '--timeout', '5', 'release-motion'], env=self.env(), capture_output=True, text=True, timeout=7)
            released = result.returncode == 0
        except subprocess.TimeoutExpired:
            released = False
        # During partial startup the manager may not exist. End only this launch
        # and its recorded descendants; the container layer owns its own fallback.
        if alive(state['supervisor_pid'], state['supervisor_ticks']):
            os.kill(state['supervisor_pid'], signal.SIGTERM)
        deadline = time.monotonic()+13
        while self.status()['running'] and time.monotonic()<deadline:
            time.sleep(.1)
        if self.status()['running'] or alive(state.get('launch_pid'), state.get('launch_ticks')):
            raise RuntimeError('Session stop unconfirmed')
        stop_descendants(owned)
        remaining = [pid for pid, ticks in owned.items() if alive(pid, ticks)]
        if remaining:
            raise RuntimeError('Owned startup processes still running: '+str(remaining))
        return {'ok': True, 'release_confirmed': released, 'message': '本项目导航会话已停止'}

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
    parser.add_argument('--no-wait', action='store_true')
    parser.add_argument('--timeout', type=float, default=120.)
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
        if (args.action in ('start', 'restart') and not args.no_wait and sys.stderr.isatty()):
            response = subprocess.run(['ros2', 'run', 'pm01_navigation_client', 'pm01_nav',
                '--timeout', str(args.timeout), 'wait-ready', '--session-id', result['session_id'],
                '--allow-initial-pose', '--status-file', str(session.root/'startup.json')], env=session.env(), stdout=subprocess.PIPE, text=True,
                timeout=args.timeout+5.)
            if response.returncode == 0:
                result['startup'] = json.loads(response.stdout.strip().splitlines()[-1])
            else:
                result['startup'] = {'ready': False, 'reason': 'Startup wait ended; inspect status'}
        print(json.dumps(result, ensure_ascii=False))
    except KeyboardInterrupt:
        print('启动等待已取消；后台会话保持运行，运动输出未自动接通 / Startup wait canceled; session remains running', file=sys.stderr)
        return 130
    except Exception as error:
        print(json.dumps({'ok': False, 'error': str(error)}, ensure_ascii=False))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
