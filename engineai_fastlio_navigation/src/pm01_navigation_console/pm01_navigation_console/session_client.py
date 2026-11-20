"""SSH handles only session lifecycle; ROS interfaces handle navigation."""
import json
import shlex
import subprocess


class SessionClient:
    def __init__(self, config):
        self.config = config

    def argv(self, action, map_yaml='', mode=None):
        if action not in ('identity', 'status', 'maps', 'start', 'stop', 'restart'):
            raise ValueError('Unsupported session action')
        if (self.config.profile, self.config.runtime) not in (('sim', 'native'), ('hardware', 'docker')):
            raise ValueError('Unsupported profile/runtime combination')
        docker = self.config.runtime == 'docker'
        config_path = self.config.container_config if docker else self.config.session_config
        args = (['python3', '__PM01_CONTAINER_TOOL__'] if docker else
                ['ros2', 'run', 'pm01_navigation_bringup', 'pm01_session'])
        args += ['--config', '__PM01_CONFIG__', '--expected-machine-id', self.config.expected_machine_id]
        args += ['session', action] if docker else [action]
        if action in ('start', 'restart') and mode is not None:
            if mode not in ('fresh', 'saved', 'mapping'):
                raise ValueError('Invalid mode')
            args += ['--mode', mode]
            if map_yaml:
                args += ['--map', map_yaml]
        # Expand ~/ on the destination, without evaluating configured text.
        def remote_path(value):
            return '"$HOME"/' + shlex.quote(value[2:]) if value.startswith('~/') else shlex.quote(value)
        command = 'exec ' if docker else 'source ' + remote_path(self.config.workspace+'/scripts/env.sh') + ' && exec '
        # Replace only complete argument placeholders; user map text stays quoted literally.
        command += ' '.join(remote_path(config_path) if arg == '__PM01_CONFIG__' else
                            remote_path(self.config.container_tool) if arg == '__PM01_CONTAINER_TOOL__' else
                            shlex.quote(arg) for arg in args)
        if self.config.ssh_host:
            return ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
                    '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=5',
                    '-o', 'ServerAliveCountMax=2', self.config.ssh_host, 'bash -lc '+shlex.quote(command)]
        return ['bash', '-lc', command]

    def call(self, action, map_yaml='', mode=None):
        try:
            result = subprocess.run(self.argv(action, map_yaml, mode), capture_output=True,
                                    text=True, timeout=40 if action in ('stop', 'restart') else 20)
        except subprocess.TimeoutExpired as error:
            raise TimeoutError('会话请求结果未确认；刷新状态后再决定是否重试') from error
        for line in reversed(result.stdout.splitlines()):
            try:
                response = json.loads(line)
            except ValueError:
                continue
            if isinstance(response, dict):
                if not response.get('ok'):
                    raise RuntimeError(response.get('error', 'Session operation failed'))
                return response
        raise RuntimeError((result.stderr or result.stdout)[-1500:] or 'No session response')
