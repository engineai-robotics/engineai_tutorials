"""Explicit deployment configuration. No credentials or implicit deployment."""
from dataclasses import dataclass
from pathlib import Path
import re
import yaml
from pm01_navigation_runtime.system.network import fastdds_xml


@dataclass(frozen=True)
class Config:
    profile: str = 'hardware'
    ssh_host: str = ''
    expected_machine_id: str = ''
    workspace: str = '~/pm01_navigation_ws'
    session_config: str = '~/.config/pm01_navigation/session.json'
    runtime: str = 'docker'
    container_tool: str = '~/pm01_navigation_deploy/pm01_container.py'
    container_config: str = '~/.config/pm01_navigation/container.json'
    ros_domain_id: int = 69
    local_addresses: tuple = ('127.0.0.1',)
    peers: tuple = ()
    rviz_config: str = ''
    request_timeout: float = 12.

    @classmethod
    def load(cls, filename):
        data = yaml.safe_load(Path(filename).expanduser().read_text())
        if not isinstance(data, dict):
            raise ValueError('Console configuration must be a mapping')
        data.setdefault('runtime', 'native' if data.get('profile') == 'sim' else 'docker')
        config = cls(**data)
        if config.profile not in ('hardware', 'sim') or not 0 <= config.ros_domain_id <= 232:
            raise ValueError('Invalid profile or ROS domain')
        if (config.profile, config.runtime) not in (('sim', 'native'), ('hardware', 'docker')):
            raise ValueError('Use native simulation or Docker hardware; rerun setup_hardware.sh for old hardware configurations')
        if config.ssh_host and not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]*', config.ssh_host):
            raise ValueError('Use an SSH config alias, without options or shell syntax')
        if config.ssh_host and not re.fullmatch(r'[0-9a-f]{32}', config.expected_machine_id):
            raise ValueError('Set expected_machine_id from the verified robot')
        if not 1 <= config.request_timeout <= 120:
            raise ValueError('request_timeout must be 1..120 seconds')
        for path in (config.workspace, config.session_config, config.container_tool, config.container_config):
            if not path.startswith(('/', '~/')) or '\n' in path:
                raise ValueError('Use an absolute or ~/ path')
        fastdds_xml(config.local_addresses, config.peers)
        return config
