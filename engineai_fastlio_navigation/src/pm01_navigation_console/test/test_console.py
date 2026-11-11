import json
from pathlib import Path
import queue
import shlex
import sys
import threading
import time
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'src/pm01_navigation_client'), str(ROOT/'src/pm01_navigation_console')]
from pm01_navigation_runtime.system.network import fastdds_xml
from pm01_navigation_console.config import Config
from pm01_navigation_console.operations import Operations
from pm01_navigation_console.session_client import SessionClient
from pm01_navigation_console.status import detail


def test_config_rejects_placeholder_wrong_domain_and_unknown_fields(tmp_path):
    p = tmp_path/'console.yaml'
    for value in [{'ssh_host': '<HOST>'}, {'ros_domain_id': 233}, {'unsupported': True},
                  {'ssh_host': '-oProxyCommand=something'}, {'local_addresses': ['not-an-ip']}]:
        p.write_text(json.dumps(value))
        with pytest.raises((ValueError, TypeError)):
            Config.load(p)


@pytest.mark.parametrize('addresses,peers,expected_peers', [
    (['127.0.0.1'], [], ['127.0.0.1']),
    (['127.0.0.1', '192.0.2.2'], ['192.0.2.3'],
     ['127.0.0.1', '192.0.2.2', '192.0.2.3']),
])
def test_profile_preserves_default_discovery_and_explicit_unicast_peers(
        addresses, peers, expected_peers):
    root = ET.fromstring(fastdds_xml(addresses, peers))
    ns = {'f': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    locators = root.findall('.//f:initialPeersList/f:locator', ns)
    assert all(len(p.findall('f:udpv4/f:address', ns)) == 1 for p in locators)
    assert [p.find('f:udpv4/f:address', ns).text for p in locators] == expected_peers
    assert root.find('.//f:useBuiltinTransports', ns).text == 'false'
    # Fast DDS adds default discovery listeners only when both lists are empty.
    assert root.find('.//f:metatrafficUnicastLocatorList', ns) is None
    assert root.find('.//f:metatrafficMulticastLocatorList', ns) is None
    assert [e.text for e in root.findall('.//f:interfaceWhiteList/f:address', ns)] == addresses


def test_remote_paths_and_map_are_arguments_not_shell_programs():
    cfg = Config(profile='sim', runtime='native', ssh_host='pm01-robot', expected_machine_id='a'*32,
                 workspace="~/robot's workspace")
    argv = SessionClient(cfg).argv('start', "/maps/a'; touch NO; '.yaml", 'saved')
    assert argv[-2] == 'pm01-robot'
    assert 'StrictHostKeyChecking=yes' in argv
    outer = shlex.split(argv[-1])
    assert outer[:2] == ['bash', '-lc']
    assert "touch NO" in outer[2]  # contained inside a quoted map argument
    after_exec = outer[2].split(' && exec ', 1)[1]
    assert shlex.split(after_exec)[-1] == "/maps/a'; touch NO; '.yaml"


def test_release_finishes_while_restart_still_waits_and_duplicate_is_refused():
    operations = Operations()
    blocked = threading.Event()
    started = threading.Event()
    def restart():
        started.set()
        assert blocked.wait(3)
        return 'restart complete'
    try:
        assert operations.submit('restart', restart)
        assert started.wait(1)
        assert not operations.submit('restart', restart)
        assert operations.submit('release', lambda: 'output withdrawn')
        name, ok, result = operations.results.get(timeout=1)
        assert (name, ok, result) == ('release', True, 'output withdrawn')
        assert 'restart' in operations.pending
    finally:
        blocked.set()
        operations.close()


def test_failed_request_remains_unknown_and_does_not_poison_release_queue():
    operations = Operations()
    def timeout():
        raise TimeoutError('outcome unknown')
    try:
        operations.submit('restart', timeout)
        assert operations.results.get(timeout=1) == ('restart', False, 'outcome unknown')
        operations.submit('release', lambda: 'confirmed')
        assert operations.results.get(timeout=1) == ('release', True, 'confirmed')
    finally:
        operations.close()


def test_map_diagnostic_age_does_not_become_a_motion_gate():
    text = detail({'motion': {'state': 'disabled', 'goal_cancel': 'unknown_timeout'},
        'diagnostics': {'/pm01/map_update_status': {'message': 'waiting_initialpose',
            'receive_age_sec': 3., 'values': {'last_successful_update_age_sec': 4.}}}})
    assert 'waiting_initialpose' in text and '7.0 秒' in text
    assert 'unknown_timeout' in text
    assert '物理控制归属：未知' in text


def test_stale_output_report_does_not_claim_current_withdrawal():
    text = detail({'motion': {'state': 'disabled', 'output_detached': True},
                   'motion_receive_age_sec': 8.})
    assert '上次输出报告（8.0 秒前）' in text
    assert '输出已撤下：当前未确认' in text


def test_late_cancel_ack_uses_already_observed_terminal_status():
    from types import SimpleNamespace
    from pm01_navigation_console.app import Backend
    backend = Backend.__new__(Backend)
    key = bytes(range(16))
    backend.goal_states = {key.hex(): 5}
    backend.cancel_request = 'unknown_timeout'
    backend.cancel_acknowledged(SimpleNamespace(return_code=0,
        goals_canceling=[SimpleNamespace(goal_id=SimpleNamespace(uuid=list(key)))]))
    assert backend.cancel_request == 'confirmed_canceled'


@pytest.mark.parametrize('custom,sim_time', [(False, False), (False, True), (True, False)])
def test_rviz_uses_shared_bringup_view_and_preserves_user_override(monkeypatch, tmp_path, custom, sim_time):
    from types import ModuleType, SimpleNamespace
    from pm01_navigation_console.app import Backend

    lookups = []
    def share(package):
        lookups.append(package)
        if package != 'pm01_navigation_bringup':
            raise LookupError('Only shared bringup resources are available')
        share_dir = tmp_path/'share'/package
        view = share_dir/'config/hardware/navigation.rviz'
        view.parent.mkdir(parents=True, exist_ok=True)
        view.write_bytes((ROOT/'config/hardware/navigation.rviz').read_bytes())
        return str(share_dir)

    index = ModuleType('ament_index_python')
    packages = ModuleType('ament_index_python.packages')
    packages.get_package_share_directory = share
    monkeypatch.setitem(sys.modules, 'ament_index_python', index)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)

    override = tmp_path/'custom.rviz'
    if custom:
        override.write_text('Visualization Manager: {}\n')
    started, stopped = [], []
    backend = Backend.__new__(Backend)
    backend.config = SimpleNamespace(rviz_config=str(override) if custom else '',
                                     profile='sim' if sim_time else 'hardware')
    backend.rviz = SimpleNamespace(start=started.append, stop=stopped.append)
    backend.restart_rviz()

    command = started[0]
    view = Path(command[command.index('-d')+1])
    assert view.is_file()
    assert view == (override if custom else tmp_path/'share/pm01_navigation_bringup/config/hardware/navigation.rviz')
    assert lookups == ([] if custom else ['pm01_navigation_bringup'])
    assert command[-1] == 'use_sim_time:='+str(sim_time).lower()
    assert stopped == [2.]


def test_hardware_defaults_to_docker_and_native_hardware_is_retired(tmp_path):
    path=tmp_path/'console.yaml'
    path.write_text('profile: hardware\n')
    assert Config.load(path).runtime == 'docker'
    path.write_text('profile: hardware\nruntime: native\n')
    with pytest.raises(ValueError, match='setup_hardware'):
        Config.load(path)
    path.write_text('profile: sim\n')
    assert Config.load(path).runtime == 'native'
