import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

from pm01_navigation_runtime import session
SCRIPT = Path(session.__file__)


@pytest.fixture(autouse=True)
def platform_identity(monkeypatch):
    if not Path('/etc/machine-id').exists():
        monkeypatch.setattr(session, 'identity', lambda: {'machine_id': 'a'*32,
            'hostname': 'unit-test', 'architecture': 'test'})


def config(tmp_path):
    filename = tmp_path/'config.json'
    filename.write_text(json.dumps({'workspace': str(tmp_path), 'ros_domain_id': 193,
        'state_root': str(tmp_path/'state'), 'map_root': str(tmp_path/'maps'),
        'expected_machine_id': session.identity()['machine_id']}))
    return filename


@pytest.mark.skipif(not Path('/proc/self/stat').exists(), reason='Linux process identity')
@pytest.mark.skipif(session.start_ticks(os.getpid()) == '0',
                   reason='QEMU synthetic self start time; process identity requires native Linux')
def test_owned_group_stop_and_duplicate_start_preserve_unrelated_process(tmp_path, monkeypatch):
    binary = tmp_path/'ros2'
    binary.write_text('#!/usr/bin/env python3\nimport subprocess,sys,time\n'
        'if sys.argv[1]=="run": sys.exit(0)\n'
        'subprocess.Popen([sys.executable,"-c","import time; time.sleep(60)"])\n'
        'time.sleep(60)\n')
    binary.chmod(0o755)
    monkeypatch.setenv('PATH', str(tmp_path)+os.pathsep+os.environ['PATH'])
    filename = config(tmp_path)
    owner = session.Session(filename)
    monkeypatch.setattr(owner, 'conflicts', lambda: [])
    unrelated = subprocess.Popen(['sleep', '60'])
    try:
        owner.start('fresh', '')
        deadline = time.monotonic()+4
        while not owner.status()['running'] and time.monotonic()<deadline:
            time.sleep(.05)
        assert owner.status()['running']
        while not owner.read().get('launch_ticks') and time.monotonic()<deadline:
            time.sleep(.05)
        original = owner.read()
        reused = owner.start('fresh', '')
        assert reused['ok'] and reused['already_running'] and reused['session_id'] == original['id']
        assert owner.read() == original  # No new launch, reset or output change.
        with pytest.raises(RuntimeError, match='already running'):
            owner.start('mapping', '')
        assert owner.stop()['ok']
        assert not owner.status()['running']
        assert not session.alive(original['launch_pid'], original['launch_ticks'])
        assert unrelated.poll() is None
    finally:
        unrelated.terminate(); unrelated.wait()
        if owner.status()['running']:
            owner.stop()


def test_wrong_robot_does_not_create_runtime_directory(tmp_path):
    filename = config(tmp_path)
    with pytest.raises(RuntimeError, match='identity mismatch'):
        session.Session(filename, '0'*32)
    assert not (tmp_path/'state').exists()


def test_conflict_refuses_launch_and_leaves_owner_untouched(tmp_path, monkeypatch):
    owner = session.Session(config(tmp_path))
    monkeypatch.setattr(owner, 'conflicts', lambda: [{'pid': 123, 'names': ['fastlio_mapping']}])
    with pytest.raises(RuntimeError, match='left untouched'):
        owner.start('fresh', '')
    assert not owner.state_path.exists()


def test_broken_saved_map_refused_before_session_state_changes(tmp_path, monkeypatch):
    owner = session.Session(config(tmp_path))
    monkeypatch.setattr(owner, 'conflicts', lambda: [])
    filename = tmp_path/'map.yaml'
    filename.write_text('image: missing.pgm\nresolution: 0.1\norigin: [0,0,0]\nfree_thresh: 0.2\noccupied_thresh: 0.7\n')
    with pytest.raises(ValueError, match='image missing'):
        owner.start('saved', str(filename))
    assert not owner.state_path.exists()


def test_sim_default_transport_is_loopback_and_does_not_inherit_hardware_profile(tmp_path, monkeypatch):
    filename = config(tmp_path)
    data = json.loads(filename.read_text()); data['profile'] = 'sim'
    filename.write_text(json.dumps(data))
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE', '/missing/hardware.xml')
    env = session.Session(filename).env()
    xml = Path(env['FASTRTPS_DEFAULT_PROFILES_FILE']).read_text()
    assert '<address>127.0.0.1</address>' in xml
    assert '<type>SHM</type>' in xml
    assert '<useBuiltinTransports>false</useBuiltinTransports>' in xml
    assert env['ROS_DOMAIN_ID'] == '193'


@pytest.mark.skipif(not Path('/proc/self/stat').exists(), reason='Linux process identity')
def test_partial_startup_stop_without_motion_service_cleans_only_owned_processes(tmp_path, monkeypatch):
    binary = tmp_path/'ros2'
    binary.write_text('#!/usr/bin/env python3\nimport subprocess,sys,time\n'
        'if sys.argv[1]=="run": sys.exit(1)\n'
        'subprocess.Popen([sys.executable,"-c","import time; time.sleep(60)"],start_new_session=True)\n'
        'time.sleep(60)\n')
    binary.chmod(0o755)
    monkeypatch.setenv('PATH', str(tmp_path)+os.pathsep+os.environ['PATH'])
    owner = session.Session(config(tmp_path))
    monkeypatch.setattr(owner, 'conflicts', lambda: [])
    unrelated = subprocess.Popen(['sleep', '60'])
    try:
        owner.start('fresh', '')
        end = time.monotonic()+4.
        owned = {}
        while time.monotonic()<end:
            state = owner.read()
            owned = session.descendants(state.get('launch_pid'), state.get('launch_ticks'))
            if len(owned)>=2:
                break
            time.sleep(.05)
        assert len(owned)>=2
        result = owner.stop()
        assert result['ok'] and not result['release_confirmed']
        assert not any(session.alive(pid,ticks) for pid,ticks in owned.items())
        assert unrelated.poll() is None
    finally:
        unrelated.terminate(); unrelated.wait()
        if owner.status()['running']: owner.stop()


def test_stop_cancels_supervisor_that_has_not_started(tmp_path, monkeypatch):
    owner = session.Session(config(tmp_path))
    monkeypatch.setattr(owner, 'conflicts', lambda: [])
    launched = []
    monkeypatch.setattr(session.subprocess, 'Popen', lambda *a, **k: launched.append(a))
    pending = owner.start('fresh', '')
    assert len(launched) == 1  # Supervisor not scheduled yet.
    assert owner.stop()['ok']
    assert owner.read()['phase'] == 'stopped'
    assert owner.supervise(pending['session_id'])['ok']
    assert len(launched) == 1  # No navigation process after stop.


def test_robot_shm_upgrade_matches_clean_setup_and_preserves_peers(tmp_path):
    from pm01_navigation_runtime.system.network import fastdds_xml
    from pm01_navigation_runtime.deployment.hardware_apply import merge_dds
    from pm01_navigation_runtime.containers.manager import enable_managed_local_transport
    addresses = ['127.0.0.1', '192.0.2.12']
    peers = ['192.0.2.13', '192.0.2.14']
    old = fastdds_xml(addresses, peers)
    updated = fastdds_xml(addresses, peers, shm_id='pm01_robot_shm')
    assert '<type>SHM</type>' not in old and merge_dds(old, updated) == updated
    profile = tmp_path/'fastdds-robot.xml'; profile.write_text(old)
    session = tmp_path/'session.json'; session.write_text(json.dumps(dict(profile='hardware', dds_profile=str(profile))))
    enable_managed_local_transport(session)
    assert profile.read_text() == updated
    assert merge_dds(updated, updated) == updated
    enable_managed_local_transport(session)
    assert profile.read_text() == updated
