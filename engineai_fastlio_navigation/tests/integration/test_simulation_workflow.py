"""Opt-in GPU integration; never starts the hardware profile."""
import os
from pathlib import Path
import socket
import subprocess
import sys
import pytest

ROOT=Path(__file__).resolve().parents[2]
SUPPORT=Path(__file__).with_name('support')


def test_fresh_and_saved_navigation_cancel_release_and_single_tf_owner(tmp_path,monkeypatch):
    if os.environ.get('PM01_RUN_GAZEBO')!='1':
        pytest.skip('Enable on the isolated GPU acceptance runner after building sim')
    monkeypatch.setenv('PM01_USE_SIM_TIME','true')
    monkeypatch.setenv('ROS_DOMAIN_ID',os.environ.get('PM01_TEST_DOMAIN','196'))
    from pm01_navigation_runtime.system.network import write_profile
    monkeypatch.setenv('ROS_LOCALHOST_ONLY','0')
    monkeypatch.setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE',write_profile(
        tmp_path/'fastdds-loopback.xml',['127.0.0.1'],[]))
    monkeypatch.delenv('FASTDDS_DEFAULT_PROFILES_FILE',raising=False)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    monkeypatch.setenv('GAZEBO_MASTER_URI','http://127.0.0.1:'+str(port))
    build=tmp_path/'probe-build'
    subprocess.run(['cmake','-S',str(SUPPORT),'-B',str(build)],check=True)
    subprocess.run(['cmake','--build',str(build),'-j2'],check=True)
    sys.path.insert(0,str(SUPPORT))
    from gazebo_workflow import run
    output=Path(os.environ.get('PM01_INTEGRATION_RESULTS',str(tmp_path/'results')))
    fresh=run(ROOT,output/'fresh',build/'tf_owner_probe')
    saved=run(ROOT,output/'saved',build/'tf_owner_probe',mode='saved',map_yaml=fresh['saved_map'])
    assert fresh['passed'] and saved['passed']
