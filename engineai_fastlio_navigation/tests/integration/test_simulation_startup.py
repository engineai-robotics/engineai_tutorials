"""Opt-in startup regression: never enable motion or send a navigation goal."""
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('policy_delay', [0, 12], ids=['normal', 'delayed'])
def test_standing_from_spawn_and_navigation_map(tmp_path, monkeypatch, policy_delay):
    if os.environ.get('PM01_RUN_GAZEBO') != '1':
        pytest.skip('Enable after building sim on an isolated GPU host')
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from gazebo_msgs.msg import ModelStates
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Bool
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from pm01_navigation_runtime.system.network import write_profile

    monkeypatch.setenv('ROS_DOMAIN_ID', str(196 if policy_delay == 0 else 197))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '0')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE', write_profile(
        tmp_path/'loopback.xml', ['127.0.0.1'], []))
    monkeypatch.delenv('FASTDDS_DEFAULT_PROFILES_FILE', raising=False)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
    monkeypatch.setenv('GAZEBO_MASTER_URI', f'http://127.0.0.1:{port}')
    result = dict(max_tilt=0., min_height=10., maps=0, correction=False, matched=False,
                  disabled=False, startup_held=False, startup_released=False)
    rclpy.init(); node = rclpy.create_node('pm01_startup_regression')
    def imu(msg):
        q = msg.orientation
        tilt = math.acos(max(-1., min(1., 1.-2.*(q.x*q.x+q.y*q.y))))
        result['max_tilt'] = max(result['max_tilt'], tilt)
    def models(msg):
        if 'pm01_lidar_v1' in msg.name:
            height = msg.pose[msg.name.index('pm01_lidar_v1')].position.z
            result['min_height'] = min(result['min_height'], height)
    def grid(msg):
        if msg.info.width and msg.info.height and any(v >= 0 for v in msg.data):
            result['maps'] += 1
    def transforms(msg):
        if any(t.header.frame_id == 'map' and t.child_frame_id == 'odom' for t in msg.transforms):
            result['correction'] = True
    def status(msg):
        for s in msg.status:
            if s.name == 'pm01/map_matching': result['matched'] = s.message == 'tracking'
            values = {v.key: v.value for v in s.values}
            if 'output_detached' in values:
                result['disabled'] = (s.message == 'disabled' and values['output_detached'] == 'true'
                                      and values['output_publishers'] == '0')
    def held(msg):
        if msg.data:
            result['startup_held'] = True
        elif result['startup_held']:
            result['startup_released'] = True
    subscriptions = [
        node.create_subscription(Imu, '/pm01/imu', imu, qos_profile_sensor_data),
        node.create_subscription(ModelStates, '/gazebo/model_states', models, qos_profile_sensor_data),
        node.create_subscription(OccupancyGrid, '/navigation_map', grid,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(TFMessage, '/tf', transforms, 100),
        node.create_subscription(DiagnosticArray, '/pm01/localization_status', status, 10),
        node.create_subscription(DiagnosticArray, '/pm01/navigation/motion_status', status, 10),
        node.create_subscription(Bool, '/pm01/sim_startup_held', held,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]
    started = time.monotonic(); ready = None
    log = (tmp_path/'launch.log').open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'pm01_navigation_bringup', 'navigation.launch.py',
        'profile:=sim', 'mode:=fresh', 'gui:=false', 'rviz:=false',
        f'start_delay:={float(policy_delay)}'], cwd=ROOT, stdout=log,
        stderr=subprocess.STDOUT, start_new_session=True)
    try:
        while time.monotonic()-started < 180:
            rclpy.spin_once(node, timeout_sec=.05)
            assert process.poll() is None, (tmp_path/'launch.log').read_text()[-6000:]
            # Detect the former early fall, even if a subsequent reset restores standing.
            assert result['max_tilt'] < .25 and result['min_height'] > .65, result
            if ready is None and result['maps'] >= 3 and result['correction'] and result['matched']:
                ready = time.monotonic(); result['ready_after_sec'] = ready-started
            if ready is not None and time.monotonic()-ready >= 60:
                break
        assert ready is not None and result['maps'] >= 30 and result['disabled'], result
        assert result['startup_held'] and result['startup_released'], result
        assert node.count_publishers('/pm01/walking_cmd_vel') == 0
        result['passed'] = True
    finally:
        # Terminate only the process group created for this test.
        for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
            if process.poll() is not None: break
            try: os.killpg(process.pid, sig)
            except ProcessLookupError: pass
            try: process.wait(timeout)
            except subprocess.TimeoutExpired: pass
        log.close(); node.destroy_node(); rclpy.shutdown()
        (tmp_path/'result.json').write_text(json.dumps(result, indent=2)+'\n')
