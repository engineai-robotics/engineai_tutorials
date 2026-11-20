"""Isolated ROS test: only a named fake consumer can satisfy bridge readiness."""
import os
import subprocess
import time
import uuid

import pytest



@pytest.mark.skipif(os.environ.get('PM01_RUN_ADAPTER_TESTS') != '1',
                    reason='requires isolated ROS domain and built hardware adapter')
def test_recorder_is_not_consumer_and_pre_ready_command_is_discarded():
    rclpy = pytest.importorskip('rclpy')
    pytest.importorskip('interface_protocol.msg')
    from ament_index_python.packages import get_package_prefix
    from pathlib import Path
    from geometry_msgs.msg import Twist
    from interface_protocol.msg import BodyVelCmd
    from std_srvs.srv import Trigger
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0', '69')
    exe = Path(get_package_prefix('pm01_navigation_hardware'))/'lib/pm01_navigation_hardware/cmd_vel_bridge'
    service = '/pm01/test_ready_'+uuid.uuid4().hex
    rclpy.init()
    observer = rclpy.create_node('test_recorder')
    seen = []
    observer.create_subscription(BodyVelCmd, '/motion/body_vel_cmd',
                                 lambda m: seen.append(m.linear_velocity[0]), 10)
    pub = observer.create_publisher(Twist, '/cmd_vel', 10)
    client = observer.create_client(Trigger, service)
    worker = subprocess.Popen([str(exe), '--ros-args', '-p', 'ready_service:='+service],
                              stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    consumer = None

    def spin(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(observer, timeout_sec=.02)

    def ready():
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic()+3.
        while not future.done() and time.monotonic()<deadline:
            rclpy.spin_once(observer, timeout_sec=.02)
        assert future.done()
        return future.result()

    try:
        assert client.wait_for_service(timeout_sec=5.)
        spin(.3)
        assert seen and not ready().success  # Recorder discovery cannot arm hardware.
        command = Twist(); command.linear.x = .4
        for _ in range(5): pub.publish(command); spin(.02)
        consumer = rclpy.create_node('locomotion_interface_node')
        consumer.create_subscription(BodyVelCmd, '/motion/body_vel_cmd', lambda m: None, 10)
        deadline=time.monotonic()+4.
        while not ready().success:
            assert time.monotonic()<deadline
            spin(.05)
        seen.clear(); spin(.1)
        assert seen and all(v==0. for v in seen)
        pub.publish(command); spin(.1)
        assert any(abs(v-.4)<1e-6 for v in seen)
    finally:
        worker.terminate(); worker.wait(timeout=3.)
        if consumer is not None: consumer.destroy_node()
        observer.destroy_node(); rclpy.shutdown()
