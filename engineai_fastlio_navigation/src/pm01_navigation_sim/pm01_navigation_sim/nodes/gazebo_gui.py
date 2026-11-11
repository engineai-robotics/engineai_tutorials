#!/usr/bin/env python3
"""Open the Gazebo client after the world and robot IMU are publishing.

Only the GUI process is managed here. A GUI failure never resets the world,
robot, localization or navigation action. Closing the window normally does
not reopen it; unexpected exits have a bounded retry budget.
"""
import json
import math
import signal
import subprocess
import time

import rclpy
from sensor_msgs.msg import Imu
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


class GazeboGui(Node):
    def __init__(self):
        super().__init__('pm01_gazebo_gui')
        self.timeout = self.declare_parameter('ready_timeout_sec', 45.0).value
        self.max_restarts = self.declare_parameter('max_restarts', 1).value
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('ready_timeout_sec must be finite and positive')
        if not isinstance(self.max_restarts, int) or not 0 <= self.max_restarts <= 3:
            raise ValueError('max_restarts must be an integer from zero to three')
        self.clock_stamp = self.imu_stamp = None
        self.last_clock_progress = self.last_imu = 0.0
        self.first_imu = None
        self.imu_count = 0
        self.child = None
        self.create_subscription(Clock, '/clock', self.clock_update, qos_profile_sensor_data)
        self.create_subscription(Imu, '/livox/imu', self.imu_update, qos_profile_sensor_data)

    def event(self, state, **values):
        self.get_logger().info(json.dumps(dict(gui_state=state, **values)))

    def clock_update(self, msg):
        stamp = msg.clock.sec + msg.clock.nanosec * 1e-9
        if stamp != self.clock_stamp:
            self.last_clock_progress = time.monotonic()
        self.clock_stamp = stamp

    def imu_update(self, msg):
        a, w = msg.linear_acceleration, msg.angular_velocity
        if not all(math.isfinite(v) for v in [a.x, a.y, a.z, w.x, w.y, w.z]):
            return
        self.imu_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_imu = time.monotonic()
        if self.first_imu is None:
            self.first_imu = self.last_imu
        self.imu_count += 1

    def wait_ready(self):
        start = time.monotonic()
        self.event('waiting_for_backend', timeout_sec=self.timeout)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            clock_fresh = self.clock_stamp is not None and now-self.last_clock_progress < 1.0
            imu_fresh = self.imu_count >= 3 and now-self.last_imu < 1.0
            synchronized = (self.imu_stamp is not None and self.clock_stamp is not None and
                            abs(self.clock_stamp-self.imu_stamp) < 0.5)
            settled = self.first_imu is not None and now-self.first_imu >= 1.0
            if clock_fresh and imu_fresh and synchronized and settled:
                # Do not wait for GPU LiDAR/LIO: the GUI must remain available
                # to inspect a sensor startup failure. This is not a Nav2 gate.
                self.event('scene_data_ready', elapsed_sec=now-start)
                return True
            if now-start >= self.timeout:
                if not clock_fresh:
                    raise RuntimeError('No advancing simulation clock; GUI not started')
                self.get_logger().warning(
                    'Simulation clock is advancing but robot IMU data is not ready; '
                    'opening the GUI for inspection. Navigation readiness is not confirmed.')
                self.event('inspection_only', elapsed_sec=now-start)
                return True
        return False

    def run_gui(self):
        if not self.wait_ready():
            return 0
        for attempt in range(self.max_restarts+1):
            if not rclpy.ok():
                return 0
            self.child = subprocess.Popen(
                ['gzclient', '--gui-client-plugin=libgazebo_ros_eol_gui.so'],
                stdin=subprocess.DEVNULL)
            self.event('running', pid=self.child.pid, attempt=attempt+1)
            while rclpy.ok() and self.child.poll() is None:
                rclpy.spin_once(self, timeout_sec=0.1)
            if not rclpy.ok():
                return 0
            code = self.child.returncode
            self.child = None
            if code in (0, -signal.SIGINT):
                self.event('closed', return_code=code)
                return 0
            if attempt == self.max_restarts:
                self.event('failed', return_code=code, restart_budget=self.max_restarts)
                self.get_logger().error(
                    'Gazebo GUI retry budget exhausted. Simulation and navigation '
                    'remain running; inspect the GUI log and cancel any active goal separately.')
                return 1
            self.event('restarting', return_code=code, next_attempt=attempt+2)
            deadline = time.monotonic()+2.0
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
        return 1

    def stop_child(self):
        if self.child is None or self.child.poll() is not None:
            return
        for sig, timeout in [(signal.SIGINT, 2), (signal.SIGTERM, 1), (signal.SIGKILL, 1)]:
            try:
                self.child.send_signal(sig)
                self.child.wait(timeout=timeout)
                return
            except ProcessLookupError:
                return
            except subprocess.TimeoutExpired:
                pass


def main():
    rclpy.init()
    node = GazeboGui()
    result = 0
    try:
        result = node.run_gui()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Launch and the terminal can both deliver SIGINT to this process
        # group. Keep the second signal from interrupting bounded child reaping.
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.stop_child()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            signal.signal(signal.SIGINT, previous)
    raise SystemExit(result)


if __name__ == '__main__':
    main()
