#!/usr/bin/env python3
# Copyright 2026 EngineAI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers
from rclpy.qos import qos_profile_sensor_data


# SDK default_joint_q order: J00 through J23.
NOMINAL_POSITIONS = [
    -0.06, 0.0, 0.0, 0.12, -0.06, 0.0,
    -0.06, 0.0, 0.0, 0.12, -0.06, 0.0,
    0.0,
    0.0, 0.15, 0.0, -0.25, 0.0,
    0.0, -0.15, 0.0, -0.25, 0.0,
    0.0,
]


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('pm01_initial_pose')
        command_topic = self.declare_parameter(
            'command_topic', '/pm01_sdk_pd_controller/commands'
        ).value
        self.duration = float(self.declare_parameter('duration', 8.0).value)
        publish_rate = float(self.declare_parameter('publish_rate', 100.0).value)
        self.finished = False
        self.deadline = time.monotonic() + 90.0
        self.started_at = None
        self.sim_time = None
        self.controllers_active = False
        self.controller_query = None
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_timer(0.5, self.check_controllers)
        self.publisher = self.create_publisher(
            Float64MultiArray, command_topic, 10
        )
        self.timer = self.create_timer(
            1.0 / max(publish_rate, 1.0), self.publish_pose
        )

    def on_joints(self, message):
        self.sim_time = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def check_controllers(self):
        if self.controller_query is not None:
            if not self.controller_query.done():
                return
            response = self.controller_query.result()
            active = {c.name for c in response.controller if c.state == 'active'} if response else set()
            self.controllers_active = {'joint_state_broadcaster', 'pm01_sdk_pd_controller'} <= active
            self.controller_query = None
        if self.controllers.service_is_ready():
            self.controller_query = self.controllers.call_async(ListControllers.Request())

    def publish_pose(self):
        self.publisher.publish(Float64MultiArray(data=NOMINAL_POSITIONS))
        if time.monotonic() > self.deadline:
            raise RuntimeError('Initial pose timed out waiting for active controllers and simulation time')
        if not self.controllers_active or self.sim_time is None:
            self.started_at = None
            return
        if self.started_at is None or self.sim_time < self.started_at:
            self.started_at = self.sim_time
        if self.sim_time - self.started_at >= self.duration:
            self.get_logger().info('Initial pose command published')
            self.timer.cancel()
            self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
