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

"""Coordinate a complete PM01 Gazebo standing-state reset."""

import math
import threading

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty


NOMINAL_POSITIONS = [
    -0.06, 0.0, 0.0, 0.12, -0.06, 0.0,
    -0.06, 0.0, 0.0, 0.12, -0.06, 0.0,
    0.0,
    0.0, 0.15, 0.0, -0.25, 0.0,
    0.0, -0.15, 0.0, -0.25, 0.0,
    0.0,
]


class PM01StandReset(Node):
    """Expose a service that restores the complete PM01 simulation state."""

    def __init__(self):
        super().__init__('pm01_reset_stand')
        self.model_name = self.declare_parameter(
            'model_name', 'pm01_lidar_v1'
        ).value
        self.spawn_x = float(self.declare_parameter('x', 0.0).value)
        self.spawn_y = float(self.declare_parameter('y', 0.0).value)
        self.spawn_z = float(self.declare_parameter('z', 0.82).value)
        self.spawn_yaw = float(self.declare_parameter('yaw', 0.0).value)
        self.command_burst_count = max(
            1,
            int(self.declare_parameter('command_burst_count', 5).value),
        )
        command_topic = self.declare_parameter(
            'command_topic', '/pm01_sdk_pd_controller/commands'
        ).value
        controller_reset_service = self.declare_parameter(
            'controller_reset_service', '/pm01_joint_control_test/reset'
        ).value

        self.service_group = MutuallyExclusiveCallbackGroup()
        self.client_group = ReentrantCallbackGroup()
        self.pause_client = self.create_client(
            Empty, '/pause_physics', callback_group=self.client_group
        )
        self.unpause_client = self.create_client(
            Empty, '/unpause_physics', callback_group=self.client_group
        )
        self.snap_client = self.create_client(
            SetEntityState,
            '/pm01/snap_stand',
            callback_group=self.client_group,
        )
        self.controller_reset_client = self.create_client(
            Empty,
            controller_reset_service,
            callback_group=self.client_group,
        )
        self.command_publisher = self.create_publisher(
            Float64MultiArray, command_topic, 10
        )
        self.create_service(
            Empty,
            '/pm01/reset_stand',
            self.on_reset,
            callback_group=self.service_group,
        )
        self.get_logger().info(
            'Call /pm01/reset_stand to restore the complete PM01 state'
        )

    def call_service(self, client, request, timeout=1.0):
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=timeout):
                return None
        future = client.call_async(request)
        finished = threading.Event()
        future.add_done_callback(lambda _: finished.set())
        if not finished.wait(timeout):
            return None
        if future.exception() is not None:
            return None
        return future.result()

    def initial_pose(self):
        half_yaw = self.spawn_yaw / 2.0
        return Pose(
            position=Point(
                x=self.spawn_x, y=self.spawn_y, z=self.spawn_z
            ),
            orientation=Quaternion(
                x=0.0,
                y=0.0,
                z=math.sin(half_yaw),
                w=math.cos(half_yaw),
            ),
        )

    def publish_nominal_commands(self):
        message = Float64MultiArray(data=NOMINAL_POSITIONS)
        for _ in range(self.command_burst_count):
            self.command_publisher.publish(message)

    def on_reset(self, request, response):
        del request
        self.get_logger().info('Resetting PM01 to the SDK standing state')

        if self.call_service(self.pause_client, Empty.Request(), 1.0) is None:
            self.get_logger().error('Failed to pause Gazebo physics')
            return response

        # Optional: reset whichever high-level controller currently owns the
        # command stream. Absence is valid when only the simulator is running.
        controller_reset_response = self.call_service(
            self.controller_reset_client, Empty.Request(), timeout=0.5
        )
        if controller_reset_response is None:
            self.get_logger().warning(
                'High-level controller reset service was unavailable'
            )
        self.publish_nominal_commands()

        state = EntityState(
            name=self.model_name,
            pose=self.initial_pose(),
            twist=Twist(linear=Vector3(), angular=Vector3()),
            reference_frame='world',
        )
        snap_request = SetEntityState.Request()
        snap_request.state = state
        snap_response = self.call_service(
            self.snap_client, snap_request, timeout=2.0
        )
        if snap_response is None or not snap_response.success:
            self.get_logger().error(
                'Gazebo failed to restore PM01 joints and base state'
            )
        else:
            # Refresh ros2_control's command queue after Gazebo has updated
            # every joint and link, before physics is allowed to run again.
            self.publish_nominal_commands()

        if self.call_service(
            self.unpause_client, Empty.Request(), 1.0
        ) is None:
            self.get_logger().error('Failed to unpause Gazebo physics')
        else:
            self.get_logger().info('PM01 complete state reset finished')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PM01StandReset()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
