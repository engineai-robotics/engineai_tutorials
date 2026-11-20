#!/usr/bin/env python3
"""Publish one automatically calibrated ground model for both lidar consumers."""
from collections import deque
import json
import math
import time
import uuid

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from pm01_navigation_core.nodes.stamped_clouds import StampedClouds

from pm01_navigation_core.ground.ground_model import GroundTracker, height_at
from pm01_navigation_core.localization.map_localization_health import (
    local_observation_support, observation_health_current)
from pm01_navigation_core.pointcloud import points_array
from pm01_navigation_core.geometry import rotation


class AutomaticGround(Node):
    def __init__(self):
        super().__init__('automatic_ground')
        defaults = {'input_topic': '/livox/points', 'output_topic': '/pm01/ground_plane_status',
                    'reference_frame': 'odom', 'base_frame': 'base_footprint',
                    'frequency': 5., 'max_data_age': 1., 'max_range': 12.,
                    'min_sensor_height': .7, 'max_sensor_height': 1.5,
                    'calibration_samples': 3, 'calibration_span': .4,
                    'max_ground_height_rate': .02, 'max_anchor_offset': .08}
        self.declare_parameters('', list(defaults.items()))
        self.config = {k: self.get_parameter(k).value for k in defaults}
        if (not 0 < self.config['max_data_age'] <= 1. or self.config['frequency'] <= 0 or
                self.config['max_range'] <= .6):
            raise ValueError('Invalid automatic ground frequency or age')
        self.tracker = GroundTracker(min_height=self.config['min_sensor_height'],
            max_height=self.config['max_sensor_height'], samples=self.config['calibration_samples'],
            calibration_span=self.config['calibration_span'], max_age=self.config['max_data_age'],
            max_height_rate=self.config['max_ground_height_rate'],
            max_anchor_offset=self.config['max_anchor_offset'])
        self.pending = StampedClouds(self, self.on_input, 8)
        self.tf = self.pending.tf
        self.health = None
        self.health_history = deque(maxlen=32)
        self.lio_observation_support = 'unsupported'
        self.epoch = uuid.uuid4().hex
        self.reason, self.last_processed = 'waiting_for_lio', -math.inf
        self.details, self.sensor = {}, None
        self.pub = self.create_publisher(DiagnosticArray, self.config['output_topic'], 10)
        self.create_subscription(PointCloud2, self.config['input_topic'], self.pending.append,
                                 qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/lio/status', self.on_health, qos_profile_sensor_data)
        self.create_timer(1./self.config['frequency'], self.update)

    def on_health(self, msg):
        statuses = [s for s in msg.status if s.name == 'fast_lio/geometry']
        supported = len(statuses) == 1
        self.lio_observation_support = 'unsupported'
        for status in statuses:
            level = status.level[0] if isinstance(status.level, bytes) else int(status.level)
            self.lio_observation_support = local_observation_support(
                level, status.message, {v.key: v.value for v in status.values})
            supported &= self.lio_observation_support != 'unsupported'
        stamp_ns = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
        self.health = time.monotonic(), stamp_ns, supported
        self.health_history.append(self.health)
        self.pending.wake()

    def on_input(self):
        self.update(heartbeat=False)

    def update(self, heartbeat=True):
        began = time.monotonic()
        now_ns = self.get_clock().now().nanoseconds
        now = now_ns * 1e-9
        age = self.config['max_data_age']
        health=self.health
        if health is not None and health[2] and health[1] > now_ns + 50_000_000:
            health=next((h for h in reversed(self.health_history) if h[1] <= now_ns + 50_000_000),None)
        if not observation_health_current(health, now_ns, began, age):
            changed = self.reason != 'local_observation_unhealthy_or_stale'
            self.tracker.suspend('local_observation_unhealthy_or_stale')
            self.reason = self.tracker.reason
            if heartbeat or changed:
                self.publish(now, False)
            return
        chosen = self.pending.select(now_ns)
        if chosen is None:
            if heartbeat:
                if now-self.last_processed > age:
                    self.reason = self.pending.state
                self.publish(now, now-self.last_processed <= age)
            return
        msg, stamp, sensor, root = chosen
        try:
            t = sensor.translation
            self.sensor = np.array([t.x, t.y, t.z])
            points = points_array(msg)@rotation(sensor.rotation).T+self.sensor
            distance = np.linalg.norm(points-self.sensor, axis=1)
            points = points[np.isfinite(points).all(axis=1) & (distance >= .6) &
                            (distance <= self.config['max_range']) &
                            (np.linalg.norm(points[:, :2]-[root.x, root.y], axis=1) > .45)]
            self.tracker.update(points, [root.x, root.y], self.sensor, stamp)
            self.last_processed = stamp
            self.reason = self.tracker.reason
            self.pending.consumed(msg, now_ns)
            self.details = {'input_points': len(points), 'source_stamp': stamp,
                            'wall_seconds': time.monotonic()-began}
            self.publish(now, True)
        except (ValueError, MemoryError, OverflowError) as error:
            self.tracker.suspend('invalid_cloud')
            self.reason = 'invalid_cloud'
            self.get_logger().error(str(error))
            self.publish(now, False)

    def publish(self, now, usable):
        plane = self.tracker.current(now) if usable else None
        msg = DiagnosticArray(); msg.header.frame_id = self.config['reference_frame']
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(); status.name = 'pm01/automatic_ground'
        status.hardware_id = 'localized_lidar'
        status.level = bytes([0 if plane is not None else 1])
        status.message = self.reason
        values = dict(self.details, **self.pending.diagnostics(), mode='automatic', epoch=self.epoch,
            ground_valid=plane is not None, ground_rejection=self.tracker.rejection,
            observation_usable=plane is not None,
            local_observation_usable=plane is not None,
            source_age_sec=(None if 'source_stamp' not in self.details else
                            now-self.details['source_stamp']),
            lio_source_stamp=None if self.health is None else self.health[1] * 1e-9,
            lio_observation_support=self.lio_observation_support,
            calibration_samples=len(self.tracker.candidates),
            model_id=self.epoch+':'+str(self.tracker.revision),
            ground_plane=None if plane is None else plane.summary(),
            plane_stamp=None if plane is None else self.tracker.plane_stamp,
            sensor_height=(None if plane is None or self.sensor is None else
                           float(self.sensor[2]-height_at(plane, self.sensor[:2]))),
            configured_ground_z_used=False)
        status.values = [KeyValue(key=k, value=json.dumps(v)) for k, v in values.items()]
        msg.status = [status]; self.pub.publish(msg)


def main():
    rclpy.init(); node = AutomaticGround()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
