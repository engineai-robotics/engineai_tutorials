#!/usr/bin/env python3
"""Ground-relative obstacle marks; preserve the original lidar frame/origin."""
from collections import deque
import json
import math
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener

from pm01_navigation_core.ground.flat_ground import estimate_ground, near_ground_noise
from pm01_navigation_core.ground.ground_model import GroundInbox, transform_plane
from pm01_navigation_core.pointcloud import points_array
from pm01_navigation_core.geometry import rotation


class NavigationObstacleCloud(Node):
    def __init__(self):
        super().__init__('navigation_obstacle_cloud')
        defaults = {'input_topic': '/livox/points', 'output_topic': '/pm01/obstacle_points',
            'reference_frame': 'odom', 'ground_z': -.81, 'obstacle_height': .025,
            'max_height': 1.8, 'max_range': 12., 'frequency': 5., 'max_data_age': .75,
            'near_ground_filter': False, 'near_ground_band_max': .06,
            'near_ground_support_radius': .20, 'use_shared_ground': False,
            'ground_model_topic': '/pm01/ground_plane_status'}
        self.declare_parameters('', list(defaults.items()))
        self.config = {k: self.get_parameter(k).value for k in defaults}
        if self.config['frequency'] <= 0 or not 0 < self.config['obstacle_height'] < self.config['max_height']:
            raise ValueError('Invalid obstacle filtering limits')
        if self.config['near_ground_filter'] and (
                not np.isfinite([self.config['near_ground_band_max'],
                                 self.config['near_ground_support_radius']]).all() or
                not self.config['obstacle_height'] < self.config['near_ground_band_max'] <= self.config['max_height'] or
                self.config['near_ground_support_radius'] <= 0):
            raise ValueError('Invalid near-ground filter configuration')
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.ground = GroundInbox(self.config['max_data_age'])
        self.pending = deque(maxlen=10)
        self.last_stamp, self.reason, self.details = -math.inf, 'waiting_for_cloud', {}
        self.pub = self.create_publisher(PointCloud2, self.config['output_topic'], 2)
        self.status = self.create_publisher(DiagnosticArray, '/pm01/ground_filter_status', 10)
        self.create_subscription(PointCloud2, self.config['input_topic'], self.pending.append, qos_profile_sensor_data)
        if self.config['use_shared_ground']:
            self.create_subscription(DiagnosticArray, self.config['ground_model_topic'],
                                     self.on_ground, qos_profile_sensor_data)
        self.create_timer(1/self.config['frequency'], self.update)
        self.create_timer(1., self.diagnostic)

    def on_ground(self, msg):
        if not msg.status:
            return
        status = msg.status[0]
        level = status.level[0] if isinstance(status.level, bytes) else int(status.level)
        self.ground.receive(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                            msg.header.frame_id, level, {v.key: v.value for v in status.values})

    def update(self):
        now = self.get_clock().now().nanoseconds*1e-9
        chosen = None
        for msg in reversed(self.pending):
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if stamp <= self.last_stamp or not 0 <= now-stamp <= self.config['max_data_age']:
                continue
            try:
                when = rclpy.time.Time.from_msg(msg.header.stamp)
                tf = self.tf.lookup_transform(self.config['reference_frame'], msg.header.frame_id, when).transform
                root = self.tf.lookup_transform(self.config['reference_frame'], 'base_footprint', when).transform.translation
                chosen = msg, stamp, tf, root
                break
            except Exception:
                continue
        if chosen is None:
            # Waiting for the next timestamped TF is normal between outputs.
            # Report unavailable only once the last accepted cloud is stale.
            if now-self.last_stamp > self.config['max_data_age']:
                self.reason = 'cloud_or_timestamped_tf_unavailable'
                self.details.update(ground_valid=False, observation_usable=False)
            return
        msg, stamp, tf, root = chosen
        began = time.monotonic()
        raw = points_array(msg)
        offset = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
        points = raw@rotation(tf.rotation).T+offset
        ranges = np.linalg.norm(points-offset, axis=1)
        valid = (np.isfinite(points).all(axis=1) & (ranges >= .6) &
            (ranges <= self.config['max_range']) &
            (np.linalg.norm(points[:, :2]-[root.x, root.y], axis=1) > .45))
        raw, points = raw[valid], points[valid]
        model_details = {}
        if self.config['use_shared_ground']:
            model = self.ground.get(stamp, now)
            plane = None
            if model is not None:
                plane, model_stamp, frame, model_id = model
                try:
                    if frame != self.config['reference_frame']:
                        t = self.tf.lookup_transform(self.config['reference_frame'], frame,
                                                    rclpy.time.Time.from_msg(msg.header.stamp)).transform
                        plane = transform_plane(plane, rotation(t.rotation),
                            [t.translation.x, t.translation.y, t.translation.z])
                    model_details = {'ground_model_id': model_id, 'ground_model_stamp': model_stamp}
                except Exception:
                    plane = None
        else:
            plane = estimate_ground(points, [root.x, root.y], self.config['ground_z'])
        self.details = {'input_points': len(points), 'stamp': stamp, 'source_stamp': stamp,
            'ground_truth_used': False, 'obstacle_height_m': self.config['obstacle_height'],
            'ground_source': 'shared_automatic' if self.config['use_shared_ground'] else 'manual_reference',
            **model_details}
        if plane is None:
            # Missing evidence is neither an obstacle nor observed free space.
            # The navigation readiness gate pauses motion while sensing resumes.
            self.reason = 'ground_unavailable'
            self.details.update(ground_valid=False, observation_usable=False,
                                obstacle_points=None)
            self.diagnostic()
            return
        height = plane.heights(points)
        keep = (height >= self.config['obstacle_height']) & (height <= self.config['max_height'])
        noise = (near_ground_noise(points, plane, self.config['obstacle_height'],
            band_max=self.config['near_ground_band_max'],
            support_radius=self.config['near_ground_support_radius'])
            if self.config['near_ground_filter'] else np.zeros(len(points), bool))
        keep &= ~noise
        self.reason = 'filtering'
        self.details.update(ground_valid=True, observation_usable=True,
            ground_plane=plane.summary(),
            near_ground_filter_enabled=self.config['near_ground_filter'],
            near_ground_noise_rejected=int(noise.sum()))
        result = PointCloud2()
        result.header = msg.header
        result.height, result.width = 1, int(keep.sum())
        result.fields = [PointField(name=k, offset=i*4, datatype=7, count=1) for i, k in enumerate(('x', 'y', 'z'))]
        result.point_step, result.row_step = 12, int(keep.sum())*12
        result.is_dense, result.is_bigendian = True, False
        result.data = np.asarray(raw[keep], dtype='<f4').tobytes()
        self.pub.publish(result)
        self.last_stamp = stamp; self.pending.clear()
        self.details.update(obstacle_points=int(keep.sum()), wall_seconds=time.monotonic()-began)

    def diagnostic(self):
        msg = DiagnosticArray(); msg.header.stamp = self.get_clock().now().to_msg()
        now = self.get_clock().now().nanoseconds*1e-9
        self.details['source_age_sec'] = (None if 'source_stamp' not in self.details else
                                         now-self.details['source_stamp'])
        status = DiagnosticStatus(); status.name = 'pm01/navigation_ground_filter'
        status.hardware_id = 'lidar'; status.level = bytes([0 if self.reason == 'filtering' else 1])
        status.message = self.reason
        status.values = [KeyValue(key=k, value=json.dumps(v)) for k, v in self.details.items()]
        msg.status = [status]; self.status.publish(msg)


def main():
    rclpy.init(); node = NavigationObstacleCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
