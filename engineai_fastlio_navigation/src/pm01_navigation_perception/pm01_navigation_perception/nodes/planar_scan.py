#!/usr/bin/env python3
"""Project filtered lidar hits into a yaw-only scan for SLAM/AMCL.

Owns odom->navigation_base, while the existing estimator still owns the full
odom->base_footprint transform. It never publishes odometry or velocity.
"""
from collections import deque
import json
import math
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from pm01_navigation_perception.scan_projection import MappingScans, ranges, read_xyz, rotation, yaw


class PlanarScan(Node):
    def __init__(self):
        super().__init__('planar_scan')
        defaults = dict(input_topic='/pm01/obstacle_points', output_topic='/scan',
                        odom_topic='/odom', odom_frame='odom', base_frame='base_footprint',
                        scan_frame='navigation_base', beams=720, range_min=.6, range_max=12.,
                        frequency=5., max_data_age=.75, minimum_valid_beams=12,
                        mapping_topic='/mapping_scan', publish_mapping_scan=True,
                        keyframe_distance=.2, keyframe_angle=.15, mapping_retry_period=1.,
                        matching_status_topic='/pm01/slam_match_update',
                        require_ground_status=True, ground_status_max_age=2.5)
        self.declare_parameters('', list(defaults.items()))
        self.config = {k: self.get_parameter(k).value for k in defaults}
        c = self.config
        ranges(np.empty((0, 3)), beams=c['beams'], range_min=c['range_min'], range_max=c['range_max'])
        if (c['frequency'] <= 0 or c['max_data_age'] <= 0 or
                not 0 < c['minimum_valid_beams'] <= c['beams'] or
                len({c[k] for k in ('odom_frame', 'base_frame', 'scan_frame')}) != 3):
            raise ValueError('Invalid planar scan configuration')
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.broadcaster = TransformBroadcaster(self)
        self.pending = deque(maxlen=5)
        self.mapping_scans = MappingScans(c['keyframe_distance'], c['keyframe_angle'], c['mapping_retry_period'])
        self.ground = None
        self.last_stamp, self.reason, self.details = -math.inf, 'waiting_for_cloud', {}
        self.last_projection_stamp = -math.inf
        self.pub = self.create_publisher(LaserScan, c['output_topic'], qos_profile_sensor_data)
        self.mapping_pub = self.create_publisher(LaserScan, c['mapping_topic'], qos_profile_sensor_data)
        self.status = self.create_publisher(DiagnosticArray, '/pm01/scan_status', 10)
        self.create_subscription(PointCloud2, c['input_topic'], self.pending.append, qos_profile_sensor_data)
        self.create_subscription(Odometry, c['odom_topic'], self.on_odom, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/pm01/ground_filter_status', self.on_ground, 10)
        self.create_subscription(String, c['matching_status_topic'],
                                 lambda msg: self.mapping_scans.observe_match(msg.data), 20)
        self.create_timer(1./c['frequency'], self.update)
        self.create_timer(.2, self.publish_status)

    def on_odom(self, msg):
        c = self.config
        if msg.header.frame_id != c['odom_frame'] or msg.child_frame_id != c['base_frame']:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        try:
            angle = yaw(q.x, q.y, q.z, q.w)
            if not math.isfinite(p.x+p.y):
                return
        except ValueError:
            return
        tf = TransformStamped()
        tf.header = msg.header; tf.child_frame_id = c['scan_frame']
        tf.transform.translation.x, tf.transform.translation.y = p.x, p.y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(angle/2), math.cos(angle/2)
        self.broadcaster.sendTransform(tf)

    def on_ground(self, msg):
        good = bool(msg.status)
        for status in msg.status:
            values = {v.key: v.value for v in status.values}
            good &= status.message == 'filtering' and values.get('ground_valid') == 'true'
        self.ground = time.monotonic(), good

    def update(self):
        c = self.config
        now = self.get_clock().now().nanoseconds*1e-9
        if now-self.last_projection_stamp > c['max_data_age']:
            self.mapping_scans.request_recovery()
        if c['require_ground_status'] and (not self.ground or not self.ground[1] or
                time.monotonic()-self.ground[0] > c['ground_status_max_age']):
            self.reason = 'waiting_for_valid_ground_filter'
            self.mapping_scans.request_recovery()
            return
        for msg in reversed(self.pending):
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if stamp <= self.last_stamp or not 0 <= now-stamp <= c['max_data_age']:
                continue
            try:
                when = rclpy.time.Time.from_msg(msg.header.stamp)
                tf = self.tf.lookup_transform(c['scan_frame'], msg.header.frame_id, when).transform
                base = self.tf.lookup_transform(c['odom_frame'], c['scan_frame'], when).transform
                q, t = tf.rotation, tf.translation
                points = read_xyz(msg)@rotation(q.x, q.y, q.z, q.w).T+[t.x, t.y, t.z]
                data = ranges(points, beams=c['beams'], range_min=c['range_min'], range_max=c['range_max'])
            except Exception as error:
                self.reason = 'waiting_for_timestamped_tf'
                self.details = {'error': str(error)}
                continue
            self.last_stamp = stamp
            count = int(np.isfinite(data).sum())
            self.details = dict(valid_beams=count, total_beams=c['beams'], cloud_stamp=stamp,
                                cloud_age_sec=now-stamp, unobserved_bins='NaN')
            if count < c['minimum_valid_beams']:
                self.reason = 'insufficient_observed_geometry'
                self.mapping_scans.request_recovery()
                return
            scan = LaserScan()
            scan.header.stamp = msg.header.stamp; scan.header.frame_id = c['scan_frame']
            scan.angle_min = -math.pi; scan.angle_increment = 2*math.pi/c['beams']
            scan.angle_max = scan.angle_min+(c['beams']-1)*scan.angle_increment
            scan.range_min, scan.range_max = c['range_min'], c['range_max']
            scan.scan_time = 1./c['frequency']; scan.ranges = data.tolist()
            self.pub.publish(scan)
            self.last_projection_stamp = stamp
            q, p = base.rotation, base.translation
            selection = ''
            if c['publish_mapping_scan']:
                selection = self.mapping_scans.select(p.x, p.y, yaw(q.x, q.y, q.z, q.w),
                    msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec)
                if selection:
                    self.mapping_pub.publish(scan)
            self.details.update(mapping_selection=selection,
                                mapping_recovering=self.mapping_scans.recovering)
            self.reason = 'projecting'
            # Publish the measurement status with the scan, rather than leave
            # consumers one diagnostic timer period behind the actual data.
            self.publish_status()
            while self.pending and self.pending[0].header.stamp.sec+self.pending[0].header.stamp.nanosec*1e-9 <= stamp:
                self.pending.popleft()
            return
        if now-self.last_stamp > c['max_data_age']:
            self.reason = 'cloud_or_timestamped_tf_stale'

    def publish_status(self):
        msg = DiagnosticArray(); msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(name='pm01/planar_scan', message=self.reason,
                                  level=bytes([0 if self.reason == 'projecting' else 1]))
        status.values = [KeyValue(key=k, value=json.dumps(v)) for k, v in self.details.items()]
        msg.status = [status]; self.status.publish(msg)


def main():
    rclpy.init(); node = PlanarScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
