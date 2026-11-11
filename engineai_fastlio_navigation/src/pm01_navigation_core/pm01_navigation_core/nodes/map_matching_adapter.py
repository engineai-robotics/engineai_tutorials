#!/usr/bin/env python3
"""Adapt matched SLAM poses to map/odom and report matching observations.

SLAM Toolbox Humble 2.6.10 dates its own TF from the last keyframe. We disable
that publisher and hold each genuine matched map/odom correction between
keyframes. Sensor and matched-pose stamps are never rewritten. AMCL publishes
its own correction and uses this node only for diagnostics.
"""
import json
import math
import time
import uuid

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from pm01_navigation_core.localization.map_alignment import Pose, map_to_odom, yaw_from_quaternion
from pm01_navigation_core.localization.slam_updates import SlamUpdate, SnapshotInbox
from pm01_navigation_core.localization.map_localization_health import MotionSinceMatch


class MapMatchingAdapter(Node):
    def __init__(self):
        super().__init__('map_matching_adapter')
        defaults = dict(backend='slam', max_data_age=.75, max_position_std=.65,
                        max_yaw_std=.5, max_unmatched_travel=1., max_unmatched_turn=.7,
                        motion_resolution_m=.02, motion_resolution_rad=.01)
        self.declare_parameters('', list(defaults.items()))
        self.config = {k: self.get_parameter(k).value for k in defaults}
        if self.config['backend'] not in ('slam', 'amcl') or any(
                not math.isfinite(self.config[k]) or self.config[k] <= 0 for k in defaults if k != 'backend'):
            raise ValueError('Invalid matching diagnostic configuration')
        if (self.config['motion_resolution_m'] >= self.config['max_unmatched_travel'] or
                self.config['motion_resolution_rad'] >= self.config['max_unmatched_turn']):
            raise ValueError('Motion resolutions must be below unmatched-motion limits')
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.broadcaster = TransformBroadcaster(self) if self.config['backend'] == 'slam' else None
        self.pending_pose = None
        self.match_updates = {}
        self.backend_state = 'waiting_for_validated_backend'
        self.backend_stamp = -1
        self.latest_backend_accepted = False
        self.backend_epoch = None
        self.retired_epochs = set()
        self.map_snapshots = SnapshotInbox()
        self.map_optimization_revision = 0
        self.awaiting_map_revision = False
        self.correction = None
        self.correction_stamp = None
        self.applied_map_from_odom = None
        self.anchor_id = None
        self.anchor_pose = None
        self.epoch, self.revision = uuid.uuid4().hex, 0
        self.optimization_revision = 0
        self.scan_stamp = self.scan_received = self.pose_received = None
        self.odom = self.odom_at_match = None
        self.unmatched_motion = MotionSinceMatch(self.config['motion_resolution_m'],
                                                self.config['motion_resolution_rad'])
        self.unmatched_path = self.unmatched_rotation = 0.
        self.covariance = None
        self.map_received = False
        self.pub = self.create_publisher(DiagnosticArray, '/pm01/localization_status', 10)
        pose_topic = '/amcl_pose' if self.config['backend'] == 'amcl' else '/pm01/slam_pose'
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.on_pose, 10)
        if self.broadcaster is not None:
            self.create_subscription(String, '/pm01/slam_match_update', self.on_match_update, 100)
            self.create_subscription(String, '/pm01/slam_map_revision', self.on_map_revision,
                                     QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.2, self.tick)

    def on_scan(self, msg):
        self.scan_stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.scan_received = time.monotonic()

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        current = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        if not all(math.isfinite(v) for v in current):
            return
        if self.odom_at_match is not None:
            if self.unmatched_motion.position is None:
                self.unmatched_motion.reset(self.odom_at_match)
            self.unmatched_path, self.unmatched_rotation = self.unmatched_motion.update(current)
        self.odom = current

    def on_pose(self, msg):
        if msg.header.frame_id != 'map': return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.correction_stamp is not None and stamp <= self.correction_stamp:
            return
        if self.broadcaster is not None:
            if self.pending_pose is None or (msg.header.stamp.sec, msg.header.stamp.nanosec) > (
                    self.pending_pose.header.stamp.sec, self.pending_pose.header.stamp.nanosec):
                self.pending_pose = msg
        else:
            self.pose_received = time.monotonic()
            self.covariance = [msg.pose.covariance[k] for k in (0, 7, 35)]
            self.odom_at_match = self.odom
            self.unmatched_motion.reset(self.odom_at_match)
            self.unmatched_path = self.unmatched_rotation = 0.

    def on_match_update(self, msg):
        try:
            update = SlamUpdate.parse(msg.data)
        except (ValueError, TypeError, KeyError):
            self.backend_state = 'invalid_backend_metadata'
            return
        if update.accepted and update.anchor_id < 0:
            self.backend_state = 'invalid_backend_metadata'
            return
        if update.epoch in self.retired_epochs:
            return
        if self.backend_epoch is not None and update.epoch != self.backend_epoch:
            self.retired_epochs.add(self.backend_epoch)
            self.backend_state = 'backend_restarted_restart_session'
            self.pending_pose = None
            self.match_updates.clear()
            return
        self.backend_epoch = update.epoch
        if update.stamp_ns > self.backend_stamp:
            self.backend_stamp = update.stamp_ns
            self.latest_backend_accepted = update.accepted
            if not update.accepted:
                self.backend_state = update.reason
            elif self.backend_state != 'tracking':
                # Acceptance metadata can arrive before its pose or exact-time
                # odometry. Recovery is not complete until that pose is applied.
                # An already healthy ordinary stream keeps its held alignment.
                self.backend_state = 'waiting_for_accepted_pose'
        if update.accepted:
            self.match_updates[update.stamp_ns] = update
            while len(self.match_updates) > 128:
                self.match_updates.pop(min(self.match_updates))

    def on_map(self, msg):
        self.map_received = bool(msg.info.width and msg.info.height and any(v >= 0 for v in msg.data))
        if self.broadcaster is not None and self.map_received:
            try:
                self.adopt_map_revision(self.map_snapshots.map(
                    msg.header.stamp.sec*10**9+msg.header.stamp.nanosec, True))
            except ValueError:
                self.backend_state = 'backend_restarted_restart_session'

    def on_map_revision(self, msg):
        try:
            self.adopt_map_revision(self.map_snapshots.update(SlamUpdate.parse(msg.data)))
        except (ValueError, KeyError, TypeError):
            self.backend_state = 'invalid_map_revision'

    def adopt_map_revision(self, pair):
        if pair is not None:
            self.map_optimization_revision = pair[1].optimization_revision

    def tick(self):
        c = self.config; wall = time.monotonic(); now = self.get_clock().now().nanoseconds*1e-9
        if self.pending_pose is not None:
            msg = self.pending_pose
            try:
                update = self.match_updates.get(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)
                if update is None:
                    raise ValueError('Waiting for exact backend acceptance metadata')
                self.awaiting_map_revision = update.optimization_revision > self.map_optimization_revision
                if self.awaiting_map_revision:
                    raise ValueError('Waiting for the optimized map snapshot')
                tf = self.tf.lookup_transform('odom', 'navigation_base',
                                              rclpy.time.Time.from_msg(msg.header.stamp)).transform
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                pose_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
                if (not all(math.isfinite(v) for v in (p.x, p.y, pose_yaw)) or
                        math.hypot(p.x-update.pose[0], p.y-update.pose[1]) > 1e-6 or
                        abs(math.atan2(math.sin(pose_yaw-update.pose[2]),
                                       math.cos(pose_yaw-update.pose[2]))) > 1e-6):
                    raise ValueError('Pose and backend revision disagree')
                t, r = tf.translation, tf.rotation
                odom_pose = Pose(t.x, t.y, yaw_from_quaternion(r.x, r.y, r.z, r.w))
                if (math.hypot(odom_pose.x-update.odom[0], odom_pose.y-update.odom[1]) > 1e-4 or
                        abs(math.atan2(math.sin(odom_pose.yaw-update.odom[2]),
                                       math.cos(odom_pose.yaw-update.odom[2]))) > 1e-4):
                    raise ValueError('Backend and adapter odometry disagree')
                if not all(math.isfinite(v) and v >= -1e-9 for v in
                           [msg.pose.covariance[k] for k in (0, 7, 35)]):
                    raise ValueError('Invalid covariance')
                correction = map_to_odom(Pose(p.x, p.y, pose_yaw), odom_pose)
                # Publish one applied alignment tuple only after metadata,
                # stamped pose, optimized map and exact odometry agree. A
                # candidate arrival must not relabel old evidence with a new
                # graph anchor before its transform is actually in effect.
                self.correction = correction
                self.epoch, self.revision = update.epoch, update.revision
                self.optimization_revision = update.optimization_revision
                self.applied_map_from_odom = [correction.x, correction.y, correction.z, correction.yaw]
                self.anchor_id = update.anchor_id
                self.anchor_pose = list(update.pose)
                self.pose_received = time.monotonic()
                self.covariance = [msg.pose.covariance[k] for k in (0, 7, 35)]
                self.odom_at_match = odom_pose.x, odom_pose.y, odom_pose.yaw
                self.unmatched_motion.reset(self.odom_at_match)
                if self.odom is not None:
                    self.unmatched_path, self.unmatched_rotation = self.unmatched_motion.update(self.odom)
                else:
                    self.unmatched_path = self.unmatched_rotation = 0.
                self.correction_stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                self.pending_pose = None
                if self.latest_backend_accepted and update.stamp_ns == self.backend_stamp:
                    self.backend_state = 'tracking'
            except Exception:
                pass  # A queued TF may still arrive; never substitute latest TF.
        if self.broadcaster is not None and self.correction is not None:
            tf = TransformStamped(); tf.header.frame_id = 'map'; tf.child_frame_id = 'odom'
            # A constant alignment is valid between keyframes. The source
            # measurement time remains separately reported below.
            tf.header.stamp = (self.get_clock().now()+Duration(seconds=.3)).to_msg()
            tf.transform.translation.x, tf.transform.translation.y = self.correction.x, self.correction.y
            tf.transform.rotation.z = math.sin(self.correction.yaw/2)
            tf.transform.rotation.w = math.cos(self.correction.yaw/2)
            self.broadcaster.sendTransform(tf)
        reason, tf_age = 'tracking', None
        if self.awaiting_map_revision:
            reason = 'waiting_for_optimized_map'
        elif self.broadcaster is not None and self.backend_state != 'tracking':
            reason = self.backend_state
        elif not self.map_received:
            reason = 'waiting_for_map'
        elif self.pose_received is None:
            reason = 'waiting_for_first_map_match'
        elif self.scan_received is None or wall-self.scan_received > c['max_data_age'] or not (
                -.05 <= now-self.scan_stamp <= c['max_data_age']):
            reason = 'scan_stale'
        elif (not all(math.isfinite(v) and v >= -1e-9 for v in self.covariance) or
              (c['backend'] == 'amcl' and (max(self.covariance[:2]) > c['max_position_std']**2 or
                                         self.covariance[2] > c['max_yaw_std']**2))):
            reason = 'map_match_uncertain'
        elif self.odom is not None and self.odom_at_match is not None:
            a, b = self.odom, self.odom_at_match
            turn = abs(math.atan2(math.sin(a[2]-b[2]), math.cos(a[2]-b[2])))
            if math.hypot(a[0]-b[0], a[1]-b[1]) > c['max_unmatched_travel'] or turn > c['max_unmatched_turn']:
                reason = 'map_match_not_advancing'
        try:
            tf = self.tf.lookup_transform('map', 'odom', rclpy.time.Time())
            tf_age = now-(tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9)
            # AMCL intentionally dates its correction slightly into the future.
            if reason == 'tracking' and not -.6 <= tf_age <= c['max_data_age']:
                reason = 'map_transform_stale'
        except Exception:
            if reason == 'tracking': reason = 'waiting_for_map_transform'
        travel = turn = None
        if self.odom is not None and self.odom_at_match is not None:
            travel, turn = self.unmatched_path, self.unmatched_rotation
            if reason == 'tracking' and (travel > c['max_unmatched_travel'] or turn > c['max_unmatched_turn']):
                reason = 'map_match_not_advancing'
        temporary_rejection = self.backend_state in (
            'recovering', 'yaw_residual', 'translation_residual', 'weak_response',
            'insufficient_support', 'ambiguous_peak', 'cumulative_yaw_residual',
            'waiting_for_accepted_pose')
        held_alignment_usable = bool(self.correction is not None and temporary_rejection and
            not self.awaiting_map_revision and travel is not None and
            travel <= c['max_unmatched_travel'] and turn <= c['max_unmatched_turn'] and
            tf_age is not None and -.6 <= tf_age <= c['max_data_age'] and
            self.scan_received is not None and wall-self.scan_received <= c['max_data_age'] and
            -.05 <= now-self.scan_stamp <= c['max_data_age'])
        msg = DiagnosticArray(); msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(name='pm01/map_matching', message=reason,
                                  level=bytes([0 if reason == 'tracking' else 1]))
        values = dict(mode=c['backend'], matched=reason == 'tracking', epoch=self.epoch,
                      revision=self.revision, map_to_odom_age_sec=tf_age,
                      optimization_revision=self.optimization_revision,
                      map_optimization_revision=self.map_optimization_revision,
                      awaiting_map_revision=self.awaiting_map_revision,
                      correction_source_stamp=self.correction_stamp,
                      map_from_odom=self.applied_map_from_odom,
                      anchor_id=self.anchor_id, anchor_pose=self.anchor_pose,
                      scan_receive_age_sec=None if self.scan_received is None else wall-self.scan_received,
                      pose_receive_age_sec=None if self.pose_received is None else wall-self.pose_received,
                      covariance_xy_yaw=self.covariance, motion_output_controlled=False)
        values.update(held_alignment_usable=held_alignment_usable,
                      unmatched_travel_m=travel, unmatched_turn_rad=turn)
        values['backend_state'] = self.backend_state if self.broadcaster is not None else 'amcl'
        status.values = [KeyValue(key=k, value=json.dumps(v)) for k, v in values.items()]
        msg.status = [status]; self.pub.publish(msg)


def main():
    rclpy.init(); node = MapMatchingAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
