#!/usr/bin/env python3
"""Build a fresh session map or update an explicitly supplied prior using lidar."""
from array import array
from collections import deque
from copy import deepcopy
import datetime
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from pm01_navigation_core.mapping.online_occupancy import empty_grid
from pm01_navigation_core.mapping.continuous_occupancy import ContinuousOccupancy
from pm01_navigation_core.localization.map_localization_health import (
    local_observation_support, localization_support, observation_health_current)
from pm01_navigation_core.pointcloud import points_array
from pm01_navigation_core.geometry import rotation
from pm01_navigation_core.ground.ground_model import GroundInbox, height_at, transform_plane
from pm01_navigation_core.localization.slam_updates import SlamUpdate, SnapshotInbox


class OnlineNavigationMap(Node):
    def __init__(self):
        super().__init__('online_navigation_map')
        defaults = {'enabled': True, 'seed_topic': '/map', 'output_topic': '/navigation_map',
                    'cloud_topic': '/livox/points', 'ground_z': 0.0,
                    'obstacle_height': 0.12, 'max_height': 1.8, 'max_range': 12.0,
                    'update_frequency': 2.0, 'max_data_age': 0.75,
                    'height_resolution': 0.10, 'ray_clearance': 0.15, 'adaptive_ground': False,
                    'near_ground_filter': False, 'near_ground_band_max': .06,
                    'near_ground_support_radius': .20,
                    'min_sensor_height': 0.70, 'max_sensor_height': 1.50,
                    'save_root': str(Path.home()/'.local/share/pm01_navigation/maps'),
                    'fresh_map': False, 'prior_alignment_mode': 'map_matching', 'dynamic_seed': False,
                    'map_resolution': 0.1, 'initial_map_size_m': 30.,
                    'map_growth_step_m': 5., 'max_map_cells': 1_000_000,
                    'use_shared_ground': False, 'ground_model_topic': '/pm01/ground_plane_status'}
        self.declare_parameters('', list(defaults.items()))
        self.config = {k: self.get_parameter(k).value for k in defaults}
        if self.config['prior_alignment_mode'] != 'map_matching' or self.config['fresh_map']:
            raise ValueError('Use the SLAM/AMCL navigation launch for mapping and localization')
        if (self.config['update_frequency'] <= 0 or self.config['max_data_age'] <= 0 or
                self.config['seed_topic'] == self.config['output_topic']):
            raise ValueError('Invalid update frequency, age or map topics')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(OccupancyGrid, self.config['output_topic'], qos)
        self.status_publisher = self.create_publisher(
            DiagnosticArray, '/pm01/map_update_status', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ground = GroundInbox(self.config['max_data_age'])
        self.completion = None
        self.seed = None
        self.seed_hash = None
        self.slam_snapshots = SnapshotInbox()
        self.slam_snapshot = None
        self.pending_snapshot = None
        self.graph_anchors = {}
        self.applied_alignments = deque(maxlen=64)
        self.pending = deque(maxlen=8)
        self.health = {}
        self.health_modes = {}
        self.manual_revision = None
        self.alignment_epoch = None
        self.alignment_optimization_revision = None
        self.lio_epoch = None
        self.lio_publisher_gid = None
        self.last_lio_publisher_check = -math.inf
        self.last_processed = -math.inf
        self.last_correction = None
        self.correction_resets = 0
        self.latched_error = None
        self.reason = 'waiting_for_seed'
        self.last_observation = {}
        self.last_successful_update_stamp = None
        self.last_successful_update_monotonic = None
        self.last_successful_observation_stamp = None
        self.last_successful_snapshot = None
        self.rejected = {}
        self.create_subscription(OccupancyGrid, self.config['seed_topic'], self.on_seed, qos)
        if self.config['dynamic_seed']:
            self.create_subscription(String, '/pm01/slam_map_revision', self.on_map_revision,
                                     QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(String, '/pm01/slam_graph_anchors', self.on_graph_anchors, qos)
        self.create_subscription(PointCloud2, self.config['cloud_topic'], self.pending.append,
                                 qos_profile_sensor_data)
        if self.config['use_shared_ground']:
            self.create_subscription(DiagnosticArray, self.config['ground_model_topic'],
                                     self.on_ground, qos_profile_sensor_data)
        self.health_topics = ('/lio/status', '/pm01/localization_status')
        for topic in self.health_topics:
            self.create_subscription(DiagnosticArray, topic,
                                     lambda msg, topic=topic: self.diagnostics(topic, msg),
                                     qos_profile_sensor_data)
        self.create_timer(1/self.config['update_frequency'], self.update)
        self.create_timer(1.0, self.publish_status)
        self.create_service(Trigger, '/pm01/save_navigation_map', self.save)

    def on_seed(self, msg):
        if not self.config['dynamic_seed']:
            self.seed_map(msg)
            return
        try:
            self.adopt_snapshot(self.slam_snapshots.map(
                msg.header.stamp.sec*10**9+msg.header.stamp.nanosec, msg))
        except ValueError as exc:
            self.latched_error = str(exc)

    def on_map_revision(self, msg):
        try:
            self.adopt_snapshot(self.slam_snapshots.update(SlamUpdate.parse(msg.data)))
        except (ValueError, TypeError, KeyError) as exc:
            self.latched_error = str(exc)

    def on_graph_anchors(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('schema') != 1 or data.get('backend') != 'pm01-slam-2.6.10':
                raise ValueError('Unsupported graph anchor metadata')
            key = (str(data['epoch']), int(data['optimization_revision']))
            anchors = {}
            for item in data['anchors']:
                anchor, pose = int(item['id']), tuple(item['pose'])
                if anchor < 0 or anchor in anchors or len(pose) != 3 or not np.isfinite(pose).all():
                    raise ValueError('Invalid graph anchor pose')
                anchors[anchor] = pose
            self.graph_anchors[key] = anchors
            while len(self.graph_anchors) > 4:
                del self.graph_anchors[next(iter(self.graph_anchors))]
            if self.pending_snapshot is not None:
                self.adopt_snapshot(self.pending_snapshot)
        except (ValueError, TypeError, KeyError) as exc:
            self.latched_error = str(exc)

    def adopt_snapshot(self, snapshot):
        if snapshot is None:
            return
        msg, update = snapshot
        changed = (self.slam_snapshot is not None and
                   update.optimization_revision != self.slam_snapshot.optimization_revision)
        key = (update.epoch, update.optimization_revision)
        if update.optimization_revision and key not in self.graph_anchors:
            self.pending_snapshot = snapshot
            self.reason = 'waiting_for_graph_anchors'
            return
        optimized = self.graph_anchors.get(key) if changed else None
        self.seed_map(msg, update.correction, optimized_anchors=optimized)
        self.slam_snapshot = update
        if update.anchor_id >= 0:
            self.completion.register_anchor(update.anchor_id, update.anchor_pose)
        self.pending_snapshot = None

    def seed_map(self, msg, correction=None, *, optimized_anchors=None):
        try:
            if (msg.header.frame_id != 'map' or msg.info.width*msg.info.height != len(msg.data)
                    or not msg.info.width or not msg.info.height):
                raise ValueError('Expected a complete seed grid in map frame')
            q = msg.info.origin.orientation
            if abs(q.x) > 1e-6 or abs(q.y) > 1e-6:
                raise ValueError('Seed map must have a horizontal origin')
            yaw = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
            digest = hashlib.sha256(np.asarray(msg.data, dtype=np.int8).tobytes()).hexdigest()
            if self.seed is not None:
                if digest != self.seed_hash or msg.info != self.seed.info or correction is not None:
                    if not self.config['dynamic_seed']:
                        self.latched_error = 'seed_changed_restart_required'
                    elif msg.info.width*msg.info.height > self.config['max_map_cells']:
                        self.latched_error = 'map_capacity_exceeded'
                    else:
                        self.completion.rebase(np.asarray(msg.data).reshape(msg.info.height, msg.info.width),
                            msg.info.resolution, (msg.info.origin.position.x, msg.info.origin.position.y, yaw),
                            msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                            map_from_odom=correction, optimized_anchors=optimized_anchors)
                        self.seed, self.seed_hash = deepcopy(msg), digest
                        self.publish_map()
                return
            self.seed, self.seed_hash = deepcopy(msg), digest
            self.reset_completion()
            if correction is not None:
                self.completion.project(correction)
            self.pending.clear()
            self.publish_map()
            self.get_logger().info('Seed loaded; updating '+self.config['output_topic'])
        except Exception as exc:
            self.latched_error = 'invalid_seed'
            self.get_logger().error(str(exc))

    def reset_completion(self):
        self.last_observation = {}
        self.last_successful_update_stamp = None
        self.last_successful_update_monotonic = None
        self.last_successful_observation_stamp = None
        self.last_successful_snapshot = None
        msg = self.seed
        q = msg.info.origin.orientation
        yaw = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
        self.completion = ContinuousOccupancy(
            np.array(msg.data).reshape(msg.info.height, msg.info.width),
            msg.info.resolution,
            (msg.info.origin.position.x, msg.info.origin.position.y, yaw),
            initial_size=self.config['initial_map_size_m'], max_cells=self.config['max_map_cells'],
            growth_step=self.config['map_growth_step_m'], require_anchors=self.config['dynamic_seed'],
            **{k: self.config[k] for k in ('ground_z', 'obstacle_height', 'max_height', 'max_range',
                'height_resolution', 'ray_clearance', 'adaptive_ground',
                'near_ground_filter', 'near_ground_band_max', 'near_ground_support_radius')})

    def diagnostics(self, topic, msg):
        ok = bool(msg.status)
        modes = []
        for s in msg.status:
            level = s.level[0] if isinstance(s.level, bytes) else int(s.level)
            values = {v.key: v.value for v in s.values}
            if topic == '/pm01/localization_status':
                self.alignment_epoch = self.alignment_optimization_revision = None
                try:
                    epoch = json.loads(values.get('epoch', 'null'))
                    optimized = json.loads(values.get('optimization_revision', 'null'))
                    if (isinstance(epoch, str) and epoch and type(optimized) is int and optimized >= 0):
                        self.alignment_epoch, self.alignment_optimization_revision = epoch, optimized
                except (TypeError, ValueError):
                    pass
                try:
                    decoded = {k: json.loads(v) for k, v in values.items()}
                    correction = decoded.get('map_from_odom')
                    anchor = decoded.get('anchor_id')
                    pose = decoded.get('anchor_pose')
                    source = decoded.get('correction_source_stamp')
                    if (decoded.get('mode') == 'slam' and correction is not None and len(correction) == 4 and
                            pose is not None and len(pose) == 3 and type(anchor) is int and anchor >= 0 and
                            source is not None and np.isfinite([source, *correction, *pose]).all() and
                            self.alignment_epoch is not None):
                        record = (float(source), self.alignment_epoch, self.alignment_optimization_revision,
                                  int(decoded['revision']), anchor, tuple(pose), tuple(correction))
                        if not self.applied_alignments or self.applied_alignments[-1] != record:
                            self.applied_alignments.append(record)
                except (TypeError, ValueError, KeyError):
                    pass
                revision = (values.get('epoch'), values.get('revision'))
                if revision != self.manual_revision:
                    self.manual_revision = revision
                    # Ordinary accepted alignment updates apply only to new
                    # observations; historical map-frame evidence stays fixed.
            if topic == '/lio/status':
                epoch = values.get('epoch') or values.get('session_id')
                previous = self.health.get(topic)
                stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                changed = epoch is not None and self.lio_epoch is not None and epoch != self.lio_epoch
                rewound = previous is not None and stamp < previous[1]-1.
                replaced = False
                if time.monotonic()-self.last_lio_publisher_check >= .5:
                    self.last_lio_publisher_check = time.monotonic()
                    publishers = self.get_publishers_info_by_topic('/lio/status')
                    if len(publishers) == 1:
                        gid = tuple(publishers[0].endpoint_gid)
                        replaced = self.lio_publisher_gid is not None and gid != self.lio_publisher_gid
                        self.lio_publisher_gid = gid
                if (changed or rewound or replaced) and self.seed is not None:
                    self.reset_completion()
                    self.pending.clear()
                    self.last_processed = -math.inf
                    self.last_correction = None
                    self.correction_resets += 1
                if epoch is not None:
                    self.lio_epoch = epoch
            mode = (local_observation_support(level, s.message, values) if topic == '/lio/status'
                    else localization_support(topic, level, s.message, values))
            modes.append(mode)
            ok &= mode != 'unsupported'
        self.health_modes[topic] = modes
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.health[topic] = (time.monotonic(), stamp, ok)

    def observation_eligibility(self, now, received_now):
        """Local sensing and map alignment must each be usable, independently."""
        local, alignment = (observation_health_current(self.health.get(topic), now,
                            received_now, self.config['max_data_age'],
                            future_tolerance=.2 if topic == '/pm01/localization_status' else .05)
                            for topic in self.health_topics)
        if self.config['dynamic_seed']:
            # Ordinary scan revisions need not match a periodically published
            # map. A graph optimization changes coordinates, so its map and TF
            # must belong to the same epoch and optimization before any writes.
            alignment &= (self.slam_snapshot is not None and
                          self.snapshot_identity() == (self.alignment_epoch,
                                                       self.alignment_optimization_revision))
        return local, bool(alignment)

    def snapshot_identity(self):
        return (None if self.slam_snapshot is None else
                (self.slam_snapshot.epoch, self.slam_snapshot.optimization_revision))

    def reject(self, reason):
        self.reason = reason
        self.rejected[reason] = self.rejected.get(reason, 0)+1

    def on_ground(self, msg):
        if not msg.status:
            return
        status = msg.status[0]
        level = status.level[0] if isinstance(status.level, bytes) else int(status.level)
        self.ground.receive(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                            msg.header.frame_id, level, {v.key: v.value for v in status.values})

    def update(self):
        update_began = time.monotonic()
        if self.completion is None:
            return
        if not self.config['enabled']:
            self.reason = 'updates_disabled'
            return
        if self.latched_error:
            self.reject(self.latched_error)
            return
        now = self.get_clock().now().nanoseconds*1e-9
        local_usable, alignment_usable = self.observation_eligibility(now, update_began)
        if not local_usable:
            self.reject('local_observation_unhealthy_or_stale')
            return
        if not alignment_usable:
            self.reject('map_alignment_unhealthy_or_stale')
            return
        if not self.pending:
            self.reject('waiting_for_cloud')
            return
        # Look for the newest cloud whose exact timestamp has TF coverage.
        chosen = None
        for msg in reversed(self.pending):
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if stamp <= self.last_processed:
                continue
            if not 0 <= now-stamp <= self.config['max_data_age']:
                continue
            try:
                when = rclpy.time.Time.from_msg(msg.header.stamp)
                anchor_id = 0
                applied_correction = None
                if self.slam_snapshot is not None:
                    eligible = [v for v in self.applied_alignments if v[0] <= stamp and
                                (v[1], v[2]) == self.snapshot_identity()]
                    if not eligible:
                        continue
                    alignment = max(eligible, key=lambda v: (v[0], v[3]))
                    anchor_id, anchor_pose, applied_correction = alignment[4:]
                    self.completion.register_anchor(anchor_id, anchor_pose)
                    sensor = self.snapshot_transform(msg.header.frame_id, when, applied_correction)
                    root = self.snapshot_transform('base_footprint', when, applied_correction)
                    correction = TransformStamped()
                    x, y, z, a = applied_correction
                    correction.transform.translation.x, correction.transform.translation.y = x, y
                    correction.transform.translation.z = z
                    correction.transform.rotation.z = math.sin(a/2)
                    correction.transform.rotation.w = math.cos(a/2)
                else:
                    sensor = self.tf_buffer.lookup_transform('map', msg.header.frame_id, when)
                    root = self.tf_buffer.lookup_transform('map', 'base_footprint', when)
                    correction = self.tf_buffer.lookup_transform('map', 'odom', when)
                chosen = msg, stamp, sensor, root, correction, anchor_id, applied_correction
                break
            except Exception:
                continue
        if chosen is None:
            self.reject('cloud_or_timestamped_tf_unavailable')
            return
        msg, stamp, sensor, root, correction, anchor_id, applied_correction = chosen
        try:
            ct, cq = correction.transform.translation, correction.transform.rotation
            correction_pose = np.array([ct.x, ct.y, ct.z,
                math.atan2(2*(cq.w*cq.z+cq.x*cq.y), 1-2*(cq.y*cq.y+cq.z*cq.z))])
            self.last_correction = correction_pose
            t = sensor.transform.translation
            origin = np.array([t.x, t.y, t.z])
            plane, model_details = None, {}
            if self.config['use_shared_ground']:
                model = self.ground.get(stamp, now)
                if model is None:
                    self.reject('waiting_for_shared_ground')
                    return
                plane, model_stamp, frame, model_id = model
                if frame != 'map':
                    when = rclpy.time.Time.from_msg(msg.header.stamp)
                    tf = (self.snapshot_transform(frame, when, applied_correction) if self.slam_snapshot is not None
                          else self.tf_buffer.lookup_transform('map', frame, when)).transform
                    plane = transform_plane(plane, rotation(tf.rotation),
                        [tf.translation.x, tf.translation.y, tf.translation.z])
                model_details = {'ground_model_id': model_id, 'ground_model_stamp': model_stamp}
            sensor_height = origin[2]-(height_at(plane, origin[:2]) if plane is not None else self.config['ground_z'])
            if not self.config['min_sensor_height'] <= sensor_height <= self.config['max_sensor_height']:
                self.reject('sensor_height_inconsistent_with_ground')
                return
            points = points_array(msg)@rotation(sensor.transform.rotation).T+origin
            rt = root.transform.translation
            # The robot's own torso and limbs must not become map obstacles.
            points = points[np.linalg.norm(points[:, :2]-[rt.x, rt.y], axis=1) > 0.45]
            began = time.monotonic()
            result = self.completion.integrate(points, origin, stamp, ground_plane=plane,
                                               map_from_odom=correction_pose, anchor_id=anchor_id)
            self.last_processed = stamp
            # A newer scan may already be queued but still waiting for its
            # timestamped TF. Preserve it for the next timer instead of
            # discarding it whenever an older ready scan has been processed.
            while self.pending:
                first_stamp = (self.pending[0].header.stamp.sec+
                               self.pending[0].header.stamp.nanosec*1e-9)
                if first_stamp > stamp:
                    break
                self.pending.popleft()
            self.last_observation = {**result, 'stamp': stamp,
                                     **model_details,
                                     'sensor_height_above_ground_m': float(sensor_height),
                                     'integration_wall_sec': time.monotonic()-began,
                                     'update_wall_sec': time.monotonic()-update_began,
                                     'cloud_age_at_start_sec': now-stamp,
                                     'queued_newer_scans': len(self.pending)}
            self.reason = 'updating' if result['accepted'] else result['reason']
            if result['accepted']:
                self.publish_map()
                self.last_successful_observation_stamp = stamp
                self.last_successful_snapshot = self.snapshot_identity()
                self.last_successful_update_stamp = self.get_clock().now().nanoseconds*1e-9
                self.last_successful_update_monotonic = time.monotonic()
                self.publish_status()
        except MemoryError as exc:
            self.reject('map_capacity_exceeded')
            self.get_logger().error(str(exc))
        except Exception as exc:
            self.reject('invalid_cloud')
            self.get_logger().error(str(exc))

    def snapshot_transform(self, frame, when, correction=None):
        """Project exact-time local TF using an actually applied map alignment."""
        tf = deepcopy(self.tf_buffer.lookup_transform('odom', frame, when))
        t, q = tf.transform.translation, tf.transform.rotation
        x, y, z, a = self.slam_snapshot.correction if correction is None else correction
        c, s = math.cos(a), math.sin(a)
        t.x, t.y, t.z = c*t.x-s*t.y+x, s*t.x+c*t.y+y, t.z+z
        c, s = math.cos(a/2), math.sin(a/2)
        q.x, q.y, q.z, q.w = c*q.x-s*q.y, s*q.x+c*q.y, c*q.z+s*q.w, c*q.w-s*q.z
        tf.header.frame_id = 'map'
        return tf

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = self.seed.header.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = deepcopy(self.seed.info)
        msg.info.height, msg.info.width = self.completion.data.shape
        msg.info.resolution = self.completion.resolution
        x, y, yaw = self.completion.origin
        msg.info.origin.position.x, msg.info.origin.position.y = x, y
        msg.info.origin.orientation.z, msg.info.origin.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        msg.data = array('b', self.completion.data.astype(np.int8).tobytes())
        self.publisher.publish(msg)

    def publish_status(self):
        now = self.get_clock().now().nanoseconds*1e-9
        local_usable, alignment_usable = self.observation_eligibility(now, time.monotonic())
        source_age = (None if self.last_successful_observation_stamp is None else
                      now-self.last_successful_observation_stamp)
        observation_usable = bool(local_usable and alignment_usable and
            self.latched_error is None and source_age is not None and
            -.05 <= source_age <= self.config['max_data_age'] and
            self.last_successful_snapshot == self.snapshot_identity())
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        s = DiagnosticStatus()
        s.name = 'pm01/online_navigation_map'
        s.hardware_id = 'localized_lidar'
        s.level = bytes([0 if self.reason in ('updating', 'updates_disabled') else 1])
        s.message = self.reason
        values = {'seed_topic': self.config['seed_topic'],
                  'fresh_map': self.config['fresh_map'],
                  'localization_mode': self.config['prior_alignment_mode'],
                  'manual_alignment_revision': self.manual_revision,
                  'slam_map_revision': None if self.slam_snapshot is None else self.slam_snapshot.revision,
                  'map_revision': None if self.slam_snapshot is None else self.slam_snapshot.revision,
                  'map_epoch': None if self.slam_snapshot is None else self.slam_snapshot.epoch,
                  'map_optimization_revision': (None if self.slam_snapshot is None else
                                                self.slam_snapshot.optimization_revision),
                  'alignment_epoch': self.alignment_epoch,
                  'alignment_optimization_revision': self.alignment_optimization_revision,
                  'local_observation_usable': local_usable,
                  'alignment_usable': alignment_usable,
                  'map_alignment_usable': alignment_usable,
                  'observation_usable': observation_usable,
                  'source_stamp': self.last_successful_observation_stamp,
                  'source_age_sec': source_age,
                  'output_topic': self.config['output_topic'],
                  'ground_truth_used': False, 'mode': 'continuous_hit_miss_with_height_memory',
                  'ground_source': 'shared_automatic' if self.config['use_shared_ground'] else 'manual_reference',
                  'configured_ground_z': None if self.config['use_shared_ground'] else self.config['ground_z'],
                  'ray_clearance_m': self.config['ray_clearance'],
                  'localization_correction_resets': self.correction_resets,
                  'lio_reference_resets': self.correction_resets,
                  'localization_support': self.health_modes,
                  'last_successful_update_stamp': self.last_successful_update_stamp,
                  'last_successful_update_age_sec': (
                      None if self.last_successful_update_monotonic is None else
                      time.monotonic()-self.last_successful_update_monotonic),
                  'observation_age_sec': (self.get_clock().now().nanoseconds*1e-9-
                      self.last_observation['stamp'] if 'stamp' in self.last_observation else None),
                  'last_observation': self.last_observation, 'rejected_updates': self.rejected,
                  'health_checks': {topic: {
                      'valid': h[2], 'receive_age_sec': time.monotonic()-h[0],
                      'stamp_age_sec': self.get_clock().now().nanoseconds*1e-9-h[1]}
                      for topic, h in self.health.items()}}
        if self.completion is not None:
            values.update(self.completion.statistics())
            values.update({'map_width': self.completion.data.shape[1],
                           'map_height': self.completion.data.shape[0],
                           'map_resolution': self.completion.resolution,
                           'map_origin': self.completion.origin})
        s.values = [KeyValue(key=k, value=json.dumps(v)) for k, v in values.items()]
        msg.status = [s]
        self.status_publisher.publish(msg)

    def save(self, request, response):
        del request
        if self.completion is None:
            response.success, response.message = False, 'No seed map received'
            return response
        if not np.any(self.completion.data >= 0):
            response.success, response.message = False, 'Map contains only unknown cells; wait for a valid update'
            return response
        try:
            directory = Path(self.config['save_root']).expanduser()/datetime.datetime.now(
                datetime.timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
            directory.mkdir(parents=True, exist_ok=False)
            data = self.completion.data
            image = np.full(data.shape, 205, np.uint8)
            image[(data >= 0) & (data < 65)] = 254
            image[data >= 65] = 0
            height, width = image.shape
            (directory/'map.pgm').write_bytes(
                f'P5\n{width} {height}\n255\n'.encode()+np.flipud(image).tobytes())
            metadata = {'image': 'map.pgm', 'resolution': self.completion.resolution,
                        'origin': list(self.completion.origin), 'negate': 0,
                        'occupied_thresh': 0.65, 'free_thresh': 0.196, 'mode': 'trinary'}
            (directory/'map.yaml').write_text(json.dumps(metadata, indent=2)+'\n')
            (directory/'completion.json').write_text(json.dumps({
                'seed_data_sha256': self.seed_hash, 'config': self.config,
                **self.completion.statistics(), 'ground_truth_used': False}, indent=2)+'\n')
            response.success, response.message = True, str(directory/'map.yaml')
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response


def main():
    rclpy.init()
    node = OnlineNavigationMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
