"""Exercise the ground, obstacle and live-map nodes with measured weak geometry."""
from array import array
import json
from types import SimpleNamespace

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField

from pm01_navigation_core.ground.flat_ground import GroundPlane
from pm01_navigation_core.localization.slam_updates import SlamUpdate
from pm01_navigation_core.nodes.automatic_ground import AutomaticGround
from pm01_navigation_core.nodes.navigation_obstacle_cloud import NavigationObstacleCloud
from pm01_navigation_core.nodes.online_navigation_map import OnlineNavigationMap
from pm01_navigation_core.pointcloud import points_array


@pytest.fixture
def nodes():
    rclpy.init()
    created = []
    def make(cls):
        node = cls()
        created.append(node)
        return node
    try:
        yield make
    finally:
        for node in reversed(created):
            node.destroy_node()
        rclpy.shutdown()


def stamp(seconds):
    return rclpy.time.Time(seconds=seconds).to_msg()


def set_time(node, seconds):
    node.get_clock = lambda: SimpleNamespace(now=lambda: rclpy.time.Time(seconds=seconds))


def capture(node, attribute):
    messages = []
    setattr(node, attribute, SimpleNamespace(publish=messages.append))
    return messages


def transforms(buffer, *, future_stamp=None):
    for parent, child, z in [('odom', 'lidar', 1.), ('odom', 'base_footprint', 0.),
                              ('map', 'odom', 0.)]:
        tf = TransformStamped()
        tf.header.frame_id = parent; tf.child_frame_id = child
        tf.transform.translation.z = z; tf.transform.rotation.w = 1.
        if future_stamp is None:
            buffer.set_transform_static(tf, 'fixture')
        else:
            tf.header.stamp = stamp(future_stamp)
            buffer.set_transform(tf, 'fixture')


def cloud(seconds):
    x, y = np.meshgrid(np.linspace(-3., 3., 41), np.linspace(-3., 3., 41))
    # A sensor one metre above a broad observed floor, plus two genuine obstacles.
    points = np.r_[np.c_[x.ravel(), y.ravel(), np.full(x.size, -1.)],
                   [[2., .1, -.6], [2., -.1, -.6]]]
    msg = PointCloud2()
    msg.header.frame_id = 'lidar'; msg.header.stamp = stamp(seconds)
    msg.height = 1; msg.width = len(points)
    msg.fields = [PointField(name=k, offset=i*4, datatype=7, count=1)
                  for i, k in enumerate(('x', 'y', 'z'))]
    msg.point_step = 12; msg.row_step = 12*len(points)
    msg.is_dense = True; msg.data = np.asarray(points, dtype='<f4').tobytes()
    return msg


def diagnostic(seconds, name, reason, values, level=0, frame=''):
    msg = DiagnosticArray()
    msg.header.stamp = stamp(seconds); msg.header.frame_id = frame
    item = DiagnosticStatus(name=name, message=reason, level=bytes([level]))
    item.values = [KeyValue(key=k, value=json.dumps(v)) for k, v in values.items()]
    msg.status = [item]
    return msg


def lio(seconds, **overrides):
    values = dict(pose_rank=5, motion_supported=False, lidar_update_applied=True,
                  lidar_correction_rejected=False, recovery_candidate_rejected=False,
                  velocity_norm_mps=.2)
    values.update(overrides)
    return diagnostic(seconds, 'fast_lio/geometry', 'weak_geometry', values, 1)


def ground(seconds, valid=True):
    return diagnostic(seconds, 'pm01/automatic_ground', 'tracking' if valid else 'ground_unavailable',
        dict(epoch='fixture', model_id='fixture:1', plane_stamp=seconds,
             ground_plane=GroundPlane(0., 0., 0., 1000, 1500, .001).summary()),
        0 if valid else 1, 'odom')


def values(message):
    return {v.key: json.loads(v.value) for v in message.status[0].values}


def test_actual_ground_node_fits_rank_five_but_rejects_failed_lidar_updates(nodes):
    node = nodes(AutomaticGround)
    transforms(node.tf)
    output = capture(node, 'pub')
    for seconds in (10., 10.2, 10.4, 10.6):
        set_time(node, seconds); node.on_health(lio(seconds))
        node.pending.append(cloud(seconds)); node.update()
    result = values(output[-1])
    assert result['ground_valid'] and result['local_observation_usable']
    assert result['lio_observation_support'] == 'weak_geometry_observation'
    assert result['ground_plane']['inliers'] > 100
    assert result['ground_plane']['rms_m'] < .002
    revision = node.tracker.revision
    set_time(node, 10.8)
    node.on_health(lio(10.8, lidar_correction_rejected=True))
    node.pending.append(cloud(10.8)); node.update()
    assert not values(output[-1])['ground_valid']
    assert node.tracker.revision == revision
    set_time(node, 11.)
    node.on_health(lio(11.)); node.pending.append(cloud(11.)); node.update()
    assert values(output[-1])['ground_valid']
    assert node.tracker.anchor.c == pytest.approx(0., abs=.002)


def test_ground_node_does_not_substitute_latest_tf_for_measurement_time(nodes):
    node = nodes(AutomaticGround)
    transforms(node.tf, future_stamp=11.)
    output = capture(node, 'pub')
    set_time(node, 10.); node.on_health(lio(10.))
    node.pending.append(cloud(10.)); node.update()
    assert not values(output[-1])['ground_valid']
    assert node.tracker.revision == 0
    assert node.reason == 'cloud_or_timestamped_tf_unavailable'


def test_missing_ground_publishes_no_fake_obstacles_or_empty_success_cloud(nodes):
    node = nodes(NavigationObstacleCloud)
    node.config['use_shared_ground'] = True
    transforms(node.tf)
    output = capture(node, 'pub'); state = capture(node, 'status')
    set_time(node, 10.); node.pending.append(cloud(10.)); node.update()
    assert not output
    assert values(state[-1])['observation_usable'] is False
    node.on_ground(ground(10.)); node.update()
    assert len(output) == 1
    assert len(points_array(output[-1])) == 2  # Only the measured raised returns.
    assert node.details['ground_valid']
    set_time(node, 10.2); node.on_ground(ground(10.2, valid=False))
    node.pending.append(cloud(10.2)); node.update()
    assert len(output) == 1
    assert not values(state[-1])['ground_valid']
    set_time(node, 10.4); node.on_ground(ground(10.4))
    node.pending.append(cloud(10.4)); node.update()
    assert len(output) == 2 and len(points_array(output[-1])) == 2


def test_live_map_accepts_weak_local_observations_only_with_usable_alignment(nodes):
    node = nodes(OnlineNavigationMap)
    node.config['use_shared_ground'] = True
    transforms(node.tf_buffer)
    capture(node, 'publisher'); state = capture(node, 'status_publisher')
    set_time(node, 10.)
    seed = OccupancyGrid(); seed.header.frame_id = 'map'; seed.header.stamp = stamp(10.)
    seed.info.width = seed.info.height = 80; seed.info.resolution = .1
    seed.info.origin.position.x = seed.info.origin.position.y = -4.
    seed.info.origin.orientation.w = 1.; seed.data = array('b', [-1]*6400)
    node.on_seed(seed)
    def update(seconds, matched=True, held=False, lidar_applied=True, mode='slam', **metadata):
        set_time(node, seconds)
        node.diagnostics('/lio/status', lio(seconds, lidar_update_applied=lidar_applied))
        node.diagnostics('/pm01/localization_status', diagnostic(seconds, 'pm01/map_matching',
            'tracking' if matched else 'recovering',
            dict(mode=mode, matched=matched, held_alignment_usable=held,
                 map_from_odom=[0., 0., 0., 0.], anchor_id=metadata.get('revision', 0),
                 anchor_pose=[0., 0., 0.], correction_source_stamp=seconds, **metadata),
            0 if matched else 1))
        node.on_ground(ground(seconds)); node.pending.append(cloud(seconds)); node.update()
        node.publish_status()
        return values(state[-1])
    result = update(10.)
    assert result['local_observation_usable'] and result['map_alignment_usable']
    assert result['observation_usable'] and result['source_stamp'] == 10.
    first = node.completion.statistics()['accepted_scans']
    result = update(10.2, matched=False, held=True)
    assert result['observation_usable'] and result['source_stamp'] == 10.2
    assert node.completion.statistics()['accepted_scans'] == first+1
    result = update(10.4, matched=False, held=False)
    assert result['local_observation_usable'] and not result['map_alignment_usable']
    assert not result['observation_usable'] and result['source_stamp'] == 10.2
    assert node.completion.statistics()['accepted_scans'] == first+1
    result = update(10.6, lidar_applied=False)
    assert result['map_alignment_usable'] and not result['local_observation_usable']
    assert not result['observation_usable']
    assert node.completion.statistics()['accepted_scans'] == first+1
    # Saved-map AMCL has no SLAM revision and must still accept observations.
    result = update(10.8, mode='amcl')
    assert result['observation_usable'] and result['source_stamp'] == 10.8

    node.config['dynamic_seed'] = True
    def snapshot(revision, optimized):
        return SlamUpdate('fixture', revision, 10_000_000_000, True, 'accepted', 'accepted',
                          (0., 0., 0.), (0., 0., 0.), optimized, revision)
    node.graph_anchors[('fixture', 2)] = {10: (0., 0., 0.)}
    node.adopt_snapshot((node.seed, snapshot(10, 2)))
    result = update(11., epoch='fixture', revision=100, optimization_revision=2)
    # Periodic map revisions may lag accepted ordinary scans without a pause.
    assert result['observation_usable'] and result['map_revision'] == 10
    accepted = node.completion.statistics()['accepted_scans']
    result = update(11.2, epoch='fixture', revision=101, optimization_revision=3)
    assert not result['map_alignment_usable'] and not result['observation_usable']
    assert node.completion.statistics()['accepted_scans'] == accepted
    node.graph_anchors[('fixture', 3)] = {100: (0., 0., 0.), 101: (0., 0., 0.)}
    node.adopt_snapshot((node.seed, snapshot(101, 3)))
    node.publish_status(); result = values(state[-1])
    assert result['map_alignment_usable']
    # Loading the corresponding map is not a new accepted lidar observation.
    assert not result['observation_usable'] and result['source_stamp'] == 11.
    result = update(11.4, epoch='fixture', revision=102, optimization_revision=3)
    assert result['observation_usable'] and result['source_stamp'] == 11.4
    result = update(11.6, epoch='another-session', revision=102, optimization_revision=3)
    assert not result['map_alignment_usable'] and not result['observation_usable']


def test_fresh_status_header_does_not_renew_the_last_ground_measurement(nodes):
    node = nodes(AutomaticGround)
    transforms(node.tf)
    output = capture(node, 'pub')
    for seconds in (10., 10.2, 10.4, 10.6):
        set_time(node, seconds); node.on_health(lio(seconds))
        node.pending.append(cloud(seconds)); node.update()
    set_time(node, 12.); node.on_health(lio(12.)); node.update()
    result = values(output[-1])
    assert not result['observation_usable']
    assert result['source_stamp'] == 10.6
    assert result['source_age_sec'] == pytest.approx(1.4)


def test_published_and_saved_geometry_include_measured_area_outside_seed(nodes, tmp_path):
    node = nodes(OnlineNavigationMap)
    node.config.update(use_shared_ground=True, save_root=str(tmp_path))
    transforms(node.tf_buffer)
    output = capture(node, 'publisher'); capture(node, 'status_publisher')
    seed = OccupancyGrid(); seed.header.frame_id = 'map'; seed.header.stamp = stamp(10.)
    seed.info.width = seed.info.height = 10; seed.info.resolution = .1
    seed.info.origin.orientation.w = 1.; seed.data = array('b', [-1]*100)
    node.on_seed(seed)
    for seconds in (10., 10.2, 10.4):
        set_time(node, seconds)
        node.diagnostics('/lio/status', lio(seconds))
        node.diagnostics('/pm01/localization_status', diagnostic(seconds, 'pm01/map_matching',
            'tracking', dict(mode='amcl', matched=True)))
        node.on_ground(ground(seconds)); node.pending.append(cloud(seconds)); node.update()
    msg = output[-1]
    assert msg.info.width > seed.info.width and msg.info.height > seed.info.height
    assert msg.info.width*msg.info.height == len(msg.data)
    assert msg.info.origin.position.x < 0. and msg.info.origin.position.y < 0.
    assert msg.info.width < 200 and -1 in msg.data and 0 in msg.data
    response = node.save(None, SimpleNamespace(success=False, message=''))
    assert response.success
    from pathlib import Path
    yaml = Path(response.message); saved = json.loads(yaml.read_text())
    assert saved['origin'] == list(node.completion.origin)
    dimensions = (yaml.parent/'map.pgm').read_bytes().split(b'\n', 3)[1]
    assert dimensions == f'{msg.info.width} {msg.info.height}'.encode()


def test_latest_applied_alignment_is_used_without_waiting_for_periodic_seed(nodes):
    node = nodes(OnlineNavigationMap)
    node.config.update(use_shared_ground=True, dynamic_seed=True)
    transforms(node.tf_buffer)
    capture(node, 'publisher'); capture(node, 'status_publisher')
    seed = OccupancyGrid(); seed.header.frame_id = 'map'; seed.header.stamp = stamp(10.)
    seed.info.width = seed.info.height = 100; seed.info.resolution = .1
    seed.info.origin.position.x = seed.info.origin.position.y = -5.
    seed.info.origin.orientation.w = 1.; seed.data = array('b', [-1]*10000)
    update = SlamUpdate('fixture', 1, 10_000_000_000, True, 'accepted', 'accepted',
                        (0., 0., 0.), (0., 0., 0.), 0, 1)
    node.adopt_snapshot((seed, update))
    for seconds in (10.2, 10.4):
        set_time(node, seconds)
        node.diagnostics('/lio/status', lio(seconds))
        node.diagnostics('/pm01/localization_status', diagnostic(seconds+.1, 'pm01/map_matching',
            'tracking', dict(mode='slam', matched=True, epoch='fixture', revision=2,
                optimization_revision=0, map_from_odom=[1., 0., 0., 0.],
                anchor_id=2, anchor_pose=[1., 0., 0.], correction_source_stamp=10.2)))
        node.on_ground(ground(seconds)); node.pending.append(cloud(seconds)); node.update()
    assert node.completion.frames == 2
    # Real raised returns at local x=2 are written using the newly applied
    # alignment x=1, rather than the periodic map snapshot's old x=0.
    col, row = node.completion.cell(3.05, .15)
    assert node.completion.data[row, col] == 100
    assert node.last_correction[0] == 1.
    assert node.slam_snapshot.revision == 1


def test_new_optimization_waits_for_matching_anchor_table(nodes):
    from std_msgs.msg import String
    node = nodes(OnlineNavigationMap); node.config['dynamic_seed'] = True
    capture(node, 'publisher')
    seed = OccupancyGrid(); seed.header.frame_id = 'map'; seed.info.width = seed.info.height = 10
    seed.info.resolution = .1; seed.info.origin.orientation.w = 1.; seed.data = array('b', [-1]*100)
    first = SlamUpdate('fixture', 1, 10_000_000_000, True, 'accepted', 'accepted',
                       (0., 0., 0.), (0., 0., 0.), 0, 1)
    node.adopt_snapshot((seed, first))
    next_update = SlamUpdate('fixture', 2, 11_000_000_000, True, 'loop', 'accepted',
                             (1., 0., 0.), (0., 0., 0.), 2, 2)
    node.adopt_snapshot((seed, next_update))
    assert node.slam_snapshot is first and node.pending_snapshot is not None
    node.on_graph_anchors(String(data=json.dumps(dict(schema=1, backend='pm01-slam-2.6.10',
        epoch='another-session', revision=2, optimization_revision=2,
        anchors=[dict(id=1, pose=[1., 0., 0.]), dict(id=2, pose=[1., 0., 0.])]))))
    assert node.slam_snapshot is first
    node.on_graph_anchors(String(data=json.dumps(dict(schema=1, backend='pm01-slam-2.6.10',
        epoch='fixture', revision=2, optimization_revision=2,
        anchors=[dict(id=1, pose=[1., 0., 0.]), dict(id=2, pose=[1., 0., 0.])]))))
    assert node.slam_snapshot is next_update and node.pending_snapshot is None
