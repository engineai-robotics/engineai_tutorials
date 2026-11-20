"""Transient delivery errors do not permanently disable valid map snapshots."""
import pytest
import json
pytest.importorskip('rclpy')
import rclpy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from pm01_navigation_core.nodes.online_navigation_map import OnlineNavigationMap


def grid():
    msg=OccupancyGrid();msg.header.frame_id='map'
    msg.info.width=msg.info.height=10;msg.info.resolution=.1
    msg.info.origin.orientation.w=1.;msg.data=[0]*100
    return msg


def test_valid_seed_recovers_after_malformed_packet():
    rclpy.init();node=OnlineNavigationMap()
    try:
        node.seed_map(grid());before=node.seed_hash
        node.seed_map(OccupancyGrid())
        assert node.latched_error=='invalid_seed'
        node.seed_map(grid())
        assert node.latched_error is None and node.seed_hash==before
    finally:node.destroy_node();rclpy.shutdown()


def test_metadata_backlog_and_invalid_packet_do_not_latch_mapping_off():
    rclpy.init();node=OnlineNavigationMap()
    try:
        for _ in range(300):node.receive('revision',String(data='{}'))
        assert node.inbox_overflows>0 and node.latched_error is None
        node.on_map_revision(String(data='{}'))
        assert node.latched_error is None
        node.seed_map(grid())
        assert node.completion is not None
    finally:node.destroy_node();rclpy.shutdown()


def test_map_backlog_rebuilds_only_latest_complete_revision(monkeypatch):
    rclpy.init();node=OnlineNavigationMap()
    try:
        node.config['dynamic_seed'] = True
        calls = []
        original = node.seed_map
        newest_cloud = object()
        def seed(*args, **kwargs):
            calls.append(args[0].header.stamp.sec)
            original(*args, **kwargs)
            node.receive('cloud', newest_cloud)
        monkeypatch.setattr(node, 'seed_map', seed)
        for revision in range(1, 11):
            msg = grid(); msg.header.stamp.sec = revision
            node.receive('seed', msg)
            node.receive('revision', String(data=json.dumps(dict(schema=1,
                backend='pm01-slam-2.6.10', epoch='test', revision=revision,
                stamp_ns=revision*10**9, accepted=True, event='loop_closure', reason='accepted',
                pose=[0.,0.,0.], odom=[0.,0.,0.], optimization_revision=revision, anchor_id=0))))
            node.receive('anchors', String(data=json.dumps(dict(schema=1,
                backend='pm01-slam-2.6.10', epoch='test', optimization_revision=revision,
                anchors=[dict(id=0, pose=[0.,0.,0.])]))))
        node.receive_pending()
        assert calls == [10]
        assert node.slam_snapshot.revision == 10 and node.pending_snapshot is None
        assert list(node.pending) == [newest_cloud]
    finally:node.destroy_node();rclpy.shutdown()
