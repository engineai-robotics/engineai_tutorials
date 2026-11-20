"""Matched poses keep their measurement times; map alignment remains valid."""
from pathlib import Path
import math
import json
import sys
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

from pm01_navigation_core.localization.map_alignment import Pose, compose
from pm01_navigation_core.nodes.map_matching_adapter import MapMatchingAdapter


def test_slam_pose_uses_exact_time_odometry_and_held_tf_stays_current():
    from types import SimpleNamespace
    rclpy.init()
    node = MapMatchingAdapter()
    captured = []
    try:
        node.pub = SimpleNamespace(publish=captured.append)
        stamp = node.get_clock().now()-Duration(seconds=1.)
        for when, x, y, yaw in [(stamp, 2., -1., .2),
                                (stamp+Duration(seconds=.8), 20., 30., .9)]:
            tf = TransformStamped(); tf.header.frame_id = 'odom'; tf.child_frame_id = 'navigation_base'
            tf.header.stamp = when.to_msg(); tf.transform.translation.x=x; tf.transform.translation.y=y
            tf.transform.rotation.z=math.sin(yaw/2); tf.transform.rotation.w=math.cos(yaw/2)
            node.tf.set_transform(tf, 'fixture')
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id='map'; msg.header.stamp=stamp.to_msg()
        msg.pose.pose.position.x=5.; msg.pose.pose.position.y=3.
        msg.pose.pose.orientation.z=math.sin(.5); msg.pose.pose.orientation.w=math.cos(.5)
        node.on_pose(msg); node.tick()
        assert node.correction is None  # Unvalidated apt backend cannot drive TF.
        node.on_match_update(String(data=json.dumps(dict(schema=1, backend='pm01-slam-2.6.10',
            epoch='fixture', revision=1, anchor_id=0, stamp_ns=stamp.nanoseconds, accepted=True,
            event='accepted', reason='accepted', pose=[5., 3., 1.], odom=[2., -1., .2]))))
        node.tick()
        result = compose(node.correction, Pose(2., -1., .2))
        assert (result.x, result.y, result.yaw) == pytest.approx((5., 3., 1.))
        applied = {v.key: json.loads(v.value) for v in captured[-1].status[0].values}
        assert applied['map_from_odom'] == pytest.approx([
            node.correction.x, node.correction.y, 0., .8])
        assert applied['anchor_id'] == 0
        assert applied['anchor_pose'] == [5., 3., 1.]
        assert applied['epoch'] == 'fixture' and applied['revision'] == 1
        assert applied['correction_source_stamp'] == pytest.approx(stamp.nanoseconds*1e-9)
        source_stamp = node.correction_stamp
        deadline = time.monotonic()+1.
        while time.monotonic() < deadline:
            node.tick(); rclpy.spin_once(node, timeout_sec=.02)
        tf = node.tf.lookup_transform('map','odom',rclpy.time.Time())
        age = node.get_clock().now().nanoseconds*1e-9-(tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9)
        assert -.6 < age < .5
        assert node.correction_stamp == source_stamp  # No fake new matches.
        held = {v.key: json.loads(v.value) for v in captured[-1].status[0].values}
        assert {k: held[k] for k in ('map_from_odom', 'anchor_id', 'anchor_pose',
            'epoch', 'revision', 'correction_source_stamp')} == {
            k: applied[k] for k in ('map_from_odom', 'anchor_id', 'anchor_pose',
            'epoch', 'revision', 'correction_source_stamp')}
        assert node.count_publishers('/motion/body_vel_cmd') == 0
    finally:
        node.destroy_node();rclpy.shutdown()


def test_amcl_owns_its_transform_without_a_second_broadcaster():
    rclpy.init(args=['--ros-args','-p','backend:=amcl'])
    node=MapMatchingAdapter()
    try:
        assert node.broadcaster is None and node.correction is None
        assert node.applied_map_from_odom is None and node.anchor_id is None
    finally:
        node.destroy_node();rclpy.shutdown()


def test_loop_pose_waits_for_matching_map_even_when_later_ordinary_pose_arrives():
    rclpy.init()
    node=MapMatchingAdapter()
    try:
        when=node.get_clock().now()-Duration(seconds=.2)
        tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='navigation_base'
        tf.header.stamp=when.to_msg();tf.transform.rotation.w=1.
        node.tf.set_transform(tf,'fixture')
        data=dict(schema=1,backend='pm01-slam-2.6.10',epoch='fixture',revision=3,
                  optimization_revision=2,anchor_id=2,stamp_ns=when.nanoseconds,accepted=True,
                  event='accepted',reason='accepted',pose=[0.,0.,1.],odom=[0.,0.,0.])
        msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=when.to_msg()
        msg.pose.pose.orientation.z=math.sin(.5);msg.pose.pose.orientation.w=math.cos(.5)
        node.on_match_update(String(data=json.dumps(data)));node.on_pose(msg);node.tick()
        assert node.correction is None and node.awaiting_map_revision
        assert node.applied_map_from_odom is None and node.anchor_id is None
        node.on_map_revision(String(data=json.dumps(data)));node.tick()
        assert node.correction is None  # Metadata alone is not a map snapshot.
        assert node.anchor_pose is None
        grid=OccupancyGrid();grid.header.stamp=when.to_msg();grid.info.width=grid.info.height=1;grid.data=[0]
        node.on_map(grid);node.tick()
        assert node.correction.yaw == pytest.approx(1.) and not node.awaiting_map_revision
        assert node.applied_map_from_odom == pytest.approx([0., 0., 0., 1.])
        assert (node.anchor_id, node.anchor_pose) == (2, [0., 0., 1.])
        assert node.optimization_revision == 2
    finally:
        node.destroy_node();rclpy.shutdown()


def test_unmatched_rotation_does_not_disappear_after_a_complete_turn():
    from nav_msgs.msg import Odometry
    rclpy.init();node=MapMatchingAdapter()
    try:
        node.odom_at_match=(0.,0.,0.)
        for i in range(81):
            yaw=i*2*math.pi/80
            msg=Odometry();msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
            node.on_odom(msg)
        assert node.unmatched_rotation == pytest.approx(2*math.pi)
        assert node.unmatched_path == 0.
    finally:
        node.destroy_node();rclpy.shutdown()


def test_stationary_observations_remain_healthy_but_unmatched_turn_degrades():
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from types import SimpleNamespace
    rclpy.init();node=MapMatchingAdapter()
    captured=[]
    try:
        node.pub=SimpleNamespace(publish=captured.append)
        node.map_received=True;node.pose_received=time.monotonic()
        node.covariance=[0.,0.,0.];node.backend_state='tracking'
        node.odom_at_match=(0.,0.,0.);node.correction=Pose(0.,0.,0.)
        def odom(x,y,yaw):
            msg=Odometry();msg.pose.pose.position.x=x;msg.pose.pose.position.y=y
            msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
            node.on_odom(msg)
        def status():
            scan=LaserScan();scan.header.stamp=node.get_clock().now().to_msg();node.on_scan(scan)
            tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom'
            tf.header.stamp=scan.header.stamp;tf.transform.rotation.w=1.
            node.tf.set_transform(tf,'fixture');node.tick()
            return captured[-1].status[0].message
        for i in range(6000):
            odom(.003*math.sin(i),.003*math.cos(i),.002*math.sin(i*.7))
        assert status()=='tracking'
        for i in range(81):
            odom(0.,0.,i*2*math.pi/80)
        assert status()=='map_match_not_advancing'
        assert node.unmatched_rotation > 6.
    finally:
        node.destroy_node();rclpy.shutdown()


def test_recovery_waits_for_accepted_pose_and_exact_tf_without_flapping_ordinary_tracking():
    from sensor_msgs.msg import LaserScan
    from types import SimpleNamespace
    rclpy.init(); node = MapMatchingAdapter(); captured = []
    try:
        node.pub = SimpleNamespace(publish=captured.append)
        node.map_received = True
        start = node.get_clock().now()-Duration(seconds=.4)
        def metadata(when, revision, accepted=True):
            node.on_match_update(String(data=json.dumps(dict(schema=1,
                backend='pm01-slam-2.6.10', epoch='fixture', revision=revision,
                stamp_ns=when.nanoseconds, accepted=accepted, optimization_revision=0, anchor_id=revision-1,
                event='accepted' if accepted else 'rejected',
                reason='accepted' if accepted else 'weak_response',
                pose=[0.,0.,0.], odom=[0.,0.,0.]))))
        def pose(when):
            msg = PoseWithCovarianceStamped(); msg.header.frame_id = 'map'
            msg.header.stamp = when.to_msg(); msg.pose.pose.orientation.w = 1.
            node.on_pose(msg)
        def odom_tf(when):
            tf = TransformStamped(); tf.header.frame_id = 'odom'
            tf.child_frame_id = 'navigation_base'; tf.header.stamp = when.to_msg()
            tf.transform.rotation.w = 1.; node.tf.set_transform(tf, 'fixture')
        def tick():
            scan = LaserScan(); scan.header.stamp = node.get_clock().now().to_msg()
            node.on_scan(scan)
            tf = TransformStamped(); tf.header.frame_id = 'map'; tf.child_frame_id = 'odom'
            tf.header.stamp = scan.header.stamp; tf.transform.rotation.w = 1.
            node.tf.set_transform(tf, 'fixture'); node.tick()
            return {v.key: json.loads(v.value) for v in captured[-1].status[0].values}
        metadata(start, 1); pose(start); odom_tf(start)
        original = tick()
        assert original['matched'] and node.backend_state == 'tracking'
        applied_keys = ('map_from_odom', 'anchor_id', 'anchor_pose', 'epoch',
                        'revision', 'optimization_revision', 'correction_source_stamp')
        original_applied = {k: original[k] for k in applied_keys}
        assert original_applied['anchor_id'] == 0
        rejected = start+Duration(seconds=.1)
        metadata(rejected, 2, False)
        rejected_status = tick()
        assert not rejected_status['matched']
        assert {k: rejected_status[k] for k in applied_keys} == original_applied
        recovered = start+Duration(seconds=.2)
        metadata(recovered, 3)
        awaiting_pose = tick()
        assert not awaiting_pose['matched'] and node.backend_state == 'waiting_for_accepted_pose'
        assert {k: awaiting_pose[k] for k in applied_keys} == original_applied
        pose(recovered)
        awaiting_tf = tick()
        assert not awaiting_tf['matched']  # The newest TF is still the old accepted scan.
        assert {k: awaiting_tf[k] for k in applied_keys} == original_applied
        assert node.revision == 1
        odom_tf(recovered)
        applied_recovery = tick()
        assert applied_recovery['matched'] and node.revision == 3
        assert applied_recovery['anchor_id'] == 2
        assert applied_recovery['correction_source_stamp'] == pytest.approx(recovered.nanoseconds*1e-9)
        ordinary = start+Duration(seconds=.3)
        metadata(ordinary, 4)
        pending_ordinary = tick()
        assert pending_ordinary['matched']  # No artificial pause on every normal metadata arrival.
        assert {k: pending_ordinary[k] for k in applied_keys} == {
            k: applied_recovery[k] for k in applied_keys}
        metadata(start+Duration(seconds=.35), 5, False)
        pose(ordinary); odom_tf(ordinary)
        assert not tick()['matched']
        assert node.backend_state == 'weak_response'  # A late older pose cannot undo rejection.
    finally:
        node.destroy_node(); rclpy.shutdown()


@pytest.mark.parametrize('anchor', [None, -1])
def test_accepted_metadata_without_a_graph_anchor_cannot_supply_an_alignment(anchor):
    rclpy.init(); node = MapMatchingAdapter()
    try:
        when = node.get_clock().now()-Duration(seconds=.1)
        data = dict(schema=1, backend='pm01-slam-2.6.10', epoch='fixture',
            revision=1, stamp_ns=when.nanoseconds, accepted=True, event='accepted',
            reason='accepted', pose=[0., 0., 0.], odom=[0., 0., 0.])
        if anchor is not None:
            data['anchor_id'] = anchor
        tf = TransformStamped(); tf.header.frame_id = 'odom'
        tf.child_frame_id = 'navigation_base'; tf.header.stamp = when.to_msg()
        tf.transform.rotation.w = 1.; node.tf.set_transform(tf, 'fixture')
        pose = PoseWithCovarianceStamped(); pose.header.frame_id = 'map'
        pose.header.stamp = when.to_msg(); pose.pose.pose.orientation.w = 1.
        node.on_match_update(String(data=json.dumps(data))); node.on_pose(pose); node.tick()
        assert node.backend_state == 'invalid_backend_metadata'
        assert node.correction is None and node.applied_map_from_odom is None
        assert node.anchor_id is None and node.anchor_pose is None
        assert node.revision == 0
    finally:
        node.destroy_node(); rclpy.shutdown()
