"""Exercise the real projection callback without a running ROS graph."""
from collections import deque
import json
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2
from pm01_navigation_perception.nodes import planar_scan


def test_new_scan_immediately_publishes_its_source_status(monkeypatch):
    node = object.__new__(planar_scan.PlanarScan)
    stamp = Time(sec=100)
    node.config = dict(max_data_age=.75, require_ground_status=False,
        scan_frame='navigation_base', odom_frame='odom', beams=720,
        range_min=.6, range_max=12., minimum_valid_beams=1, frequency=5.,
        publish_mapping_scan=False)
    clock = SimpleNamespace(nanoseconds=100050000000, to_msg=lambda: Time(sec=100, nanosec=50000000))
    node.get_clock = lambda: SimpleNamespace(now=lambda: clock)
    cloud = PointCloud2(); cloud.header.stamp = stamp; cloud.header.frame_id = 'lidar'
    node.pending = deque([cloud])
    node.last_stamp = node.last_projection_stamp = -float('inf')
    node.mapping_scans = SimpleNamespace(request_recovery=lambda: None, recovering=False)
    transform = SimpleNamespace(translation=SimpleNamespace(x=0., y=0., z=0.),
                                rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))
    node.tf = SimpleNamespace(lookup_transform=lambda *args: SimpleNamespace(transform=transform))
    monkeypatch.setattr(planar_scan, 'read_xyz', lambda msg: np.array([[2., 0., 0.]]))
    published = []
    node.pub = SimpleNamespace(publish=lambda msg: published.append(('scan', msg)))
    node.status = SimpleNamespace(publish=lambda msg: published.append(('status', msg)))

    node.update()

    assert [kind for kind, _ in published] == ['scan', 'status']
    scan, status = published[0][1], published[1][1].status[0]
    values = {value.key: json.loads(value.value) for value in status.values}
    assert status.message == 'projecting'
    assert scan.header.stamp == stamp
    assert values['cloud_stamp'] == 100.
    assert values['valid_beams'] == 1
    assert node.last_projection_stamp == 100.
