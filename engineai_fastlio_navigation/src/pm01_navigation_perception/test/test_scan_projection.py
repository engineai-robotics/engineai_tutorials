"""Virtual scan visibility and sensor/frame geometry, without ROS."""
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from pm01_navigation_perception.scan_projection import Keyframes, MappingScans, ranges, read_xyz, rotation, yaw


def test_only_observed_directions_receive_ranges_and_nearest_hit_wins():
    scan = ranges([[2., 0., 1.], [1., 0., .5], [-3., 0., .8], [np.nan, 1., 0.]])
    assert np.count_nonzero(np.isfinite(scan)) == 2
    assert scan[360] == 1. and scan[0] == 3.
    assert np.isnan(scan[180])  # Missing returns are not a max-range free ray.


def test_physical_range_limits_and_empty_cloud_do_not_fabricate_clear_space():
    assert np.isnan(ranges([[.1, 0., 0.], [13., 0., 0.], [1., np.inf, 0.]])).all()
    assert np.isnan(ranges(np.empty((0, 3)))).all()


def test_tilted_sensor_is_rotated_before_horizontal_projection():
    angle = math.pi/3
    r = rotation(0., math.sin(angle/2), 0., math.cos(angle/2))
    world = np.array([[3., 1., .4]])
    sensor = np.array([.12, 0., 1.])
    raw = (world-sensor)@r
    restored = raw@r.T+sensor
    scan = ranges(restored)
    assert np.nanmin(scan) == pytest.approx(math.sqrt(10.))
    assert yaw(0., math.sin(angle/2), 0., math.cos(angle/2)) == pytest.approx(0.)


def test_organized_big_endian_cloud_with_padding_keeps_xyz():
    data = bytearray(80)
    expected = [[1., 2., 3.], [4., 5., 6.], [7., 8., 9.], [10., 11., 12.]]
    for i, xyz in enumerate(expected): struct.pack_into('>fff', data, (i//2)*40+(i%2)*16, *xyz)
    msg = SimpleNamespace(height=2, width=2, is_bigendian=True, data=data, row_step=40, point_step=16,
                          fields=[SimpleNamespace(name=k, offset=i*4, datatype=7) for i, k in enumerate(('x','y','z'))])
    np.testing.assert_allclose(read_xyz(msg), expected)


@pytest.mark.parametrize('kwargs', [{'range_min': 0.}, {'range_max': .1}, {'beams': 3}])
def test_invalid_scan_geometry_is_rejected(kwargs):
    with pytest.raises(ValueError): ranges([], **kwargs)


def test_keyframes_keep_rotation_without_unbounded_stationary_growth():
    frames = Keyframes(bootstrap=1)
    assert frames.accept(0., 0., math.pi-.05)
    assert not frames.accept(0., 0., -math.pi+.05)  # Wrapped turn is only .1.
    assert frames.accept(0., 0., -math.pi+.2)
    assert not any(frames.accept(0., 0., -math.pi+.2) for _ in range(100))
    assert not frames.accept(.19, 0., -math.pi+.2)
    assert frames.accept(.21, 0., -math.pi+.2)


def match_update(stamp, accepted, epoch='session'):
    return dict(schema=1, backend='pm01-slam-2.6.10', epoch=epoch,
                stamp_ns=round(stamp*1e9), accepted=accepted)


def test_stationary_retries_require_rejection_and_fresh_measurements():
    selector = MappingScans(bootstrap=1)
    assert selector.select(0., 0., 0., 1_000_000_000) == 'keyframe'
    assert selector.observe_match(match_update(1, True))
    assert not any(selector.select(0., 0., 0., n*200_000_000) for n in range(6, 3006))
    assert selector.observe_match(match_update(601, False))
    assert selector.select(0., 0., 0., 602_000_000_000) == 'recovery_observation'
    assert not selector.select(0., 0., 0., 602_000_000_000)
    assert not selector.select(0., 0., 0., 602_800_000_000)
    assert selector.select(0., 0., 0., 603_000_000_000) == 'recovery_observation'
    assert selector.select(0., 0., 0., 604_000_000_000) == 'recovery_observation'
    assert selector.observe_match(match_update(604, True))
    assert not selector.select(0., 0., 0., 606_000_000_000)


def test_motion_keyframes_do_not_wait_for_recovery_timer_or_send_twice():
    selector = MappingScans(bootstrap=1)
    assert selector.select(0., 0., 0., 1_000_000_000) == 'keyframe'
    selector.observe_match(match_update(1, False))
    assert selector.select(.21, 0., 0., 1_200_000_000) == 'keyframe'
    assert not selector.select(.21, 0., 0., 2_000_000_000)
    assert selector.select(.21, 0., 0., 2_200_000_000) == 'recovery_observation'
    assert not selector.select(.21, 0., 0., 2_200_000_000)


def test_old_or_invalid_backend_updates_cannot_cancel_newer_recovery():
    selector = MappingScans(bootstrap=1)
    assert selector.observe_match(match_update(10, False))
    assert not selector.observe_match(match_update(9, True))
    assert not selector.observe_match(match_update(10, True))
    assert selector.recovering
    assert not selector.observe_match('not json')
    assert not selector.observe_match(dict(match_update(11, True), accepted='false'))
    assert not selector.observe_match(dict(match_update(11, True), backend='other'))
    assert selector.observe_match(match_update(12, True, epoch='new-session'))
    assert not selector.observe_match(match_update(13, False))
    assert not selector.recovering


def test_invalid_pose_does_not_consume_mapping_retry():
    selector = MappingScans(bootstrap=1)
    selector.select(0., 0., 0., 1_000_000_000)
    selector.observe_match(match_update(1, False))
    assert not selector.select(float('nan'), 0., 0., 2_000_000_000)
    assert selector.select(0., 0., 0., 2_000_000_000) == 'recovery_observation'


def test_initial_mapping_acknowledgement_missing_retries_without_unbounded_bootstrap():
    selector = MappingScans()
    sent = [(n, selector.select(0., 0., 0., n*200_000_000)) for n in range(1, 21)]
    assert [n for n, kind in sent if kind == 'keyframe'] == [1, 2, 3, 4, 5]
    assert [n for n, kind in sent if kind == 'recovery_observation'] == [10, 15, 20]
    selector.observe_match(match_update(4, True))
    assert not selector.select(0., 0., 0., 5_000_000_000)


def test_input_gap_retries_even_without_backend_rejection_and_ignores_late_ack():
    selector = MappingScans(bootstrap=1)
    selector.select(0., 0., 0., 1_000_000_000)
    selector.observe_match(match_update(1, True))
    assert not selector.select(0., 0., 0., 10_000_000_000)
    selector.request_recovery()
    assert selector.observe_match(match_update(9, True))
    assert selector.recovering
    assert selector.observe_match(match_update(10, True))
    assert selector.recovering
    assert selector.select(0., 0., 0., 11_000_000_000) == 'recovery_observation'
    assert selector.observe_match(match_update(11, True))
    assert not selector.recovering
    assert not selector.select(0., 0., 0., 12_000_000_000)


def test_new_backend_epoch_does_not_inherit_old_ack_barrier():
    selector = MappingScans(bootstrap=1)
    selector.select(0., 0., 0., 10_000_000_000)
    selector.observe_match(match_update(10, True))
    selector.request_recovery()
    selector.observe_match(match_update(5, False, epoch='new-session'))
    assert selector.recovering
    selector.observe_match(match_update(6, True, epoch='new-session'))
    assert not selector.recovering
    assert not selector.observe_match(match_update(12, False))
