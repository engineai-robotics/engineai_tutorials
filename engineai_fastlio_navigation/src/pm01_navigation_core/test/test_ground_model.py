"""Calibration without a fixed map Z, model continuity and consumer fail-closed."""
from pathlib import Path
import json
import math
import sys

import numpy as np
import pytest

from pm01_navigation_core.ground.flat_ground import GroundPlane
from pm01_navigation_core.ground.ground_model import GroundInbox, GroundTracker, bootstrap_ground, height_at, transform_plane
from pm01_navigation_core.mapping.online_occupancy import OccupancyUpdater


def floor(z=-.98, slope=.0):
    x, y = np.meshgrid(np.linspace(-4., 4., 61), np.linspace(-4., 4., 61))
    return np.c_[x.ravel(), y.ravel(), z+slope*x.ravel()]


@pytest.mark.parametrize('z', [-.6, -.98, -1.2])
def test_bootstrap_uses_mounting_height_rather_than_a_previous_map_datum(z):
    plane = bootstrap_ground(floor(z, .01), [0., 0.], [0., 0., .2])
    assert plane is not None and abs(plane.c-z) < .002
    assert abs(plane.a-.01) < .001


def test_missing_ground_sparse_cloud_and_table_above_height_band_are_rejected():
    for points in ([], [[1., 0., -.98]], floor(-.2)):
        assert bootstrap_ground(points, [0., 0.], [0., 0., .2]) is None


def test_dense_low_object_does_not_replace_spatially_broad_floor():
    rng = np.random.default_rng(5)
    points = floor()
    box = np.c_[rng.uniform(1.4, 1.8, 10000), rng.uniform(-.2, .2, 10000), np.full(10000, -.93)]
    plane = bootstrap_ground(np.r_[points, box], [0., 0.], [0., 0., .2])
    assert plane is not None and abs(plane.c+.98) < .002
    assert np.min(plane.heights(box)) > .04


def calibrated():
    tracker = GroundTracker()
    for stamp in (10., 10.2, 10.4, 10.6):
        tracker.update(floor(), [0., 0.], [0., 0., .2], stamp)
    assert tracker.plane is not None
    return tracker


def test_calibration_requires_consistent_frames_spanning_time_and_resets_for_motion():
    tracker = GroundTracker()
    for stamp, x in ((1., 0.), (1.2, .1), (1.4, .2), (1.6, .3)):
        assert tracker.update(floor(), [x, 0.], [x, 0., .2], stamp) is None
    assert tracker.anchor is None
    for stamp in (2., 2.2, 2.4, 2.6):
        tracker.update(floor(), [.3, 0.], [.3, 0., .2], stamp)
    assert tracker.anchor is not None
    revision = tracker.revision
    assert tracker.update(floor(), [.3, 0.], [.3, 0., .2], 2.5) is None
    assert tracker.revision == revision


def test_tracking_allows_small_changes_but_rejects_ground_jumps_and_no_silent_rebase():
    tracker = calibrated()
    assert tracker.update(floor(-.978), [0., 0.], [0., 0., .2], 10.8) is not None
    held = tracker.plane
    tracker.update(floor(-.93), [0., 0.], [0., 0., .2], 11.)
    assert tracker.reason == 'ground_change_rejected' and tracker.plane == held
    for stamp in (11.2, 11.4, 11.6, 12.):
        tracker.update(floor(-.93), [0., 0.], [0., 0., .2], stamp)
    assert tracker.current(12.) is None
    assert abs(tracker.anchor.c+.98) < .002


def test_localization_suspension_invalidates_a_fresh_model_immediately():
    tracker = calibrated()
    assert tracker.current(10.7) is not None
    tracker.suspend('localization_unhealthy_or_stale')
    assert tracker.current(10.7) is None
    # An unrelated higher floor after loss cannot acquire a new anchor.
    assert tracker.update(floor(-.7), [0., 0.], [0., 0., .2], 12.) is None
    assert tracker.anchor.c == pytest.approx(-.98)
    assert tracker.update(floor(), [0., 0.], [0., 0., .2], 12.2) is not None


def packet(plane_stamp=10., **changes):
    p = GroundPlane(.01, -.005, -.98, 300, 400, .004).summary()
    p.update(changes)
    return {'epoch': json.dumps('session-a'), 'model_id': json.dumps('session-a:4'),
            'plane_stamp': json.dumps(plane_stamp), 'ground_plane': json.dumps(p)}


def test_shared_consumers_select_the_same_stamped_model_and_reject_stale_or_future_models():
    a, b = GroundInbox(), GroundInbox()
    for inbox in (a, b):
        inbox.receive(10.05, 'odom', 0, packet(), received=20.)
    assert a.get(10.1, 10.2, 20.1) == b.get(10.1, 10.2, 20.1)
    assert a.get(9.8, 10.2, 20.1) is None
    assert a.get(10.9, 10.9, 20.1) is None
    assert a.get(10.1, 10.2, 20.9) is None
    a.receive(10.04, 'odom', 1, {}, received=20.1)
    assert a.get(10.1, 10.2, 20.1) is not None  # delayed old packet
    a.receive(10.1, 'odom', 1, {'epoch': json.dumps('session-a')}, received=20.1)
    assert a.get(10.2, 10.2, 20.1) is None


@pytest.mark.parametrize('changes', [{'rms_m': .02}, {'a': 1.}, {'c': math.nan}, {'inliers': 5}])
def test_invalid_shared_models_never_reach_obstacle_or_map_consumers(changes):
    inbox = GroundInbox()
    inbox.receive(10.05, 'odom', 0, packet(**changes), received=20.)
    assert inbox.get(10.1, 10.2, 20.1) is None


def test_plane_frame_transform_preserves_ground_relative_heights():
    plane = GroundPlane(.01, .02, -.98, 300, 400, .004)
    R = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    t = np.array([2., -3., .15])
    points = np.array([[1., 2., height_at(plane, [1., 2.])], [-1., 0., -.99]])
    changed = transform_plane(plane, R, t)
    assert np.allclose(changed.heights(points@R.T+t), plane.heights(points))
    with pytest.raises(ValueError):
        transform_plane(plane, np.array([[1.,0.,0.],[0.,0.,-1.],[0.,1.,0.]]), t)


def test_shared_plane_preserves_three_centimetre_obstacles_despite_wrong_legacy_ground_z():
    points = floor()
    patch = (abs(points[:, 0]-1.6)<.25) & (abs(points[:, 1])<.2)
    points[patch, 2] += .03
    plane = bootstrap_ground(points, [0.,0.], [0.,0.,.2])
    assert plane is not None
    model = OccupancyUpdater(np.full((100,100), -1), .1, (-5.,-5.,0.),
        ground_z=99., obstacle_height=.025, adaptive_ground=True, near_ground_filter=True)
    for stamp in range(1,5):
        assert model.integrate(points, (0.,0.,.2), stamp, ground_plane=plane)['accepted']
    assert np.count_nonzero(model.data >= 65)>0
