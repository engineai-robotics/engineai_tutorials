"""Ground estimation must distinguish low objects from floor/pose errors."""
from pathlib import Path
import sys

import pytest
import numpy as np
import yaml

from pm01_navigation_core.ground.flat_ground import GroundPlane, estimate_ground, near_ground_noise
from pm01_navigation_core.mapping.online_occupancy import OccupancyUpdater


def floor_cloud(offset=.0, slope=.0):
    x, y = np.meshgrid(np.linspace(-4., 4., 61), np.linspace(-4., 4., 61))
    z = -.81+offset+slope*x
    return np.c_[x.ravel(), y.ravel(), z.ravel()]


def test_ground_tracks_offset_and_small_tilt_without_calling_floor_an_obstacle():
    points = floor_cloud(.025, .015)
    plane = estimate_ground(points, [0., 0.], -.81)
    assert plane is not None
    assert abs(plane.c-(-.785)) < .001
    assert np.max(np.abs(plane.heights(points))) < .001


def test_three_centimetre_box_does_not_become_ground():
    points = floor_cloud(.025, .01)
    inside = (abs(points[:, 0]-1.6) < .3) & (abs(points[:, 1]) < .2)
    points[inside, 2] += .03
    plane = estimate_ground(points, [0., 0.], -.81)
    assert plane is not None
    assert np.all(plane.heights(points[inside]) > .025)
    assert np.max(np.abs(plane.heights(points[~inside]))) < .001


def test_sparse_cloud_and_missing_floor_are_not_a_valid_plane():
    assert estimate_ground([[1., 0., -.81]], [0., 0.], -.81) is None
    points = floor_cloud(.5)
    assert estimate_ground(points, [0., 0.], -.81) is None


def test_large_tilt_and_wrong_ground_datum_are_rejected():
    assert estimate_ground(floor_cloud(.12), [0., 0.], -.81) is None
    assert estimate_ground(floor_cloud(0., .2), [0., 0.], -.81) is None


def test_dense_obstacle_returns_do_not_outvote_spatially_broad_floor():
    points = floor_cloud()
    rng = np.random.default_rng(5)
    box = np.c_[rng.uniform(1.4, 1.8, 10000), rng.uniform(-.2, .2, 10000), np.full(10000, -.76)]
    plane = estimate_ground(np.r_[points, box], [0., 0.], -.81)
    assert plane is not None and abs(plane.c+.81) < .001
    assert np.min(plane.heights(box)) > .04


def test_adaptive_map_marks_three_cm_obstacle_on_shifted_ground():
    points = floor_cloud(.02)
    patch = (abs(points[:, 0]-1.6) < .25) & (abs(points[:, 1]) < .2)
    points[patch, 2] += .03
    model = OccupancyUpdater(np.zeros((100, 100), int), .1, (-5., -5., 0.),
        ground_z=-.81, obstacle_height=.025, adaptive_ground=True, height_resolution=.05)
    for stamp in range(1, 5):
        result = model.integrate(points, (0., 0., .3), float(stamp))
        assert result['accepted']
    assert np.count_nonzero(model.data >= 65) > 0
    # Unsupported ground observations cannot clear learned obstacles.
    saved = model.data.copy()
    result = model.integrate([[2., 0., -.76]], (0., 0., .3), 5.)
    assert not result['accepted'] and result['reason'] == 'ground_plane_unavailable'
    assert np.array_equal(saved, model.data)



def support_floor():
    x, y = np.meshgrid(np.linspace(.5, 2, 61), np.linspace(-1, 1, 81))
    return np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]


def support_plane():
    return GroundPlane(0, 0, 0, 1000, 1000, .004)


def test_isolated_three_cm_returns_are_rejected_with_measured_floor_support():
    noise = np.array([[.95, -.27, .028], [.96, .01, .031], [.94, .33, .026]])
    points = np.r_[support_floor(), noise]
    reject = near_ground_noise(points, support_plane(), .025)
    assert not reject[:-3].any()
    assert reject[-3:].all()


def test_three_cm_object_surface_survives_including_small_dense_patch():
    f = support_floor()
    footprint = (abs(f[:, 0]-1) <= .05) & (abs(f[:, 1]) <= .05)
    f[footprint, 2] = .03
    reject = near_ground_noise(f, support_plane(), .025)
    assert (~reject[footprint]).sum() >= 4
    assert not reject[~footprint].any()


def test_existing_sparse_three_cm_box_still_survives():
    x, y = np.meshgrid(np.linspace(-4, 4, 61), np.linspace(-4, 4, 61))
    points = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
    box = (abs(points[:, 0]-1.6) < .25) & (abs(points[:, 1]) < .2)
    points[box, 2] = .03
    reject = near_ground_noise(points, support_plane(), .025)
    assert (~reject[box]).sum() == box.sum()


def test_sparse_uncertain_returns_are_preserved_without_floor_evidence():
    points = np.array([[1., 0., .03], [2., 0., .028], [3., 0., .031]])
    assert not near_ground_noise(points, support_plane(), .025).any()


def test_tall_returns_never_use_near_ground_support_rule():
    objects = np.array([[.95, 0., .061], [.95, .1, .3], [.95, .2, 1.]])
    points = np.r_[support_floor(), objects]
    assert not near_ground_noise(points, support_plane(), .025)[-3:].any()


def test_duplicating_an_outlier_does_not_create_elevated_surface_support():
    points = np.r_[support_floor(), np.tile([.95, .01, .028], (100, 1))]
    assert near_ground_noise(points, support_plane(), .025)[-100:].all()


def test_rotated_tilted_plane_uses_ground_relative_height():
    points = np.r_[support_floor(), [[.95, .01, .028]]]
    angle = .7
    rotation = np.array([[np.cos(angle), -np.sin(angle)],
                         [np.sin(angle), np.cos(angle)]])
    points[:, :2] = points[:, :2]@rotation.T+[4, -2]
    points[:, 2] += .01*points[:, 0]-.005*points[:, 1]-.9
    p = GroundPlane(.01, -.005, -.9, 1000, 1000, .004)
    assert near_ground_noise(points, p, .025)[-1]


def test_new_filter_keeps_three_cm_box_in_occupancy_map():
    f = support_floor()
    f[:, 2] -= .81
    box = (abs(f[:, 0]-1.4) < .1) & (abs(f[:, 1]) < .1)
    f[box, 2] += .03
    model = OccupancyUpdater(np.zeros((60, 60)), .1, (-1., -3., 0.),
        ground_z=-.81, obstacle_height=.025, adaptive_ground=True,
        near_ground_filter=True, height_resolution=.05)
    for stamp in (1., 2., 3.):
        assert model.integrate(f, (0., 0., .2), stamp)['accepted']
    col, row = model.cell(1.4, 0.)
    assert np.any(model.data[row-1:row+2, col-1:col+2] >= 65)


def test_rejected_returns_never_become_free_space_evidence(monkeypatch):
    from pm01_navigation_core.mapping import online_occupancy
    model = OccupancyUpdater(np.full((40, 40), -1), .1, (-1., -2., 0.),
        obstacle_height=.025, near_ground_filter=True)
    monkeypatch.setattr(online_occupancy, 'near_ground_noise',
                        lambda points, *args, **kwargs: np.ones(len(points), bool))
    saved = model.data.copy()
    result = model.integrate([[1.5, 0., .03]], (0., 0., 1.), 1.)
    assert not result['accepted']
    assert np.array_equal(model.data, saved)


def test_enabled_filter_rejects_invalid_configuration():
    with pytest.raises(ValueError):
        OccupancyUpdater(np.zeros((10, 10)), .1, (0., 0., 0.),
            obstacle_height=.025, near_ground_filter=True, near_ground_support_radius=0.)

def test_mixed_floor_and_wall_tiles_do_not_raise_the_floor_seed():
    # A narrow visible floor strip at a wall: floor and elevated returns share
    # XY tiles. Counts must not turn one wall layer into the ground surface.
    rng = np.random.default_rng(27)
    points = []
    centres = np.arange(-3.9, 4.0, .2)
    for x in centres:
        for y in (-2.9, 2.9):
            for _ in range(4):
                points.append([x+rng.uniform(-.03,.03), y+rng.uniform(-.03,.03), -.98])
            for height in np.linspace(.04,.14,16):
                points.append([x+rng.uniform(-.03,.03), y+rng.uniform(-.03,.03), -.98+height])
    plane = estimate_ground(points, [0.,0.], -.98)
    assert plane is not None
    assert abs(plane.c+.98)<.005
    assert plane.rms<.01
