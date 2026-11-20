"""Sensor evidence, rather than seed/component geometry, controls occupancy."""
import math
from pathlib import Path
import sys

import numpy as np

from pm01_navigation_core.mapping.online_occupancy import OccupancyUpdater, empty_grid
import pytest


def grid(prior=None, **kwargs):
    return OccupancyUpdater(np.full((40, 70), -1) if prior is None else prior,
                            .1, (-1., -2., 0.), max_range=8., **kwargs)


def integrate(model, points, count=3, origin=(0., .05, 1.)):
    for _ in range(count):
        model.integrate(points, origin, model.last_stamp+1. if model.frames else 1.)


def at(model, x, y=.05):
    col, row = model.cell(x, y)
    return model.data[row, col]


def test_new_session_is_unknown_even_after_a_previous_map_was_populated():
    data, origin = empty_grid(.1, 4.)
    old = OccupancyUpdater(data, .1, origin)
    old.data[:] = 100
    data2, origin2 = empty_grid(.1, 4.)
    new = OccupancyUpdater(data2, .1, origin2)
    assert np.all(new.data == -1)
    assert not new.observed.any() and not new.voxel_occupied.any()
    assert np.all(new.evidence == 0)


def test_growth_preserves_obstacles_height_memory_and_world_coordinates():
    model=grid()
    integrate(model, [[2.05,.05,.5]], 3)
    old_frames=model.frames
    assert model.grow_to_include([[6.2,-3.,0.]], (0.,.05,1.), step_m=1.)
    col,row=model.cell(2.05,.05)
    assert model.data[row,col] == 100
    assert model.voxel_occupied[:,row*model.data.shape[1]+col].any()
    assert model.frames == old_frames
    assert at(model,6.2,-3.) == -1
    assert not model.grow_to_include([[6.2,-3.,0.]], (0.,.05,1.), step_m=1.)


def test_growth_capacity_failure_preserves_the_existing_map():
    model=grid();before=model.data.copy();origin=model.origin
    with pytest.raises(MemoryError):
        model.grow_to_include([[6.2,-3.,0.]], (0.,.05,1.), max_cells=model.data.size)
    assert model.origin == origin and np.array_equal(model.data,before)


def test_growth_of_rotated_grid_preserves_cell_world_location():
    yaw=.6;origin=(-1.,-2.,yaw)
    model=OccupancyUpdater(np.full((4,4),-1), .1, origin, max_range=8.)
    x=origin[0]+math.cos(yaw)*.15-math.sin(yaw)*.15
    y=origin[1]+math.sin(yaw)*.15+math.cos(yaw)*.15
    model.data[1,1]=100
    assert model.grow_to_include([[x+2.,y-2.,0.]], (x,y,1.), step_m=1.)
    col,row=model.cell(x,y)
    assert model.data[row,col]==100


@pytest.mark.parametrize('resolution,size', [(0.,4.),(.1,0.),(float('nan'),4.)])
def test_empty_map_rejects_invalid_geometry(resolution,size):
    with pytest.raises(ValueError):empty_grid(resolution,size)


def test_floor_only_marks_observed_low_ray_cells_free():
    model = grid()
    integrate(model, [[5.05, .05, 0.]])
    assert at(model, 4.95) == 0
    assert at(model, 2.05) == -1  # Beam still above a possible low obstacle.
    assert at(model, 4.95, 1.) == -1
    assert at(model, 5.55) == -1


def test_duplicate_scan_and_duplicate_points_are_not_independent_evidence():
    model = grid()
    for _ in range(4):
        model.integrate([[5.05, .05, 0.]]*100, (0., .05, 1.), 1.)
    assert model.frames == 1
    assert at(model, 4.95) == -1


def test_no_returns_or_overhead_points_never_erase_occupied_seed():
    model = grid(np.full((40, 70), 100))
    integrate(model, [], 20)
    integrate(model, [[5., 0., 3.], [float('nan'), 0., 0.]], 20)
    assert np.all(model.data == 100)


def test_large_connected_seed_obstacles_clear_where_floor_is_observed():
    prior = np.zeros((40, 70), int)
    prior[19:24, 29:39] = 100  # 0.5 m2, far beyond former speckle limit.
    model = grid(prior)
    floor = [[x, y, 0.] for x in np.arange(1.95, 2.86, .1)
             for y in np.arange(-.05, .36, .1)]
    integrate(model, floor, 8)
    assert np.all(model.data[19:24, 29:39] == 100)
    integrate(model, floor, 1)
    assert np.all(model.data[19:24, 29:39] == 0)
    assert model.statistics()['cleared_seed_obstacle_cells'] == 50


def test_real_returns_keep_a_seed_wall_occupied():
    prior = np.zeros((40, 70), int)
    prior[20, 30] = 100
    model = grid(prior)
    integrate(model, [[2.05, .05, .5]], 30)
    assert at(model, 2.05) == 100


def test_new_obstacles_are_written_even_on_previously_free_seed():
    model = grid(np.zeros((40, 70), int))
    integrate(model, [[2.05, .05, .5]], 2)
    assert at(model, 2.05) == 100
    assert model.statistics()['new_obstacles_on_seed_free_cells'] == 1


def test_low_ray_cannot_clear_an_observed_overhanging_obstacle():
    model = grid()
    integrate(model, [[2.05, .05, .5]], 3)
    integrate(model, [[2.05, .05, 0.]], 30)
    assert at(model, 2.05) == 100


def test_high_ray_cannot_clear_an_observed_low_obstacle():
    model = grid()
    integrate(model, [[2.05, .05, .2]], 3)
    integrate(model, [[5.05, .05, 0.]], 30)
    assert at(model, 2.05) == 100


def test_removed_obstacle_clears_only_after_its_height_is_observed_free():
    model = grid()
    integrate(model, [[2.05, .05, .5]], 3)
    # One real ground return in the column, another beam crossing the old hit
    # height on the way to its own actual ground return farther away.
    points = [[2.05, .05, 0.], [4.10, .05, 0.]]
    integrate(model, points, 1)
    assert at(model, 2.05) == 100
    integrate(model, points, 18)
    assert at(model, 2.05) == 0
    integrate(model, [[2.05, .05, .5]], 4)
    assert at(model, 2.05) == 100


def test_removed_obstacle_restores_known_free_ground_without_a_second_floor_observation():
    model = grid(np.zeros((40, 70), int), obstacle_height=.08,
                 height_resolution=.05, max_height=1.5)
    integrate(model, [[1.05, .05, .55]], 5, origin=(0., .05, 1.1))
    assert at(model, 1.05) == 100
    # These actual distant returns traverse the removed object's height.
    # They do not reach the floor in its column, which was already known free.
    integrate(model, [[2., .05, 0.], [2.1, .05, 0.], [2.2, .05, 0.]],
              30, origin=(0., .05, 1.1))
    assert at(model, 1.05) == 0


def test_current_hits_win_over_other_rays_in_the_same_scan():
    model = grid()
    points = [[2.05, .05, .5], [2.05, .05, 0.], [4.10, .05, 0.]]
    integrate(model, points, 30)
    assert at(model, 2.05) == 100


def test_cell_boundary_guard_uses_real_returns_and_allows_later_removal():
    prior = np.zeros((40, 70), int)
    prior[20, 30] = 100
    model = grid(prior)
    integrate(model, [[2.05, .05, 0.], [2.15, .05, .5]], 30)
    assert at(model, 2.05) == 100
    # No connected-component condition: removing the neighbouring return
    # allows exactly the same column to clear through repeated observations.
    integrate(model, [[2.05, .05, 0.]], 9)
    assert at(model, 2.05) == 0


def test_unobserved_seed_cells_are_preserved_without_time_decay():
    prior = np.full((40, 70), 100)
    model = grid(prior)
    integrate(model, [[2.05, .05, 0.]], 30)
    assert at(model, 2.05) == 0
    assert at(model, 2.05, 1.05) == 100
    assert at(model, 3.05) == 100


def test_one_frame_cannot_establish_free_or_occupied():
    model = grid()
    integrate(model, [[2.05, .05, .5]], 1)
    assert at(model, 2.05) == -1


def test_sensor_outside_seed_bounds_cannot_fill_from_outside():
    model = grid()
    result = model.integrate([[3., 0., 0.]], (-2., 0., 1.), 1.)
    assert not result['accepted']
    assert np.all(model.data == -1)


def test_rotated_map_origin_uses_grid_coordinates():
    model = OccupancyUpdater(np.full((40, 70), -1), .1, (2., -1., math.pi/2))
    assert model.cell(1.95, -.95) == (0, 0)


def test_torso_origin_ground_offset_does_not_turn_floor_into_an_obstacle():
    model = grid(ground_z=-.81)
    integrate(model, [[2.05, .05, -.81]], origin=(0., .05, .3))
    assert at(model, 2.05) == 0
    assert not np.any(model.data == 100)


def test_platform_top_remains_an_obstacle_with_torso_origin_ground_offset():
    model = grid(ground_z=-.81)
    integrate(model, [[2.05, .05, .19], [5.05, .05, -.81]], origin=(0., .05, .3))
    assert at(model, 2.05) == 100
    assert at(model, 2.55) == -1
