"""Optimized map geometry must not resurrect or retain misplaced live cells."""
from pathlib import Path
import sys

import numpy as np

from pm01_navigation_core.mapping.online_occupancy import OccupancyUpdater


def model():
    grid = OccupancyUpdater(np.full((40, 70), -1), .1, (-1., -2., 0.), max_range=8.)
    for stamp in (1., 2., 3.):
        assert grid.integrate([[2.05, .05, .5], [5.05, 1.05, 0.]], (0., .05, 1.), stamp)['accepted']
    return grid


def test_map_expansion_preserves_world_hits_and_height_memory_without_reusing_a_scan():
    grid = model()
    before = grid.statistics()
    data = np.full((60, 90), -1)
    assert grid.rebase(data, .1, (-2., -3., 0.)) > 0
    col, row = grid.cell(2.05, .05)
    assert grid.data[row, col] == 100
    assert grid.voxel_occupied[:, row*grid.data.shape[1]+col].any()
    assert grid.statistics()['accepted_scans'] == before['accepted_scans']
    assert grid.integrate([[2.05, .05, .5]], (0., .05, 1.), 3.)['reason'] == 'duplicate_or_old_scan'


def test_changed_prior_discards_old_wall_height_evidence():
    grid = model(); col, row = grid.cell(2.05, .05)
    assert grid.data[row, col] == 100
    seed = grid.prior.copy(); seed[row, col] = 0
    grid.rebase(seed, grid.resolution, grid.origin)
    assert grid.data[row, col] == 0
    assert not grid.voxel_occupied[:, row*grid.data.shape[1]+col].any()
    assert not grid.observed[row, col]


def test_repeated_identical_seed_preserves_clearing_progress():
    grid = model()
    data, evidence, voxels = grid.data.copy(), grid.evidence.copy(), grid.voxel_evidence.copy()
    grid.rebase(grid.prior, grid.resolution, grid.origin)
    np.testing.assert_array_equal(grid.data, data)
    np.testing.assert_array_equal(grid.evidence, evidence)
    np.testing.assert_array_equal(grid.voxel_evidence, voxels)


def test_fractional_grid_shift_drops_old_overlay_instead_of_resampling_obstacles():
    grid = model()
    seed = np.full_like(grid.prior, -1)
    assert grid.rebase(seed, .1, (-.95, -2., 0.)) == 0
    assert np.all(grid.data == -1) and not grid.voxel_occupied.any()
