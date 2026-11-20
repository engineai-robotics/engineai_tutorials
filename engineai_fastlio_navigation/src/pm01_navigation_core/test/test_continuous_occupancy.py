"""Localization corrections cannot erase live ray evidence or revive a prior."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

from pm01_navigation_core.mapping.continuous_occupancy import ContinuousOccupancy


def model():
    data = np.zeros((80, 80), np.int16)
    data[29:32, 49:52] = 100
    return ContinuousOccupancy(data, .1, (-3., -3., 0.), ground_z=0.,
        obstacle_height=.025, height_resolution=.05, max_range=8.)


def scan(grid, correction=(0., 0., 0., 0.), count=12):
    points = np.array([[x, y, 0.] for x in np.arange(1.5, 2.7, .05)
                       for y in np.arange(-.5, .6, .05)]+[[1.05, -1.05, .5]])
    x, y, z, yaw = correction
    c, s = math.cos(yaw), math.sin(yaw)
    matrix = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    world = points@matrix.T+[x, y, z]
    origin = np.array([0., 0., 1.])@matrix.T+[x, y, z]
    for _ in range(count):
        stamp = grid.last_stamp+1. if grid.frames else 1.
        assert grid.integrate(world, origin, stamp, map_from_odom=correction)['accepted']


def at(grid, x=2.05, y=.05):
    col, row = grid.cell(x, y)
    return grid.data[row, col]


@pytest.mark.parametrize('correction', [(.19, 0., 0., 0.), (.21, 0., 0., 0.),
    (0., 0., 0., .09), (0., 0., 0., .11), (.4, -.2, .1, -.3)])
def test_ordinary_correction_never_moves_historical_hits_or_free(correction):
    grid = model(); scan(grid)
    assert at(grid) == 0 and at(grid, 1.05, -1.05) == 100
    evidence = grid.live.voxel_evidence.copy()
    misses = grid.live.low_misses.copy()
    grid.project(correction)
    np.testing.assert_array_equal(grid.live.voxel_evidence, evidence)
    np.testing.assert_array_equal(grid.live.low_misses, misses)
    assert grid.frames == 12
    assert at(grid) == 0 and at(grid, 1.05, -1.05) == 100


def test_gradual_correction_and_repeat_seed_do_not_restart_clearing():
    grid = model(); scan(grid)
    for x in np.arange(.01, .31, .01):
        grid.project((x, 0., 0., 0.))
        grid.rebase(grid.prior, grid.resolution, grid.origin, stamp=grid.last_stamp)
        assert at(grid) == 0 and grid.frames == 12


@pytest.mark.parametrize('origin', [(-2.95, -3., 0.), (-3.1, -3.2, .05)])
def test_fractional_or_rotated_map_origin_retains_live_observations(origin):
    grid = model(); scan(grid)
    old = grid.live
    grid.rebase(grid.prior, grid.resolution, origin, stamp=grid.last_stamp)
    assert grid.live is old and grid.frames == 12
    assert at(grid) == 0 and at(grid, 1.05, -1.05) == 100


def test_new_optimized_geometry_supersedes_old_hits_but_current_hits_can_return():
    grid = model(); scan(grid)
    col, row = grid.cell(1.05, -1.05)
    # Previous SLAM classification was occupied, and is now corrected to free.
    grid.prior[row, col] = 100
    new = grid.prior.copy(); new[row, col] = 0
    grid.rebase(new, .1, grid.origin, stamp=grid.last_stamp)
    assert at(grid, 1.05, -1.05) == 0
    scan(grid, count=2)
    assert at(grid, 1.05, -1.05) == 100


def test_unobserved_space_and_seed_walls_are_not_cleared_by_resampling():
    grid = model()
    for yaw in (.0, .1, .8):
        grid.project((.03, .02, 0., yaw))
        assert at(grid) == 100
    grid.rebase(np.full((80, 80), -1), .1, (-2.95, -3., .1), stamp=10.)
    assert np.all(grid.data == -1)


def test_new_prior_obstacle_requires_nine_independent_low_observations():
    grid = model(); scan(grid, count=8)
    assert at(grid) == 100
    scan(grid, count=1)
    assert at(grid) == 0


def test_seed_coverage_loss_does_not_erase_measured_live_hits_or_free_space():
    grid = model(); scan(grid)
    assert at(grid) == 0 and at(grid, 1.05, -1.05) == 100
    # A rebuilt SLAM raster temporarily marks previously covered space unknown.
    # Neither a new physical observation nor an odom reset occurred.
    grid.rebase(np.full(grid.prior.shape, -1), grid.resolution, grid.origin,
                stamp=grid.last_stamp)
    assert at(grid) == 0 and at(grid, 1.05, -1.05) == 100
    assert grid.frames == 12


def test_map_expansion_keeps_live_voxels_and_duplicate_scan_guard():
    grid = model(); scan(grid)
    assert grid.grow_to_include([[7., -4., 0.]], (0., 0., 1.))
    assert at(grid) == 0 and at(grid, 1.05, -1.05) == 100
    assert not grid.integrate([[2., 0., 0.]], (0., 0., 1.), grid.last_stamp)['accepted']


@pytest.mark.parametrize('correction',[(0.,0.,0.,math.pi/2),(.7,-.4,0.,-.6)])
def test_atomic_rotated_prior_keeps_cleared_wall_and_live_hit(correction):
    grid=model();scan(grid)
    evidence=grid.live.voxel_evidence.copy()
    x,y,_,yaw=correction;c,s=math.cos(yaw),math.sin(yaw)
    transform=lambda a,b:(x+c*a-s*b,y+s*a+c*b)
    ox,oy=transform(*grid.origin[:2])
    grid.rebase(grid.prior.copy(),grid.resolution,(ox,oy,yaw),
                stamp=grid.last_stamp,map_from_odom=correction, optimized_anchors={0: (x,y,yaw)})
    assert at(grid,*transform(2.05,.05)) == 0
    assert at(grid,*transform(1.05,-1.05)) == 100
    assert grid.frames == 12


def test_ordinary_alignment_does_not_shift_old_free_into_unknown_wall_interior():
    grid = model(); scan(grid)
    col, row = grid.cell(3.05, .05)
    grid.prior[row, col] = -1
    grid.project((0., 0., 0., 0.))
    assert at(grid, 3.05, .05) == -1
    before = grid.data.copy()
    grid.project((1., 0., 0., 0.))
    np.testing.assert_array_equal(grid.data, before)
    assert at(grid, 3.05, .05) == -1


def anchored_hits():
    grid = ContinuousOccupancy(np.zeros((100, 100), np.int16), .1, (-5., -5., 0.),
        obstacle_height=.025, height_resolution=.1, max_range=8., require_anchors=True)
    for anchor, x in ((1, -2.05), (2, 2.05)):
        grid.register_anchor(anchor, (0., 0., 0.))
        for _ in range(2):
            stamp = grid.last_stamp+1. if grid.frames else 1.
            assert grid.integrate([[x, .05, .5]], [x, -2., 1.], stamp, anchor_id=anchor)['accepted']
    return grid


def test_distinct_graph_anchors_follow_nonrigid_optimization_independently():
    grid = anchored_hits()
    assert at(grid, -2.05, .05) == at(grid, 2.05, .05) == 100
    grid.rebase(grid.prior, .1, grid.origin, stamp=grid.last_stamp,
                optimized_anchors={1: (1., 0., 0.), 2: (-1., .5, 0.)})
    assert at(grid, -1.05, .05) == at(grid, 1.05, .55) == 100
    assert at(grid, -2.05, .05) == at(grid, 2.05, .05) == 0
    assert grid.frames == 4


def test_missing_graph_anchor_withdraws_only_its_unassociated_evidence():
    grid = anchored_hits()
    grid.rebase(grid.prior, .1, grid.origin, stamp=grid.last_stamp,
                optimized_anchors={2: (0., 0., 0.)})
    assert at(grid, -2.05, .05) == 0
    assert at(grid, 2.05, .05) == 100
    assert grid.prior.sum() == 0  # The complete seed remains available.


def test_optimization_collisions_preserve_real_hits_over_free_evidence():
    grid = model(); scan(grid)
    grid.register_anchor(2, (0., 0., 0.))
    for _ in range(2):
        grid.integrate([[3.05, .05, .5]], (3.05, -2., 1.), grid.last_stamp+1., anchor_id=2)
    grid.rebase(grid.prior, .1, grid.origin, stamp=grid.last_stamp,
                optimized_anchors={0: (0., 0., 0.), 2: (-1., 0., 0.)})
    assert at(grid, 2.05, .05) == 100


def test_observed_output_expands_beyond_seed_and_survives_smaller_seed():
    seed = np.full((10, 10), -1, np.int16)
    grid = ContinuousOccupancy(seed, .1, (0., 0., 0.), obstacle_height=.025, max_range=5.)
    points = [[x, y, 0.] for x in np.arange(1.5, 2.6, .05) for y in np.arange(.1, .9, .05)]
    for stamp in (1., 2., 3.):
        assert grid.integrate(points+[[100., 0., 0.]], (.5, .5, 1.), stamp)['accepted']
    assert at(grid, 2.05, .55) == 0
    assert grid.data.shape[1] > 10 and grid.data.shape[1] < 100
    assert np.any(grid.data < 0)  # Unmeasured expanded area is not whitened.
    grid.rebase(seed, .1, (0., 0., 0.), stamp=3.)
    assert at(grid, 2.05, .55) == 0


def test_equal_source_stamp_never_crosses_optimization_cutoff():
    grid = anchored_hits()
    col, row = grid.cell(2.05, .05)
    grid.cutoff[row, col] = grid.last_stamp
    grid.rebase(grid.prior, .1, grid.origin, stamp=grid.last_stamp,
                optimized_anchors={1: (0., 0., 0.), 2: (0., 0., 0.)})
    assert at(grid, 2.05, .05) == 0
    assert at(grid, -2.05, .05) == 100


def test_identity_graph_optimization_preserves_live_bounds_at_cell_capacity():
    grid = ContinuousOccupancy(np.zeros((20, 20), np.int16), .1, (-1., -1., 0.),
        initial_size=2., max_cells=400, obstacle_height=.025, max_range=2.)
    for stamp in (1., 2.):
        grid.integrate([[.85, .05, .5]], [0., 0., 1.], stamp)
    shape, origin = grid.live.data.shape, grid.live.origin
    for _ in range(3):
        grid.rebase(grid.prior, .1, grid.origin, stamp=2., optimized_anchors={0: (0., 0., 0.)})
        assert grid.live.data.shape == shape and grid.live.origin == origin
    assert at(grid, .85, .05) == 100


@pytest.mark.parametrize('poses', [
    [(0.01, 0., 0.), (0., 0., 0.)]*8,
    [(0., 0., 0.01), (0., 0., 0.)]*8,
    [(i*.01, 0., 0.) for i in range(1, 21)],
])
def test_repeated_subcell_optimizations_do_not_thicken_or_quantize_away_motion(poses):
    grid = anchored_hits()
    before = np.count_nonzero(grid.live.voxel_occupied)
    for pose in poses:
        grid.rebase(grid.prior, .1, grid.origin, stamp=grid.last_stamp,
                    optimized_anchors={1: pose, 2: pose})
        assert np.count_nonzero(grid.live.voxel_occupied) == before
        assert len(grid.live.hit_positions) == before
    x, y, yaw = poses[-1]
    for px in (-2.05, 2.05):
        assert at(grid, x+math.cos(yaw)*px-math.sin(yaw)*.05,
                  y+math.sin(yaw)*px+math.cos(yaw)*.05) == 100
    assert grid.frames == 4


def test_hit_positions_survive_canvas_growth_and_clear_with_real_rays():
    grid = anchored_hits()
    grid.rebase(grid.prior, .1, grid.origin, stamp=4.,
                optimized_anchors={1: (.01, 0., 0.), 2: (.01, 0., 0.)})
    before = sorted(grid.live.hit_positions.values())
    grid.live.grow_to_include([[9., 0., .5]], [8., 0., 1.], max_cells=1_000_000)
    assert sorted(grid.live.hit_positions.values()) == before
    for stamp in range(5, 17):
        grid.integrate([[3., .05, 0.]], [-4., .05, 1.], float(stamp), anchor_id=1)
    for voxel in grid.live.hit_positions:
        assert (grid.live.voxel_occupied.ravel()[voxel] or
                grid.live.voxel_evidence.ravel()[voxel] > 0)


@pytest.mark.parametrize('miss_shift, other_shift, expected_score', [
    (1., 1., 5), (1., 0., 6), (-1., 0., 6)])
def test_partial_clearing_is_retained_only_with_a_complete_coherent_transform(
        miss_shift, other_shift, expected_score):
    grid = anchored_hits()
    grid.register_anchor(3, (0., 0., 0.))
    grid.integrate([[-.05, .05, 0.]], [-4.05, .05, 1.], 5., anchor_id=3)
    col, row = grid.live.cell(-2.05, .05)
    before = grid.live.voxel_evidence[:, row*grid.live.data.shape[1]+col]
    occupied = grid.live.voxel_occupied[:, row*grid.live.data.shape[1]+col]
    assert np.any(occupied & (before < 6))
    score = before[occupied].min()
    grid.rebase(grid.prior, .1, grid.origin, stamp=5., optimized_anchors={
        1: (1., 0., 0.), 2: (other_shift, 0., 0.), 3: (miss_shift, 0., 0.)})
    assert at(grid, -1.05, .05) == 100  # Hit anchor, not the recent free-ray anchor.
    col, row = grid.live.cell(-1.05, .05)
    remaining = grid.live.voxel_evidence[:, row*grid.live.data.shape[1]+col]
    occupied = grid.live.voxel_occupied[:, row*grid.live.data.shape[1]+col]
    assert remaining[occupied].min() == expected_score
    assert score == 5  # One actual free vote follows two actual hit observations.
    if expected_score == 5:
        # A later graph optimization must still know the retained miss anchor.
        grid.rebase(grid.prior, .1, grid.origin, stamp=5., optimized_anchors={
            1: (2., 0., 0.), 2: (0., 0., 0.), 3: (-1., 0., 0.)})
        col, row = grid.live.cell(-.05, .05)
        occupied = grid.live.voxel_occupied[:, row*grid.live.data.shape[1]+col]
        scores = grid.live.voxel_evidence[:, row*grid.live.data.shape[1]+col]
        assert occupied.any() and scores[occupied].min() == 6


def test_unconfirmed_hit_remains_unconfirmed_across_optimization():
    grid = ContinuousOccupancy(np.zeros((100, 100), np.int16), .1, (-5., -5., 0.),
        obstacle_height=.025, height_resolution=.1, max_range=8.)
    grid.integrate([[1.05, .05, .5]], [1.05, -2., 1.], 1.)
    assert at(grid, 1.05, .05) == 0
    grid.rebase(grid.prior, .1, grid.origin, stamp=1., optimized_anchors={0: (1., 0., 0.)})
    assert at(grid, 2.05, .05) == 0
    grid.integrate([[2.05, .05, .5]], [2.05, -2., 1.], 2.)
    assert at(grid, 2.05, .05) == 100


def test_loop_clearing_uses_only_observations_from_each_height():
    grid = model()
    live = grid.live
    # Both layers have a clearing observation at the same XY. Each also has a
    # different neighbouring observation, and the middle height was never seen.
    live.observation_stamps.append(5.)
    live.observation_anchors.append(0)
    points = [(.05, .05), (.15, .05), (.05, .15)]
    layers = {1: {0, 1}, 3: {0, 2}}
    for level, observed in layers.items():
        for i in observed:
            col, row = live.cell(*points[i])
            cell = row * live.data.shape[1] + col
            live.voxel_observation[level, cell] = 1
            live.voxel_evidence[level, cell] = -level
    grid.rebase(grid.prior, .1, grid.origin, stamp=5.,
                optimized_anchors={0: (1., .5, math.pi / 2)})
    for i, (x, y) in enumerate(points):
        col, row = grid.live.cell(1. - y, .5 + x)
        cell = row * grid.live.data.shape[1] + col
        for level in (1, 2, 3):
            observed = i in layers.get(level, ())
            assert grid.live.voxel_observation[level, cell] == int(observed)
            assert grid.live.voxel_evidence[level, cell] == (-level if observed else 0)


def test_missing_hit_anchor_does_not_revive_older_contradicted_free_space():
    grid = model(); grid.prior[:] = -1; scan(grid)
    assert at(grid, 2.05, .05) == 0
    grid.register_anchor(2, (0., 0., 0.))
    for _ in range(2):
        grid.integrate([[2.05, .05, .5]], [2.05, -2., 1.], grid.last_stamp+1., anchor_id=2)
    assert at(grid, 2.05, .05) == 100
    grid.rebase(grid.prior, .1, grid.origin, stamp=grid.last_stamp,
                optimized_anchors={0: (0., 0., 0.)})
    assert at(grid, 2.05, .05) == -1  # Missing hit is uncertainty, not a removal observation.
    assert at(grid, 1.75, .25) == 0  # Uncontradicted free support is preserved.


def mixed_low_observations():
    grid = model()
    grid.register_anchor(1, (0., 0., 0.))
    points = np.array([[x, y, 0.] for x in np.arange(1.5, 2.7, .05)
                       for y in np.arange(-.5, .6, .05)])
    for stamp in range(1, 10):
        assert grid.integrate(points, (0., 0., 1.), float(stamp),
                              anchor_id=0 if stamp < 9 else 1)['accepted']
    assert at(grid) == 0
    return grid


@pytest.mark.parametrize('updated', [
    {0: (1., 0., 0.), 1: (0., 0., 0.)}, {1: (0., 0., 0.)}])
def test_mixed_clearing_cannot_assign_nine_votes_to_its_last_anchor(updated):
    grid = mixed_low_observations()
    seed = grid.prior.copy()
    grid.rebase(seed, .1, grid.origin, stamp=9., optimized_anchors=updated)
    assert at(grid) == 100  # The last anchor supplied one miss, not nine.
    col, row = grid.live.cell(2.05, .05)
    assert grid.live.low_misses[row, col] == 1
    assert not grid.live.low_mixed_anchors[row, col]
    assert grid.live.evidence[row, col] == -1
    # Mixed vertical clearing also cannot become three votes of the last scan.
    assert np.min(grid.live.voxel_evidence) >= -1
    assert not grid.live.voxel_mixed_anchors.any()
    points = np.array([[x, y, 0.] for x in np.arange(1.5, 2.7, .05)
                       for y in np.arange(-.5, .6, .05)])
    for stamp in range(10, 18):
        assert grid.integrate(points, (0., 0., 1.), float(stamp), anchor_id=1)['accepted']
    assert at(grid) == 0  # Fresh coherent observations can earn clearing again.


def test_rigid_optimization_preserves_mixed_provenance_for_a_later_nonrigid_change():
    grid = mixed_low_observations()
    grid.rebase(grid.prior.copy(), .1, (-2., -3., 0.), stamp=9.,
                optimized_anchors={0: (1., 0., 0.), 1: (1., 0., 0.)})
    assert at(grid, 3.05, .05) == 0
    col, row = grid.live.cell(3.05, .05)
    assert grid.live.low_misses[row, col] == 9
    assert grid.live.low_mixed_anchors[row, col]
    assert grid.live.voxel_mixed_anchors.any()
    grid.rebase(grid.prior.copy(), .1, grid.origin, stamp=9.,
                optimized_anchors={0: (2., 0., 0.), 1: (1., 0., 0.)})
    assert at(grid, 3.05, .05) == 100
    col, row = grid.live.cell(3.05, .05)
    assert grid.live.low_misses[row, col] == 1
    assert not grid.live.low_mixed_anchors[row, col]


def test_map_growth_keeps_the_bounded_mixed_provenance():
    grid = mixed_low_observations()
    grid.live.grow_to_include([[-20., 0., 0.]], (-20., 0., 1.), max_cells=1_000_000)
    col, row = grid.live.cell(2.05, .05)
    assert grid.live.low_mixed_anchors[row, col]
    assert grid.live.low_misses[row, col] == 9
    assert grid.live.voxel_mixed_anchors.any()


@pytest.mark.parametrize('yaw', [0., .35, -.7])
def test_partial_projection_matches_full_projection_after_motion_and_growth(yaw):
    """The optimized path must retain exactly the full projection evidence."""
    rng = np.random.default_rng(7)
    grid = ContinuousOccupancy(np.full((95, 120), -1, np.int16), .08,
        (-3., -3., yaw), ground_z=0., obstacle_height=.025, height_resolution=.05,
        max_range=8., initial_size=10.)
    for index in range(18):
        origin = np.array([index*.3, np.sin(index*.3), 1.])
        points = rng.uniform([-3., -3., 0.], [3., 3., 1.], (450, 3))
        points[:350, 2] = 0.
        points[:, :2] += origin[:2]
        result = grid.integrate(points, origin, 1.+index, map_from_odom=(0., 0., 0., 0.))
        assert result['accepted']
        partial = grid.data.copy()
        retained = grid.retained_cells
        grid.project((0., 0., 0., 0.))
        np.testing.assert_array_equal(grid.data, partial)
        assert grid.retained_cells == retained
        expected = np.where(np.any(grid.live.voxel_occupied, axis=0).reshape(grid.live.data.shape), 100, grid.live.column_state)
        np.testing.assert_array_equal(grid.live.data, expected)


def test_accelerated_projection_matches_source_on_consecutive_loops(monkeypatch):
    import copy
    from pm01_navigation_core.mapping import evidence_anchors
    if evidence_anchors.transfer_free is None:
        pytest.skip('Requires the installed map projection extension')
    reference = mixed_low_observations()
    accelerated = copy.deepcopy(reference)
    for dx, yaw in ((.13, .08), (-.16, -.14)):
        changed = {0: (dx, 0., yaw), 1: (-dx, .2, -yaw)}
        for grid in (reference, accelerated):
            with monkeypatch.context() as context:
                if grid is reference:
                    context.setattr(evidence_anchors, 'transfer_free', None)
                grid.rebase(grid.prior, grid.resolution, grid.origin, stamp=grid.last_stamp,
                            optimized_anchors=changed)
        for name, value in vars(reference.live).items():
            if isinstance(value, np.ndarray):
                np.testing.assert_array_equal(value, getattr(accelerated.live, name), err_msg=name)
        assert reference.live.hit_positions == accelerated.live.hit_positions
        np.testing.assert_array_equal(reference.data, accelerated.data)
