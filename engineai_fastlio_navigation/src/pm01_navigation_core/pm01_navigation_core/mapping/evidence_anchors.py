"""Reproject sufficient occupancy evidence by its own optimized graph anchor.

Only an optimization invokes this operation. No scans are stored or replayed.
Unknown/missing anchors cannot supply either an obstacle or clearing evidence.
"""
import math
import numpy as np
from pm01_navigation_core.mapping.online_occupancy import OccupancyUpdater


def transform(points, old, new):
    angle = new[2]-old[2]
    c, s = math.cos(angle), math.sin(angle)
    delta = np.asarray(points)-old[:2]
    return np.column_stack((c*delta[:, 0]-s*delta[:, 1]+new[0],
                            s*delta[:, 0]+c*delta[:, 1]+new[1]))


def centres(grid, columns):
    row, col = np.divmod(columns, grid.data.shape[1])
    x, y = (col+.5)*grid.resolution, (row+.5)*grid.resolution
    c, s = math.cos(grid.origin[2]), math.sin(grid.origin[2])
    return np.column_stack((grid.origin[0]+c*x-s*y, grid.origin[1]+s*x+c*y))


def offsets(grid):
    c, s = math.cos(grid.origin[2]), math.sin(grid.origin[2])
    return np.array([(0., 0.), (-.49999, -.49999), (-.49999, .49999),
                     (.49999, -.49999), (.49999, .49999)]) @ np.array([[c, s], [-s, c]])*grid.resolution


def sample(grid, points):
    return grid._indices(points)


def common_anchor_transform(anchors, updated):
    """Whether every retained source can carry cumulative votes unchanged.

    A mixed vote does not retain an unbounded contributor list. Missing anchors
    or differing transforms therefore require conservative revalidation; a
    complete rigid graph correction preserves both counts and their mixed flag.
    """
    reference = None
    for anchor, old in anchors.items():
        if anchor not in updated:
            return False
        new = updated[anchor]
        angle = math.atan2(math.sin(new[2]-old[2]), math.cos(new[2]-old[2]))
        c, s = math.cos(angle), math.sin(angle)
        value = (new[0]-c*old[0]+s*old[1], new[1]-s*old[0]-c*old[1], angle)
        if reference is None:
            reference = value
        elif (math.hypot(value[0]-reference[0], value[1]-reference[1]) > 1e-8 or
              abs(math.atan2(math.sin(value[2]-reference[2]),
                             math.cos(value[2]-reference[2]))) > 1e-8):
            return False
    return True


def reproject(live, anchors, updated, classify_old, classify_new, cutoff_old, max_cells):
    observations = np.asarray(live.observation_anchors, dtype=np.int32)
    stamps = np.asarray(live.observation_stamps, dtype=float)
    low_ids = live.low_observation.ravel()
    voxel_ids = np.where(live.voxel_occupied.ravel() | (live.voxel_evidence.ravel() > 0),
                         live.voxel_hit_observation.ravel(), live.voxel_observation.ravel())
    def group_ids(ids):
        nonzero = np.flatnonzero(ids)
        labels = observations[ids[nonzero]]
        order = np.argsort(labels, kind='stable')
        labels, nonzero = labels[order], nonzero[order]
        keys, starts = np.unique(labels, return_index=True)
        ends = np.r_[starts[1:], len(nonzero)]
        return {int(key): nonzero[start:end] for key, start, end in zip(keys, starts, ends)}
    low_groups, voxel_groups = group_ids(low_ids), group_ids(voxel_ids)
    active = sorted((low_groups.keys() | voxel_groups.keys()) & anchors.keys() & updated.keys())
    groups = []
    bounds = []
    size = live.data.size
    empty = np.array([], dtype=np.int64)
    for anchor in active:
        lows = low_groups.get(anchor, empty)
        voxels = voxel_groups.get(anchor, empty)
        columns = np.unique(np.concatenate((lows, voxels % size)))
        if not len(columns):
            continue
        xy = transform((centres(live, columns)[:, None, :]+offsets(live)).reshape(-1, 2),
                       anchors[anchor], updated[anchor])
        bounds.append(xy)
        groups.append((anchor, lows, voxels))
    # Retain the existing canvas, and expand only for moved observed evidence.
    old_corners = centres(live, np.array([0, live.data.shape[1]-1, size-live.data.shape[1], size-1]))
    extent = np.vstack([(old_corners[:, None, :]+offsets(live)).reshape(-1, 2), *bounds])
    resolution = live.resolution
    lo = np.floor(extent.min(axis=0)/resolution+1e-8)*resolution
    hi = np.ceil(extent.max(axis=0)/resolution-1e-8)*resolution
    width, height = np.ceil((hi-lo)/resolution).astype(int)
    if width*height > max_cells:
        raise MemoryError('Optimized evidence exceeds configured cell capacity')
    options = {k: getattr(live, k) for k in ('ground_z', 'obstacle_height', 'max_height', 'max_range',
        'height_resolution', 'ray_clearance', 'adaptive_ground', 'near_ground_filter',
        'near_ground_band_max', 'near_ground_support_radius')}
    result = OccupancyUpdater(np.full((height, width), -1, np.int16), resolution, (*lo, 0.), **options)
    result.observation_stamps, result.observation_anchors = live.observation_stamps, live.observation_anchors
    result.frames, result.last_stamp = live.frames, live.last_stamp
    dest_size = result.data.size

    def candidates(columns, anchor):
        xy = centres(live, columns)
        transformed = transform((xy[:, None, :]+offsets(live)).reshape(-1, 2), anchors[anchor], updated[anchor])
        ids, valid = sample(result, transformed)
        return np.unique(ids[valid])

    def source_samples(dest, anchor):
        xy = centres(result, dest)
        source = transform((xy[:, None, :]+offsets(result)).reshape(-1, 2), updated[anchor], anchors[anchor])
        ids, valid = sample(live, source)
        return ids.reshape(-1, 5), valid.reshape(-1, 5), source.reshape(-1, 5, 2)

    rigid = common_anchor_transform(anchors, updated)
    invalid_low = live.low_mixed_anchors.ravel() & ~np.bool_(rigid)
    invalid_voxel = live.voxel_mixed_anchors.ravel() & ~np.bool_(rigid)
    adjusted_low_misses = np.where(invalid_low, np.minimum(live.low_misses.ravel(), 1),
                                  live.low_misses.ravel())
    adjusted_low_evidence = np.where(invalid_low, np.maximum(live.evidence.ravel(), -1),
                                    live.evidence.ravel())
    adjusted_scores = live.voxel_evidence.ravel().copy()
    adjusted_occupied = live.voxel_occupied.ravel().copy()
    # Never withdraw a confirmed obstacle because mixed clearing votes moved.
    # Discard their reduction and keep its confirmation floor. Unconfirmed
    # positive/negative sums retain at most one hit/miss and must be earned again.
    positive = adjusted_scores > 0
    adjusted_scores[invalid_voxel & adjusted_occupied] = 6
    adjusted_scores[invalid_voxel & ~adjusted_occupied & positive] = 3
    adjusted_scores[invalid_voxel & ~adjusted_occupied & ~positive] = np.maximum(
        adjusted_scores[invalid_voxel & ~adjusted_occupied & ~positive], -1)
    # A miss can reduce an existing hit only while both observations still
    # cover the same physical cell after nonrigid graph optimization.
    for anchor, _, voxels in groups:
        affected = voxels[adjusted_occupied[voxels] & (adjusted_scores[voxels] < 6)]
        if not len(affected):
            continue
        votes = observations[live.voxel_observation.ravel()[affected]]
        for vote_anchor in np.unique(votes):
            chosen = affected[votes == vote_anchor]
            if vote_anchor == anchor:
                continue
            if vote_anchor not in anchors or vote_anchor not in updated:
                adjusted_scores[chosen] = 6
                continue
            footprint = (centres(live, chosen % size)[:, None, :]+offsets(live)).reshape(-1, 2)
            hit_ids, hit_valid = sample(result, transform(footprint, anchors[anchor], updated[anchor]))
            miss_ids, miss_valid = sample(result, transform(footprint, anchors[vote_anchor], updated[vote_anchor]))
            same = (hit_valid & miss_valid & (hit_ids == miss_ids)).reshape(-1, 5).all(axis=1)
            adjusted_scores[chosen[~same]] = 6

    for anchor, lows, voxels in groups:
        if len(lows):
            dest = candidates(lows, anchor)
            ids, valid, points = source_samples(dest, anchor)
            safe = np.clip(ids, 0, size-1)
            obs = low_ids[safe]
            qualified = valid & (obs != 0) & (observations[obs] == anchor)
            qualified &= stamps[obs] > cutoff_old(points.reshape(-1, 2)).reshape(-1, 5)
            # A new wall cannot be cleared using measurements that belonged to
            # free/unknown space before the graph correction.
            old_class = classify_old(points.reshape(-1, 2)).reshape(-1, 5)
            new_class = classify_new(centres(result, dest))
            qualified &= (new_class[:, None] < 65) | (old_class >= 65)
            use = qualified.all(axis=1)
            if np.any(use):
                dest, safe, obs = dest[use], safe[use], obs[use]
                # The weakest covered source supplies a conservative decision.
                counts = adjusted_low_misses[safe].min(axis=1)
                source_observation = obs[np.arange(len(obs)), np.argmin(stamps[obs], axis=1)]
                newer = stamps[source_observation] > result.low_stamp.ravel()[dest]
                dest, safe, counts, source_observation = dest[newer], safe[newer], counts[newer], source_observation[newer]
                result.low_observation.ravel()[dest] = source_observation
                result.low_stamp.ravel()[dest] = stamps[source_observation]
                result.low_misses.ravel()[dest] = counts
                result.evidence.ravel()[dest] = adjusted_low_evidence[safe].max(axis=1)
                result.low_mixed_anchors.ravel()[dest] = np.any(
                    live.low_mixed_anchors.ravel()[safe] & ~invalid_low[safe], axis=1)
                source_free = (live.data.ravel()[safe] == 0).all(axis=1)
                result.column_state.ravel()[dest] = np.where(
                    (result.evidence.ravel()[dest] <= -3) & source_free, 0, -1)
        # Reproject each height independently. Positive occupied evidence wins
        # collisions; negative evidence requires all five footprint samples.
        for level in np.unique(voxels//size):
            source_voxels = voxels[voxels//size == level]
            columns = source_voxels % size
            dest = candidates(columns, anchor)
            ids, valid, points = source_samples(dest, anchor)
            safe = np.clip(ids, 0, size-1)+level*size
            obs = voxel_ids[safe]
            qualified = valid & (obs != 0) & (observations[obs] == anchor)
            qualified &= stamps[obs] > cutoff_old(points.reshape(-1, 2)).reshape(-1, 5)
            old_class = classify_old(points.reshape(-1, 2)).reshape(-1, 5)
            new_class = classify_new(centres(result, dest))
            occupied = adjusted_occupied[safe] & qualified
            scores = adjusted_scores[safe]
            positive = (scores > 0) & qualified
            # A changed optimized prior supersedes an old geometric obstacle.
            obsolete = (new_class[:, None] == 0) & (old_class >= 65)
            occupied &= ~obsolete
            positive &= ~obsolete
            hits = occupied.any(axis=1)
            partial = positive.any(axis=1) & ~hits
            frees = (qualified & (scores <= 0)).all(axis=1) & ~hits & ~partial
            use = hits | partial | frees
            if not np.any(use):
                continue
            target = dest[use]+level*dest_size
            obs = obs[use]; occupied = occupied[use]; positive = positive[use]
            scores = scores[use]; source_voxels = safe[use]
            is_hit = hits[use]; is_partial = partial[use]
            preferred = np.where(is_hit[:, None], occupied,
                                 np.where(is_partial[:, None], positive, np.ones_like(occupied)))
            strongest = np.max(np.where(preferred, scores, -32768), axis=1)
            strongest_support = preferred & (scores == strongest[:, None])
            hit_pick = np.argmax(np.where(strongest_support, stamps[obs], -math.inf), axis=1)
            free_pick = np.argmin(stamps[obs], axis=1)
            pick = np.where(is_hit | is_partial, hit_pick, free_pick)
            source_observation = obs[np.arange(len(obs)), pick]
            old_obs = result.voxel_observation.ravel()[target]
            old_hit = result.voxel_occupied.ravel()[target]
            old_positive = result.voxel_evidence.ravel()[target] > 0
            old_obs = np.where(old_hit | old_positive, result.voxel_hit_observation.ravel()[target], old_obs)
            priority = np.where(is_hit, 2, np.where(is_partial, 1, 0))
            old_priority = np.where(old_hit, 2, np.where(old_positive, 1, 0))
            replace = (priority > old_priority) | ((priority == old_priority) &
                       (stamps[source_observation] > stamps[old_obs]))
            selected_score = np.max(np.where(preferred, scores, -32768), axis=1)
            selected_source = source_voxels[np.arange(len(obs)), pick]
            selected_vote = live.voxel_observation.ravel()[selected_source]
            coherent = (~invalid_voxel[selected_source] &
                        (adjusted_scores[selected_source] == live.voxel_evidence.ravel()[selected_source]))
            selected_vote = np.where((is_hit | is_partial) & coherent, selected_vote, source_observation)
            target, source_observation = target[replace], source_observation[replace]
            is_hit, is_partial, selected_score = is_hit[replace], is_partial[replace], selected_score[replace]
            result.voxel_observation.ravel()[target] = selected_vote[replace]
            result.voxel_hit_observation.ravel()[target] = np.where(is_hit | is_partial, source_observation, 0)
            result.voxel_occupied.ravel()[target] = is_hit
            result.voxel_evidence.ravel()[target] = selected_score
            result.voxel_mixed_anchors.ravel()[target] = (
                live.voxel_mixed_anchors.ravel()[selected_source] & ~invalid_voxel[selected_source])[replace]
    result.data[:] = result.column_state
    occupied = np.any(result.voxel_occupied, axis=0)
    result.data.ravel()[occupied] = 100
    hit_voxels = np.flatnonzero(result.voxel_occupied)
    np.maximum.at(result.hit_stamp.ravel(), hit_voxels % dest_size,
                  stamps[result.voxel_hit_observation.ravel()[hit_voxels]])
    result.observed[:] = (result.low_observation != 0) | occupied.reshape(result.data.shape)
    result.observed_stamp[:] = np.maximum(result.low_stamp, result.hit_stamp)
    return result
