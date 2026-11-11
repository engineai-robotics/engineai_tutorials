"""Localized ray integration for every cell of a seeded navigation map.

The seed is a finite prior, not a permanent obstacle mask. Actual low rays
update column free evidence; obstacle returns update occupancy. Height voxels
remember observed obstacles so rays above/below them cannot erase them. No
component size, isolation, elapsed-time decay or simulator truth is used.
"""
import math

import numpy as np
from pm01_navigation_core.ground.flat_ground import GroundPlane, estimate_ground, near_ground_noise


def empty_grid(resolution, size_m, max_cells=1_000_000):
    """A new session starts unknown, with no occupied or free prior cells."""
    if not np.isfinite([resolution, size_m]).all() or resolution <= 0 or size_m <= 0:
        raise ValueError('Invalid new-map geometry')
    side = math.ceil(size_m/resolution)
    if side*side > max_cells:
        raise MemoryError('New map exceeds configured cell capacity')
    return np.full((side, side), -1, np.int16), (-side*resolution/2, -side*resolution/2, 0.)


class OccupancyUpdater:
    def __init__(self, data, resolution, origin, *, ground_z=0.0,
                 obstacle_height=0.12, max_height=1.8, max_range=12.0,
                 height_resolution=0.10, ray_clearance=0.15, adaptive_ground=False,
                 near_ground_filter=False, near_ground_band_max=.06,
                 near_ground_support_radius=.20):
        self.prior = np.asarray(data, dtype=np.int16).copy()
        if (self.prior.ndim != 2 or not self.prior.size or
                np.any((self.prior < -1) | (self.prior > 100))):
            raise ValueError('Expected a nonempty ROS occupancy grid')
        if (len(origin) != 3 or not np.isfinite([resolution, *origin, ground_z,
                obstacle_height, max_height, max_range, height_resolution, ray_clearance]).all() or
                resolution <= 0 or obstacle_height <= 0 or max_height <= obstacle_height or
                max_range <= 0.6 or height_resolution <= 0 or ray_clearance < 0):
            raise ValueError('Invalid grid geometry or lidar limits')
        self.resolution, self.origin = float(resolution), tuple(origin)
        self.ground_z, self.obstacle_height = ground_z, obstacle_height
        self.max_height, self.max_range = max_height, max_range
        self.height_resolution = height_resolution
        self.ray_clearance = ray_clearance
        self.adaptive_ground = adaptive_ground
        self.near_ground_filter = near_ground_filter
        self.near_ground_band_max = near_ground_band_max
        self.near_ground_support_radius = near_ground_support_radius
        if near_ground_filter and (
                not np.isfinite([near_ground_band_max, near_ground_support_radius]).all() or
                not obstacle_height < near_ground_band_max <= max_height or
                near_ground_support_radius <= 0):
            raise ValueError('Invalid near-ground filter configuration')
        self.height_bins = math.ceil((max_height-obstacle_height)/height_resolution)
        self.data = self.prior.copy()
        self.column_state = self.prior.copy()
        # All cells use the same bounded hit/miss evidence. Even occupied seed
        # cells can clear; only genuinely observed space can change the prior.
        self.evidence = np.where(self.prior >= 65, 6,
                                 np.where(self.prior >= 0, -3, 0)).astype(np.int16)
        shape = (self.height_bins, self.prior.size)
        self.voxel_evidence = np.zeros(shape, np.int16)
        self.voxel_occupied = np.zeros(shape, bool)
        self.observed = np.zeros_like(self.prior, bool)
        # Observation metadata is independent of the seed and survives grid
        # growth. Used by the continuous odom layer when adopting a SLAM map.
        self.observed_stamp = np.full_like(self.prior, -math.inf, dtype=float)
        self.hit_stamp = np.full_like(self.prior, -math.inf, dtype=float)
        self.low_stamp = np.full_like(self.prior, -math.inf, dtype=float)
        self.low_misses = np.zeros_like(self.prior, dtype=np.uint8)
        # Compact IDs retain the exact float64 source time and keyframe anchor
        # without adding two large arrays to every height voxel.
        self.observation_stamps = [float('-inf')]
        self.observation_anchors = [-1]
        self.low_observation = np.zeros_like(self.prior, dtype=np.uint32)
        self.voxel_observation = np.zeros(shape, dtype=np.uint32)
        self.voxel_hit_observation = np.zeros(shape, dtype=np.uint32)
        # Bounded provenance: cumulative votes can span multiple graph anchors.
        # A nonrigid optimization cannot assign that whole sum to its last
        # observation. One byte per cell/voxel marks sums needing revalidation.
        self.low_mixed_anchors = np.zeros_like(self.prior, dtype=bool)
        self.voxel_mixed_anchors = np.zeros(shape, dtype=bool)
        self.frames, self.last_stamp = 0, -math.inf

    def cell(self, x, y):
        dx, dy = x-self.origin[0], y-self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return (math.floor((c*dx+s*dy)/self.resolution),
                math.floor((-s*dx+c*dy)/self.resolution))

    def rebase(self, data, resolution, origin):
        """Adopt an optimized SLAM grid without retaining corrected ghosts.

        Live observations survive only where grids align in world coordinates
        AND the old and new priors agree. Changed prior cells discard their
        old height memory; arbitrary resampling also discards the live overlay.
        """
        options = {k: getattr(self, k) for k in (
            'ground_z', 'obstacle_height', 'max_height', 'max_range', 'height_resolution',
            'ray_clearance', 'adaptive_ground', 'near_ground_filter',
            'near_ground_band_max', 'near_ground_support_radius')}
        new = OccupancyUpdater(data, resolution, origin, **options)
        angle = math.atan2(math.sin(origin[2]-self.origin[2]), math.cos(origin[2]-self.origin[2]))
        compatible = abs(resolution-self.resolution) < 1e-9 and abs(angle) < 1e-8
        preserved = 0
        if compatible:
            c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
            dx, dy = origin[0]-self.origin[0], origin[1]-self.origin[1]
            shift = np.array([(c*dx+s*dy)/resolution, (-s*dx+c*dy)/resolution])
            compatible = np.max(np.abs(shift-np.rint(shift))) < 1e-4
        if compatible:
            dx, dy = np.rint(shift).astype(int)
            oh, ow = self.prior.shape; nh, nw = new.prior.shape
            x0, x1, y0, y1 = max(0, -dx), min(nw, ow-dx), max(0, -dy), min(nh, oh-dy)
            if x0 < x1 and y0 < y1:
                old_slice = np.s_[y0+dy:y1+dy, x0+dx:x1+dx]
                new_slice = np.s_[y0:y1, x0:x1]
                unchanged = self.prior[old_slice] == new.prior[new_slice]
                preserved = int((unchanged & self.observed[old_slice]).sum())
                for field in ('data', 'column_state', 'evidence', 'observed',
                              'observed_stamp', 'hit_stamp', 'low_stamp', 'low_misses', 'low_observation',
                              'low_mixed_anchors'):
                    target = getattr(new, field)[new_slice]
                    np.copyto(target, getattr(self, field)[old_slice], where=unchanged)
                for field in ('voxel_evidence', 'voxel_occupied', 'voxel_observation',
                              'voxel_hit_observation', 'voxel_mixed_anchors'):
                    target = getattr(new, field).reshape(new.height_bins, nh, nw)[:, new_slice[0], new_slice[1]]
                    source = getattr(self, field).reshape(self.height_bins, oh, ow)[:, old_slice[0], old_slice[1]]
                    np.copyto(target, source, where=unchanged[None, :, :])
        new.frames, new.last_stamp = self.frames, self.last_stamp
        new.observation_stamps, new.observation_anchors = self.observation_stamps, self.observation_anchors
        self.__dict__.update(new.__dict__)
        return preserved

    def grow_to_include(self, points, sensor_origin, step_m=5., max_cells=1_000_000):
        """Expand in whole cells, preserving all hit/miss and height memory."""
        if not math.isfinite(step_m) or step_m <= 0 or max_cells <= 0:
            raise ValueError('Invalid map growth limits')
        sensor = np.asarray(sensor_origin, dtype=float)
        if sensor.shape != (3,) or not np.isfinite(sensor).all():
            raise ValueError('Invalid sensor origin')
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        valid = np.isfinite(points).all(axis=1)
        points = points[valid]
        points = points[np.linalg.norm(points-sensor, axis=1) <= self.max_range]
        xy = np.vstack((points[:, :2], sensor[None, :2]))
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        dx, dy = xy[:, 0]-self.origin[0], xy[:, 1]-self.origin[1]
        cols = np.floor((c*dx+s*dy)/self.resolution).astype(np.int64)
        rows = np.floor((-s*dx+c*dy)/self.resolution).astype(np.int64)
        height, width = self.data.shape
        step = max(1, math.ceil(step_m/self.resolution))
        pad = lambda n: max(0, math.ceil(int(n)/step)*step)
        left, right = pad(-cols.min()), pad(cols.max()-width+1)
        bottom, top = pad(-rows.min()), pad(rows.max()-height+1)
        if not any((left, right, bottom, top)):
            return False
        new_shape = height+bottom+top, width+left+right
        if math.prod(new_shape) > max_cells:
            raise MemoryError('Map growth exceeds configured cell capacity')
        padding = ((bottom, top), (left, right))
        expanded = {}
        for name, unknown in (('prior', -1), ('data', -1), ('column_state', -1),
                              ('evidence', 0), ('observed', False),
                              ('observed_stamp', -math.inf), ('hit_stamp', -math.inf),
                              ('low_stamp', -math.inf), ('low_misses', 0), ('low_observation', 0),
                              ('low_mixed_anchors', False)):
            expanded[name] = np.pad(getattr(self, name), padding, constant_values=unknown)
        for name in ('voxel_evidence', 'voxel_occupied', 'voxel_observation',
                     'voxel_hit_observation', 'voxel_mixed_anchors'):
            field = getattr(self, name).reshape(self.height_bins, height, width)
            field = np.pad(field, ((0, 0), *padding), constant_values=0)
            expanded[name] = field.reshape(self.height_bins, -1)
        self.__dict__.update(expanded)
        self.origin = (self.origin[0]-c*left*self.resolution+s*bottom*self.resolution,
                       self.origin[1]-s*left*self.resolution-c*bottom*self.resolution,
                       self.origin[2])
        return True

    def _indices(self, points):
        dx, dy = points[:, 0]-self.origin[0], points[:, 1]-self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        x = np.floor((c*dx+s*dy)/self.resolution).astype(int)
        y = np.floor((-s*dx+c*dy)/self.resolution).astype(int)
        h, w = self.data.shape
        return y*w+x, (x >= 0) & (x < w) & (y >= 0) & (y < h)

    def _clearance_columns(self, hits):
        # Endpoint/pose discretization is uncertain at cell boundaries. Do not
        # accumulate clearing immediately beside actual returns. This guard
        # applies to all obstacles, never expands occupied map cells, and goes
        # away when the physical object no longer supplies returns.
        h, w = self.data.shape
        ys, xs = np.divmod(hits, w)
        radius = math.ceil(self.ray_clearance/self.resolution)
        columns = []
        for dy in range(-radius, radius+1):
            for dx in range(-radius, radius+1):
                if math.hypot(dx, dy)*self.resolution > self.ray_clearance+1e-9:
                    continue
                x, y = xs+dx, ys+dy
                valid = (x >= 0) & (x < w) & (y >= 0) & (y < h)
                columns.append(y[valid]*w+x[valid])
        return np.unique(np.concatenate(columns))

    def integrate(self, points, sensor_origin, stamp, *, ground_plane=None, anchor_id=0):
        """One independent observation per cell/voxel and unique scan stamp."""
        if not math.isfinite(stamp) or stamp <= self.last_stamp:
            return {'accepted': False, 'reason': 'duplicate_or_old_scan'}
        origin = np.asarray(sensor_origin, dtype=float)
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        if origin.shape != (3,) or not np.isfinite(origin).all():
            raise ValueError('Invalid sensor origin')
        _, inside = self._indices(origin[None, :])
        if not inside[0]:
            return {'accepted': False, 'reason': 'sensor_outside_seed_map'}
        relative = points-origin
        ranges = np.linalg.norm(relative, axis=1)
        valid = (np.isfinite(points).all(axis=1) & (ranges >= 0.6) &
                 (ranges <= self.max_range))
        points, relative, ranges = points[valid], relative[valid], ranges[valid]
        if not len(points):
            return {'accepted': False, 'reason': 'no_valid_returns'}
        plane = ground_plane
        if plane is None:
            plane = (estimate_ground(points, origin[:2], self.ground_z) if self.adaptive_ground
                     else GroundPlane(0., 0., self.ground_z, 0, 0, 0.))
        if plane is None:
            return {'accepted': False, 'reason': 'ground_plane_unavailable'}
        heights = plane.heights(points)
        # A measured endpoint above the navigation band still proves the
        # traversed part of its beam free. Keep the ray; only mark endpoints
        # inside the obstacle band below. Never extrapolate past a return.
        valid = heights >= -0.08
        points, relative, ranges = points[valid], relative[valid], ranges[valid]
        if not len(points):
            return {'accepted': False, 'reason': 'no_valid_returns'}
        # Drop contradicted sensor returns before BOTH hit and free-ray
        # integration. Never reinterpret a rejected endpoint as free space.
        noise = (near_ground_noise(points, plane, self.obstacle_height,
            band_max=self.near_ground_band_max,
            support_radius=self.near_ground_support_radius)
            if self.near_ground_filter else np.zeros(len(points), bool))
        noise_count = int(noise.sum())
        points, relative, ranges = points[~noise], relative[~noise], ranges[~noise]
        if not len(points):
            return {'accepted': False, 'reason': 'no_supported_returns'}
        # Keep separate elevation rays. A far floor return and a near obstacle
        # at another elevation are both real observations, not one 2D laser ray.
        angle_step = min(self.resolution, self.height_resolution)/(4*self.max_range)
        angles = np.column_stack((np.floor(np.arctan2(relative[:, 1], relative[:, 0])/angle_step),
            np.floor(np.arctan2(relative[:, 2], np.linalg.norm(relative[:, :2], axis=1))/angle_step)))
        order = np.argsort(ranges, kind='stable')
        _, first = np.unique(angles[order], axis=0, return_index=True)
        points, relative, ranges = points[order[first]], relative[order[first]], ranges[order[first]]
        columns, inside = self._indices(points)
        heights = plane.heights(points)
        hit_mask = inside & (heights >= self.obstacle_height) & (heights <= self.max_height)
        hit_columns = np.unique(columns[hit_mask])
        clearing_guard = self._clearance_columns(hit_columns)
        levels = np.minimum(self.height_bins-1, np.floor(
            (heights[hit_mask]-self.obstacle_height)/self.height_resolution).astype(int))
        hit_voxels = np.unique(levels*self.data.size+columns[hit_mask])
        free_columns, free_voxels = [], []
        # Trace only the portion before a rising ray leaves the navigation
        # band. The real endpoint was already used for range validation and
        # angular visibility; the clipped point is never an obstacle hit.
        ray_relative, ray_ranges = relative, ranges
        origin_height = float(plane.heights(origin[None, :])[0])
        above = heights > self.max_height
        if origin_height < self.max_height and np.any(above):
            fraction = np.ones(len(points))
            fraction[above] = ((self.max_height-origin_height)/
                               (heights[above]-origin_height))
            ray_relative = relative*fraction[:, None]
            ray_ranges = ranges*fraction
        # Conservative subvoxel sampling: missed grazing cells get no evidence.
        # Samples never extend beyond an actual return. Endpoint voxels carrying
        # hits are removed below, including conflicting rays in the same scan.
        step = min(self.resolution, self.height_resolution)/2
        for start in range(0, len(points), 128):
            end = start+128
            lengths = ray_ranges[start:end]
            counts = np.ceil(lengths/step).astype(int)
            fractions = np.arange(int(counts.max())+1)[None, :]/counts[:, None]
            use = fractions <= 1.
            samples = origin+ray_relative[start:end, None, :]*fractions[:, :, None]
            samples = samples[use]
            ids, in_map = self._indices(samples)
            z = plane.heights(samples)
            low = in_map & (z >= -0.08) & (z <= self.obstacle_height)
            free_columns.append(np.unique(ids[low]))
            mid = in_map & (z >= self.obstacle_height) & (z < self.max_height)
            bins = np.floor((z[mid]-self.obstacle_height)/self.height_resolution).astype(int)
            free_voxels.append(np.unique(bins*self.data.size+ids[mid]))
        misses = np.setdiff1d(np.unique(np.concatenate(free_columns)), clearing_guard)
        voxel_misses = np.setdiff1d(np.unique(np.concatenate(free_voxels)), hit_voxels)
        voxel_misses = voxel_misses[~np.isin(voxel_misses % self.data.size, clearing_guard)]
        observation = len(self.observation_stamps)
        if observation >= np.iinfo(np.uint32).max:
            raise OverflowError('Observation identifier capacity exceeded')
        self.observation_stamps.append(float(stamp))
        self.observation_anchors.append(int(anchor_id))
        voxels = self.voxel_evidence.ravel()
        voxels[voxel_misses] = np.maximum(-3, voxels[voxel_misses]-1)
        voxels[hit_voxels] = np.minimum(9, voxels[hit_voxels]+3)
        touched_voxels = np.union1d(voxel_misses, hit_voxels)
        anchors = np.asarray(self.observation_anchors)
        previous = self.voxel_observation.ravel()[touched_voxels]
        self.voxel_mixed_anchors.ravel()[touched_voxels] |= (
            (previous != 0) & (anchors[previous] != anchor_id))
        self.voxel_observation.ravel()[touched_voxels] = observation
        self.voxel_hit_observation.ravel()[hit_voxels] = observation
        occupied = self.voxel_occupied.ravel()
        occupied[touched_voxels[voxels[touched_voxels] >= 6]] = True
        occupied[touched_voxels[voxels[touched_voxels] <= -3]] = False
        scores = self.evidence.ravel()
        scores[misses] = np.maximum(-6, scores[misses]-1)
        # Live obstacle returns are represented by height voxels only.
        # Also latching the unheighted column would keep a removed obstacle
        # occupied after every recorded height has been observed free, unless
        # a second beam happens to reach the floor in the same column.
        # Seed occupancy still requires actual low-ray evidence to clear.
        touched = np.union1d(misses, hit_columns)
        self.observed.ravel()[touched] = True
        self.observed_stamp.ravel()[touched] = stamp
        self.hit_stamp.ravel()[hit_columns] = stamp
        self.low_stamp.ravel()[misses] = stamp
        previous = self.low_observation.ravel()[misses]
        self.low_mixed_anchors.ravel()[misses] |= (previous != 0) & (anchors[previous] != anchor_id)
        self.low_observation.ravel()[misses] = observation
        low_misses = self.low_misses.ravel()
        low_misses[misses] = np.minimum(9, low_misses[misses]+1)
        low_misses[hit_columns] = 0
        state = self.column_state.ravel()
        state[touched[scores[touched] <= -3]] = 0
        state[touched[scores[touched] >= 6]] = 100
        previous = self.data.copy()
        self.data[:] = self.column_state
        self.data.ravel()[np.any(self.voxel_occupied, axis=0)] = 100
        self.frames += 1
        self.last_stamp = stamp
        return {'accepted': True, 'rays': len(points), 'free_evidence': len(misses),
                'hit_evidence': len(hit_columns), 'voxel_free_evidence': len(voxel_misses),
                'ground_plane': plane.summary(),
                'near_ground_filter_enabled': self.near_ground_filter,
                'near_ground_noise_rejected': noise_count,
                'changed_cells': int(np.count_nonzero(previous != self.data))}

    def statistics(self):
        unknown, seed_occupied = self.prior == -1, self.prior >= 65
        return {'accepted_scans': self.frames,
                'seed_unknown_cells': int(np.count_nonzero(unknown)),
                'unknown_cells': int(np.count_nonzero(self.data == -1)),
                'completed_free_cells': int(np.count_nonzero(unknown & (self.data == 0))),
                'completed_occupied_cells': int(np.count_nonzero(unknown & (self.data >= 65))),
                'seed_known_cells_changed': int(np.count_nonzero(~unknown & (self.data != self.prior))),
                'cleared_seed_obstacle_cells': int(np.count_nonzero(seed_occupied & (self.data == 0))),
                'new_obstacles_on_seed_free_cells': int(np.count_nonzero(
                    (self.prior >= 0) & (self.prior < 65) & (self.data >= 65))),
                'observed_cells': int(np.count_nonzero(self.observed)),
                'occupied_height_voxels': int(np.count_nonzero(self.voxel_occupied))}
