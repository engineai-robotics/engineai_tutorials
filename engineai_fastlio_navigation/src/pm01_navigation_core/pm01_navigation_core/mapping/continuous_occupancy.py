"""Map-frame evidence with independently anchored graph-optimization updates.

Ordinary map->odom corrections never move previously measured hit/free evidence.
Unknown cells never supply clearing. Five footprint samples conservatively
require free support across a destination cell, rather than copying one free
source pixel across an unobserved boundary.
"""
import math
import time

import numpy as np

from pm01_navigation_core.ground.flat_ground import GroundPlane
from pm01_navigation_core.mapping.evidence_anchors import reproject, centres
from pm01_navigation_core.mapping.online_occupancy import OccupancyUpdater, empty_grid


class ContinuousOccupancy:
    def __init__(self, data, resolution, origin, *, initial_size=30.,
                 max_cells=1_000_000, growth_step=5., require_anchors=False, **options):
        self.prior = np.asarray(data, np.int16).copy()
        if (self.prior.ndim != 2 or not self.prior.size or self.prior.size > max_cells or
                len(origin) != 3 or not np.isfinite([resolution, *origin]).all() or resolution <= 0 or
                np.any((self.prior < -1) | (self.prior > 100))):
            raise ValueError('Invalid seed map geometry or contents')
        self.data = self.prior.copy()
        self.resolution, self.origin = float(resolution), tuple(origin)
        self.max_cells, self.growth_step = max_cells, growth_step
        empty, live_origin = empty_grid(resolution, initial_size, max_cells)
        self.live = OccupancyUpdater(empty, resolution, live_origin, **options)
        self.options = options
        self.cutoff = np.full(self.prior.shape, -math.inf)
        self.correction = (0., 0., 0., 0.)
        self.anchors = {} if require_anchors else {0: (0., 0., 0.)}
        self.reproject_wall_sec = 0.
        self.optimizations = 0
        self.projections = 0
        self.projection_wall_sec = 0.
        self.retained_cells = 0
        self._projection_key = None
        self._projection_indices = []
        self._retained = np.zeros(self.prior.shape, bool)

    @property
    def frames(self):
        return self.live.frames

    @property
    def last_stamp(self):
        return self.live.last_stamp

    def cell(self, x, y):
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        dx, dy = x-self.origin[0], y-self.origin[1]
        return math.floor((c*dx+s*dy)/self.resolution), math.floor((-s*dx+c*dy)/self.resolution)

    def register_anchor(self, anchor_id, pose):
        if anchor_id < 0 or len(pose) != 3 or not np.isfinite(pose).all():
            raise ValueError('Invalid observation anchor')
        pose = tuple(float(v) for v in pose)
        if anchor_id in self.anchors and not np.allclose(self.anchors[anchor_id], pose, atol=1e-9, rtol=0):
            raise ValueError('Anchor pose changed without a graph optimization')
        self.anchors[anchor_id] = pose

    def _sample(self, points, data, resolution, origin, default=-1):
        ids, valid = self._indices(points[:, 0], points[:, 1], data.shape, resolution, origin)
        result = np.full(len(points), default, dtype=data.dtype)
        result[valid] = data.ravel()[ids[valid]]
        return result

    def _seed_on_published_grid(self, seed, origin):
        # Sample the new seed at cell centres of the published grid. Equal
        # spacing and orientation make this a single integer offset on each axis.
        yaw = self.origin[2]
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = origin[0]-self.origin[0], origin[1]-self.origin[1]
        col = -math.floor(.5 - (c*dx+s*dy)/self.resolution)
        row = -math.floor(.5 - (-s*dx+c*dy)/self.resolution)
        left, bottom = min(0, col), min(0, row)
        height = max(self.prior.shape[0], row+seed.shape[0])-bottom
        width = max(self.prior.shape[1], col+seed.shape[1])-left
        if height*width > self.max_cells:
            # Drop unused padding before rejecting a map for lack of space.
            return seed, origin, None
        if (height, width) == seed.shape:
            view = seed
        else:
            view = np.full((height, width), -1, np.int16)
            y, x = row-bottom, col-left
            view[y:y+seed.shape[0], x:x+seed.shape[1]] = seed
        new_origin = (self.origin[0]+(c*left-s*bottom)*self.resolution,
                      self.origin[1]+(s*left+c*bottom)*self.resolution, yaw)
        old_region = np.s_[-bottom:self.prior.shape[0]-bottom, -left:self.prior.shape[1]-left]
        return view, new_origin, old_region

    def rebase(self, data, resolution, origin, stamp=None, map_from_odom=None, *, optimized_anchors=None):
        new = np.asarray(data, np.int16)
        if (new.ndim != 2 or not new.size or new.size > self.max_cells or resolution <= 0 or
                len(origin) != 3 or not np.isfinite([resolution, *origin]).all() or
                np.any((new < -1) | (new > 100))):
            raise ValueError('Invalid optimized map geometry')
        old_region = None
        old_prior, old_cutoff = self.prior, self.cutoff
        old_resolution, old_origin = self.resolution, self.origin
        if optimized_anchors is not None:
            started = time.monotonic()
            self.live = reproject(self.live, self.anchors, optimized_anchors,
                lambda p: self._sample(p, old_prior, old_resolution, old_origin),
                lambda p: self._sample(p, new, resolution, origin),
                lambda p: self._sample(p, old_cutoff, old_resolution, old_origin, -math.inf),
                self.max_cells, cutoff_grid=(old_cutoff, old_resolution, old_origin))
            self.reproject_wall_sec = time.monotonic()-started
            self.anchors = dict(optimized_anchors)
            self.optimizations += 1
        elif resolution == self.resolution and origin[2] == self.origin[2]:
            new, origin, old_region = self._seed_on_published_grid(new, origin)
        cutoff = np.full(new.shape, -math.inf)
        if old_region is not None:
            # Both rasters use the same indices here, including existing padding.
            target = new[old_region]
            changed = (target >= 0) & ((old_prior < 0) | ((old_prior >= 65) != (target >= 65)))
            cut = cutoff[old_region]
            cut[:] = old_cutoff
            cut[changed] = np.maximum(cut[changed], self.last_stamp if stamp is None else float(stamp))
        elif optimized_anchors is None:
            for rows, xx, yy in self._centres(new.shape, resolution, origin):
                ids, inside = self._indices(xx, yy, old_prior.shape, old_resolution, old_origin)
                old = np.full(ids.shape, -1, np.int16)
                old[inside] = old_prior.ravel()[ids[inside]]
                cut = cutoff.ravel()[rows]
                cut[inside] = old_cutoff.ravel()[ids[inside]]
                classify = lambda a: np.where(a < 0, -1, np.where(a >= 65, 1, 0))
                changed = inside & (new.ravel()[rows] >= 0) & (classify(old) != classify(new.ravel()[rows]))
                cut[changed] = np.maximum(cut[changed], self.last_stamp if stamp is None else float(stamp))
        self.prior, self.cutoff = new.copy(), cutoff
        self.resolution, self.origin = float(resolution), tuple(origin)
        # A periodic seed may shrink. Previously observed coverage is still
        # part of the output canvas, and unobserved added cells stay unknown.
        self._include_observed()
        self.project(self.correction if map_from_odom is None else map_from_odom)
        return self.retained_cells

    def _include_observed(self):
        rows, cols = np.nonzero(self.live.observed)
        if not len(rows):
            return
        columns = np.array([r*self.live.data.shape[1]+c for r in (rows.min(), rows.max())
                            for c in (cols.min(), cols.max())])
        xy = centres(self.live, columns)
        self._grow_canvas(xy)

    def _grow_canvas(self, points):
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        points = np.asarray(points).reshape(-1, 2)
        dx, dy = points[:, 0]-self.origin[0], points[:, 1]-self.origin[1]
        cols = np.floor((c*dx+s*dy)/self.resolution).astype(int)
        rows = np.floor((-s*dx+c*dy)/self.resolution).astype(int)
        step = max(1, math.ceil(self.growth_step/self.resolution))
        pad = lambda v: max(0, math.ceil(int(v)/step)*step)
        h, w = self.prior.shape
        left, right, bottom, top = pad(-cols.min()), pad(cols.max()-w+1), pad(-rows.min()), pad(rows.max()-h+1)
        if not any((left, right, bottom, top)):
            return False
        if (h+bottom+top)*(w+left+right) > self.max_cells:
            raise MemoryError('Map growth exceeds configured cell capacity')
        padding = ((bottom, top), (left, right))
        self.prior = np.pad(self.prior, padding, constant_values=-1)
        self.cutoff = np.pad(self.cutoff, padding, constant_values=-math.inf)
        self.origin = (self.origin[0]-c*left*self.resolution+s*bottom*self.resolution,
                       self.origin[1]-s*left*self.resolution-c*bottom*self.resolution, self.origin[2])
        return True

    @staticmethod
    def _centres(shape, resolution, origin):
        width = shape[1]
        c, s = math.cos(origin[2]), math.sin(origin[2])
        for start in range(0, math.prod(shape), 32768):
            end = min(start+32768, math.prod(shape))
            rows = slice(start, end)
            y, x = np.divmod(np.arange(start, end), width)
            x, y = (x+.5)*resolution, (y+.5)*resolution
            yield rows, origin[0]+c*x-s*y, origin[1]+s*x+c*y

    @staticmethod
    def _indices(x, y, shape, resolution, origin):
        c, s = math.cos(origin[2]), math.sin(origin[2])
        dx, dy = x-origin[0], y-origin[1]
        col = np.floor((c*dx+s*dy)/resolution).astype(np.int64)
        row = np.floor((-s*dx+c*dy)/resolution).astype(np.int64)
        return row*shape[1]+col, (col >= 0) & (row >= 0) & (col < shape[1]) & (row < shape[0])

    def project(self, correction, touched=None):
        began = time.monotonic()
        if not np.isfinite(correction).all() or len(correction) != 4:
            raise ValueError('Expected finite planar map->odom correction')
        self.correction = tuple(float(v) for v in correction)
        key = (self.prior.shape, self.resolution, self.origin,
               self.live.data.shape, self.live.resolution, self.live.origin)
        changed_geometry = key != self._projection_key
        if changed_geometry:
            self._projection_key = key
            self._projection_indices = []
            mc, ms = math.cos(self.origin[2]), math.sin(self.origin[2])
            for ox, oy in ((0., 0.), (-.49999, -.49999), (-.49999, .49999),
                           (.49999, -.49999), (.49999, .49999)):
                ids = np.full(self.prior.size, -1, np.int64)
                for rows, mx, my in self._centres(self.prior.shape, self.resolution, self.origin):
                    src, inside = self._indices(mx+self.resolution*(mc*ox-ms*oy),
                        my+self.resolution*(ms*ox+mc*oy), self.live.data.shape,
                        self.live.resolution, self.live.origin)
                    ids[rows] = np.where(inside, src, -1)
                self._projection_indices.append(ids)
        full = touched is None or changed_geometry or self.data.shape != self.prior.shape
        if full:
            rows = np.arange(self.prior.size)
            result = self.prior.copy()
            self._retained = np.zeros(self.prior.shape, bool)
        else:
            xy = centres(self.live, touched)
            if not len(xy):
                return
            c, si = math.cos(self.origin[2]), math.sin(self.origin[2])
            dx, dy = xy[:, 0]-self.origin[0], xy[:, 1]-self.origin[1]
            cols = np.floor((c*dx+si*dy)/self.resolution).astype(int)
            rr = np.floor((-si*dx+c*dy)/self.resolution).astype(int)
            pad = 2+math.ceil(self.live.resolution/self.resolution)
            left, right = max(0, cols.min()-pad), min(self.prior.shape[1], cols.max()+pad+1)
            bottom, top = max(0, rr.min()-pad), min(self.prior.shape[0], rr.max()+pad+1)
            rows = (np.arange(bottom, top)[:, None]*self.prior.shape[1]+
                    np.arange(left, right)[None, :]).ravel()
            result = self.data.copy()
            result.ravel()[rows] = self.prior.ravel()[rows]
        for start in range(0, len(rows), 32768):
            dst = rows[start:start+32768]
            free, strong_free = np.ones(len(dst), bool), np.ones(len(dst), bool)
            occupied, observed = np.zeros(len(dst), bool), np.zeros(len(dst), bool)
            cutoff = self.cutoff.ravel()[dst]
            for indices in self._projection_indices:
                ids = indices[dst]
                inside = ids >= 0
                valid, src = np.flatnonzero(inside), ids[inside]
                sf, ss = np.zeros(len(dst), bool), np.zeros(len(dst), bool)
                values = self.live.data.ravel()[src]
                sf[valid] = (values == 0) & (self.live.low_stamp.ravel()[src] > cutoff[valid])
                ss[valid] = sf[valid] & (self.live.low_misses.ravel()[src] >= 9)
                occupied[valid] |= (values >= 65) & (self.live.hit_stamp.ravel()[src] > cutoff[valid])
                observed[valid] |= self.live.observed.ravel()[src] & (self.live.observed_stamp.ravel()[src] > cutoff[valid])
                free &= sf
                strong_free &= ss
            target = result.ravel()[dst].copy()
            target[free & ((target < 65) | strong_free)] = 0
            target[occupied] = 100
            result.ravel()[dst] = target
            self._retained.ravel()[dst] = observed
        self.data = result
        self.retained_cells = int(np.count_nonzero(self._retained))
        self.projections += 1
        self.projection_wall_sec = time.monotonic()-began

    def integrate(self, points, sensor_origin, stamp, *, ground_plane=None,
                  map_from_odom=(0., 0., 0., 0.), anchor_id=0):
        if anchor_id not in self.anchors:
            return {'accepted': False, 'reason': 'observation_anchor_unavailable'}
        # Points and the shared plane already use this observation's accepted
        # map alignment. Keep historical evidence at those map coordinates.
        points = np.asarray(points).reshape(-1, 3)
        origin = np.asarray(sensor_origin)
        if ground_plane is None and not self.live.adaptive_ground:
            ground_plane = GroundPlane(0., 0., self.options.get('ground_z', 0.), 0, 0, 0.)
        self.live.grow_to_include(points, origin, self.growth_step, self.max_cells)
        result = self.live.integrate(points, origin, stamp, ground_plane=ground_plane, anchor_id=anchor_id)
        if result['accepted']:
            self._include_observed()
            self.project(map_from_odom, touched=self.live.last_touched)
        return result

    def grow_to_include(self, points, sensor_origin, step_m=5., max_cells=1_000_000):
        changed = self._grow_canvas(np.vstack((np.asarray(points).reshape(-1, 3), sensor_origin))[:, :2])
        if changed:
            self.project(self.correction)
        return changed

    def statistics(self):
        unknown, occupied = self.prior < 0, self.prior >= 65
        return {**self.live.statistics(), 'accepted_scans': self.frames,
                'seed_unknown_cells': int(np.sum(unknown)),
                'seed_known_cells_changed': int(np.sum(~unknown & (self.data != self.prior))),
                'unknown_cells': int(np.sum(self.data < 0)),
                'cleared_seed_obstacle_cells': int(np.sum(occupied & (self.data == 0))),
                'completed_free_cells': int(np.sum(unknown & (self.data == 0))),
                'completed_occupied_cells': int(np.sum(unknown & (self.data >= 65))),
                'new_obstacles_on_seed_free_cells': int(np.sum((self.prior >= 0) &
                    (self.prior < 65) & (self.data >= 65))),
                'evidence_frame': 'map', 'evidence_anchor_count': len(self.anchors),
                'evidence_optimizations': self.optimizations, 'map_projections': self.projections,
                'retained_observed_cells': self.retained_cells,
                'projection_wall_sec': self.projection_wall_sec}
