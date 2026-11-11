"""Estimate a nearly horizontal indoor ground plane from lidar, not truth."""
from dataclasses import dataclass
import math

import numpy as np


def near_ground_noise(points, plane, obstacle_height, *, band_max=.06,
                      support_radius=.20):
    """Reject near-floor returns contradicted by a dense local floor surface.

    This is a sensor-return decision, never a map-cell/component deletion.
    Search at most 12 spatially nearest distinct returns within 20 cm. Reject
    a 2.5--6 cm candidate only with at least 6 actual floor neighbours and
    insufficient coherent elevated support. Sparse/ambiguous scenes and every
    return above the near-floor band are preserved. Rejected rays must not be
    reused as free-space evidence by callers.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if support_radius <= 0 or not np.isfinite([band_max, support_radius]).all():
        raise ValueError('Invalid near-ground support limits')
    heights = plane.heights(points)
    reject = np.zeros(len(points), dtype=bool)
    candidates = np.flatnonzero((heights >= obstacle_height) & (heights < band_max))
    pool = points[np.isfinite(points).all(axis=1) & (heights >= -.02) & (heights < band_max+.015)]
    pool = np.unique(pool, axis=0)
    if not len(candidates) or len(pool) < 7:
        return reject
    pool_heights = plane.heights(pool)
    count = min(13, len(pool))  # includes the query itself when it is in pool
    for start in range(0, len(candidates), 64):
        ids = candidates[start:start+64]
        delta = points[ids, None, :2]-pool[None, :, :2]
        distances = np.sum(delta*delta, axis=2)
        nearest = np.argpartition(distances, count-1, axis=1)[:, :count]
        distance = np.take_along_axis(distances, nearest, axis=1)
        nh = pool_heights[nearest]
        # Exclude the identical return so duplicating it cannot manufacture
        # support. Distinct returns at the same XY but another height count.
        identical = (distance < 1e-16) & (np.abs(nh-heights[ids, None]) < 1e-9)
        valid = (distance <= support_radius**2) & ~identical
        floor = valid & (np.abs(nh) <= .012)
        elevated = valid & (nh >= obstacle_height) & (np.abs(nh-heights[ids, None]) <= .012)
        floor_count = floor.sum(axis=1)
        elevated_count = elevated.sum(axis=1)
        coherent = (elevated_count >= 3) & (elevated_count >= .5*valid.sum(axis=1))
        reject[ids] = (floor_count >= 6) & ~coherent
    return reject


@dataclass(frozen=True)
class GroundPlane:
    a: float
    b: float
    c: float
    inliers: int
    candidates: int
    rms: float

    def heights(self, points):
        return points[:, 2]-(self.a*points[:, 0]+self.b*points[:, 1]+self.c)

    def summary(self):
        return {'a': self.a, 'b': self.b, 'c': self.c, 'inliers': self.inliers,
                'candidates': self.candidates, 'rms_m': self.rms,
                'tilt_deg': math.degrees(math.atan(math.hypot(self.a, self.b)))}


def estimate_ground(points, centre, expected_z, *, radius=5., min_inliers=100):
    """RANSAC plus inlier refit, balanced across XY tiles.

    Search near the configured flat-ground datum, enforce <=5 degree tilt and
    <=8 cm offset at the robot, and fail explicitly without sufficient support.
    A tile quantile avoids letting a dense box surface dominate near-field data.
    """
    points = np.asarray(points, float).reshape(-1, 3)
    centre = np.asarray(centre, float)[:2]
    valid = np.isfinite(points).all(axis=1)
    distance = np.linalg.norm(points[:, :2]-centre, axis=1)
    valid &= (distance >= .5) & (distance <= radius) & (np.abs(points[:, 2]-expected_z) < .15)
    candidates = points[valid]
    if len(candidates) < min_inliers:
        return None
    tiles = np.floor((candidates[:, :2]-centre)/.20).astype(int)
    keys = tiles[:, 0]*10000+tiles[:, 1]
    order = np.lexsort((candidates[:, 2], keys))
    _, starts, counts = np.unique(keys[order], return_index=True, return_counts=True)
    # Mixed floor/wall tiles need a low seed; all spatial/refit checks remain.
    balanced = candidates[order[starts+counts//10]]
    if len(balanced) < 30:
        return None
    xy = balanced[:, :2]-centre
    design = np.c_[xy, np.ones(len(xy))]
    rng = np.random.default_rng(41)
    # Three scalar columns avoid BLAS thread startup on every candidate plane.
    def predicted(matrix, fit):
        return matrix[:, 0]*fit[0]+matrix[:, 1]*fit[1]+fit[2]

    best, best_count = None, 0
    for ids in rng.integers(0, len(balanced), size=(128, 3)):
        try:
            fit = np.linalg.solve(design[ids], balanced[ids, 2])
        except np.linalg.LinAlgError:
            continue
        if math.hypot(*fit[:2]) > math.tan(math.radians(5)) or abs(fit[2]-expected_z) > .08:
            continue
        mask = np.abs(predicted(design, fit)-balanced[:, 2]) <= .010
        if int(mask.sum()) > best_count:
            best, best_count = fit, int(mask.sum())
    if best is None or best_count < max(30, .25*len(balanced)):
        return None
    full_design = np.c_[candidates[:, :2]-centre, np.ones(len(candidates))]
    mask = np.abs(predicted(full_design, best)-candidates[:, 2]) <= .010
    for _ in range(2):
        if int(mask.sum()) < min_inliers:
            return None
        fit = np.linalg.lstsq(full_design[mask], candidates[mask, 2], rcond=None)[0]
        mask = np.abs(predicted(full_design, fit)-candidates[:, 2]) <= .010
    if (int(mask.sum()) < min_inliers or abs(fit[2]-expected_z) > .08 or
            math.hypot(*fit[:2]) > math.tan(math.radians(5))):
        return None
    residual = predicted(full_design[mask], fit)-candidates[mask, 2]
    return GroundPlane(float(fit[0]), float(fit[1]),
        float(fit[2]-np.dot(fit[:2], centre)), int(mask.sum()), len(candidates),
        float(np.sqrt(np.mean(residual**2))))
