"""Mark measured free space; every backend uses the same subvoxel spacing."""
import atexit
import logging
import math
import os
from threading import Lock

import numpy as np

try:
    from _pm01_evidence import trace_rays
except ImportError:
    trace_rays = None

_lock = Lock()
_cuda = None
_try_cuda = True


def trace_free(grid, rays, lengths, origin, plane):
    global _cuda, _try_cuda
    h, w = grid.data.shape
    bins = grid.height_bins
    counts = np.ceil(lengths/(min(grid.resolution, grid.height_resolution)/2)).astype(np.int32)
    params = np.array([*origin, *grid.origin[:2], math.cos(grid.origin[2]), math.sin(grid.origin[2]),
                       grid.resolution, plane.a, plane.b, plane.c, grid.obstacle_height,
                       grid.max_height, grid.height_resolution], dtype=np.float64)
    rays = np.ascontiguousarray(rays, dtype=np.float64)
    if _try_cuda and os.environ.get('PM01_RAY_BACKEND', 'auto') != 'cpu':
        with _lock:
            if _try_cuda:
                from .cuda_rays import CudaRays, CudaError
                try:
                    if _cuda is None:
                        _cuda = CudaRays()
                        atexit.register(_cuda.close)
                    lo, mid = _cuda.trace(rays, counts, params, w, h, bins)
                    return lo, mid, 'cuda'
                except (OSError, CudaError, AttributeError) as exc:
                    _try_cuda = False
                    logging.getLogger(__name__).info('Using CPU for map rays: %s', exc)
                    if _cuda is not None:
                        try:
                            _cuda.close()
                        except CudaError:
                            pass
    low = np.zeros(w*h, dtype=bool)
    mid = np.zeros(w*h*bins, dtype=bool)
    if trace_rays is not None:
        trace_rays(rays, counts, params, w, h, bins, low, mid)
        return low, mid, 'cpu'
    reference(grid, rays, lengths, origin, plane, low, mid)
    return low, mid, 'numpy'


def reference(grid, ray_relative, ray_ranges, origin, plane, free_columns, free_voxels):
    step = min(grid.resolution, grid.height_resolution)/2
    for start in range(0, len(ray_ranges), 128):
        end = start+128
        lengths = ray_ranges[start:end]
        counts = np.ceil(lengths/step).astype(int)
        fractions = np.arange(int(counts.max())+1)[None, :]/counts[:, None]
        use = fractions <= 1.
        samples = origin+ray_relative[start:end, None, :]*fractions[:, :, None]
        samples = samples[use]
        ids, in_map = grid._indices(samples)
        z = plane.heights(samples)
        low = in_map & (z >= -0.08) & (z <= grid.obstacle_height)
        free_columns[ids[low]] = True
        mid = in_map & (z >= grid.obstacle_height) & (z < grid.max_height)
        bins = np.floor((z[mid]-grid.obstacle_height)/grid.height_resolution).astype(int)
        free_voxels[bins*grid.data.size+ids[mid]] = True
