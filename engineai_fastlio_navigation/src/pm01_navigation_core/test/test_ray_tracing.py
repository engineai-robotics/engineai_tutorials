import numpy as np
import pytest

from pm01_navigation_core.ground.flat_ground import GroundPlane
from pm01_navigation_core.mapping import ray_tracing
from pm01_navigation_core.mapping.online_occupancy import OccupancyUpdater


@pytest.mark.parametrize('yaw', [0., .27])
def test_free_cells_match_reference(yaw, monkeypatch):
    if ray_tracing.trace_rays is None:
        pytest.skip('requires installed CPU extension')
    monkeypatch.setenv('PM01_RAY_BACKEND', 'cpu')
    grid = OccupancyUpdater(np.full((37, 41), -1), .1, (-1., -1.2, yaw))
    plane = GroundPlane(0., 0., 0., 0, 0, 0.)
    origin = np.array([.0, .0, .12])
    rays = np.vstack(([1., 0., 0.], [0., 1., 1.68], [-1., -1., -.2],
                      np.random.default_rng(9).uniform(-2.5, 2.5, (300, 3))))
    ranges = np.linalg.norm(rays, axis=1)
    low, mid, backend = ray_tracing.trace_free(grid, rays, ranges, origin, plane)
    expected = [np.zeros_like(a) for a in (low, mid)]
    ray_tracing.reference(grid, rays, ranges, origin, plane, *expected)
    assert backend == 'cpu'
    for got, want in zip((low, mid), expected):
        np.testing.assert_array_equal(got, want)


def test_cuda_failure_recomputes_on_cpu(monkeypatch):
    from pm01_navigation_core.mapping.cuda_rays import CudaError

    class FailedDevice:
        def trace(self, *args):
            raise CudaError('device unavailable')

        def close(self):
            pass

    monkeypatch.setenv('PM01_RAY_BACKEND', 'auto')
    monkeypatch.setattr(ray_tracing, '_cuda', FailedDevice())
    monkeypatch.setattr(ray_tracing, '_try_cuda', True)
    grid = OccupancyUpdater(np.full((16, 16), -1), .1, (0, 0, 0))
    origin = np.array([.1, .1, .8])
    plane = GroundPlane(0., 0., 0., 0, 0, 0.)
    rays = np.array([[1., .2, -.75]])
    lengths = np.linalg.norm(rays, axis=1)
    low, mid, backend = ray_tracing.trace_free(grid, rays, lengths, origin, plane)
    reference = [np.zeros_like(a) for a in (low, mid)]
    ray_tracing.reference(grid, rays, lengths, origin, plane, *reference)
    assert backend in ('numpy', 'cpu') and not ray_tracing._try_cuda
    for a, b in zip((low, mid), reference):
        np.testing.assert_array_equal(a, b)
