"""Check the installed backend's metadata ABI, including existing PM01 images."""
from pathlib import Path

BACKEND_ID = b'pm01-slam-2.6.10'


def require_backend(prefix):
    prefix = Path(prefix)
    executable = prefix/'lib/slam_toolbox/async_slam_toolbox_node'
    library = prefix/'lib/libtoolbox_common.so'
    # Check compiled code, not a source-tree marker that could exist even when
    # colcon skipped the package. Older PM01 bundles also expose this ABI.
    if (not executable.is_file() or not library.is_file() or
            BACKEND_ID not in library.read_bytes()):
        raise RuntimeError(
            f'SLAM backend at {prefix} lacks PM01 map revision support. '
            'Run bash scripts/build.sh sim (or hardware), then reload scripts/env.sh. '
            'The system slam_toolbox package cannot provide the navigation map.')
    return prefix
