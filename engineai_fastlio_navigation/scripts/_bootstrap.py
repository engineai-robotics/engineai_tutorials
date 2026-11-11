"""Locate the single runtime implementation before colcon or in an offline bundle."""
from pathlib import Path
import sys

_directory = Path(__file__).resolve().parent
_workspace = _directory.parent if _directory.name == 'scripts' else _directory
_source = _workspace/'src/pm01_navigation_runtime'
if (_source/'pm01_navigation_runtime').is_dir():
    sys.path.insert(0, str(_source))
elif (_directory/'pm01_navigation_runtime').is_dir():
    sys.path.insert(0, str(_directory))

def workspace():
    return _workspace
