#!/usr/bin/env python3
"""Run project tests from source; pass paths and options through to pytest."""
import hashlib
import os
from pathlib import Path
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    for package in sorted((root/'src').glob('pm01_navigation_*')):
        if (package/package.name).is_dir():
            sys.path.insert(0, str(package))
    import pytest
    os.chdir(root)
    cache = Path.home()/'.cache/pm01_navigation/pytest'/hashlib.sha256(str(root).encode()).hexdigest()[:12]
    return pytest.main(['-c', str(root/'tests/pytest.ini'), '--rootdir', str(root),
                        '-o', 'cache_dir='+str(cache), *sys.argv[1:]])


if __name__ == '__main__':
    raise SystemExit(main())
