#!/usr/bin/env python3
from pathlib import Path
import sys
from _bootstrap import workspace

source = workspace()/'src/pm01_navigation_client'
if source.is_dir():
    sys.path.insert(0, str(source))
from pm01_navigation_client.diagnostics.mapping_recording import main

if __name__ == '__main__':
    raise SystemExit(main())
