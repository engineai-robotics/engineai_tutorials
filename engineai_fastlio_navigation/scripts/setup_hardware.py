#!/usr/bin/env python3
from _bootstrap import workspace
from pm01_navigation_runtime.deployment.setup_hardware import main

if __name__ == "__main__":
    raise SystemExit(main(workspace()))
