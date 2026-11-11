#!/usr/bin/env bash
# First-use preparation only: never starts sensors, navigation, or motion output.
set -eo pipefail
pm01_setup_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
exec /usr/bin/python3 "$pm01_setup_root/scripts/setup_hardware.py" "$@"
