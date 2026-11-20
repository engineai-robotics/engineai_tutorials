#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /opt/pm01_navigation/install/local_setup.bash
export LD_LIBRARY_PATH="/opt/pm01_navigation/.deps/prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
exec "$@"
