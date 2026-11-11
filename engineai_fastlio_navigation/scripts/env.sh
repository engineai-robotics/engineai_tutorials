#!/usr/bin/env bash
# source scripts/env.sh; set PM01_DOMAIN_ID before sourcing to isolate experiments.
pm01_project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ! -f "$pm01_project_root/install/setup.bash" ]]; then echo 'Run scripts/build.sh first' >&2; return 1; fi
source /opt/ros/humble/setup.bash || return 1
if [[ -f /usr/share/gazebo/setup.sh ]]; then source /usr/share/gazebo/setup.sh; fi
source "$pm01_project_root/install/local_setup.bash" || return 1
export CMAKE_PREFIX_PATH="$pm01_project_root/.deps/prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$pm01_project_root/.deps/prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
# Last assignment wins: environment overlays cannot silently switch this project.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
export ROS_DOMAIN_ID=${PM01_DOMAIN_ID:-184}
export ROS_LOCALHOST_ONLY=${PM01_LOCALHOST_ONLY:-1}
export GAZEBO_MASTER_URI=${PM01_GAZEBO_MASTER_URI:-http://127.0.0.1:11378}
export GAZEBO_MODEL_DATABASE_URI=
export PYTHONUNBUFFERED=1
export PM01_USE_SIM_TIME=${PM01_USE_SIM_TIME:-true}
if [[ ${1:-} == --config ]]; then
 if [[ -z ${2:-} ]]; then echo 'Usage: source scripts/env.sh --config FILE' >&2; return 2; fi
 pm01_deployment_values=$(python3 -m pm01_navigation_runtime.system.environment "$2") || return 1
 unset CYCLONEDDS_URI FASTDDS_DEFAULT_PROFILES_FILE
 while IFS= read -r pm01_setting; do export "$pm01_setting"; done <<< "$pm01_deployment_values"
 unset pm01_setting pm01_deployment_values
elif [[ ${PM01_LOCALHOST_ONLY:-1} == 1 ]]; then
 # Explicit loopback UDP avoids relying on stale/shared SHM transport state.
 # ROS_LOCALHOST_ONLY stays 0 because isolation is enforced by this profile.
 unset FASTDDS_DEFAULT_PROFILES_FILE
 pm01_loopback_profile=$(python3 -m pm01_navigation_runtime.system.network --loopback) || return 1
 export FASTRTPS_DEFAULT_PROFILES_FILE="$pm01_loopback_profile"
 export ROS_LOCALHOST_ONLY=0
 unset pm01_loopback_profile
fi
unset pm01_project_root
