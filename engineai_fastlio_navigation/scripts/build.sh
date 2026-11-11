#!/usr/bin/env bash
set -euo pipefail
mode=${1:-sim}
stage=${2:-all}
root=$(cd "$(dirname "$0")/.." && pwd)
case "$mode" in sim|hardware|desktop|core-client) ;; *) echo 'Usage: build.sh [sim|hardware|desktop|core-client]' >&2; exit 2;; esac
case "$stage" in all|sdk|driver|lio|navigation) ;; *) echo 'Invalid build stage' >&2; exit 2;; esac
if [[ $stage != all && $mode != hardware ]]; then echo 'Individual stages are hardware-only' >&2; exit 2; fi
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'First run scripts/install_dependencies.sh' >&2; exit 2; }
# ROS shell setup may access unset variables. Set the transport AFTER all overlays.
set +u; source /opt/ros/humble/setup.bash; set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
export CMAKE_BUILD_PARALLEL_LEVEL=${PM01_BUILD_JOBS:-2}
export MAKEFLAGS="-j${CMAKE_BUILD_PARALLEL_LEVEL}"
cd "$root"
manifest() {
 python3 - "$root" "$mode" "$@" <<'MANIFEST'
import sys
from pathlib import Path
root=Path(sys.argv[1]); sys.path.insert(0,str(root/'src/pm01_navigation_runtime'))
from pm01_navigation_runtime.build.manifest import main
sys.argv=sys.argv[1:]; main(root)
MANIFEST
}
manifest --prepare-layout --stage "$stage" >/dev/null
/usr/bin/python3 - "$root" <<'CONFIG_META'
import json, sys
from pathlib import Path
root=Path(sys.argv[1]); (root/'build').mkdir(exist_ok=True)
(root/'build/pm01-config.meta').write_text(json.dumps({'names': {
    'pm01_navigation_bringup': {'cmake-args': ['-DPM01_CONFIG_ROOT='+str(root/'config')]}}}))
CONFIG_META
if [[ $mode == desktop || $mode == core-client ]]; then
 mapfile -t paths < <(manifest)
 colcon build --base-paths "${paths[@]}" --metas "$root/build/pm01-config.meta" --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
 exit
fi
# Bind legacy ROS interface generation and modern CMake targets to the same
# system Python. Do not guess a version or inherit a separate Python environment.
python_config=$(/usr/bin/python3 - <<'PY'
from pathlib import Path
import sysconfig
library = Path(sysconfig.get_config_var('LIBDIR'))/sysconfig.get_config_var('LDLIBRARY')
include = Path(sysconfig.get_path('include'))
if not library.is_file() or not (include/'Python.h').is_file():
    raise SystemExit('System Python development files missing; install the selected build dependencies first')
print(library)
print(include)
PY
)
mapfile -t python_paths <<< "$python_config"
python_cmake_args=(-DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
 "-DPYTHON_LIBRARY=${python_paths[0]}" "-DPYTHON_INCLUDE_DIR=${python_paths[1]}")
if [[ $mode == sim && $(uname -m) != x86_64 ]]; then echo 'sim requires x86_64 (bundled MNN ABI)' >&2; exit 2; fi
if [[ $mode == sim ]]; then
 # Remove retired simulation-only aiding from previous isolated installs.
 rm -f install/pm01_navigation_sim/lib/pm01_navigation_sim/pm01_leg_velocity.py
 for module in install/pm01_navigation_sim/local/lib/python*/dist-packages/pm01_navigation_sim install/pm01_navigation_sim/lib/python*/site-packages/pm01_navigation_sim; do
   rm -rf "$module/estimation"
   rm -f "$module/nodes/leg_velocity.py"
 done
 python3 scripts/fetch_sim_dependencies.py
 sdk="$root/.deps/sim_assets/livox_sdk2"
else
 python3 scripts/fetch_hardware_dependencies.py --verify-only
 sdk="$root/.deps/hardware/livox_sdk2"
fi
prefix="$root/.deps/prefix"
export CMAKE_PREFIX_PATH="$prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
# Resolve SDK layout from the pinned repository, never a private /opt installation.
if [[ ! -f "$sdk/CMakeLists.txt" ]]; then sdk="$sdk/source"; fi
if [[ $stage == all || $stage == sdk ]]; then
 bash scripts/build_sdk.sh "$sdk" "$prefix"
fi
[[ $stage != sdk ]] || exit 0
if [[ $stage == all || $stage == driver ]]; then
 cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml
 colcon build --base-paths src/livox_ros_driver2 --executor sequential --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release "${python_cmake_args[@]}"
fi
[[ $stage != driver ]] || exit 0
set +u; source install/local_setup.bash; set -u
if [[ $stage == all || $stage == lio ]]; then
 colcon build --base-paths src/FAST_LIO_ROS2 --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release "${python_cmake_args[@]}"
fi
[[ $stage != lio ]] || exit 0
set +u; source install/local_setup.bash; set -u
mapfile -t paths < <(manifest --stage navigation)
# Legacy Python hints are needed only by rosidl interface generation. Other
# packages use FindPython3 and must not receive unused PYTHON_* command flags.
/usr/bin/python3 - "$root" "${python_cmake_args[@]:1}" <<'META'
import json, sys
from pathlib import Path
root=Path(sys.argv[1])
(root/'build').mkdir(exist_ok=True)
names = {name: {'cmake-args': sys.argv[2:]} for name in ('slam_toolbox', 'interface_protocol')}
for name in ('simulator', 'rl_walking'):
    names[name] = {'cmake-args': ['-DPM01_SIM_ASSETS_ROOT='+str(root/'.deps/sim_assets')]}
(root/'build/pm01-python.meta').write_text(json.dumps({'names': names}))
META
# A missing root package.xml previously let colcon build nested karto_sdk and
# report success while silently selecting apt's SLAM at runtime.
colcon list --base-paths "${paths[@]}" --names-only | /usr/bin/python3 -c '
import sys
names=set(sys.stdin.read().split())
if "slam_toolbox" not in names:
    raise SystemExit("Project slam_toolbox was not discovered; build aborted")'
colcon build --base-paths "${paths[@]}" --metas "$root/build/pm01-python.meta" "$root/build/pm01-config.meta" --allow-overriding slam_toolbox --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
set +u; source install/local_setup.bash; set -u
/usr/bin/python3 - "$root" <<'VERIFY'
import sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix
sys.path.insert(0,str(Path(sys.argv[1])/'src/pm01_navigation_runtime'))
from pm01_navigation_runtime.build.slam_backend import require_backend
prefix=require_backend(get_package_prefix('slam_toolbox'))
if prefix.resolve() != (Path(sys.argv[1])/'install/slam_toolbox').resolve():
    raise SystemExit('SLAM overlay was shadowed: '+str(prefix))
VERIFY
printf 'Build complete. source %s/scripts/env.sh\n' "$root"
