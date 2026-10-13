#!/usr/bin/env bash
set -euo pipefail
rig_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
rig_build="${RIG_BUILD_DIR:-$rig_root/build/omni}"
rig_prefix="${RIG_DEPENDENCY_PREFIX:-}"
if [[ -z "$rig_prefix" && -f "$HOME/OmniLIVO_ws/devel/share/livox_ros_driver/cmake/livox_ros_driverConfig.cmake" ]]; then
  rig_prefix="$HOME/OmniLIVO_ws/devel"
fi
rig_prefix="${rig_prefix:+$rig_prefix;}/opt/ros/noetic;/usr/local"
cmake -S "$rig_root/src/Omni-LIVO" -B "$rig_build" \
  -DCMAKE_PREFIX_PATH="$rig_prefix" -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_ENABLE_TESTING=ON
cmake --build "$rig_build" -j "${RIG_BUILD_JOBS:-2}"
(cd "$rig_build" && ctest --output-on-failure)
printf 'Executable: %s/devel/lib/omni_livo/rig_mapping\n' "$rig_build"
