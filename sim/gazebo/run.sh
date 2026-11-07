#!/usr/bin/env bash
# Build and start PX4 SITL with the Gazebo advanced_plane model. Opens the PX4 shell (pxh>).
# Set HEADLESS=1 to run without the Gazebo window.
set -e
cd "$(dirname "$0")/../../external/PX4-Autopilot"
# PX4 makes gz_* targets only for worlds present at configure time, so fetch them first.
git submodule update --init Tools/simulation/gz
make px4_sitl

# No ground station here, so PX4 must not need one to arm (NAV_DLL_ACT=0).
# The airframe sets NAV_DLL_ACT=2 during startup, and a PX4_PARAM_ env override cannot set a
# value back to its firmware default. So set it with the PX4 client after startup (logger starts last).
(
  for _ in $(seq 120); do
    if build/px4_sitl_default/bin/px4-logger status >/dev/null 2>&1; then
      build/px4_sitl_default/bin/px4-param set NAV_DLL_ACT 0 >/dev/null
      exit
    fi
    sleep 1
  done
) &

make px4_sitl gz_advanced_plane
