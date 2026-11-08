#!/usr/bin/env bash
# Build and start PX4 SITL with a Gazebo fixed-wing. Opens the PX4 shell (pxh>).
# Usage: run.sh [world] [model]   (defaults: default, advanced_plane). Set HEADLESS=1 to run without the Gazebo window.
# Models: advanced_plane (PX4's), advanced_plane_stereo (with a stereo pair, see make_stereo_model.py;
# STEREO_ARGS are passed to it, e.g. STEREO_ARGS="--baseline 1.0 --width 1280 --height 720 --hfov 1.5708").
set -e

export PX4_GZ_WORLD=${1:-default}
MODEL=${2:-advanced_plane}
POSE="0,0,0,0,0,0"
case $PX4_GZ_WORLD in
  baylands)
    # Flat, clear 50 m strip facing west (found from the collision mesh). Trees reach ~33 m.
    # Ground there is z=-0.44 (measured with a dropped box). A model spawned inside the mesh falls through.
    POSE="60.5,7.1,-0.1,0,0,3.14159"
    export PX4_PARAM_MIS_TAKEOFF_ALT=50
    ;;
esac
export PX4_GZ_MODEL_POSE=$POSE

# With ROS 2 sourced and ros_gz installed, `gz` is the ROS vendor copy, which only knows the transport and
# msgs commands. Add the system Gazebo config so `gz sim` (used by PX4 to start Gazebo) is found.
export GZ_CONFIG_PATH=${GZ_CONFIG_PATH:+$GZ_CONFIG_PATH:}/usr/share/gz

SIM_DIR=$(cd "$(dirname "$0")" && pwd)
cd "$SIM_DIR/../../external/PX4-Autopilot"
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

if [ "$MODEL" = advanced_plane ]; then
  make px4_sitl gz_advanced_plane
  exit
fi

# Other models: PX4 only spawns models from its own folder. So start Gazebo, spawn the model, and let PX4
# attach to it (PX4 standalone mode). gz_env.sh gives Gazebo PX4's models, plugins and server config.
MODELS=$SIM_DIR/generated
python3 "$SIM_DIR/make_stereo_model.py" "$MODELS" $STEREO_ARGS > /dev/null
source build/px4_sitl_default/rootfs/gz_env.sh
export GZ_SIM_RESOURCE_PATH=$MODELS:$GZ_SIM_RESOURCE_PATH
gz sim --verbose=1 -r -s "$PX4_GZ_WORLDS/$PX4_GZ_WORLD.sdf" &
[ -z "$HEADLESS" ] && gz sim -g &
until gz service -l 2>/dev/null | grep -q "/world/$PX4_GZ_WORLD/create"; do sleep 1; done
IFS=, read -r x y z roll pitch yaw <<< "$POSE"
qz=$(python3 -c "import math; print(math.sin($yaw / 2))")
qw=$(python3 -c "import math; print(math.cos($yaw / 2))")
gz service -s "/world/$PX4_GZ_WORLD/create" --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 10000 \
  --req "sdf_filename: \"$MODELS/$MODEL/model.sdf\", name: \"$MODEL\", pose: {position: {x: $x, y: $y, z: $z}, orientation: {z: $qz, w: $qw}}"
cd build/px4_sitl_default/rootfs
PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_NAME=$MODEL PX4_SYS_AUTOSTART=4008 GZ_IP=127.0.0.1 ../bin/px4
