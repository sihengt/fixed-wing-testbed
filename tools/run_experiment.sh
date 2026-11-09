#!/usr/bin/env bash
# Fly one experiment headless (sim, planner, PX4) and score it. Usage: tools/run_experiment.sh NAME [SECONDS_ON_PATH]
# Env: SCENARIO (fw_planner config name, default baylands_demo), ALGO (planner algorithm, default hybrid_astar),
#      RVIZ=true (RViz view; screenshot of its window 90 s into the flight -> rviz.png),
#      LAUNCH=mavros (fly through MAVROS: cf_adapters mavros_fly.launch.py; hybrid_astar only),
#      LAUNCH=pid_position | pid_attitude (cf_drone attitude_pid on the plane, attitude_pid_fly.launch.py),
#      LAUNCH=droan | super (cf_drone DROAN with the fixed-wing library / SUPER: *_fly.launch.py; use WORLD=pillars or forest and the
#      stereo model; LAUNCH_ARGS="name:=value ..." passes extra launch arguments), WORLD (default baylands),
#      RECORD=1 (video of the RViz window -> video.mp4; implies RViz),
#      MODEL=advanced_plane_stereo + STEREO_ARGS (see sim/gazebo/run.sh): also runs cf_drone stereo and
#      scores it against ground-truth depth -> stereo_eval.{json,png}; DUMP_EVERY=N saves every N-th frame raw.
# Output: agent/experiments/NAME/{px4.log, ros.log, plan.json, flight.ulg, flight.png, result.txt}
NAME=$1
DUR=${2:-200}
REPO=$(cd "$(dirname "$0")/.." && pwd)
OUT=$REPO/agent/experiments/$NAME
mkdir -p "$OUT"
SCENARIO=${SCENARIO:-baylands_demo}
ALGO=${ALGO:-hybrid_astar}
RVIZ=${RVIZ:-false}
LAUNCH=${LAUNCH:-dds}
MODEL=${MODEL:-advanced_plane}
WORLD=${WORLD:-baylands}
RECORD=${RECORD:-0}
[[ $RECORD == 1 ]] && RVIZ=true
CONFIG=$REPO/ros2_ws/src/fw_planner/config/$SCENARIO.yaml

stop() { bash "$REPO/tools/stop_sim.sh"; }

wait_for() {  # wait_for FILE PATTERN TIMEOUT_S
  for _ in $(seq "$3"); do grep -q "$2" "$1" 2>/dev/null && return 0; sleep 1; done
  echo "TIMEOUT waiting for '$2' in $1"; stop; exit 1
}

stop
# The outer redirect stops the background PX4 from holding this script's output open after it exits.
(cd "$REPO" && tail -f /dev/null | HEADLESS=1 STEREO_ARGS="$STEREO_ARGS" sim/gazebo/run.sh "$WORLD" "$MODEL" > "$OUT/px4.log" 2>&1 &) > /dev/null 2>&1
wait_for "$OUT/px4.log" 'Ready for takeoff' 300

# No `set -u` in this script: the ROS setup files fail under it.
source /opt/ros/jazzy/setup.bash
source "$REPO/ros2_ws/install/setup.bash"
if [[ $ALGO == jps3d* ]]; then
  source "$REPO/external/cf_drone/install/setup.bash"
  source "$REPO/cf_ws/install/setup.bash"
  (ros2 launch cf_adapters jps3d_world.launch.py config:="$CONFIG" > "$OUT/jps3d.log" 2>&1 &) > /dev/null 2>&1
fi
BASELINE=$(echo "$STEREO_ARGS" | grep -oP '(?<=--baseline )[0-9.]+' || true)
PITCH=$(echo "$STEREO_ARGS" | grep -oP '(?<=--pitch )[0-9.]+' || true)
if [[ $LAUNCH == droan || $LAUNCH == super ]]; then
  source "$REPO/external/cf_drone/install/setup.bash"
  source "$REPO/cf_ws/install/setup.bash"
  [[ $RECORD == 1 ]] && { Xvfb :99 -screen 0 1600x1000x24 > /dev/null 2>&1 & sleep 1; }
  (cd "$OUT" && ros2 launch cf_adapters ${LAUNCH}_fly.launch.py rviz_display:="$( [[ $RECORD == 1 ]] && echo :99 || echo "$DISPLAY")" \
    baseline:="${BASELINE:-0.12}" camera_pitch:="${PITCH:-0.35}" \
    world:="$WORLD" rviz:="$RVIZ" plan_file:="$OUT/plan.json" $LAUNCH_ARGS > "$OUT/ros.log" 2>&1 &) > /dev/null 2>&1
elif [[ $LAUNCH == pid_* ]]; then
  source "$REPO/external/cf_drone/install/setup.bash"
  source "$REPO/cf_ws/install/setup.bash"
  (cd "$OUT" && ros2 launch cf_adapters attitude_pid_fly.launch.py config:="$CONFIG" mode:="${LAUNCH#pid_}" \
    plan_file:="$OUT/plan.json" > "$OUT/ros.log" 2>&1 &) > /dev/null 2>&1
elif [[ $LAUNCH == mavros ]]; then
  source "$REPO/external/cf_drone/install/setup.bash"
  source "$REPO/cf_ws/install/setup.bash"
  (cd "$OUT" && ros2 launch cf_adapters mavros_fly.launch.py config:="$CONFIG" plan_file:="$OUT/plan.json" \
    > "$OUT/ros.log" 2>&1 &) > /dev/null 2>&1
else
(cd "$OUT" && ros2 launch fw_planner plan_and_fly.launch.py config:="$CONFIG" algorithm:="$ALGO" \
  rviz:="$RVIZ" plan_file:="$OUT/plan.json" > "$OUT/ros.log" 2>&1 &) > /dev/null 2>&1
fi
wait_for "$OUT/ros.log" 'switching to offboard' 300
if [[ $MODEL == advanced_plane_stereo && $LAUNCH != droan && $LAUNCH != super ]]; then
  source "$REPO/external/cf_drone/install/setup.bash"
  source "$REPO/cf_ws/install/setup.bash"
  (ros2 launch cf_adapters stereo_eval.launch.py baseline:="${BASELINE:-0.12}" out:="$OUT/stereo_eval" dump_every:="${DUMP_EVERY:-0}" \
    > "$OUT/stereo.log" 2>&1 &) > /dev/null 2>&1
fi
if [[ $RECORD == 1 ]]; then
  # RViz renders on the virtual display :99 (the real screen may be off), filmed whole.
  sleep 15
  ffmpeg -loglevel error -y -f x11grab -framerate 15 -video_size 1600x1000 -i :99 -t "$((DUR - 20))" \
    -c:v libx264 -preset veryfast -pix_fmt yuv420p "$OUT/video.mp4" || echo "WARNING: video recording failed"
  sleep 5
elif [[ $RVIZ == true ]]; then
  sleep 90
  WINDOW=$(xwininfo -root -tree | grep -i '"rviz' | grep -v '0x0+' | head -1 | awk '{print $1}')
  [[ -n $WINDOW ]] && import -window "$WINDOW" "$OUT/rviz.png" || echo "WARNING: no RViz window found"
  sleep $((DUR - 90))
else
  sleep "$DUR"
fi
grep -hE 'Disarmed|Landing detected|Failsafe' "$OUT/px4.log" && echo "WARNING: failure events above"
stop

ULOG=$(ls -t "$REPO"/external/PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/*.ulg | head -1)
cp "$ULOG" "$OUT/flight.ulg"
ros2 run fw_planner plot_flight "$OUT/plan.json" "$OUT/flight.ulg" -o "$OUT/flight.png" | tee "$OUT/result.txt"
