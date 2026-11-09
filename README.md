# fixed-wing-testbed

Fixed-wing prototyping with PX4 SITL and ROS 2 (Jazzy). Simulator: Gazebo Harmonic for now.

## Setup

Open the repo in the devcontainer (needs the NVIDIA Container Toolkit on the host), then:

```bash
git submodule update --init external/PX4-Autopilot ros2_ws/src/px4_msgs
cd ros2_ws && colcon build --symlink-install
```

The cf_drone experiments below also need `external/cf_drone` (private, DinoHub; needs an SSH key with access)
and, for the offline stereo tests, `external/Fast-FoundationStereo`.

## Fly the waypoint loop

```bash
# Terminal 1: PX4 + Gazebo (first run builds PX4). World: default or baylands (textured park).
sim/gazebo/run.sh baylands

# Terminal 2: ROS 2 bridge + waypoint node
source ros2_ws/install/setup.bash
ros2 launch waypoint_flyer fly.launch.py
```

The plane takes off from the runway, then loops a 300 m square at 60 m altitude (placed over the baylands park).
Edit `WAYPOINTS` in `ros2_ws/src/waypoint_flyer/waypoint_flyer/waypoint_flyer.py` to change the path.

## Fly a planned path

`fw_planner` plans a closed tour through goal poses around circular no-fly zones (Hybrid A* with Dubins
shots), and `waypoint_flyer` follows it with PX4 path following.

```bash
# Terminal 1
sim/gazebo/run.sh baylands

# Terminal 2: bridge, planner and follower. Writes the plan to planned_path.json.
source ros2_ws/install/setup.bash
ros2 launch fw_planner plan_and_fly.launch.py

# After the flight: plot the flown track against the plan, and print the cross-track error.
ros2 run fw_planner plot_flight planned_path.json \
  "$(ls -t external/PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/*.ulg | head -1)" -o flight.png
```

Goals, no-fly zones, bounds and turn radius are in `ros2_ws/src/fw_planner/config/baylands_demo.yaml`.
Pass another file with `config:=<path>` (e.g. `baylands_clutter.yaml`, nine no-fly zones).

Options of `plan_and_fly.launch.py`:
- `algorithm:=hybrid_astar` (default), `rrt_star` (OMPL), or `jps3d_raw | jps3d_fillet | jps3d_guided`
  (cf_drone's jps3d, see below).
- `rviz:=true`: RViz view of the plane, the planned path, the no-fly zones and the flown track.

Compare planners without flying: `ros2 run fw_planner bench_planners <config.yaml> --plot plans.png`.

## Stereo camera on the plane

`sim/gazebo/run.sh baylands advanced_plane_stereo` flies PX4's plane with a stereo pair and a ground-truth
depth camera (`/stereo/{left,right,depth}/...` in Gazebo). Camera settings: `STEREO_ARGS="--baseline 1.0
--width 1280 --height 720"` (see `sim/gazebo/make_stereo_model.py`).

## cf_drone modules (optional)

The drone stack is the private submodule `external/cf_drone`, branch `fixed-wing-experiments`: DROAN with a
fixed-wing trajectory library and SUPER with a goal fix for planes (its `droan_local_planner` and `super_planner`
submodules are on branches of the same name). Its adapters live in `cf_ws`, built on top of it:

```bash
git submodule update --init external/cf_drone
git -C external/cf_drone submodule update --init
cd external/cf_drone && colcon build --packages-skip px4_gazebo_sim px4_msgs px4_ros_com
cd ../../cf_ws && source ../external/cf_drone/install/setup.bash && colcon build --symlink-install
```

Overlay order: `/opt/ros/jazzy` -> `ros2_ws` -> `external/cf_drone/install` -> `cf_ws/install`.

## cf_drone experiments

`tools/run_experiment.sh NAME SECONDS` starts the sim headless, flies, stops everything and scores the flight into
`agent/experiments/NAME/` (`flight.ulg`, `result.txt`, `flight.png`, `video.mp4` with `RECORD=1`). Settings come
from environment variables (see its header). The commands behind the reported results:

```bash
STEREO_1M="--baseline 1.0 --width 960 --height 540 --hfov 1.5708 --pitch 0.09"   # plane-scale pair, 1 m
ZEDX_4MM="--baseline 0.12 --width 1920 --height 1200 --hfov 1.3963 --pitch 0.09"  # real ZED X, 4 mm lens

# DROAN through the forest (23 pillars), and past the pillar line
WORLD=forest MODEL=advanced_plane_stereo STEREO_ARGS="$STEREO_1M" LAUNCH=droan RECORD=1 tools/run_experiment.sh droan_forest 150
WORLD=pillars MODEL=advanced_plane_stereo STEREO_ARGS="$STEREO_1M" LAUNCH=droan RECORD=1 tools/run_experiment.sh droan_pillars 150
# SUPER past the pillar line
WORLD=pillars MODEL=advanced_plane_stereo STEREO_ARGS="$STEREO_1M" LAUNCH=super RECORD=1 tools/run_experiment.sh super_pillars 180
# Stereo frames for the offline tests: our follower flies past the pillars, every 30th frame is saved raw
WORLD=pillars MODEL=advanced_plane_stereo SCENARIO=pillars_pass DUMP_EVERY=30 STEREO_ARGS="$ZEDX_4MM" \
  tools/run_experiment.sh stereo_pillars_zedx40 200

# Top view of flights
python3 tools/plot_pillar_flights.py forest.png agent/experiments/droan_forest:"DROAN"
```

The sim is not deterministic: repeated runs can take different lines (compare several, not single tracks).

Offline stereo tests (cf_drone's matcher vs Fast-FoundationStereo, and with added camera faults) need the
Fast-FoundationStereo submodule, its own Python venv and its weights:

```bash
git submodule update --init external/Fast-FoundationStereo
tools/install_ffs.sh   # external/ffs-venv/ and external/ffs-weights/ (gitignored)
external/ffs-venv/bin/python tools/stereo_methods.py --out agent/experiments/stereo_methods \
  agent/experiments/stereo_pillars_zedx40:0.12 agent/experiments/stereo_pillars_pair1m:1.0
external/ffs-venv/bin/python tools/stereo_degrade.py --out agent/experiments/stereo_degrade.json \
  agent/experiments/stereo_pillars_zedx40:0.12 agent/experiments/stereo_pillars_pair1m:1.0
```

## Layout

- `external/` simulator-independent upstream code (PX4, cf_drone, Fast-FoundationStereo)
- `sim/<simulator>/` everything specific to one simulator
- `ros2_ws/` ROS 2 packages; they talk only to PX4 and standard ROS 2 topics
- `cf_ws/` adapters between this testbed and the cf_drone stack (needs `external/cf_drone`)
- `tools/` experiment runner and scoring scripts; results go to `agent/` (gitignored)
