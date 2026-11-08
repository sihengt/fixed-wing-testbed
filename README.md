# fixed-wing-testbed

Fixed-wing prototyping with PX4 SITL and ROS 2 (Jazzy). Simulator: Gazebo Harmonic for now.

## Setup

Open the repo in the devcontainer (needs the NVIDIA Container Toolkit on the host), then:

```bash
git submodule update --init
cd ros2_ws && colcon build --symlink-install
```

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
Pass another file with `config:=<path>`.

## Layout

- `external/` simulator-independent upstream code (PX4)
- `sim/<simulator>/` everything specific to one simulator
- `ros2_ws/` ROS 2 packages; they talk only to PX4 and standard ROS 2 topics
