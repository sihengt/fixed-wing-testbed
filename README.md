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
# Terminal 1: PX4 + Gazebo (first run builds PX4)
sim/gazebo/run.sh

# Terminal 2: ROS 2 bridge + waypoint node
source ros2_ws/install/setup.bash
ros2 launch waypoint_flyer fly.launch.py
```

The plane takes off from the runway, then loops a 400 m square at 100 m altitude.
Edit `WAYPOINTS` in `ros2_ws/src/waypoint_flyer/waypoint_flyer/waypoint_flyer.py` to change the path.

## Layout

- `external/` simulator-independent upstream code (PX4)
- `sim/<simulator>/` everything specific to one simulator
- `ros2_ws/` ROS 2 packages; they talk only to PX4 and standard ROS 2 topics
