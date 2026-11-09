#!/usr/bin/env bash
# Stop every sim, ROS and experiment process. Patterns use [x] so they never match this script's caller.
for p in waypoint_flyer planner MicroXRCEAgent ros2 px4 jps3d_node world_octomap rviz2 viz mavros_node; do pkill -x $p; done
pkill -f '[m]avros_relay.py'; pkill -f '[s]tereo_eval.py'; pkill -f '[t]racking_point.py'
pkill -f '[s]tereo_disparity_node'; pkill -f '[p]id_controller_node'; pkill -f '[p]arameter_bridge'; pkill -f '[d]roan_bridge.py'; pkill -x droan; pkill -x fsm_node; pkill -f '[w]aypoint_mission'; pkill -f '[s]tatic_transform_publisher'
pkill -f 'sim/gazebo/run[.]sh'; pkill -f '^tail -f /dev/null$'; pkill -f 'gz sim --verbos[e]'; pkill -f 'gz sim -[g]'
pkill -x Xvfb
sleep 3
# Killed ROS processes leave Fast DDS segments in /dev/shm (Docker default 64 MB); full, DDS falls back to UDP.
source /opt/ros/jazzy/setup.bash && timeout 30 fastdds shm clean > /dev/null 2>&1
