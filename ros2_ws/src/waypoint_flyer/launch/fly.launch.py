"""Start the PX4 <-> ROS 2 bridge and the waypoint node. Works with any simulator running PX4 SITL."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], output='screen'),
        Node(package='waypoint_flyer', executable='waypoint_flyer', output='screen'),
    ])
