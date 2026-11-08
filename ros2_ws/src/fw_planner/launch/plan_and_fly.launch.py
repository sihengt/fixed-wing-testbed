"""Plan a tour and fly it: PX4 bridge, waypoint_flyer (follows /planned_path) and the planner."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planner_share = get_package_share_directory('fw_planner')
    flyer_share = get_package_share_directory('waypoint_flyer')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(planner_share, 'config', 'baylands_demo.yaml')),
        DeclareLaunchArgument('plan_file', default_value='planned_path.json'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(flyer_share, 'launch', 'fly.launch.py'))),
        Node(
            package='fw_planner', executable='planner', output='screen',
            parameters=[LaunchConfiguration('config'), {'plan_file': LaunchConfiguration('plan_file')}],
        ),
    ])
