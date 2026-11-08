"""Plan a tour and fly it: PX4 bridge, waypoint_flyer (follows /planned_path) and the planner."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    planner_share = get_package_share_directory('fw_planner')
    flyer_share = get_package_share_directory('waypoint_flyer')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(planner_share, 'config', 'baylands_demo.yaml')),
        DeclareLaunchArgument('plan_file', default_value='planned_path.json'),
        # hybrid_astar, rrt_star, jps3d_raw, jps3d_fillet (jps3d_*: start cf_adapters jps3d_world.launch.py too)
        DeclareLaunchArgument('algorithm', default_value='hybrid_astar'),
        DeclareLaunchArgument('rrt_time', default_value='1.0', description='[s] RRT* time per leg'),
        DeclareLaunchArgument('rviz', default_value='false', description='true = RViz view (fw_viz)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('fw_viz'), 'launch', 'view.launch.py'])),
            condition=IfCondition(LaunchConfiguration('rviz'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(flyer_share, 'launch', 'fly.launch.py'))),
        Node(
            package='fw_planner', executable='planner', output='screen',
            parameters=[LaunchConfiguration('config'), {
                'plan_file': LaunchConfiguration('plan_file'),
                'algorithm': LaunchConfiguration('algorithm'),
                'rrt_time': ParameterValue(LaunchConfiguration('rrt_time'), value_type=float),
            }],
        ),
    ])
