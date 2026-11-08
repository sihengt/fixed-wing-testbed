"""cf_drone's jps3d node, set up for the fixed-wing planner world (same no-fly circles, bounds and margin).

Map box = planner bounds shrunk by the margin, one voxel layer at flight altitude (a plane cannot climb
steeply, so no 3D detours). Inflation = margin. The fw_planner node then calls /plan for each leg.
"""
import math

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    config = LaunchConfiguration('config').perform(context)
    resolution = float(LaunchConfiguration('resolution').perform(context))
    world = yaml.safe_load(open(config))['planner']['ros__parameters']
    margin, altitude = world['margin'], world['altitude']
    x_min, x_max, y_min, y_max = world['bounds']
    z = -altitude  # PX4 local NED
    return [
        Node(package='cf_adapters', executable='world_octomap', parameters=[{
            'obstacles': world['obstacles'], 'z': z, 'resolution': resolution}]),
        Node(package='jps3d', executable='jps3d_node', output='screen', parameters=[{
            'map_origin_x': x_min + margin, 'map_size_x': x_max - x_min - 2 * margin,
            'map_origin_y': y_min + margin, 'map_size_y': y_max - y_min - 2 * margin,
            'map_origin_z': z - resolution / 2, 'map_size_z': resolution,
            'map_resolution': resolution,
            'inflate_cell_size': math.ceil(margin / resolution),
            'block_unknown': False,
            'eps': float(LaunchConfiguration('eps').perform(context)),
            'use_jps': LaunchConfiguration('use_jps').perform(context) == 'true',
        }]),
    ]


def generate_launch_description():
    default_config = get_package_share_directory('fw_planner') + '/config/baylands_demo.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('resolution', default_value='2.0', description='[m] voxel size'),
        DeclareLaunchArgument('eps', default_value='1.0', description='heuristic weight (1 = optimal)'),
        DeclareLaunchArgument('use_jps', default_value='true', description='false = plain A* on the same grid'),
        OpaqueFunction(function=nodes),
    ])
