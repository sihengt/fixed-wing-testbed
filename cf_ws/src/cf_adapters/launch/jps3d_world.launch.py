"""cf_drone's jps3d node, set up for the fixed-wing planner world (same no-fly circles, bounds and margin).

Map box = planner bounds shrunk by the margin. Inflation = margin. The fw_planner node then calls /plan per leg.
dim:=2: one voxel layer at flight altitude, no-fly circles. dim:=3: the altitude band, cylinders with heights
(obstacles3d, altitudes in the config).
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
    margin = world['margin']
    x_min, x_max, y_min, y_max = world['bounds']
    if LaunchConfiguration('dim').perform(context) == '3':
        low, high = world['altitudes']  # PX4 local NED: z = -altitude
        cylinders = world['obstacles3d']
        cylinders = [min(v, high + margin + resolution) if i % 5 == 4 else v for i, v in enumerate(cylinders)]  # no voxels above the map
        z_origin, z_size, octomap = -high - margin, high - low + 2 * margin, {'obstacles3d': cylinders}
    else:
        z_origin, z_size = -world['altitude'] - resolution / 2, resolution
        octomap = {'obstacles': world['obstacles'], 'z': -world['altitude']}
    return [
        Node(package='cf_adapters', executable='world_octomap', parameters=[{**octomap, 'resolution': resolution}]),
        Node(package='jps3d', executable='jps3d_node', output='screen', parameters=[{
            'map_origin_x': x_min + margin, 'map_size_x': x_max - x_min - 2 * margin,
            'map_origin_y': y_min + margin, 'map_size_y': y_max - y_min - 2 * margin,
            'map_origin_z': z_origin, 'map_size_z': z_size,
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
        DeclareLaunchArgument('dim', default_value='2', description='2: one altitude layer; 3: altitude band'),
        DeclareLaunchArgument('eps', default_value='1.0', description='heuristic weight (1 = optimal)'),
        DeclareLaunchArgument('use_jps', default_value='true', description='false = plain A* on the same grid'),
        OpaqueFunction(function=nodes),
    ])
