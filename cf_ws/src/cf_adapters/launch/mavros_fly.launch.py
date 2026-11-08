"""Plan a tour and fly it with every PX4 command going through MAVROS (the channel cf_drone uses).

PX4 state still comes in over DDS for waypoint_flyer; its setpoints and commands are remapped to /relay/*,
and mavros_relay sends them to PX4 through MAVROS. PX4 SITL's offboard MAVLink link: udp 14540 <-> 14580.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planner_config = os.path.join(get_package_share_directory('fw_planner'), 'config', 'baylands_demo.yaml')
    mavros_launch = os.path.join(get_package_share_directory('mavros'), 'launch', 'px4.launch')
    relay = [(f'/fmu/in/{topic}', f'/relay/{topic}')
             for topic in ('trajectory_setpoint', 'offboard_control_mode', 'vehicle_command')]
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=planner_config),
        DeclareLaunchArgument('plan_file', default_value='planned_path.json'),
        ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888']),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(mavros_launch),
                                 launch_arguments={'fcu_url': 'udp://:14540@127.0.0.1:14580'}.items()),
        Node(package='waypoint_flyer', executable='waypoint_flyer', output='screen', remappings=relay),
        Node(package='cf_adapters', executable='mavros_relay.py', output='screen'),
        Node(package='fw_planner', executable='planner', output='screen',
             parameters=[LaunchConfiguration('config'), {'plan_file': LaunchConfiguration('plan_file')}]),
    ])
