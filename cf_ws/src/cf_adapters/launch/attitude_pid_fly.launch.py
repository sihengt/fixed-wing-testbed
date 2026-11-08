"""Fly the plane with cf_drone's attitude_pid controller (multicopter), through MAVROS.

tracking_point.py stands in for AirStack's trajectory_controller (a point on the planned path, with velocity
and acceleration). mavros_relay.py only does takeoff and the offboard switch here (nothing on /relay/*).
mode:=position  attitude_pid forwards the tracking point as setpoint_raw/local (cf_drone's default).
mode:=attitude  attitude_pid's cascaded PID sends roll, pitch and thrust (setpoint_raw/attitude).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# MAVROS with cf_drone's own config (it sends the map -> base_link TF that attitude_pid needs). Found through
# the source tree (symlink install): cf_ws/src/cf_adapters/launch -> repo root.
REPO = os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', '..'))
MAVROS_ARGS = {
    'fcu_url': 'udp://:14540@127.0.0.1:14580', 'gcs_url': '', 'tgt_system': '1', 'tgt_component': '1',
    'fcu_protocol': 'v2.0', 'respawn_mavros': 'false', 'namespace': '', 'log_output': 'screen',
    'pluginlists_yaml': os.path.join(get_package_share_directory('mavros'), 'launch', 'px4_pluginlists.yaml'),
    'config_yaml': os.path.join(REPO, 'external/cf_drone/src/px4_gazebo_sim/scripts/config/px4_config.yaml'),
}


def nodes(context):
    attitude = LaunchConfiguration('mode').perform(context) == 'attitude'
    pid_share = get_package_share_directory('attitude_pid')
    params = os.path.join(pid_share, 'pid_params.yaml' if attitude else 'position_params.yaml')
    return [Node(package='attitude_pid', executable='pid_controller_node', output='screen', parameters=[params, {
        'use_attitude': attitude, 'use_sim_time': False,
        'odom_topic': '/mavros/local_position/odom',
        'setpoint_topic': '/mavros/setpoint_raw/attitude',
        'setpoint_position_topic': '/mavros/setpoint_raw/local',
    }])]


def generate_launch_description():
    planner_config = os.path.join(get_package_share_directory('fw_planner'), 'config', 'baylands_demo.yaml')
    mavros_launch = os.path.join(get_package_share_directory('mavros'), 'launch', 'node.launch')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=planner_config),
        DeclareLaunchArgument('plan_file', default_value='planned_path.json'),
        DeclareLaunchArgument('mode', default_value='position', description='position | attitude'),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(mavros_launch),
                                 launch_arguments=MAVROS_ARGS.items()),
        Node(package='cf_adapters', executable='mavros_relay.py', output='screen'),
        Node(package='cf_adapters', executable='tracking_point.py', output='screen'),
        Node(package='fw_planner', executable='planner', output='screen',
             parameters=[LaunchConfiguration('config'), {'plan_file': LaunchConfiguration('plan_file')}]),
        OpaqueFunction(function=nodes),
    ])
