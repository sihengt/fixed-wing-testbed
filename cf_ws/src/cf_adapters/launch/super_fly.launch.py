"""Fly the plane with cf_drone's SUPER planner in the pillars world, through MAVROS, as cf_drone runs it.

Chain: Gazebo stereo pair -> stereo_disparity (/points) -> SUPER fsm_node (rog_map, super_plane.yaml) ->
MAVROS setpoint_raw/local -> PX4. Goals: mission_planner waypoint_mission (route plane_pillars.txt, added to the
cf_drone clone because it only reads routes from its own data folder). mavros_relay.py does takeoff and offboard.
"""
import math
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO = os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'sim/gazebo'))
from make_pillars_world import PILLAR_RADIUS, PILLARS  # noqa: E402

MAVROS_ARGS = {
    'fcu_url': 'udp://:14540@127.0.0.1:14580', 'gcs_url': '', 'tgt_system': '1', 'tgt_component': '1',
    'fcu_protocol': 'v2.0', 'respawn_mavros': 'false', 'namespace': '', 'log_output': 'screen',
    'pluginlists_yaml': os.path.join(get_package_share_directory('mavros'), 'launch', 'px4_pluginlists.yaml'),
    'config_yaml': os.path.join(REPO, 'external/cf_drone/src/px4_gazebo_sim/scripts/config/px4_config.yaml'),
}


def nodes(context):
    baseline = float(LaunchConfiguration('baseline').perform(context))
    pitch = float(LaunchConfiguration('camera_pitch').perform(context))
    bridged = [f'/stereo/{eye}/{topic}@sensor_msgs/msg/{kind}[gz.msgs.{kind}'
               for eye in ('left', 'right') for topic, kind in (('image', 'Image'), ('camera_info', 'CameraInfo'))]
    stereo_params = os.path.join(get_package_share_directory('stereo_disparity'), 'config', 'params.yaml')
    super_config = os.path.join(get_package_share_directory('cf_adapters'), 'config', 'super_plane.yaml')
    obstacles = [v for x, y in PILLARS for v in (x, y, PILLAR_RADIUS)]
    return [
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=bridged),
        Node(package='stereo_disparity', executable='stereo_disparity_node', output='screen', parameters=[stereo_params, {
            'topic_image_left': '/stereo/left/image', 'topic_image_right': '/stereo/right/image',
            'topic_camera_info_left': '/stereo/left/camera_info', 'topic_camera_info_right': '/stereo/right/camera_info',
            't_rel': [baseline, 0.0, 0.0], 'depth_min': 0.0, 'depth_max': 150.0}]),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=[
            '--x', '0.3', '--y', str(baseline / 2), '--pitch', str(pitch), '--frame-id', 'base_link', '--child-frame-id', 'base_link/zedx_left']),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=[
            '--roll', str(-math.pi / 2), '--yaw', str(-math.pi / 2), '--frame-id', 'base_link/zedx_left', '--child-frame-id', 'base_link/zedx_left/optical']),
        # rog_map inserts clouds in "world" (hard-coded); MAVROS publishes map -> base_link.
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['--frame-id', 'world', '--child-frame-id', 'map']),
        Node(package='super_planner', executable='fsm_node', output='screen', parameters=[{'config_name': super_config}]),
        Node(package='mission_planner', executable='waypoint_mission', output='screen', parameters=[{
            'config_name': 'super_plane_waypoints.yaml', 'data_name': 'plane_pillars.txt'}]),
        # Pillars as markers and the plan file for plot_flight (droan_bridge's helpers, flying disabled).
        Node(package='cf_adapters', executable='droan_bridge.py', output='screen', parameters=[{
            'fly': False, 'name': 'SUPER', 'goal': [1200.0, 0.0], 'obstacles': obstacles,
            'plan_file': LaunchConfiguration('plan_file').perform(context)}]),
    ]


def generate_launch_description():
    viz = os.path.join(get_package_share_directory('fw_viz'), 'rviz', 'droan.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('baseline', default_value='1.0', description='[m] must match the plane model'),
        DeclareLaunchArgument('camera_pitch', default_value='0.09', description='[rad] down, must match the model'),
        DeclareLaunchArgument('plan_file', default_value='planned_path.json'),
        DeclareLaunchArgument('rviz', default_value='false'),
        # X display for RViz only (e.g. a virtual :99 for recording); DROAN keeps the GPU display.
        DeclareLaunchArgument('rviz_display', default_value=os.environ.get('DISPLAY', ':0')),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mavros'), 'launch', 'node.launch')), launch_arguments=MAVROS_ARGS.items()),
        # 40 s of loiter first: SUPER's PlanFromRest needs known free space around the vehicle (a hover start).
        Node(package='cf_adapters', executable='mavros_relay.py', output='screen', parameters=[{'loiter_time': 40.0}]),
        OpaqueFunction(function=nodes),
        ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='fw_viz', executable='viz', condition=IfCondition(LaunchConfiguration('rviz')), parameters=[{
            'body_frame': 'plane_body', 'plane_mesh': 'file://' + os.path.join(
                REPO, 'external/PX4-Autopilot/Tools/simulation/gz/models/rc_cessna/meshes/body.dae')}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', viz], condition=IfCondition(LaunchConfiguration('rviz')),
             additional_env={'DISPLAY': LaunchConfiguration('rviz_display'), '__GLX_VENDOR_LIBRARY_NAME': 'mesa'}),
    ])
