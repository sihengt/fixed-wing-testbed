"""Fly the plane with cf_drone's DROAN local planner (fixed-wing library) in the pillars world.

Chain: Gazebo stereo pair -> stereo_disparity -> DROAN (droan_plane.yaml) -> droan_bridge.py (stand-in for AirStack's
trajectory_controller) -> MAVROS setpoints -> PX4. mavros_relay.py does takeoff and the offboard switch.
Needs run.sh pillars advanced_plane_stereo with the same STEREO_ARGS baseline and pitch.
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
    obstacles = [v for x, y in PILLARS for v in (x, y, PILLAR_RADIUS)]
    return [
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=bridged),
        Node(package='stereo_disparity', executable='stereo_disparity_node', output='screen', parameters=[stereo_params, {
            'topic_image_left': '/stereo/left/image', 'topic_image_right': '/stereo/right/image',
            'topic_camera_info_left': '/stereo/left/camera_info', 'topic_camera_info_right': '/stereo/right/camera_info',
            't_rel': [baseline, 0.0, 0.0], 'depth_min': 0.0, 'depth_max': 300.0}]),
        # Camera on the plane (base_link, FLU), pitched down, then the optical frame (z forward, x right). Names fixed by
        # stereo_disparity, which stamps its output base_link/zedx_left/optical.
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=[
            '--x', '0.3', '--y', str(baseline / 2), '--pitch', str(pitch), '--frame-id', 'base_link', '--child-frame-id', 'base_link/zedx_left']),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=[
            '--roll', str(-math.pi / 2), '--yaw', str(-math.pi / 2), '--frame-id', 'base_link/zedx_left', '--child-frame-id', 'base_link/zedx_left/optical']),
        Node(package='droan_gl', executable='droan', output='screen',
             parameters=[os.path.join(get_package_share_directory('cf_adapters'), 'config', 'droan_plane.yaml')],
             remappings=[('/px4vision_custom_0/zedx_right/camera_info', '/droan/right_camera_info')]),
        Node(package='cf_adapters', executable='droan_bridge.py', output='screen', parameters=[{
            'baseline': baseline, 'goal': [float(LaunchConfiguration('goal_x').perform(context)), 0.0],
            'obstacles': obstacles, 'plan_file': LaunchConfiguration('plan_file').perform(context)}]),
    ]


def generate_launch_description():
    viz = os.path.join(get_package_share_directory('fw_viz'), 'rviz', 'droan.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('baseline', default_value='1.0', description='[m] must match the plane model'),
        DeclareLaunchArgument('camera_pitch', default_value='0.09', description='[rad] down, must match the model'),
        DeclareLaunchArgument('goal_x', default_value='1200.0', description='[m] ENU east of the takeoff point'),
        DeclareLaunchArgument('plan_file', default_value='planned_path.json'),
        DeclareLaunchArgument('rviz', default_value='false'),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mavros'), 'launch', 'node.launch')), launch_arguments=MAVROS_ARGS.items()),
        Node(package='cf_adapters', executable='mavros_relay.py', output='screen'),
        OpaqueFunction(function=nodes),
        # RViz: plane (from PX4 over DDS), flown track, pillars, DROAN's trajectories and camera image.
        ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='fw_viz', executable='viz', condition=IfCondition(LaunchConfiguration('rviz')), parameters=[{
            'body_frame': 'plane_body', 'plane_mesh': 'file://' + os.path.join(
                REPO, 'external/PX4-Autopilot/Tools/simulation/gz/models/rc_cessna/meshes/body.dae')}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', viz], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
