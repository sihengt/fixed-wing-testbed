"""cf_drone's stereo_disparity on the plane's stereo pair, scored against ground-truth depth.

Needs the plane model from sim/gazebo/make_stereo_model.py (run.sh baylands advanced_plane_stereo).
Uses cf_drone's own params.yaml; only the topics, the baseline and the depth range are changed: depth is
kept out to 1000 m so the score shows the raw error (the stack cuts at 6-20 m indoors).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    baseline = float(LaunchConfiguration('baseline').perform(context))
    bridged = [f'/stereo/{eye}/{topic}' for eye in ('left', 'right') for topic in ('image', 'camera_info')]
    bridged.append('/stereo/depth/image')
    types = {'image': 'sensor_msgs/msg/Image[gz.msgs.Image',
             'camera_info': 'sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'}
    params = os.path.join(get_package_share_directory('stereo_disparity'), 'config', 'params.yaml')
    return [
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=[f'{t}@{types[t.rsplit("/", 1)[1]]}' for t in bridged]),
        Node(package='stereo_disparity', executable='stereo_disparity_node', output='screen', parameters=[params, {
            'topic_image_left': '/stereo/left/image', 'topic_image_right': '/stereo/right/image',
            'topic_camera_info_left': '/stereo/left/camera_info',
            'topic_camera_info_right': '/stereo/right/camera_info',
            't_rel': [baseline, 0.0, 0.0], 'depth_min': 0.0, 'depth_max': 1000.0,
        }]),
        Node(package='cf_adapters', executable='stereo_eval.py', output='screen',
             parameters=[{'out': LaunchConfiguration('out').perform(context),
                          'dump_every': int(LaunchConfiguration('dump_every').perform(context))}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('baseline', default_value='0.12', description='[m] must match the model'),
        DeclareLaunchArgument('out', default_value='stereo_eval', description='output path without extension'),
        DeclareLaunchArgument('dump_every', default_value='0', description='save every N-th frame raw (0: off)'),
        OpaqueFunction(function=nodes),
    ])
