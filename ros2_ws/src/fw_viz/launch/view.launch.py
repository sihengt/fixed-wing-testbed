"""RViz with the plane, planned path, no-fly zones and flown track. Run next to a flight."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# Same mesh as the Gazebo plane, from the PX4 submodule. Found through the source tree (symlink install).
REPO = os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', '..'))
MESH = os.path.join(REPO, 'external/PX4-Autopilot/Tools/simulation/gz/models/rc_cessna/meshes/body.dae')


def generate_launch_description():
    config = os.path.join(get_package_share_directory('fw_viz'), 'rviz', 'flight.rviz')
    return LaunchDescription([
        Node(package='fw_viz', executable='viz',
             parameters=[{'plane_mesh': 'file://' + MESH if os.path.exists(MESH) else ''}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', config]),
    ])
