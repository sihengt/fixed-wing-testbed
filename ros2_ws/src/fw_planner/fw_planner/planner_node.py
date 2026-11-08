"""Plan a closed fixed-wing tour around no-fly zones and publish it as a nav_msgs/Path (PX4 local NED)."""
import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .dubins import sample
from .hybrid_astar import World, plan_tour


def triples(values):
    values = values or []
    return [tuple(values[i:i + 3]) for i in range(0, len(values), 3)]


class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        # Flat lists in the PX4 local frame [m]: goals [x, y, heading_deg, ...], obstacles [x, y, r, ...],
        # bounds [x_min, x_max, y_min, y_max].
        goals = self.declare_parameter('goals', Parameter.Type.DOUBLE_ARRAY).value
        obstacles = triples(self.declare_parameter('obstacles', Parameter.Type.DOUBLE_ARRAY).value)
        bounds = self.declare_parameter('bounds', Parameter.Type.DOUBLE_ARRAY).value
        radius = self.declare_parameter('turn_radius', 40.0).value
        margin = self.declare_parameter('margin', 15.0).value
        altitude = self.declare_parameter('altitude', 60.0).value
        step = self.declare_parameter('sample_step', 5.0).value
        plan_file = self.declare_parameter('plan_file', '').value

        goal_poses = [(x, y, math.radians(h)) for x, y, h in triples(goals)]
        segments = plan_tour(goal_poses, radius, World(obstacles, bounds, margin))
        points = sample(goal_poses[0], segments, step)
        self.get_logger().info(f'Planned {sum(length for _, length in segments):.0f} m tour, {len(points)} points')

        path = Path()
        path.header.frame_id = 'px4_local_ned'
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, psi, _ in points:
            pose = PoseStamped(header=path.header)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, -altitude
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(psi / 2), math.cos(psi / 2)  # yaw about NED z
            path.poses.append(pose)
        # Latched, so a follower that starts later still gets the path.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Path, '/planned_path', qos)
        self.publisher.publish(path)

        if plan_file:
            with open(plan_file, 'w') as f:
                json.dump({'path': [p[:2] for p in points], 'obstacles': obstacles,
                           'bounds': bounds, 'margin': margin}, f)
            self.get_logger().info(f'Wrote {plan_file}')


def main():
    rclpy.init()
    rclpy.spin(Planner())


if __name__ == '__main__':
    main()
