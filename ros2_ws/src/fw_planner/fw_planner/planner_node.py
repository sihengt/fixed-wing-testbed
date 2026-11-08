"""Plan a closed fixed-wing tour around no-fly zones and publish it as a nav_msgs/Path (PX4 local NED)."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from . import planners
from .dubins import sample
from .jps_client import GetPlanClient
from .world import World


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
        # See fw_planner.planners. jps3d_* call a nav_msgs/GetPlan service (cf_adapters jps3d_world.launch.py).
        algorithm = self.declare_parameter('algorithm', 'hybrid_astar').value
        rrt_time = self.declare_parameter('rrt_time', 1.0).value  # [s] per leg

        goal_poses = [(x, y, math.radians(h)) for x, y, h in triples(goals)]
        world = World(obstacles, bounds, margin)
        jps = GetPlanClient(self, altitude, obstacles) if algorithm.startswith('jps3d') else None
        started = time.monotonic()
        start, segments = planners.plan(algorithm, goal_poses, radius, world, jps and jps.leg, rrt_time)
        seconds = jps.plan_seconds if jps else time.monotonic() - started
        points = sample(start, segments, step)
        length = sum(length for _, length in segments)
        self.get_logger().info(f'Planned {length:.0f} m tour, {len(points)} points ({algorithm}, {seconds:.2f} s)')
        if not world.path_free(start, segments):
            self.get_logger().warn('The tour leaves the free space (no-fly zone + margin, or bounds)')

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
                json.dump({'path': [p[:2] for p in points], 'obstacles': obstacles, 'bounds': bounds,
                           'margin': margin, 'algorithm': algorithm, 'plan_seconds': seconds,
                           'length': length}, f)
            self.get_logger().info(f'Wrote {plan_file}')


def main():
    rclpy.init()
    rclpy.spin(Planner())


if __name__ == '__main__':
    main()
