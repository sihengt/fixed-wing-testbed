"""Plan a closed fixed-wing tour around no-fly zones and publish it as a nav_msgs/Path (PX4 local NED)."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from . import planners
from .dubins import sample
from .jps_client import GetPlanClient
from .world import World


def triples(values):
    values = values or []
    return [tuple(values[i:i + 3]) for i in range(0, len(values), 3)]


def world_markers(header, goals, obstacles, bounds, margin, altitude):
    """No-fly zones (cylinder up to above the flight altitude, margin ring), bounds and goal arrows, in NED."""
    markers = MarkerArray()

    def marker(kind, rgba, **fields):
        m = Marker(header=header, ns='world', id=len(markers.markers), type=kind, **fields)
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.pose.orientation.w = 1.0
        markers.markers.append(m)
        return m

    def ring(cx, cy, r):
        return [Point(x=cx + r * math.cos(a), y=cy + r * math.sin(a), z=-altitude)
                for a in (2 * math.pi * i / 72 for i in range(73))]

    top = altitude + 20.0  # [m] cylinder height above ground
    for x, y, r in obstacles:
        m = marker(Marker.CYLINDER, (0.85, 0.2, 0.2, 0.5))
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, -top / 2
        m.scale.x, m.scale.y, m.scale.z = 2 * r, 2 * r, top
        marker(Marker.LINE_STRIP, (0.85, 0.2, 0.2, 0.9), points=ring(x, y, r + margin)).scale.x = 1.0
    if bounds:
        x0, x1, y0, y1 = bounds
        corners = [(x0, y0), (x0, y1), (x1, y1), (x1, y0), (x0, y0)]
        m = marker(Marker.LINE_STRIP, (0.7, 0.7, 0.7, 0.8), points=[Point(x=x, y=y, z=-altitude) for x, y in corners])
        m.scale.x = 1.5
    for x, y, psi in goals:
        m = marker(Marker.ARROW, (1.0, 1.0, 1.0, 1.0))
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, -altitude
        m.pose.orientation.z, m.pose.orientation.w = math.sin(psi / 2), math.cos(psi / 2)  # yaw about NED z
        m.scale.x, m.scale.y, m.scale.z = 30.0, 6.0, 6.0
    return markers


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
        self.marker_publisher = self.create_publisher(MarkerArray, '/planner_markers', qos)
        self.marker_publisher.publish(world_markers(path.header, goal_poses, obstacles, bounds, margin, altitude))

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
