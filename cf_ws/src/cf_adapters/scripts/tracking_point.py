#!/usr/bin/env python3
"""Stand-in for AirStack's trajectory_controller: a tracking point on the planned path, for attitude_pid.

The point is LOOKAHEAD metres past the closest path point, in ENU like MAVROS odometry, with the path
velocity (CRUISE_SPEED along the tangent) and the centripetal acceleration of the path curvature, as a
multicopter trajectory would carry them. /planned_path is PX4 local NED.
"""
import math

import rclpy
from airstack_msgs.msg import Odometry as TrackingPoint
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

CRUISE_SPEED = 15.0  # [m/s]
LOOKAHEAD = 15.0  # [m]


class TrackingPointPublisher(Node):
    def __init__(self):
        super().__init__('tracking_point')
        ns = self.declare_parameter('mavros_ns', '/mavros').value
        self.path, self.position = None, None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/planned_path', self.on_path, latched)
        self.create_subscription(Odometry, f'{ns}/local_position/odom', self.on_odom, qos_profile_sensor_data)
        self.pub = self.create_publisher(TrackingPoint, 'tracking_point', 1)
        self.create_timer(0.05, self.publish)

    def on_path(self, msg):
        # NED -> ENU: (x, y, z) -> (y, x, -z); heading from north toward east -> angle from east toward north.
        poses = [(p.pose.position.y, p.pose.position.x, -p.pose.position.z,
                  math.pi / 2 - 2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w)) for p in msg.poses]
        self.path = []
        for (x, y, z, yaw), (x1, y1, _, yaw1) in zip(poses, poses[1:] + poses[:1]):
            ds = math.hypot(x1 - x, y1 - y)
            turn = math.atan2(math.sin(yaw1 - yaw), math.cos(yaw1 - yaw))
            self.path.append((x, y, z, yaw, turn / ds if ds > 1e-3 else 0.0))  # curvature > 0: left (ENU)
        self.step = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.path, self.path[1:])) / len(self.path)

    def on_odom(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def publish(self):
        if not self.path or self.position is None:
            return
        closest = min(range(len(self.path)),
                      key=lambda i: math.hypot(self.path[i][0] - self.position[0], self.path[i][1] - self.position[1]))
        x, y, z, yaw, k = self.path[(closest + round(LOOKAHEAD / self.step)) % len(self.path)]
        tp = TrackingPoint()
        tp.header.stamp = self.get_clock().now().to_msg()
        tp.header.frame_id = 'map'
        tp.child_frame_id = 'base_link'
        tp.pose.position.x, tp.pose.position.y, tp.pose.position.z = x, y, z
        tp.pose.orientation.z, tp.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        tp.twist.linear.x, tp.twist.linear.y = CRUISE_SPEED * math.cos(yaw), CRUISE_SPEED * math.sin(yaw)
        a = CRUISE_SPEED ** 2 * k  # toward the turn centre: left normal for k > 0
        tp.acceleration.x, tp.acceleration.y = -a * math.sin(yaw), a * math.cos(yaw)
        self.pub.publish(tp)


def main():
    rclpy.init()
    rclpy.spin(TrackingPointPublisher())


if __name__ == '__main__':
    main()
