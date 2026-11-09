#!/usr/bin/env python3
"""Stand-in for AirStack's trajectory_controller, so cf_drone's DROAN can fly the plane through MAVROS.

To DROAN: the global plan ('global_plan', straight line to the goal), the look-ahead state ('look_ahead',
'tracking_point') and the frames look_ahead_point_stabilized / base_link_stabilized (position, heading only).
From DROAN: trajectory segments ('trajectory_segment_to_add', in the look-ahead frame). The newest segment is
flown with path setpoints through MAVROS (position + direction; no acceleration, PX4 v1.17 reads its turn
sign backwards). Before DROAN answers, the plane follows the global plan.
Also republishes the right camera info with the stereo baseline in P[3] (Gazebo leaves it 0; DROAN reads the
baseline from it), and writes a plan file for plot_flight (global plan and pillars in PX4 NED).
All positions here are ENU (MAVROS, map frame).
"""
import json
import math

import rclpy
from airstack_msgs.msg import Odometry as AirstackOdometry
from airstack_msgs.msg import TrajectoryXYZVYaw
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

LOOKAHEAD_TIME = 0.5  # [s] DROAN plans from where the plane will be
CARROT = 30.0  # [m] setpoint distance ahead on the global plan before DROAN answers
IGNORE = (PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
          | PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)


def yaw_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class DroanBridge(Node):
    def __init__(self):
        super().__init__('droan_bridge')
        ns = self.declare_parameter('mavros_ns', '/mavros').value
        self.goal = self.declare_parameter('goal', [1200.0, 0.0]).value  # ENU x, y [m]
        self.altitude = self.declare_parameter('altitude', 60.0).value
        self.baseline = self.declare_parameter('baseline', 1.0).value
        self.obstacles = self.declare_parameter('obstacles', [0.0]).value  # ENU x, y, r triples (for the plot)
        plan_file = self.declare_parameter('plan_file', '').value
        self.fly_enabled = self.declare_parameter('fly', True).value  # false: only markers and plan file
        self.name = self.declare_parameter('name', 'droan (fixed-wing library)').value  # for the plot title
        self.odom, self.segment, self.start = None, None, None
        self.create_subscription(Odometry, f'{ns}/local_position/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(TrajectoryXYZVYaw, 'trajectory_segment_to_add', self.on_segment, 1)
        self.create_subscription(CameraInfo, '/stereo/right/camera_info', self.on_camera_info, 1)
        self.setpoint_pub = self.create_publisher(PositionTarget, f'{ns}/setpoint_raw/local', qos_profile_sensor_data)
        self.plan_pub = self.create_publisher(Path, 'global_plan', 1)
        self.look_ahead_pub = self.create_publisher(AirstackOdometry, 'look_ahead', 1)
        self.tracking_pub = self.create_publisher(AirstackOdometry, 'tracking_point', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/droan/right_camera_info', 1)
        self.tf = TransformBroadcaster(self)
        self.plan_file = plan_file
        latched = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_publisher(MarkerArray, '/planner_markers', latched).publish(self.pillar_markers())
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.publish_plan)

    def on_odom(self, msg):
        self.odom = msg
        if self.start is None:
            p = msg.pose.pose.position
            self.start = (p.x, p.y)

    def on_camera_info(self, msg):
        msg.p[3] = -msg.p[0] * self.baseline
        self.info_pub.publish(msg)

    def on_segment(self, msg):
        """Segment points are in the look-ahead frame (heading-only): rotate and shift them into map."""
        if not msg.waypoints or self.look_ahead is None:
            return
        (lx, ly, lz), yaw = self.look_ahead
        c, s = math.cos(yaw), math.sin(yaw)
        self.segment = [(lx + c * w.position.x - s * w.position.y, ly + s * w.position.x + c * w.position.y,
                         lz + w.position.z) for w in msg.waypoints]

    look_ahead = None

    def tick(self):
        if self.odom is None or not self.fly_enabled:
            return
        p, v = self.odom.pose.pose.position, self.odom.twist.twist.linear
        stamp = self.get_clock().now().to_msg()
        # MAVROS odom twist is in the body frame (base_link FLU): rotate to map with the attitude's heading.
        q = self.odom.pose.pose.orientation
        heading = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        vx, vy = v.x * math.cos(heading) - v.y * math.sin(heading), v.x * math.sin(heading) + v.y * math.cos(heading)
        speed = math.hypot(vx, vy)
        course = math.atan2(vy, vx) if speed > 1.0 else heading
        ahead = (p.x + vx * LOOKAHEAD_TIME, p.y + vy * LOOKAHEAD_TIME, p.z + v.z * LOOKAHEAD_TIME)
        self.look_ahead = (ahead, course)
        self.send_tf(stamp, 'look_ahead_point_stabilized', ahead, course)
        self.send_tf(stamp, 'base_link_stabilized', (p.x, p.y, p.z), course)
        for pub in (self.look_ahead_pub, self.tracking_pub):
            msg = AirstackOdometry()
            msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, 'map', 'map'
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = ahead
            (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
             msg.pose.orientation.w) = yaw_quaternion(course)
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, v.z
            pub.publish(msg)
        self.fly((p.x, p.y))

    def fly(self, xy):
        """Setpoint on DROAN's newest segment (closest point + a few metres), else on the global plan."""
        if self.segment and len(self.segment) > 1:
            i = min(range(len(self.segment)), key=lambda k: math.dist(self.segment[k][:2], xy))
            i = min(i + 3, len(self.segment) - 1)
            a, b = self.segment[max(i - 1, 0)], self.segment[i]
            target, direction = b, math.atan2(b[1] - a[1], b[0] - a[0])
        else:
            gx, gy = self.goal
            direction = math.atan2(gy - xy[1], gx - xy[0])
            target = (xy[0] + CARROT * math.cos(direction), xy[1] + CARROT * math.sin(direction), self.altitude)
        sp = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED, type_mask=IGNORE)
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.position.x, sp.position.y, sp.position.z = float(target[0]), float(target[1]), float(target[2])
        sp.velocity.x, sp.velocity.y = math.cos(direction), math.sin(direction)  # PX4 uses the direction only
        self.setpoint_pub.publish(sp)

    def pillar_markers(self):
        markers = MarkerArray()
        for i in range(0, len(self.obstacles) - 2, 3):
            x, y, r = self.obstacles[i:i + 3]
            m = Marker(type=Marker.CYLINDER, ns='pillars', id=i)
            m.header.frame_id = 'map'
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 75.0
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = 2 * r, 2 * r, 150.0
            m.color.r, m.color.g, m.color.b, m.color.a = 0.85, 0.3, 0.2, 0.6
            markers.markers.append(m)
        return markers

    def send_tf(self, stamp, child, xyz, yaw):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'map', child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(c) for c in xyz)
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = yaw_quaternion(yaw)
        self.tf.sendTransform(t)

    plans_sent = 0

    def publish_plan(self):
        # A few times at start only: DROAN restarts its plan (and its progress along it) for every new message.
        if self.start is None or self.plans_sent >= 5:
            return
        self.plans_sent += 1
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        (sx, sy), (gx, gy) = self.start, self.goal
        n = max(2, int(math.dist((sx, sy), (gx, gy)) / 10))
        points = [(sx + (gx - sx) * i / (n - 1), sy + (gy - sy) * i / (n - 1)) for i in range(n)]
        for x, y in points:
            pose = PoseStamped(header=path.header)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, self.altitude
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.plan_pub.publish(path)
        if self.plan_file:  # plot_flight wants PX4 NED: north = ENU y, east = ENU x
            obstacles = [[self.obstacles[i + 1], self.obstacles[i], self.obstacles[i + 2]]
                         for i in range(0, len(self.obstacles) - 2, 3)]
            with open(self.plan_file, 'w') as f:
                json.dump({'path': [[y, x] for x, y in points], 'obstacles': obstacles, 'bounds': None,
                           'margin': 15.0, 'algorithm': self.name, 'length': 0.0}, f)
            self.plan_file = ''


def main():
    rclpy.init()
    rclpy.spin(DroanBridge())


if __name__ == '__main__':
    main()
