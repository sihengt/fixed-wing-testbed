"""RViz view of the flight: the plane (pose from PX4), and the track it has flown.

The planned path (/planned_path) and the world (/planner_markers) come from the planner. PX4 data stays in
its NED frame (px4_local_ned). One static transform puts it under RViz's ENU frame (map).
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker

NED_FRAME = 'px4_local_ned'
TRACK_STEP = 2.0  # [m] add a track point after moving this far
TRACK_POINTS = 5000  # keep the last points only


def quaternion_multiply(a, b):
    """Hamilton product of (w, x, y, z) quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


# PX4 attitude rotates body FRD into NED. The mesh is drawn in body FLU: FRD -> FLU is 180 deg about x.
FRD_TO_FLU = (0.0, 1.0, 0.0, 0.0)
# map (ENU) <- px4_local_ned: swap x and y, flip z (180 deg about the (1, 1, 0) axis).
ENU_FROM_NED = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)


class Viz(Node):
    def __init__(self):
        super().__init__('fw_viz')
        self.mesh = self.declare_parameter('plane_mesh', '').value  # file:// URI; empty = arrow
        self.scale = self.declare_parameter('plane_scale', 30.0).value  # true size is invisible at 500 m
        # Plane body frame (FLU, like the Gazebo model). Rename it when MAVROS also publishes base_link.
        self.body_frame = self.declare_parameter('body_frame', 'base_link').value

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.on_position, qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.on_attitude, qos)
        self.tf = TransformBroadcaster(self)
        self.track_pub = self.create_publisher(Path, '/flown_path', 1)
        self.plane_pub = self.create_publisher(Marker, '/plane_marker', 1)
        # Resent with a fresh stamp: RViz drops a marker whose frame has no transform yet, and never retries.
        self.create_timer(1.0, lambda: self.plane_pub.publish(self.plane_marker()))

        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(self.transform('map', NED_FRAME, (0.0, 0.0, 0.0), ENU_FROM_NED))
        self.position = None
        self.track = Path()
        self.track.header.frame_id = NED_FRAME

    def transform(self, parent, child, xyz, wxyz):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        r = t.transform.rotation
        r.w, r.x, r.y, r.z = wxyz
        return t

    def plane_marker(self):
        m = Marker(frame_locked=True)  # follow the body frame between resends
        m.header.frame_id = self.body_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = 'plane', 0
        m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.9, 0.9, 1.0
        m.pose.orientation.w = 1.0
        if self.mesh:
            m.type, m.mesh_resource = Marker.MESH_RESOURCE, self.mesh
            m.mesh_use_embedded_materials = False
            s = 0.1 * self.scale  # the Gazebo model draws the mesh at 0.1, offset (0.07, 0, -0.08)
            m.scale.x = m.scale.y = m.scale.z = s
            m.pose.position.x, m.pose.position.z = 0.07 * self.scale, -0.08 * self.scale
        else:
            m.type = Marker.ARROW
            m.scale.x, m.scale.y, m.scale.z = 2.0 * self.scale, 0.6 * self.scale, 0.6 * self.scale
        return m

    def on_position(self, msg):
        self.position = (float(msg.x), float(msg.y), float(msg.z))
        last = self.track.poses[-1].pose.position if self.track.poses else None
        if last is None or math.dist((last.x, last.y, last.z), self.position) > TRACK_STEP:
            pose = PoseStamped()
            pose.header.frame_id = NED_FRAME
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.position
            self.track.poses = self.track.poses[-TRACK_POINTS + 1:] + [pose]
            self.track.header.stamp = self.get_clock().now().to_msg()
            self.track_pub.publish(self.track)

    def on_attitude(self, msg):
        if self.position is None:
            return
        body = quaternion_multiply(tuple(float(v) for v in msg.q), FRD_TO_FLU)
        self.tf.sendTransform(self.transform(NED_FRAME, self.body_frame, self.position, body))


def main():
    rclpy.init()
    rclpy.spin(Viz())


if __name__ == '__main__':
    main()
