"""Take off, then fly with PX4 offboard setpoints (fixed-wing).

Without a planned path: fly a fixed loop of waypoints, straight to each one.
With a path on /planned_path (closed loop): follow it with PX4 path following, from its closest point.
"""
import math

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

# Local NED waypoints [m] from the EKF origin (the takeoff point). z < 0 is up.
# 300 m square at 60 m that stays over the baylands park from its spawn point (see sim/gazebo/run.sh).
WAYPOINTS = [(200.0, -40.0, -60.0), (200.0, 260.0, -60.0), (-100.0, 260.0, -60.0), (-100.0, -40.0, -60.0)]
ACCEPT_RADIUS = 60.0  # [m] go to the next waypoint inside this horizontal distance
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0
NAN = float('nan')

# Path following
# PX4 v1.17 takes the turn direction from the offboard acceleration with the wrong sign (fixed on PX4 main).
# Set to 1.0 after a PX4 upgrade, or the plane orbits beside the path instead of following it.
PX4_TURN_SIGN = -1.0
SEARCH_SAMPLES = 20  # closest-point search window, so progress only moves forward along the loop


class WaypointFlyer(Node):
    def __init__(self):
        super().__init__('waypoint_flyer')
        # PX4 uXRCE-DDS topics use this QoS.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.on_status, qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.on_position, qos)
        path_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/planned_path', self.on_path, path_qos)

        self.status = None
        self.position = None
        self.wp_index = 0
        self.path = None  # [(x, y, z, psi, curvature)]
        self.path_index = None  # None until the vehicle joins the path
        # Offboard needs a steady setpoint stream (> 2 Hz), also before the mode switch.
        self.create_timer(0.1, self.publish_setpoint)
        # Mode and arm commands are retried until PX4 accepts them.
        self.create_timer(1.0, self.manage_mode)

    def on_status(self, msg):
        self.status = msg

    def on_position(self, msg):
        self.position = msg

    def on_path(self, msg):
        points = [(p.pose.position.x, p.pose.position.y, p.pose.position.z,
                   2.0 * math.atan2(p.pose.orientation.z, p.pose.orientation.w)) for p in msg.poses]
        # Curvature [1/m] from the heading change to the next point (> 0 turns right). The path is a closed loop.
        self.path = []
        for (x, y, z, psi), (x1, y1, _, psi1) in zip(points, points[1:] + points[:1]):
            ds = math.hypot(x1 - x, y1 - y)
            dpsi = math.atan2(math.sin(psi1 - psi), math.cos(psi1 - psi))
            self.path.append((x, y, z, psi, dpsi / ds if ds > 1e-3 else 0.0))
        self.path_index = None
        self.get_logger().info(f'Got a planned path with {len(self.path)} points')

    def manage_mode(self):
        if self.status is None:
            return
        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        nav = self.status.nav_state
        if not armed and nav != VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)
        elif not armed:
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        elif nav == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            # PX4 loiters when takeoff is done.
            self.get_logger().info('Takeoff done, switching to offboard')
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD)

    def publish_setpoint(self):
        if self.position is None:
            return
        in_offboard = self.status is not None and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if self.path is None:
            position, velocity, acceleration = self.loop_setpoint(in_offboard), [NAN] * 3, [NAN] * 3
        else:
            position, velocity, acceleration = self.path_setpoint()

        now_us = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(OffboardControlMode(timestamp=now_us, position=True))
        self.setpoint_pub.publish(TrajectorySetpoint(
            timestamp=now_us,
            position=position,
            velocity=velocity,
            acceleration=acceleration,
            jerk=[NAN] * 3,
            yaw=NAN,
            yawspeed=NAN,
        ))

    def loop_setpoint(self, in_offboard):
        x, y, z = WAYPOINTS[self.wp_index]
        if in_offboard and math.hypot(x - self.position.x, y - self.position.y) < ACCEPT_RADIUS:
            self.wp_index = (self.wp_index + 1) % len(WAYPOINTS)
            x, y, z = WAYPOINTS[self.wp_index]
            self.get_logger().info(f'Next waypoint {self.wp_index}: ({x:.0f}, {y:.0f}, {z:.0f})')
        return [x, y, z]

    def path_setpoint(self):
        def distance(i):
            return math.hypot(self.path[i][0] - self.position.x, self.path[i][1] - self.position.y)

        n = len(self.path)
        if self.path_index is not None:
            self.path_index = min(((self.path_index + k) % n for k in range(SEARCH_SAMPLES)), key=distance)
        if self.path_index is None or distance(self.path_index) > ACCEPT_RADIUS:
            # Not on the path yet, or far off it: go for the closest point anywhere on it.
            self.path_index = min(range(n), key=distance)
        x, y, z, psi, k = self.path[self.path_index]
        # Position + velocity puts PX4 in path following through the point along the velocity direction.
        # The acceleration (toward the turn centre) bends that path to radius |v|^2 / |a|.
        # |v| = 1 here, so |a| = curvature.
        a = PX4_TURN_SIGN * k
        acceleration = [-a * math.sin(psi), a * math.cos(psi), 0.0] if abs(k) > 1e-6 else [NAN] * 3
        return [x, y, z], [math.cos(psi), math.sin(psi), 0.0], acceleration

    def send_command(self, command, param1=NAN, param2=NAN):
        # Unused params stay NaN: PX4 reads 0 as a real value (e.g. lat/lon 0 for takeoff).
        self.command_pub.publish(VehicleCommand(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            command=command,
            param1=param1, param2=param2, param3=NAN, param4=NAN, param5=NAN, param6=NAN, param7=NAN,
            target_system=1, target_component=1,
            source_system=1, source_component=1,
            from_external=True,
        ))


def main():
    rclpy.init()
    rclpy.spin(WaypointFlyer())


if __name__ == '__main__':
    main()
