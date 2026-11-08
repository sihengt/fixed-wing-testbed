"""Take off, then fly with PX4 offboard setpoints (fixed-wing).

Without a planned path: fly a fixed loop of waypoints, straight to each one.
With a path on /planned_path (closed loop): fly to its start, then follow it with PX4 path following.
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
LOOKAHEAD = 10.0  # [m] track a point this far past the closest one, to cover control delay
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
        self.path = None  # [(x, y, z, psi)]
        self.path_index = None  # None until the vehicle joins the path
        self.lookahead = 1
        # Offboard needs a steady setpoint stream (> 2 Hz), also before the mode switch.
        self.create_timer(0.1, self.publish_setpoint)
        # Mode and arm commands are retried until PX4 accepts them.
        self.create_timer(1.0, self.manage_mode)

    def on_status(self, msg):
        self.status = msg

    def on_position(self, msg):
        self.position = msg

    def on_path(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y, p.pose.position.z,
                      2.0 * math.atan2(p.pose.orientation.z, p.pose.orientation.w)) for p in msg.poses]
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.path, self.path[1:]))
        self.path_index = None
        self.lookahead = max(1, round(LOOKAHEAD * len(self.path) / max(length, 1.0)))
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
            position, velocity, acceleration = self.path_setpoint(in_offboard)

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

    def path_setpoint(self, in_offboard):
        if self.path_index is None:
            # Fly straight to the path start first.
            x, y, z, _ = self.path[0]
            if in_offboard and math.hypot(x - self.position.x, y - self.position.y) < ACCEPT_RADIUS:
                self.path_index = 0
                self.get_logger().info('Joined the planned path')
            return [x, y, z], [NAN] * 3, [NAN] * 3

        n = len(self.path)
        ahead = [(self.path_index + k) % n for k in range(SEARCH_SAMPLES)]
        self.path_index = min(ahead, key=lambda i: math.hypot(
            self.path[i][0] - self.position.x, self.path[i][1] - self.position.y))
        x, y, z, psi = self.path[(self.path_index + self.lookahead) % n]
        # Position + velocity puts PX4 in path following along the line through the point.
        # PX4 uses only the velocity direction. No acceleration: PX4 reads it as a turn radius with
        # the opposite sign, and a sign-corrected one still tracked worse than none in tests.
        return [x, y, z], [math.cos(psi), math.sin(psi), 0.0], [NAN] * 3

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
