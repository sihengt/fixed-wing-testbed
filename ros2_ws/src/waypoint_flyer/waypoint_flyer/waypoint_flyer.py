"""Take off, then fly a fixed loop of waypoints with PX4 offboard position setpoints (fixed-wing)."""
import math

import rclpy
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
WAYPOINTS = [(400.0, 0.0, -100.0), (400.0, 400.0, -100.0), (0.0, 400.0, -100.0), (0.0, 0.0, -100.0)]
ACCEPT_RADIUS = 60.0  # [m] go to the next waypoint inside this horizontal distance
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0
NAN = float('nan')


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

        self.status = None
        self.position = None
        self.wp_index = 0
        # Offboard needs a steady setpoint stream (> 2 Hz), also before the mode switch.
        self.create_timer(0.1, self.publish_setpoint)
        # Mode and arm commands are retried until PX4 accepts them.
        self.create_timer(1.0, self.manage_mode)

    def on_status(self, msg):
        self.status = msg

    def on_position(self, msg):
        self.position = msg

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
        x, y, z = WAYPOINTS[self.wp_index]
        if self.status is not None and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if math.hypot(x - self.position.x, y - self.position.y) < ACCEPT_RADIUS:
                self.wp_index = (self.wp_index + 1) % len(WAYPOINTS)
                x, y, z = WAYPOINTS[self.wp_index]
                self.get_logger().info(f'Next waypoint {self.wp_index}: ({x:.0f}, {y:.0f}, {z:.0f})')

        now_us = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(OffboardControlMode(timestamp=now_us, position=True))
        self.setpoint_pub.publish(TrajectorySetpoint(
            timestamp=now_us,
            position=[x, y, z],
            velocity=[NAN] * 3,
            acceleration=[NAN] * 3,
            jerk=[NAN] * 3,
            yaw=NAN,
            yawspeed=NAN,
        ))

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
