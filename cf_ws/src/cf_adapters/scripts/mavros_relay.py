#!/usr/bin/env python3
"""Fly the fixed-wing through MAVROS instead of the PX4 DDS bridge, to test the channel cf_drone uses.

waypoint_flyer still computes the setpoints (PX4 NED TrajectorySetpoint) but publishes them on /relay/...
This node sends them to PX4 as MAVROS setpoint_raw/local (ENU, MAVROS converts to NED), and does arming,
runway takeoff and the switch to offboard through MAVROS services, like cf_drone's stack.
"""
import math

import rclpy
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from px4_msgs.msg import TrajectorySetpoint
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

MAV_CMD_NAV_TAKEOFF = 22
# Use position, velocity and acceleration; PX4 picks the yaw itself.
IGNORE = PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
IGNORE_ACCEL = PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ


def ned_to_enu(v):
    return v[1], v[0], -v[2]


class MavrosRelay(Node):
    def __init__(self):
        super().__init__('mavros_relay')
        ns = self.declare_parameter('mavros_ns', '/mavros').value
        self.state = None
        self.create_subscription(State, f'{ns}/state', self.on_state, 10)
        self.create_subscription(TrajectorySetpoint, '/relay/trajectory_setpoint', self.on_setpoint, 10)
        self.setpoint_pub = self.create_publisher(PositionTarget, f'{ns}/setpoint_raw/local', qos_profile_sensor_data)
        self.arm = self.create_client(CommandBool, f'{ns}/cmd/arming')
        self.command = self.create_client(CommandLong, f'{ns}/cmd/command')
        self.set_mode = self.create_client(SetMode, f'{ns}/set_mode')
        self.create_timer(1.0, self.manage_mode)

    def on_state(self, msg):
        self.state = msg

    def on_setpoint(self, msg):
        target = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED, type_mask=IGNORE)
        target.header.stamp = self.get_clock().now().to_msg()
        target.position.x, target.position.y, target.position.z = ned_to_enu(msg.position)
        if all(math.isfinite(v) for v in msg.velocity):
            target.velocity.x, target.velocity.y, target.velocity.z = ned_to_enu(msg.velocity)
        else:
            target.type_mask |= PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
        if all(math.isfinite(v) for v in msg.acceleration):
            target.acceleration_or_force.x, target.acceleration_or_force.y, target.acceleration_or_force.z = \
                ned_to_enu(msg.acceleration)
        else:
            target.type_mask |= IGNORE_ACCEL
        self.setpoint_pub.publish(target)

    def manage_mode(self):
        # Same sequence as waypoint_flyer, through MAVROS: takeoff mode, arm, offboard once PX4 loiters.
        if self.state is None or not self.state.connected:
            return
        if not self.state.armed and self.state.mode != 'AUTO.TAKEOFF':
            self.command.call_async(CommandLong.Request(command=MAV_CMD_NAV_TAKEOFF, param1=math.nan,
                                                        param4=math.nan, param5=math.nan, param6=math.nan,
                                                        param7=math.nan))
        elif not self.state.armed:
            self.arm.call_async(CommandBool.Request(value=True))
        elif self.state.mode == 'AUTO.LOITER':
            self.get_logger().info('Takeoff done, switching to offboard (MAVROS)')
            self.set_mode.call_async(SetMode.Request(custom_mode='OFFBOARD'))


def main():
    rclpy.init()
    rclpy.spin(MavrosRelay())


if __name__ == '__main__':
    main()
