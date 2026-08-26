"""Exact ROS 2 semantic port of demo_offboard_position_setpoints.cpp."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
from rclpy.node import Node

from .qos import PX4_INPUT_QOS


class PositionDemo(Node):
    def __init__(self) -> None:
        super().__init__('demo_offboard_position_setpoints')
        self._mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_INPUT_QOS)
        self._setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_INPUT_QOS)
        self.create_timer(0.1, self._publish)

    def _publish(self) -> None:
        timestamp = self.get_clock().now().nanoseconds // 1000
        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.position = True
        self._mode_pub.publish(mode)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        # Original MAVROS command was ENU (0, 0, +1); PX4 DDS uses NED.
        setpoint.position = [0.0, 0.0, -1.0]
        setpoint.velocity = [math.nan, math.nan, math.nan]
        setpoint.acceleration = [math.nan, math.nan, math.nan]
        setpoint.jerk = [math.nan, math.nan, math.nan]
        setpoint.yaw = math.nan
        setpoint.yawspeed = math.nan
        self._setpoint_pub.publish(setpoint)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PositionDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
