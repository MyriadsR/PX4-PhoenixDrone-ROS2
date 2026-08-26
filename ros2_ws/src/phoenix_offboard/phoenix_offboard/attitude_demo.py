"""Exact ROS 2 semantic port of demo_offboard_attitude_setpoints.cpp."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from px4_msgs.msg import OffboardControlMode, VehicleAttitudeSetpoint
from rclpy.node import Node

from .qos import PX4_INPUT_QOS


class AttitudeDemo(Node):
    def __init__(self) -> None:
        super().__init__('demo_offboard_attitude_setpoints')
        self._start_ns = self.get_clock().now().nanoseconds
        self._mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_INPUT_QOS)
        self._setpoint_pub = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', PX4_INPUT_QOS)
        self.create_timer(0.1, self._publish)

    def _publish(self) -> None:
        now = self.get_clock().now()
        timestamp = now.nanoseconds // 1000
        elapsed = (now.nanoseconds - self._start_ns) * 1e-9

        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.attitude = True
        self._mode_pub.publish(mode)

        pitch = 0.1 * math.sin(0.5 * elapsed)
        setpoint = VehicleAttitudeSetpoint()
        setpoint.timestamp = timestamp
        # Quaternion [w, x, y, z] for RPY(0, pitch, 0).
        setpoint.q_d = [math.cos(0.5 * pitch), 0.0, math.sin(0.5 * pitch), 0.0]
        throttle = 0.4 + 0.25 * math.sin(elapsed)
        setpoint.thrust_body = [0.0, 0.0, -throttle]
        setpoint.yaw_sp_move_rate = 0.0
        self._setpoint_pub.publish(setpoint)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AttitudeDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
