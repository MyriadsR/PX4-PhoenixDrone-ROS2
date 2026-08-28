"""Time-limited, single-channel mapping probe.  It never arms the vehicle."""

import math

import numpy as np
import rclpy
from px4_msgs.msg import ActuatorMotors, ActuatorServos, OffboardControlMode, VehicleControlMode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .qos import PX4_INPUT_QOS, PX4_OUTPUT_QOS


CONFIRMATION = 'PHOENIX_SINGLE_CHANNEL_TEST'


class ActuatorProbe(Node):
    def __init__(self):
        super().__init__('phoenix_actuator_probe')
        self.declare_parameter('enabled', False)
        self.declare_parameter('confirmation', '')
        self.declare_parameter('kind', 'motor')
        self.declare_parameter('index', 0)
        self.declare_parameter('value', 0.08)
        self.declare_parameter('motor_idle', 0.0)
        self.declare_parameter('duration_s', 3.0)
        self.activation_ns = None
        self.control_mode = None
        self.control_mode_arrival_ns = 0
        self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self._on_mode, PX4_OUTPUT_QOS)
        self.mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_INPUT_QOS)
        self.motor_publisher = self.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', PX4_INPUT_QOS)
        self.servo_publisher = self.create_publisher(
            ActuatorServos, '/fmu/in/actuator_servos', PX4_INPUT_QOS)
        self.create_timer(0.01, self._tick)
        self.get_logger().warning(
            'Probe is inert unless enabled and given the exact confirmation token')

    def _on_mode(self, message):
        self.control_mode = message
        self.control_mode_arrival_ns = self.get_clock().now().nanoseconds

    def _publish(self, timestamp, motors, servos):
        motor_message = ActuatorMotors()
        motor_message.timestamp = timestamp
        motor_message.control = list(motors) + [math.nan] * (ActuatorMotors.NUM_CONTROLS - 2)
        self.motor_publisher.publish(motor_message)
        servo_message = ActuatorServos()
        servo_message.timestamp = timestamp
        servo_message.control = list(servos) + [math.nan] * (ActuatorServos.NUM_CONTROLS - 2)
        self.servo_publisher.publish(servo_message)

    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds
        timestamp = now_ns // 1000
        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.direct_actuator = True
        self.mode_publisher.publish(mode)

        motors = np.zeros(2)
        servos = np.zeros(2)
        preconditions_met = (
            bool(self.get_parameter('enabled').value)
            and str(self.get_parameter('confirmation').value) == CONFIRMATION
            and self.control_mode is not None
            and self.control_mode.flag_armed
            and self.control_mode.flag_control_offboard_enabled
            and now_ns - self.control_mode_arrival_ns <= 500_000_000
        )
        if preconditions_met and self.activation_ns is None:
            self.activation_ns = now_ns
        duration_s = np.clip(float(self.get_parameter('duration_s').value), 0.0, 10.0)
        authorized = (
            preconditions_met
            and self.activation_ns is not None
            and (now_ns - self.activation_ns) * 1e-9 <= duration_s
        )
        if authorized:
            kind = str(self.get_parameter('kind').value).lower()
            index = int(self.get_parameter('index').value)
            value = float(self.get_parameter('value').value)
            motor_idle = float(self.get_parameter('motor_idle').value)
            if (
                index not in (0, 1)
                or kind not in ('motor', 'servo')
                or not math.isfinite(value)
                or not math.isfinite(motor_idle)
            ):
                self.get_logger().error('kind/index/value is invalid; probe output inhibited')
            elif kind == 'motor':
                motors.fill(np.clip(motor_idle, 0.0, 0.15))
                motors[index] = np.clip(value, 0.0, 0.15)
            else:
                motors.fill(np.clip(motor_idle, 0.0, 0.15))
                servos[index] = np.clip(value, -0.25, 0.25)
        self._publish(timestamp, motors, servos)


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
