"""Rectangle position setpoint demo for the PhoenixDrone offboard port."""

import math
from typing import Optional

import rclpy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .qos import PX4_INPUT_QOS, PX4_OUTPUT_QOS


class RectangleDemo(Node):
    WAYPOINTS = [
        [0.0, 0.0, -1.0],
        [0.5, 0.0, -1.0],
        [0.5, 0.3, -1.0],
        [0.0, 0.3, -1.0],
    ]
    LAND_Z = -0.04
    XY_SPEED = 0.12
    Z_SPEED = 0.20
    POSITION_TOLERANCE = 0.18
    DWELL_S = 3.0

    def __init__(self) -> None:
        super().__init__('demo_offboard_rectangle_setpoints')
        self._mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_INPUT_QOS)
        self._setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_INPUT_QOS)
        self._command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_INPUT_QOS)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._on_local_position, PX4_OUTPUT_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self._on_vehicle_status, PX4_OUTPUT_QOS)

        self._local_position: Optional[VehicleLocalPosition] = None
        self._vehicle_status: Optional[VehicleStatus] = None
        self._setpoint = list(self.WAYPOINTS[0])
        self._target = list(self.WAYPOINTS[0])
        self._waypoint_index = 0
        self._phase = 'wait_for_arm'
        self._dwell_started_ns: Optional[int] = None
        self._last_publish_ns: Optional[int] = None
        self._disarm_sent = False

        self.create_timer(0.1, self._publish)

    def _on_local_position(self, msg: VehicleLocalPosition) -> None:
        self._local_position = msg

    def _on_vehicle_status(self, msg: VehicleStatus) -> None:
        self._vehicle_status = msg

    def _publish(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        timestamp = now_ns // 1000
        dt = self._dt(now_ns)

        self._update_state(now_ns)
        self._move_setpoint(dt)

        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.position = True
        self._mode_pub.publish(mode)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.position = list(self._setpoint)
        setpoint.velocity = [math.nan, math.nan, math.nan]
        setpoint.acceleration = [math.nan, math.nan, math.nan]
        setpoint.jerk = [math.nan, math.nan, math.nan]
        setpoint.yaw = math.nan
        setpoint.yawspeed = math.nan
        self._setpoint_pub.publish(setpoint)

    def _dt(self, now_ns: int) -> float:
        if self._last_publish_ns is None:
            self._last_publish_ns = now_ns
            return 0.1
        dt = max(0.0, min((now_ns - self._last_publish_ns) * 1e-9, 0.5))
        self._last_publish_ns = now_ns
        return dt

    def _update_state(self, now_ns: int) -> None:
        armed = self._is_armed()
        if self._phase == 'wait_for_arm':
            if armed:
                self._phase = 'fly'
                self.get_logger().info('Rectangle demo started')
            return

        if self._phase == 'fly':
            if self._target_reached():
                if self._dwell_started_ns is None:
                    self._dwell_started_ns = now_ns
                    self.get_logger().info(
                        f'Waypoint {self._waypoint_index + 1}/'
                        f'{len(self.WAYPOINTS)} reached: {self._target}')
                elif now_ns - self._dwell_started_ns >= int(self.DWELL_S * 1e9):
                    self._dwell_started_ns = None
                    if self._waypoint_index + 1 < len(self.WAYPOINTS):
                        self._waypoint_index += 1
                        self._target = list(self.WAYPOINTS[self._waypoint_index])
                        self.get_logger().info(
                            f'Moving to waypoint {self._waypoint_index + 1}: '
                            f'{self._target}')
                    else:
                        self._phase = 'land'
                        self._target = [
                            self.WAYPOINTS[-1][0],
                            self.WAYPOINTS[-1][1],
                            self.LAND_Z,
                        ]
                        self.get_logger().info('Rectangle complete, descending')
            return

        if self._phase == 'land' and self._target_reached():
            self._phase = 'disarm'
            self.get_logger().info('Landing setpoint reached, disarming')

        if self._phase == 'disarm' and not self._disarm_sent:
            self._publish_disarm()
            self._disarm_sent = True

    def _is_armed(self) -> bool:
        return (
            self._vehicle_status is not None
            and self._vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def _target_reached(self) -> bool:
        local = self._local_position
        if local is None:
            return False
        position = [float(local.x), float(local.y), float(local.z)]
        setpoint_error = self._distance(self._setpoint, self._target)
        position_error = self._distance(position, self._target)
        return setpoint_error < 0.02 and position_error <= self.POSITION_TOLERANCE

    def _move_setpoint(self, dt: float) -> None:
        self._setpoint[0] = self._step(self._setpoint[0], self._target[0],
                                       self.XY_SPEED * dt)
        self._setpoint[1] = self._step(self._setpoint[1], self._target[1],
                                       self.XY_SPEED * dt)
        self._setpoint[2] = self._step(self._setpoint[2], self._target[2],
                                       self.Z_SPEED * dt)

    def _publish_disarm(self) -> None:
        command = VehicleCommand()
        command.timestamp = self.get_clock().now().nanoseconds // 1000
        command.param1 = 0.0
        command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        command.target_system = 1
        command.target_component = 1
        command.source_system = 1
        command.source_component = 1
        command.from_external = True
        self._command_pub.publish(command)

    @staticmethod
    def _step(current: float, target: float, max_delta: float) -> float:
        delta = target - current
        if abs(delta) <= max_delta:
            return target
        return current + math.copysign(max_delta, delta)

    @staticmethod
    def _distance(a: list[float], b: list[float]) -> float:
        return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RectangleDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
