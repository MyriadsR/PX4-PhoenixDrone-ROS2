"""Automated bounded position missions for alpha SITL."""

from dataclasses import dataclass
import json
import math

import numpy as np
import rclpy
from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleCommand,
    VehicleControlMode,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .qos import PX4_INPUT_QOS, PX4_OUTPUT_QOS


CONFIRMATION = 'PHOENIX_POSITION_MISSION_TEST'
LEGACY_CONFIRMATION = 'PHOENIX_CROSS_MISSION_TEST'


@dataclass(frozen=True)
class MissionPhase:
    name: str
    offset_ned: np.ndarray
    dwell_s: float


def build_cross_mission(distance_m, takeoff_height_m, dwell_s):
    """Return hover, +x, -x travel, -y, +y travel, and landing phases.

    At the nominal tailsitter hover attitude TS +z is PX4 +x, so NED +x is
    forward.  NED -y is left.  The backward and right legs return to home,
    making every commanded leg exactly ``distance_m`` long.
    """
    distance = float(distance_m)
    height = float(takeoff_height_m)
    dwell = float(dwell_s)
    if not (math.isfinite(distance) and 0.25 <= distance <= 2.0):
        raise ValueError('distance_m must be finite and in [0.25, 2.0]')
    if not (math.isfinite(height) and 0.5 <= height <= 2.0):
        raise ValueError('takeoff_height_m must be finite and in [0.5, 2.0]')
    if not (math.isfinite(dwell) and dwell > 0.0):
        raise ValueError('dwell_s must be finite and positive')
    hover = np.array([0.0, 0.0, -height])
    return (
        MissionPhase('hover', hover, dwell),
        MissionPhase('forward', hover + np.array([distance, 0.0, 0.0]), dwell),
        MissionPhase('backward', hover, dwell),
        MissionPhase('left', hover + np.array([0.0, -distance, 0.0]), dwell),
        MissionPhase('right', hover, dwell),
        MissionPhase('land', np.zeros(3), 1.5),
    )


def build_rectangle_mission(side_length_m, takeoff_height_m, dwell_s):
    """Return a square NED mission with four horizontal legs.

    The path is home-above -> +x -> +x/-y -> -y -> home-above -> land.
    In this model, +x is forward and -y is left, so each horizontal leg has
    length ``side_length_m``.
    """
    side = float(side_length_m)
    height = float(takeoff_height_m)
    dwell = float(dwell_s)
    if not (math.isfinite(side) and 0.25 <= side <= 2.0):
        raise ValueError('side_length_m must be finite and in [0.25, 2.0]')
    if not (math.isfinite(height) and 0.5 <= height <= 2.0):
        raise ValueError('takeoff_height_m must be finite and in [0.5, 2.0]')
    if not (math.isfinite(dwell) and dwell > 0.0):
        raise ValueError('dwell_s must be finite and positive')
    hover = np.array([0.0, 0.0, -height])
    return (
        MissionPhase('hover', hover, dwell),
        MissionPhase('forward', hover + np.array([side, 0.0, 0.0]), dwell),
        MissionPhase('forward_left', hover + np.array([side, -side, 0.0]), dwell),
        MissionPhase('left', hover + np.array([0.0, -side, 0.0]), dwell),
        MissionPhase('home', hover, dwell),
        MissionPhase('land', np.zeros(3), 1.5),
    )


def build_mission(shape, distance_m, takeoff_height_m, dwell_s):
    shape = str(shape).lower()
    if shape == 'cross':
        return build_cross_mission(distance_m, takeoff_height_m, dwell_s)
    if shape in ('rectangle', 'square'):
        return build_rectangle_mission(distance_m, takeoff_height_m, dwell_s)
    raise ValueError('mission_shape must be cross, rectangle, or square')


def slew_setpoint(current, target, dt, horizontal_speed, vertical_speed):
    """Slew a NED position setpoint without diagonal horizontal overspeed."""
    current = np.asarray(current, dtype=float)
    target = np.asarray(target, dtype=float)
    values = np.concatenate((current, target))
    if current.shape != (3,) or target.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError('current and target must be finite NED 3-vectors')
    scalars = (float(dt), float(horizontal_speed), float(vertical_speed))
    if not all(math.isfinite(value) and value > 0.0 for value in scalars):
        raise ValueError('dt and slew speeds must be finite and positive')

    result = current.copy()
    horizontal_delta = target[:2] - current[:2]
    horizontal_norm = float(np.linalg.norm(horizontal_delta))
    horizontal_step = horizontal_speed * dt
    if horizontal_norm <= horizontal_step:
        result[:2] = target[:2]
    elif horizontal_norm > 0.0:
        result[:2] += horizontal_delta * (horizontal_step / horizontal_norm)
    vertical_delta = float(target[2] - current[2])
    result[2] += float(np.clip(
        vertical_delta, -vertical_speed * dt, vertical_speed * dt))
    return result


def slew_setpoint_with_velocity(current, target, dt, horizontal_speed,
                                vertical_speed):
    """Return the slewed position and its matching NED feed-forward velocity."""
    current = np.asarray(current, dtype=float)
    result = slew_setpoint(
        current, target, dt, horizontal_speed, vertical_speed)
    return result, (result - current) / float(dt)


def target_is_stable(position, velocity, setpoint, target,
                     horizontal_tolerance, vertical_tolerance,
                     speed_tolerance):
    position = np.asarray(position, dtype=float)
    velocity = np.asarray(velocity, dtype=float)
    setpoint = np.asarray(setpoint, dtype=float)
    target = np.asarray(target, dtype=float)
    values = np.concatenate((position, velocity, setpoint, target))
    if any(value.shape != (3,) for value in (position, velocity, setpoint, target)):
        return False
    if not np.all(np.isfinite(values)):
        return False
    return bool(
        np.linalg.norm(setpoint[:2] - target[:2]) <= 0.02
        and abs(setpoint[2] - target[2]) <= 0.02
        and np.linalg.norm(position[:2] - target[:2]) <= horizontal_tolerance
        and abs(position[2] - target[2]) <= vertical_tolerance
        and np.linalg.norm(velocity) <= speed_tolerance
    )


class PositionMissionTest(Node):
    """Arm, fly a bounded position mission, land, and disarm in SITL."""

    def __init__(self):
        super().__init__('phoenix_position_mission_test')
        self.declare_parameter('confirmation', '')
        self.declare_parameter('mission_shape', 'cross')
        self.declare_parameter('distance_m', 2.0)
        self.declare_parameter('takeoff_height_m', 2.0)
        self.declare_parameter('dwell_s', 3.0)
        self.declare_parameter('horizontal_speed_m_s', 0.15)
        self.declare_parameter('climb_speed_m_s', 0.30)
        self.declare_parameter('descent_speed_m_s', 0.18)
        self.declare_parameter('horizontal_tolerance_m', 0.30)
        self.declare_parameter('vertical_tolerance_m', 0.20)
        self.declare_parameter('speed_tolerance_m_s', 0.30)
        self.declare_parameter('phase_timeout_s', 55.0)
        self.declare_parameter('wait_timeout_s', 45.0)

        confirmation = str(self.get_parameter('confirmation').value)
        if confirmation not in (CONFIRMATION, LEGACY_CONFIRMATION):
            raise ValueError(
                f'confirmation must equal {CONFIRMATION}')
        self.mission_shape = str(
            self.get_parameter('mission_shape').value).lower()
        self.distance = float(self.get_parameter('distance_m').value)
        self.takeoff_height = float(
            self.get_parameter('takeoff_height_m').value)
        self.dwell_s = float(self.get_parameter('dwell_s').value)
        self.horizontal_speed = float(
            self.get_parameter('horizontal_speed_m_s').value)
        self.climb_speed = float(self.get_parameter('climb_speed_m_s').value)
        self.descent_speed = float(
            self.get_parameter('descent_speed_m_s').value)
        self.horizontal_tolerance = float(
            self.get_parameter('horizontal_tolerance_m').value)
        self.vertical_tolerance = float(
            self.get_parameter('vertical_tolerance_m').value)
        self.speed_tolerance = float(
            self.get_parameter('speed_tolerance_m_s').value)
        self.phase_timeout_s = float(
            self.get_parameter('phase_timeout_s').value)
        self.wait_timeout_s = float(
            self.get_parameter('wait_timeout_s').value)
        positive_values = (
            self.horizontal_speed,
            self.climb_speed,
            self.descent_speed,
            self.horizontal_tolerance,
            self.vertical_tolerance,
            self.speed_tolerance,
            self.phase_timeout_s,
            self.wait_timeout_s,
        )
        if not all(math.isfinite(value) and value > 0.0
                   for value in positive_values):
            raise ValueError('speeds, tolerances, and timeouts must be positive')
        self.phases = build_mission(
            self.mission_shape, self.distance, self.takeoff_height,
            self.dwell_s)
        self.max_horizontal_offset = max(
            float(np.linalg.norm(phase.offset_ned[:2]))
            for phase in self.phases)

        self.local_position = None
        self.control_mode = None
        self.vehicle_status = None
        self.origin = None
        self.setpoint = None
        self.setpoint_velocity = np.zeros(3)
        self.phase_index = 0
        self.phase_started_ns = None
        self.stable_started_ns = None
        self.prestream_started_ns = None
        self.created_ns = self.get_clock().now().nanoseconds
        self.last_tick_ns = None
        self.last_command_ns = 0
        self.disarm_started_ns = None
        self.done = False
        self.aborted = False
        self.exit_code = 1
        self.completed = []
        self.position_min = None
        self.position_max = None
        self.speed_peak = 0.0

        self.setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_INPUT_QOS)
        self.command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_INPUT_QOS)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._on_local_position, PX4_OUTPUT_QOS)
        self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode',
            self._on_control_mode, PX4_OUTPUT_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self._on_vehicle_status, PX4_OUTPUT_QOS)
        self.create_timer(0.05, self._tick)
        mission_names = ', '.join(phase.name for phase in self.phases)
        self.get_logger().warning(
            f'Prepared automated NED {self.mission_shape} mission: '
            f'{mission_names}')

    def _on_local_position(self, message):
        position = np.array([message.x, message.y, message.z], dtype=float)
        velocity = np.array([message.vx, message.vy, message.vz], dtype=float)
        if not np.all(np.isfinite(np.concatenate((position, velocity)))):
            return
        if not (message.xy_valid and message.z_valid
                and message.v_xy_valid and message.v_z_valid):
            return
        self.local_position = (position, velocity)
        if self.origin is None and not self._is_armed():
            self.origin = position.copy()
            self.setpoint = position.copy()
            self.setpoint_velocity.fill(0.0)
        if self.phase_started_ns is not None and not self.done:
            self.position_min = (position.copy() if self.position_min is None
                                 else np.minimum(self.position_min, position))
            self.position_max = (position.copy() if self.position_max is None
                                 else np.maximum(self.position_max, position))
            self.speed_peak = max(self.speed_peak, float(np.linalg.norm(velocity)))

    def _on_control_mode(self, message):
        self.control_mode = message

    def _on_vehicle_status(self, message):
        self.vehicle_status = message

    def _is_armed(self):
        return bool(
            self.vehicle_status is not None
            and self.vehicle_status.arming_state
            == VehicleStatus.ARMING_STATE_ARMED)

    def _is_armed_offboard(self):
        return bool(
            self._is_armed()
            and self.control_mode is not None
            and self.control_mode.flag_control_offboard_enabled)

    def _phase_target(self):
        return self.origin + self.phases[self.phase_index].offset_ned

    def _publish_setpoint(self):
        if self.setpoint is None:
            return
        message = TrajectorySetpoint()
        message.timestamp = self.get_clock().now().nanoseconds // 1000
        message.position = self.setpoint.tolist()
        message.velocity = self.setpoint_velocity.tolist()
        message.acceleration = [math.nan] * 3
        message.jerk = [math.nan] * 3
        message.yaw = math.nan
        message.yawspeed = math.nan
        self.setpoint_publisher.publish(message)

    def _publish_command(self, command, param1=0.0, param2=0.0):
        message = VehicleCommand()
        message.timestamp = self.get_clock().now().nanoseconds // 1000
        message.param1 = float(param1)
        message.param2 = float(param2)
        message.command = command
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self.command_publisher.publish(message)

    def _request_offboard_and_arm(self, now_ns):
        if now_ns - self.last_command_ns < int(1.0e9):
            return
        self.last_command_ns = now_ns
        if (self.vehicle_status is None
                or not self.vehicle_status.pre_flight_checks_pass):
            return
        if (self.vehicle_status.nav_state
                != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self._publish_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        if not self._is_armed():
            self._publish_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def _begin_flight(self, now_ns):
        self.phase_started_ns = now_ns
        self.stable_started_ns = None
        self.position_min = self.local_position[0].copy()
        self.position_max = self.local_position[0].copy()
        self.speed_peak = float(np.linalg.norm(self.local_position[1]))
        self.get_logger().info(
            f'MISSION_PHASE {self.phases[0].name} target='
            f'{self._phase_target().tolist()}')

    def _abort_to_land(self, reason, now_ns):
        if self.done or self.aborted:
            return
        self.aborted = True
        self.stable_started_ns = None
        self.phase_started_ns = now_ns
        if self.local_position is not None:
            # Descend vertically at the current horizontal location.  Never
            # remove actuator output immediately while an airborne test fails.
            self.origin[:2] = self.local_position[0][:2]
            self.setpoint[:2] = self.local_position[0][:2]
            self.setpoint_velocity[:2] = 0.0
        self.phase_index = len(self.phases) - 1
        self.get_logger().error(f'MISSION_ABORT {reason}; controlled landing')

    def _safety_violation(self):
        position, velocity = self.local_position
        horizontal = float(np.linalg.norm(position[:2] - self.origin[:2]))
        relative_z = float(position[2] - self.origin[2])
        speed = float(np.linalg.norm(velocity))
        if horizontal > self.max_horizontal_offset + 1.0:
            return f'horizontal displacement {horizontal:.3f} m'
        if -relative_z > self.takeoff_height + 0.8:
            return f'height {-relative_z:.3f} m'
        if relative_z > 0.5:
            return f'position below origin by {relative_z:.3f} m'
        if speed > 1.8:
            return f'speed {speed:.3f} m/s'
        return None

    def _request_forced_disarm(self, now_ns):
        self.last_command_ns = now_ns
        self._publish_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0,
            21196.0,
        )

    def _complete_phase(self, now_ns):
        phase = self.phases[self.phase_index]
        position, velocity = self.local_position
        target = self._phase_target()
        self.completed.append({
            'phase': phase.name,
            'target_ned_m': target.tolist(),
            'position_ned_m': position.tolist(),
            'position_error_m': float(np.linalg.norm(position - target)),
            'speed_m_s': float(np.linalg.norm(velocity)),
        })
        self.get_logger().info(
            f'MISSION_PHASE_PASS {phase.name} position={position.tolist()} '
            f'error={np.linalg.norm(position - target):.3f} m')
        if phase.name == 'land':
            self.disarm_started_ns = now_ns
            # The completed landing phase has already verified position and
            # speed.  Force-disarm keeps the automated test deterministic if a
            # final contact bounce delays PX4's landed hysteresis.
            self._request_forced_disarm(now_ns)
            return
        self.phase_index += 1
        self.phase_started_ns = now_ns
        self.stable_started_ns = None
        self.get_logger().info(
            f'MISSION_PHASE {self.phases[self.phase_index].name} target='
            f'{self._phase_target().tolist()}')

    def _finish(self):
        if self.done:
            return
        passed_names = [item['phase'] for item in self.completed]
        required_names = [phase.name for phase in self.phases]
        passed = bool(
            not self.aborted
            and passed_names == required_names
            and not self._is_armed())
        result = {
            'passed': passed,
            'aborted': self.aborted,
            'completed_phases': passed_names,
            'mission_shape': self.mission_shape,
            'distance_m': self.distance,
            'takeoff_height_m': self.takeoff_height,
            'position_min_ned_m': (None if self.position_min is None
                                   else self.position_min.tolist()),
            'position_max_ned_m': (None if self.position_max is None
                                   else self.position_max.tolist()),
            'speed_peak_m_s': self.speed_peak,
            'phase_results': self.completed,
        }
        self.get_logger().info(
            'POSITION_MISSION_RESULT ' + json.dumps(result, sort_keys=True))
        self.exit_code = 0 if passed else 2
        self.done = True

    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.last_tick_ns is None:
            dt = 0.05
        else:
            dt = float(np.clip((now_ns - self.last_tick_ns) * 1e-9,
                               0.001, 0.20))
        self.last_tick_ns = now_ns

        if self.origin is None or self.local_position is None:
            if (now_ns - self.created_ns) * 1e-9 > self.wait_timeout_s:
                self.get_logger().error('Timed out waiting for valid local position')
                self.done = True
                self.exit_code = 2
            return

        self._publish_setpoint()
        if self.phase_started_ns is None:
            if self.prestream_started_ns is None:
                self.prestream_started_ns = now_ns
            if self._is_armed_offboard():
                self.origin = self.local_position[0].copy()
                self.setpoint = self.origin.copy()
                self.setpoint_velocity.fill(0.0)
                self._begin_flight(now_ns)
            elif now_ns - self.prestream_started_ns >= int(2.0e9):
                self._request_offboard_and_arm(now_ns)
            if (now_ns - self.created_ns) * 1e-9 > self.wait_timeout_s:
                self.get_logger().error('Timed out waiting for armed offboard state')
                self.done = True
                self.exit_code = 2
            return

        if self.disarm_started_ns is not None:
            if not self._is_armed():
                self._finish()
                return
            if now_ns - self.last_command_ns >= int(1.0e9):
                self._request_forced_disarm(now_ns)
            if now_ns - self.disarm_started_ns > int(8.0e9):
                self.aborted = True
                self.get_logger().error('Disarm was not acknowledged after landing')
                self._finish()
            return

        if not self._is_armed_offboard():
            if self.aborted and not self._is_armed():
                self._finish()
                return
            if self._is_armed():
                self._abort_to_land('offboard mode lost', now_ns)
            else:
                self.aborted = True
                self.get_logger().error('Vehicle disarmed before landing completed')
                self._finish()
            return

        violation = self._safety_violation()
        if violation is not None:
            self._abort_to_land(violation, now_ns)

        phase = self.phases[self.phase_index]
        target = self._phase_target()
        vertical_speed = (self.descent_speed if phase.name == 'land'
                          else self.climb_speed)
        self.setpoint, self.setpoint_velocity = slew_setpoint_with_velocity(
            self.setpoint, target, dt,
            self.horizontal_speed, vertical_speed)
        self._publish_setpoint()

        position, velocity = self.local_position
        if (self.aborted
                and position[2] >= self.origin[2] - 0.20
                and self.disarm_started_ns is None):
            # A failed aircraft can be tilted and sliding, so requiring a low
            # horizontal speed would keep its propellers energized on ground.
            self.get_logger().warning(
                'Abort landing reached ground-height band; forcing disarm')
            self.disarm_started_ns = now_ns
            self._request_forced_disarm(now_ns)
            return
        stable = target_is_stable(
            position, velocity, self.setpoint, target,
            self.horizontal_tolerance, self.vertical_tolerance,
            self.speed_tolerance)
        if stable:
            if self.stable_started_ns is None:
                self.stable_started_ns = now_ns
            elif now_ns - self.stable_started_ns >= int(phase.dwell_s * 1e9):
                self._complete_phase(now_ns)
        else:
            self.stable_started_ns = None

        if (now_ns - self.phase_started_ns) * 1e-9 > self.phase_timeout_s:
            self._abort_to_land(f'{phase.name} phase timed out', now_ns)


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = PositionMissionTest()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        exit_code = node.exit_code
    except (KeyboardInterrupt, ExternalShutdownException):
        exit_code = 130
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    raise SystemExit(exit_code)
