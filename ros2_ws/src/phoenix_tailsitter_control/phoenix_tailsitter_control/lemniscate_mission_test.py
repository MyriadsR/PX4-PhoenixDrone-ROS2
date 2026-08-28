"""Automated Bernoulli lemniscate trajectory test for alpha SITL."""

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

from .position_mission_test import slew_setpoint_with_velocity, target_is_stable
from .qos import PX4_INPUT_QOS, PX4_OUTPUT_QOS


CONFIRMATION = 'PHOENIX_LEMNISCATE_MISSION_TEST'


@dataclass(frozen=True)
class LemniscateSample:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray
    yaw: float
    yawspeed: float


def _bernoulli_lemniscate_derivatives(parameter):
    """Return 0th through 3rd derivatives of the unit Bernoulli lemniscate."""
    u = np.asarray(parameter, dtype=float)
    sin_u = np.sin(u)
    cos_u = np.cos(u)

    denominator = 1.0 + sin_u**2
    denominator_1 = 2.0 * sin_u * cos_u
    denominator_2 = 2.0 * (cos_u**2 - sin_u**2)
    denominator_3 = -8.0 * sin_u * cos_u

    inverse = 1.0 / denominator
    inverse_1 = -denominator_1 / denominator**2
    inverse_2 = (
        2.0 * denominator_1**2 / denominator**3
        - denominator_2 / denominator**2
    )
    inverse_3 = (
        -6.0 * denominator_1**3 / denominator**4
        + 6.0 * denominator_1 * denominator_2 / denominator**3
        - denominator_3 / denominator**2
    )

    x_numerator = (cos_u, -sin_u, -cos_u, sin_u)
    y_numerator = (
        sin_u * cos_u,
        cos_u**2 - sin_u**2,
        -4.0 * sin_u * cos_u,
        -4.0 * (cos_u**2 - sin_u**2),
    )

    def quotient_derivatives(numerator):
        n_0, n_1, n_2, n_3 = numerator
        value = n_0 * inverse
        first = n_1 * inverse + n_0 * inverse_1
        second = n_2 * inverse + 2.0 * n_1 * inverse_1 + n_0 * inverse_2
        third = (
            n_3 * inverse
            + 3.0 * n_2 * inverse_1
            + 3.0 * n_1 * inverse_2
            + n_0 * inverse_3
        )
        return value, first, second, third

    x_derivatives = quotient_derivatives(x_numerator)
    y_derivatives = quotient_derivatives(y_numerator)
    return tuple(
        np.stack((x_derivatives[order], y_derivatives[order]), axis=-1)
        for order in range(4)
    )


class BernoulliLemniscateTrajectory:
    """Constant-speed Tailsitter-control Bernoulli lemniscate in NED."""

    def __init__(self, speed, lap_time, laps, takeoff_height,
                 arc_samples=20001):
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError('speed_m_s must be finite and positive')
        if not math.isfinite(lap_time) or lap_time <= 0.0:
            raise ValueError('lap_time_s must be finite and positive')
        if int(laps) != laps or laps <= 0:
            raise ValueError('laps must be a positive integer')
        if not math.isfinite(takeoff_height) or takeoff_height <= 0.0:
            raise ValueError('takeoff_height_m must be finite and positive')
        if int(arc_samples) != arc_samples or arc_samples < 1001:
            raise ValueError('arc_samples must be an integer >= 1001')

        self.speed = float(speed)
        self.lap_time = float(lap_time)
        self.laps = int(laps)
        self.takeoff_height = float(takeoff_height)
        self.total_duration = self.laps * self.lap_time
        self.path_length = self.speed * self.lap_time

        self._u_table = np.linspace(0.0, 2.0 * np.pi, int(arc_samples))
        _, tangent_unit, _, _ = _bernoulli_lemniscate_derivatives(
            self._u_table)
        parameter_speed = np.linalg.norm(tangent_unit, axis=1)
        delta_u = np.diff(self._u_table)
        delta_arc_unit = 0.5 * (
            parameter_speed[:-1] + parameter_speed[1:]) * delta_u
        arc_unit = np.concatenate(([0.0], np.cumsum(delta_arc_unit)))
        self.scale = self.path_length / arc_unit[-1]
        self._arc_table = self.scale * arc_unit

        self.start_parameter = float(np.arctan(1.0 / np.sqrt(2.0)))
        self._start_arc = float(np.interp(
            self.start_parameter, self._u_table, self._arc_table))
        start_unit, _, _, _ = _bernoulli_lemniscate_derivatives(
            self.start_parameter)
        # Shift the original Tailsitter-control curve so sample(0) is directly
        # above the takeoff point.  The curve shape, phase, speed and yaw law
        # are unchanged.
        self.center = np.array([
            -self.scale * start_unit[0],
            -self.scale * start_unit[1],
            -self.takeoff_height,
        ])

    def sample(self, t):
        t = float(t)
        if t < 0.0:
            raise ValueError('trajectory sample time cannot be negative')
        distance = np.mod(self._start_arc + self.speed * t, self.path_length)
        parameter = float(np.interp(distance, self._arc_table, self._u_table))
        position_u, first_u, second_u, third_u = (
            self.scale * derivative
            for derivative in _bernoulli_lemniscate_derivatives(parameter)
        )
        tangent_norm = float(np.linalg.norm(first_u))
        tangent_curvature = float(np.dot(first_u, second_u))
        u_dot = self.speed / tangent_norm
        u_ddot = -(self.speed**2) * tangent_curvature / tangent_norm**4
        u_dddot = -(self.speed**3) * (
            (np.dot(second_u, second_u) + np.dot(first_u, third_u))
            / tangent_norm**5
            - 4.0 * tangent_curvature**2 / tangent_norm**7
        )

        velocity_xy = first_u * u_dot
        acceleration_xy = second_u * u_dot**2 + first_u * u_ddot
        jerk_xy = (
            third_u * u_dot**3
            + 3.0 * second_u * u_dot * u_ddot
            + first_u * u_dddot
        )
        position = self.center + np.array([position_u[0], position_u[1], 0.0])
        velocity = np.array([velocity_xy[0], velocity_xy[1], 0.0])
        acceleration = np.array([acceleration_xy[0], acceleration_xy[1], 0.0])
        jerk = np.array([jerk_xy[0], jerk_xy[1], 0.0])
        yaw = float(np.arctan2(velocity[1], velocity[0]))
        yawspeed = float(
            (velocity[0] * acceleration[1] - velocity[1] * acceleration[0])
            / (velocity[0] ** 2 + velocity[1] ** 2)
        )
        values = np.concatenate((position, velocity, acceleration, jerk))
        if not np.all(np.isfinite(values)) or not math.isfinite(yaw):
            raise ValueError('lemniscate sample is not finite')
        return LemniscateSample(
            position, velocity, acceleration, jerk, yaw, yawspeed)

    def horizontal_radius_bound(self):
        positions, _, _, _ = _bernoulli_lemniscate_derivatives(self._u_table)
        shifted = self.scale * positions + self.center[:2]
        return float(np.max(np.linalg.norm(shifted, axis=1)))


class LemniscateMissionTest(Node):
    """Arm, fly a conservative lemniscate, land, and disarm in SITL."""

    def __init__(self):
        super().__init__('phoenix_lemniscate_mission_test')
        self.declare_parameter('confirmation', '')
        self.declare_parameter('speed_m_s', 0.15)
        self.declare_parameter('lap_time_s', 48.0)
        self.declare_parameter('laps', 1)
        self.declare_parameter('takeoff_height_m', 2.0)
        self.declare_parameter('climb_speed_m_s', 0.30)
        self.declare_parameter('descent_speed_m_s', 0.18)
        self.declare_parameter('start_dwell_s', 1.5)
        self.declare_parameter('horizontal_tolerance_m', 0.35)
        self.declare_parameter('vertical_tolerance_m', 0.25)
        self.declare_parameter('speed_tolerance_m_s', 0.35)
        self.declare_parameter('track_rms_limit_m', 0.80)
        self.declare_parameter('track_peak_limit_m', 1.50)
        self.declare_parameter('wait_timeout_s', 45.0)
        self.declare_parameter('takeoff_timeout_s', 35.0)
        self.declare_parameter('land_timeout_s', 35.0)

        if str(self.get_parameter('confirmation').value) != CONFIRMATION:
            raise ValueError(f'confirmation must equal {CONFIRMATION}')

        self.speed = float(self.get_parameter('speed_m_s').value)
        self.lap_time = float(self.get_parameter('lap_time_s').value)
        self.laps = int(self.get_parameter('laps').value)
        self.takeoff_height = float(
            self.get_parameter('takeoff_height_m').value)
        self.climb_speed = float(self.get_parameter('climb_speed_m_s').value)
        self.descent_speed = float(
            self.get_parameter('descent_speed_m_s').value)
        self.start_dwell_s = float(self.get_parameter('start_dwell_s').value)
        self.horizontal_tolerance = float(
            self.get_parameter('horizontal_tolerance_m').value)
        self.vertical_tolerance = float(
            self.get_parameter('vertical_tolerance_m').value)
        self.speed_tolerance = float(
            self.get_parameter('speed_tolerance_m_s').value)
        self.track_rms_limit = float(
            self.get_parameter('track_rms_limit_m').value)
        self.track_peak_limit = float(
            self.get_parameter('track_peak_limit_m').value)
        self.wait_timeout_s = float(
            self.get_parameter('wait_timeout_s').value)
        self.takeoff_timeout_s = float(
            self.get_parameter('takeoff_timeout_s').value)
        self.land_timeout_s = float(
            self.get_parameter('land_timeout_s').value)

        positive = (
            self.speed, self.lap_time, self.takeoff_height,
            self.climb_speed, self.descent_speed, self.start_dwell_s,
            self.horizontal_tolerance, self.vertical_tolerance,
            self.speed_tolerance, self.track_rms_limit, self.track_peak_limit,
            self.wait_timeout_s, self.takeoff_timeout_s, self.land_timeout_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError('all timing, speed, tolerance, and limit values must be positive')
        if self.speed > 0.30:
            raise ValueError('speed_m_s is limited to <= 0.30 for this SITL test')
        if self.takeoff_height > 2.5:
            raise ValueError('takeoff_height_m is limited to <= 2.5 for this SITL test')

        self.trajectory = BernoulliLemniscateTrajectory(
            self.speed, self.lap_time, self.laps, self.takeoff_height)
        self.start_sample = self.trajectory.sample(0.0)
        self.final_sample = self.trajectory.sample(
            self.trajectory.total_duration)
        self.horizontal_bound = self.trajectory.horizontal_radius_bound()

        self.local_position = None
        self.control_mode = None
        self.vehicle_status = None
        self.origin = None
        self.setpoint = None
        self.setpoint_velocity = np.zeros(3)
        self.phase = 'prestream'
        self.created_ns = self.get_clock().now().nanoseconds
        self.prestream_started_ns = None
        self.phase_started_ns = None
        self.track_started_ns = None
        self.stable_started_ns = None
        self.last_tick_ns = None
        self.last_command_ns = 0
        self.disarm_started_ns = None
        self.done = False
        self.aborted = False
        self.exit_code = 1
        self.records = []
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
        self.get_logger().warning(
            'Prepared automated Bernoulli lemniscate mission: '
            f'speed={self.speed:.2f} m/s, lap_time={self.lap_time:.1f} s, '
            f'laps={self.laps}, scale={self.trajectory.scale:.3f} m')

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
        if self.phase_started_ns is not None and not self.done:
            self.position_min = (position.copy() if self.position_min is None
                                 else np.minimum(self.position_min, position))
            self.position_max = (position.copy() if self.position_max is None
                                 else np.maximum(self.position_max, position))
            self.speed_peak = max(
                self.speed_peak, float(np.linalg.norm(velocity)))

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

    def _absolute_sample(self, t):
        sample = self.trajectory.sample(t)
        return LemniscateSample(
            self.origin + sample.position,
            sample.velocity,
            sample.acceleration,
            sample.jerk,
            sample.yaw,
            sample.yawspeed,
        )

    def _publish_sample(self, sample):
        message = TrajectorySetpoint()
        message.timestamp = self.get_clock().now().nanoseconds // 1000
        message.position = sample.position.tolist()
        message.velocity = sample.velocity.tolist()
        message.acceleration = sample.acceleration.tolist()
        message.jerk = sample.jerk.tolist()
        message.yaw = float(sample.yaw)
        message.yawspeed = float(sample.yawspeed)
        self.setpoint_publisher.publish(message)

    def _publish_hold_or_slew(self, position, velocity=None):
        message = TrajectorySetpoint()
        message.timestamp = self.get_clock().now().nanoseconds // 1000
        message.position = np.asarray(position, dtype=float).tolist()
        message.velocity = (
            np.zeros(3) if velocity is None else np.asarray(velocity, dtype=float)
        ).tolist()
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

    def _request_forced_disarm(self, now_ns):
        self.last_command_ns = now_ns
        self._publish_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0,
            21196.0,
        )

    def _set_phase(self, name, now_ns):
        self.phase = name
        self.phase_started_ns = now_ns
        self.stable_started_ns = None
        self.get_logger().info(f'LEMNISCATE_PHASE {name}')

    def _safety_violation(self):
        position, velocity = self.local_position
        relative = position - self.origin
        horizontal = float(np.linalg.norm(relative[:2]))
        speed = float(np.linalg.norm(velocity))
        if horizontal > self.horizontal_bound + 1.0:
            return f'horizontal displacement {horizontal:.3f} m'
        if -relative[2] > self.takeoff_height + 0.8:
            return f'height {-relative[2]:.3f} m'
        if relative[2] > 0.5:
            return f'position below origin by {relative[2]:.3f} m'
        if speed > max(1.8, 5.0 * self.speed):
            return f'speed {speed:.3f} m/s'
        return None

    def _abort_to_land(self, reason, now_ns):
        if self.done or self.aborted:
            return
        self.aborted = True
        self.get_logger().error(f'LEMNISCATE_ABORT {reason}; controlled landing')
        if self.local_position is not None:
            self.setpoint = self.local_position[0].copy()
            self.setpoint_velocity.fill(0.0)
            self.origin[:2] = self.local_position[0][:2]
        self._set_phase('land', now_ns)

    def _record_tracking_sample(self, elapsed_s, reference):
        position, velocity = self.local_position
        self.records.append({
            't_s': float(elapsed_s),
            'position_ned_m': position.tolist(),
            'reference_ned_m': reference.position.tolist(),
            'velocity_ned_m_s': velocity.tolist(),
            'reference_velocity_ned_m_s': reference.velocity.tolist(),
            'position_error_m': float(np.linalg.norm(
                position - reference.position)),
        })

    def _tracking_summary(self):
        if not self.records:
            return {
                'track_sample_count': 0,
                'track_rms_error_m': math.inf,
                'track_peak_error_m': math.inf,
                'track_passed': False,
            }
        errors = np.asarray(
            [record['position_error_m'] for record in self.records],
            dtype=float,
        )
        return {
            'track_sample_count': int(errors.size),
            'track_rms_error_m': float(np.sqrt(np.mean(errors ** 2))),
            'track_peak_error_m': float(np.max(errors)),
            'track_passed': bool(
                np.sqrt(np.mean(errors ** 2)) <= self.track_rms_limit
                and np.max(errors) <= self.track_peak_limit),
        }

    def _finish(self):
        if self.done:
            return
        summary = self._tracking_summary()
        passed = bool(
            not self.aborted
            and summary['track_passed']
            and not self._is_armed())
        result = {
            'passed': passed,
            'aborted': self.aborted,
            'speed_m_s': self.speed,
            'lap_time_s': self.lap_time,
            'laps': self.laps,
            'takeoff_height_m': self.takeoff_height,
            'scale_m': self.trajectory.scale,
            'horizontal_bound_m': self.horizontal_bound,
            'position_min_ned_m': (None if self.position_min is None
                                   else self.position_min.tolist()),
            'position_max_ned_m': (None if self.position_max is None
                                   else self.position_max.tolist()),
            'speed_peak_m_s': self.speed_peak,
        }
        result.update(summary)
        self.get_logger().info(
            'LEMNISCATE_MISSION_RESULT '
            + json.dumps(result, sort_keys=True))
        self.exit_code = 0 if passed else 2
        self.done = True

    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.last_tick_ns is None:
            dt = 0.05
        else:
            dt = float(np.clip(
                (now_ns - self.last_tick_ns) * 1e-9, 0.001, 0.20))
        self.last_tick_ns = now_ns

        if self.origin is None or self.local_position is None:
            if (now_ns - self.created_ns) * 1e-9 > self.wait_timeout_s:
                self.get_logger().error('Timed out waiting for valid local position')
                self.aborted = True
                self.done = True
                self.exit_code = 2
            return

        if self.phase == 'prestream':
            self._publish_hold_or_slew(self.origin)
            if self.prestream_started_ns is None:
                self.prestream_started_ns = now_ns
            if self._is_armed_offboard():
                self.origin = self.local_position[0].copy()
                self.setpoint = self.origin.copy()
                self.setpoint_velocity.fill(0.0)
                self._set_phase('takeoff', now_ns)
            elif now_ns - self.prestream_started_ns >= int(2.0e9):
                self._request_offboard_and_arm(now_ns)
            if (now_ns - self.created_ns) * 1e-9 > self.wait_timeout_s:
                self.get_logger().error('Timed out waiting for armed offboard state')
                self.aborted = True
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
            elif self._is_armed():
                self._abort_to_land('offboard mode lost', now_ns)
            else:
                self.aborted = True
                self.get_logger().error('Vehicle disarmed before landing completed')
                self._finish()
            return

        violation = self._safety_violation()
        if violation is not None:
            self._abort_to_land(violation, now_ns)

        if self.phase == 'takeoff':
            target = self.origin + self.start_sample.position
            self.setpoint, self.setpoint_velocity = slew_setpoint_with_velocity(
                self.setpoint, target, dt, self.speed, self.climb_speed)
            self._publish_hold_or_slew(self.setpoint, self.setpoint_velocity)
            position, velocity = self.local_position
            if target_is_stable(
                    position, velocity, self.setpoint, target,
                    self.horizontal_tolerance, self.vertical_tolerance,
                    self.speed_tolerance):
                if self.stable_started_ns is None:
                    self.stable_started_ns = now_ns
                elif now_ns - self.stable_started_ns >= int(
                        self.start_dwell_s * 1e9):
                    self.track_started_ns = now_ns
                    self._set_phase('track', now_ns)
            else:
                self.stable_started_ns = None
            if (now_ns - self.phase_started_ns) * 1e-9 > self.takeoff_timeout_s:
                self._abort_to_land('takeoff phase timed out', now_ns)
            return

        if self.phase == 'track':
            elapsed_s = (now_ns - self.track_started_ns) * 1e-9
            if elapsed_s <= self.trajectory.total_duration:
                reference = self._absolute_sample(elapsed_s)
                self._publish_sample(reference)
                self._record_tracking_sample(elapsed_s, reference)
            else:
                self.setpoint = self.origin + self.final_sample.position
                self.setpoint_velocity.fill(0.0)
                self._set_phase('land', now_ns)
            return

        if self.phase == 'land':
            target = self.origin.copy()
            self.setpoint, self.setpoint_velocity = slew_setpoint_with_velocity(
                self.setpoint, target, dt, self.speed, self.descent_speed)
            self._publish_hold_or_slew(self.setpoint, self.setpoint_velocity)
            position, velocity = self.local_position
            if (position[2] >= self.origin[2] - 0.20
                    and float(np.linalg.norm(velocity)) <= 0.60):
                self.disarm_started_ns = now_ns
                self._request_forced_disarm(now_ns)
                return
            if (now_ns - self.phase_started_ns) * 1e-9 > self.land_timeout_s:
                self.aborted = True
                self.get_logger().error('Landing phase timed out')
                self.disarm_started_ns = now_ns
                self._request_forced_disarm(now_ns)


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = LemniscateMissionTest()
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
