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
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .position_mission_test import slew_setpoint_with_velocity, target_is_stable
from .qos import PX4_INPUT_QOS, PX4_OUTPUT_QOS


def landing_descent_allowed(position, velocity, setpoint,
                            horizontal_tolerance=0.75,
                            horizontal_speed_tolerance=0.50):
    """Keep altitude until horizontal error and drift are under control."""
    return (
        np.linalg.norm(np.asarray(position)[:2] - np.asarray(setpoint)[:2])
        <= horizontal_tolerance
        and np.linalg.norm(np.asarray(velocity)[:2])
        <= horizontal_speed_tolerance
    )


def landing_disarm_allowed(position, velocity, origin, landed_confirmed):
    """Accept PX4 touchdown near the ground despite local-height bias."""
    return (
        np.linalg.norm(velocity) <= 0.60
        and (position[2] >= origin[2] - 0.20
             or (landed_confirmed and position[2] >= origin[2] - 0.60))
    )


def landed_state_confirmed(count, first_ns, last_ns, now_ns):
    """Require two fresh landed reports spanning at least half a second."""
    return (
        count >= 2
        and first_ns is not None
        and last_ns is not None
        and last_ns - first_ns >= int(0.5e9)
        and 0 <= now_ns - last_ns <= int(1.5e9)
    )


def near_ground_runaway(position, origin, horizontal_limit):
    """Detect an aborted SITL flight escaping the test area near the floor."""
    relative = np.asarray(position) - np.asarray(origin)
    return (
        relative[2] >= -0.60
        and np.linalg.norm(relative[:2]) > horizontal_limit
    )


CONFIRMATION = 'PHOENIX_LEMNISCATE_MISSION_TEST'


@dataclass(frozen=True)
class LemniscateSample:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray
    yaw: float
    yawspeed: float


def smooth_stop_sample(start_position, start_velocity, t, duration, target_z):
    """Stop along the measured horizontal velocity with quintic speed scaling."""
    position_0 = np.asarray(start_position, dtype=float)
    velocity_0 = np.asarray(start_velocity, dtype=float).copy()
    if position_0.shape != (3,) or velocity_0.shape != (3,):
        raise ValueError('smooth-stop state must contain finite 3-vectors')
    if not np.all(np.isfinite(np.concatenate((position_0, velocity_0)))):
        raise ValueError('smooth-stop state must contain finite 3-vectors')
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError('smooth-stop duration must be finite and positive')
    if not math.isfinite(target_z):
        raise ValueError('smooth-stop target z must be finite')

    velocity_0[2] = 0.0
    ratio = float(np.clip(float(t) / duration, 0.0, 1.0))
    ratio_2 = ratio * ratio
    ratio_3 = ratio_2 * ratio
    ratio_4 = ratio_3 * ratio
    ratio_5 = ratio_4 * ratio
    ratio_6 = ratio_5 * ratio
    smooth = 10.0 * ratio_3 - 15.0 * ratio_4 + 6.0 * ratio_5
    smooth_integral = 2.5 * ratio_4 - 3.0 * ratio_5 + ratio_6
    path_rate = 1.0 - smooth
    path_acceleration = -(
        30.0 * ratio_2 - 60.0 * ratio_3 + 30.0 * ratio_4
    ) / duration
    path_jerk = -(
        60.0 * ratio - 180.0 * ratio_2 + 120.0 * ratio_3
    ) / duration**2

    position = position_0 + velocity_0 * duration * (
        ratio - smooth_integral)
    position[2] = float(target_z)
    velocity = velocity_0 * path_rate
    acceleration = velocity_0 * path_acceleration
    jerk = velocity_0 * path_jerk
    yaw = float(math.atan2(velocity_0[1], velocity_0[0]))
    return LemniscateSample(position, velocity, acceleration, jerk, yaw, 0.0)


def abort_brake_sample(start_position, start_velocity, t, duration, target_z):
    """Brake horizontally while climbing smoothly to a safe height."""
    sample = smooth_stop_sample(
        start_position, start_velocity, t, duration, start_position[2])
    if not math.isfinite(target_z):
        raise ValueError('abort brake target z must be finite')
    ratio = float(np.clip(t / duration, 0.0, 1.0))
    delta_z = target_z - start_position[2]
    position = sample.position.copy()
    velocity = sample.velocity.copy()
    acceleration = sample.acceleration.copy()
    jerk = sample.jerk.copy()
    position[2] += delta_z * (
        10.0 * ratio**3 - 15.0 * ratio**4 + 6.0 * ratio**5)
    velocity[2] = delta_z * (
        30.0 * ratio**2 - 60.0 * ratio**3 + 30.0 * ratio**4
    ) / duration
    acceleration[2] = delta_z * (
        60.0 * ratio - 180.0 * ratio**2 + 120.0 * ratio**3
    ) / duration**2
    jerk[2] = delta_z * (
        60.0 - 360.0 * ratio + 360.0 * ratio**2
    ) / duration**3
    return LemniscateSample(
        position, velocity, acceleration, jerk, sample.yaw, 0.0)


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

    def entry_sample(self, t, duration):
        """Follow the curve while smoothly increasing path speed from 0 to 1."""
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError('entry duration must be finite and positive')
        ratio = float(np.clip(float(t) / duration, 0.0, 1.0))
        ratio_2 = ratio * ratio
        ratio_3 = ratio_2 * ratio
        ratio_4 = ratio_3 * ratio
        ratio_5 = ratio_4 * ratio
        ratio_6 = ratio_5 * ratio
        path_time = duration * (
            2.5 * ratio_4 - 3.0 * ratio_5 + ratio_6)
        path_rate = 10.0 * ratio_3 - 15.0 * ratio_4 + 6.0 * ratio_5
        path_acceleration = (
            30.0 * ratio_2 - 60.0 * ratio_3 + 30.0 * ratio_4
        ) / duration
        path_jerk = (
            60.0 * ratio - 180.0 * ratio_2 + 120.0 * ratio_3
        ) / duration**2

        sample = self.sample(path_time)
        velocity = sample.velocity * path_rate
        acceleration = (
            sample.acceleration * path_rate**2
            + sample.velocity * path_acceleration)
        jerk = (
            sample.jerk * path_rate**3
            + 3.0 * sample.acceleration * path_rate * path_acceleration
            + sample.velocity * path_jerk)
        return LemniscateSample(
            sample.position,
            velocity,
            acceleration,
            jerk,
            sample.yaw,
            sample.yawspeed * path_rate,
        )

    def exit_sample(self, start_time, t, duration):
        """Follow the curve while smoothly reducing path speed from 1 to 0."""
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError('exit duration must be finite and positive')
        ratio = float(np.clip(float(t) / duration, 0.0, 1.0))
        ratio_2 = ratio * ratio
        ratio_3 = ratio_2 * ratio
        ratio_4 = ratio_3 * ratio
        ratio_5 = ratio_4 * ratio
        ratio_6 = ratio_5 * ratio
        smooth_integral = 2.5 * ratio_4 - 3.0 * ratio_5 + ratio_6
        path_time = float(start_time) + duration * (ratio - smooth_integral)
        path_rate = 1.0 - (
            10.0 * ratio_3 - 15.0 * ratio_4 + 6.0 * ratio_5)
        path_acceleration = -(
            30.0 * ratio_2 - 60.0 * ratio_3 + 30.0 * ratio_4
        ) / duration
        path_jerk = -(
            60.0 * ratio - 180.0 * ratio_2 + 120.0 * ratio_3
        ) / duration**2

        sample = self.sample(path_time)
        velocity = sample.velocity * path_rate
        acceleration = (
            sample.acceleration * path_rate**2
            + sample.velocity * path_acceleration)
        jerk = (
            sample.jerk * path_rate**3
            + 3.0 * sample.acceleration * path_rate * path_acceleration
            + sample.velocity * path_jerk)
        return LemniscateSample(
            sample.position,
            velocity,
            acceleration,
            jerk,
            sample.yaw,
            sample.yawspeed * path_rate,
        )

    def horizontal_radius_bound(self):
        positions, _, _, _ = _bernoulli_lemniscate_derivatives(self._u_table)
        shifted = self.scale * positions + self.center[:2]
        return float(np.max(np.linalg.norm(shifted, axis=1)))


class LemniscateMissionTest(Node):
    """Arm, fly a conservative lemniscate, land, and disarm in SITL."""

    def __init__(self):
        super().__init__('phoenix_lemniscate_mission_test')
        self.declare_parameter('confirmation', '')
        self.declare_parameter('speed_m_s', 6.0)
        self.declare_parameter('lap_time_s', 7.0)
        self.declare_parameter('reference_rate_hz', 20.0)
        self.declare_parameter('laps', 8)
        self.declare_parameter('takeoff_height_m', 10.0)
        self.declare_parameter('staging_horizontal_speed_m_s', 0.15)
        self.declare_parameter('climb_speed_m_s', 0.30)
        self.declare_parameter('descent_speed_m_s', 0.18)
        self.declare_parameter('start_dwell_s', 1.5)
        self.declare_parameter('entry_duration_s', 4.0)
        self.declare_parameter('exit_duration_s', 4.0)
        self.declare_parameter('posttrack_braking_acceleration_m_s2', 2.0)
        self.declare_parameter('horizontal_safety_margin_m', 3.0)
        self.declare_parameter('horizontal_tolerance_m', 0.50)
        self.declare_parameter('vertical_tolerance_m', 0.50)
        self.declare_parameter('speed_tolerance_m_s', 0.80)
        self.declare_parameter('track_rms_limit_m', 10.0)
        self.declare_parameter('track_peak_limit_m', 25.0)
        self.declare_parameter('wait_timeout_s', 45.0)
        self.declare_parameter('takeoff_timeout_s', 60.0)
        self.declare_parameter('land_timeout_s', 60.0)
        self.declare_parameter('test_abort_after_track_s', 0.0)

        if str(self.get_parameter('confirmation').value) != CONFIRMATION:
            raise ValueError(f'confirmation must equal {CONFIRMATION}')

        self.reference_rate_hz = float(
            self.get_parameter('reference_rate_hz').value)
        if (not math.isfinite(self.reference_rate_hz)
                or not 1.0 <= self.reference_rate_hz <= 200.0):
            raise ValueError('reference_rate_hz must be between 1 and 200')
        self.speed = float(self.get_parameter('speed_m_s').value)
        self.lap_time = float(self.get_parameter('lap_time_s').value)
        self.laps = int(self.get_parameter('laps').value)
        self.takeoff_height = float(
            self.get_parameter('takeoff_height_m').value)
        self.climb_speed = float(self.get_parameter('climb_speed_m_s').value)
        self.descent_speed = float(
            self.get_parameter('descent_speed_m_s').value)
        self.staging_horizontal_speed = float(
            self.get_parameter('staging_horizontal_speed_m_s').value)
        self.start_dwell_s = float(self.get_parameter('start_dwell_s').value)
        self.entry_duration_s = float(
            self.get_parameter('entry_duration_s').value)
        self.exit_duration_s = float(
            self.get_parameter('exit_duration_s').value)
        self.posttrack_braking_acceleration = float(
            self.get_parameter('posttrack_braking_acceleration_m_s2').value)
        self.horizontal_safety_margin = float(
            self.get_parameter('horizontal_safety_margin_m').value)
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
        self.test_abort_after_track_s = float(
            self.get_parameter('test_abort_after_track_s').value)

        positive = (
            self.speed, self.lap_time, self.takeoff_height,
            self.staging_horizontal_speed, self.climb_speed,
            self.descent_speed, self.start_dwell_s,
            self.entry_duration_s, self.exit_duration_s,
            self.posttrack_braking_acceleration,
            self.horizontal_safety_margin,
            self.horizontal_tolerance, self.vertical_tolerance,
            self.speed_tolerance, self.track_rms_limit, self.track_peak_limit,
            self.wait_timeout_s, self.takeoff_timeout_s, self.land_timeout_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError('all timing, speed, tolerance, and limit values must be positive')
        if (not math.isfinite(self.test_abort_after_track_s)
                or self.test_abort_after_track_s < 0.0):
            raise ValueError('test_abort_after_track_s must be finite and nonnegative')
        if self.speed > self._maximum_reference_speed():
            raise ValueError('speed_m_s exceeds this SITL mission limit')
        if self.takeoff_height > 12.0:
            raise ValueError('takeoff_height_m is limited to <= 12.0 for this SITL test')

        self.trajectory = self._make_trajectory()
        self.start_sample = self.trajectory.sample(0.0)
        self.track_phase_offset_s = getattr(
            self.trajectory, 'track_phase_offset_s', 0.5 * self.entry_duration_s)
        self.track_end_phase_s = (
            self.track_phase_offset_s + self.trajectory.total_duration)
        self.final_sample = self.trajectory.sample(self.track_end_phase_s)
        self.horizontal_bound = self.trajectory.horizontal_radius_bound()
        self.exit_horizontal_bound = self.horizontal_bound

        self.local_position = None
        self.control_mode = None
        self.vehicle_status = None
        self.origin = None
        self.setpoint = None
        self.setpoint_velocity = np.zeros(3)
        self.posttrack_target = None
        self.phase = 'prestream'
        self.created_ns = self.get_clock().now().nanoseconds
        self.prestream_started_ns = None
        self.phase_started_ns = None
        self.track_started_ns = None
        self.entry_started_ns = None
        self.exit_started_ns = None
        self.abort_brake_started_ns = None
        self.abort_brake_position = None
        self.abort_brake_velocity = None
        self.abort_brake_duration_s = None
        self.abort_brake_target_z = None
        self.exit_start_position = None
        self.exit_start_velocity = None
        self.exit_braking_duration_s = None
        self.stable_started_ns = None
        self.last_tick_ns = None
        self.last_command_ns = 0
        self.disarm_started_ns = None
        self.land_detected_started_ns = None
        self.land_detected_arrival_ns = None
        self.land_detected_count = 0
        self.land_timeout_reported = False
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
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._on_land_detected, PX4_OUTPUT_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self._on_vehicle_status, PX4_OUTPUT_QOS)
        self.create_timer(1.0 / self.reference_rate_hz, self._tick)
        self.get_logger().warning(
            f'Prepared automated {getattr(self.trajectory, "name", "Bernoulli lemniscate")} mission: '
            f'speed={self.speed:.2f} m/s, lap_time={self.lap_time:.1f} s, '
            f'laps={self.laps}, scale={self.trajectory.scale:.3f} m')

    def _maximum_reference_speed(self):
        return 8.0

    def _make_trajectory(self):
        return BernoulliLemniscateTrajectory(
            self.speed, self.lap_time, self.laps, self.takeoff_height)

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

    def _on_land_detected(self, message):
        now_ns = self.get_clock().now().nanoseconds
        self.land_detected_arrival_ns = now_ns
        if message.landed:
            if self.land_detected_started_ns is None:
                self.land_detected_started_ns = now_ns
            self.land_detected_count += 1
        else:
            self.land_detected_started_ns = None
            self.land_detected_count = 0

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

    def _publish_hold_or_slew(
            self, position, velocity=None, *, tracking_feedback=False):
        message = TrajectorySetpoint()
        message.timestamp = self.get_clock().now().nanoseconds // 1000
        message.position = np.asarray(position, dtype=float).tolist()
        message.velocity = (
            np.zeros(3) if velocity is None else np.asarray(velocity, dtype=float)
        ).tolist()
        message.acceleration = (
            [0.0] * 3 if tracking_feedback else [math.nan] * 3)
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
        if horizontal > self.exit_horizontal_bound + self.horizontal_safety_margin:
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
            position, velocity = self.local_position
            self.setpoint = position.copy()
            self.setpoint_velocity.fill(0.0)
            self.origin[:2] = position[:2]
            horizontal_speed = float(np.linalg.norm(velocity[:2]))
            if (self.phase in ('entry', 'track', 'exit')
                    and self._is_armed_offboard()
                    and horizontal_speed > 1.0):
                target_z = min(
                    position[2], self.origin[2] - self.takeoff_height)
                climb = position[2] - target_z
                self.abort_brake_duration_s = max(
                    2.0,
                    1.875 * horizontal_speed
                    / self.posttrack_braking_acceleration,
                    1.875 * climb / 0.60,
                )
                self.abort_brake_started_ns = now_ns
                self.abort_brake_position = position.copy()
                self.abort_brake_velocity = velocity.copy()
                self.abort_brake_target_z = target_z
                self._set_phase('abort_brake', now_ns)
                return
        self._set_phase('land', now_ns)

    def _start_exit_braking(self, now_ns):
        position, _ = self.local_position
        self.exit_braking_duration_s = self.exit_duration_s
        self.exit_started_ns = now_ns
        self.exit_start_position = position.copy()
        exit_times = np.linspace(
            0.0, self.exit_braking_duration_s, num=101)
        exit_samples = [
            self.trajectory.exit_sample(
                self.track_end_phase_s,
                exit_time,
                self.exit_braking_duration_s,
            )
            for exit_time in exit_times
        ]
        self.posttrack_target = self.origin + exit_samples[-1].position
        self.exit_horizontal_bound = max(
            self.horizontal_bound,
            float(np.linalg.norm((position - self.origin)[:2])),
            max(float(np.linalg.norm(sample.position[:2]))
                for sample in exit_samples),
        )
        self.setpoint = self.posttrack_target.copy()
        self.setpoint_velocity.fill(0.0)
        self._set_phase('exit', now_ns)

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
            'reference_rate_hz': self.reference_rate_hz,
            'laps': self.laps,
            'takeoff_height_m': self.takeoff_height,
            'scale_m': self.trajectory.scale,
            'horizontal_bound_m': self.horizontal_bound,
            'exit_horizontal_bound_m': self.exit_horizontal_bound,
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
            dt = 1.0 / self.reference_rate_hz
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

        if self.aborted and near_ground_runaway(
                self.local_position[0], self.origin,
                self.exit_horizontal_bound + self.horizontal_safety_margin):
            self.get_logger().error(
                'SITL near-ground runaway after abort; emergency disarm')
            self.disarm_started_ns = now_ns
            self._request_forced_disarm(now_ns)
            return

        if self.phase == 'takeoff':
            target = self.origin + self.start_sample.position
            self.setpoint, self.setpoint_velocity = slew_setpoint_with_velocity(
                self.setpoint, target, dt,
                self.staging_horizontal_speed, self.climb_speed)
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
                    self.entry_started_ns = now_ns
                    self._set_phase('entry', now_ns)
            else:
                self.stable_started_ns = None
            if (now_ns - self.phase_started_ns) * 1e-9 > self.takeoff_timeout_s:
                self._abort_to_land('takeoff phase timed out', now_ns)
            return

        if self.phase == 'entry':
            elapsed_s = (now_ns - self.entry_started_ns) * 1e-9
            if elapsed_s <= self.entry_duration_s:
                reference = self.trajectory.entry_sample(
                    elapsed_s, self.entry_duration_s)
                self._publish_sample(LemniscateSample(
                    self.origin + reference.position,
                    reference.velocity,
                    reference.acceleration,
                    reference.jerk,
                    reference.yaw,
                    reference.yawspeed,
                ))
            else:
                self.track_started_ns = now_ns
                self._set_phase('track', now_ns)
            return

        if self.phase == 'track':
            elapsed_s = (now_ns - self.track_started_ns) * 1e-9
            if (self.test_abort_after_track_s > 0.0
                    and elapsed_s >= self.test_abort_after_track_s):
                self._abort_to_land('requested test abort', now_ns)
                return
            if elapsed_s <= self.trajectory.total_duration:
                reference = self._absolute_sample(
                    self.track_phase_offset_s + elapsed_s)
                self._publish_sample(reference)
                self._record_tracking_sample(elapsed_s, reference)
            else:
                self._start_exit_braking(now_ns)
            return

        if self.phase == 'exit':
            elapsed_s = (now_ns - self.exit_started_ns) * 1e-9
            if elapsed_s <= self.exit_braking_duration_s:
                reference = self.trajectory.exit_sample(
                    self.track_end_phase_s,
                    elapsed_s,
                    self.exit_braking_duration_s,
                )
                self._publish_sample(LemniscateSample(
                    self.origin + reference.position,
                    reference.velocity,
                    reference.acceleration,
                    reference.jerk,
                    reference.yaw,
                    reference.yawspeed,
                ))
            else:
                self.setpoint = self.posttrack_target.copy()
                self.setpoint_velocity.fill(0.0)
                self._set_phase('land', now_ns)
            return

        if self.phase == 'abort_brake':
            elapsed_s = (now_ns - self.abort_brake_started_ns) * 1e-9
            reference = abort_brake_sample(
                self.abort_brake_position, self.abort_brake_velocity,
                min(elapsed_s, self.abort_brake_duration_s),
                self.abort_brake_duration_s, self.abort_brake_target_z)
            if elapsed_s <= self.abort_brake_duration_s:
                self._publish_sample(reference)
            else:
                self.setpoint = reference.position.copy()
                self.setpoint_velocity.fill(0.0)
                self.origin[:2] = reference.position[:2]
                self._set_phase('land', now_ns)
            return

        if self.phase == 'land':
            target = self.origin.copy()
            position, velocity = self.local_position
            if not landing_descent_allowed(position, velocity, self.setpoint):
                target[2] = self.setpoint[2]
            self.setpoint, self.setpoint_velocity = slew_setpoint_with_velocity(
                self.setpoint, target, dt,
                self.staging_horizontal_speed, self.descent_speed)
            self._publish_hold_or_slew(
                self.setpoint,
                self.setpoint_velocity,
                tracking_feedback=False,
            )
            landed_confirmed = landed_state_confirmed(
                self.land_detected_count, self.land_detected_started_ns,
                self.land_detected_arrival_ns, now_ns)
            if landing_disarm_allowed(
                    position, velocity, self.origin, landed_confirmed):
                self.disarm_started_ns = now_ns
                self._request_forced_disarm(now_ns)
                return
            if ((now_ns - self.phase_started_ns) * 1e-9 > self.land_timeout_s
                    and not self.land_timeout_reported):
                self.aborted = True
                self.land_timeout_reported = True
                self.get_logger().error(
                    'Landing timed out above ground; holding offboard control')


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
