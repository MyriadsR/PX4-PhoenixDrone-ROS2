"""Bounded attitude-step publisher and response checker for SITL/tethered tests."""

import json
import math

import numpy as np
import rclpy
from px4_msgs.msg import VehicleAngularVelocity, VehicleAttitude, VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleControlMode, VehicleLandDetected
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .config import PhoenixHoverConfig
from .debug import (
    ACTUATOR_FEEDBACK_DEBUG_FIELDS,
    ALPHA_MODEL_DEBUG_FIELDS,
    CONTROL_DEBUG_FIELDS,
)
from .frames import attitude_px4_to_ts, vector_px4_to_ts
from .math_utils import normalize_quaternion, quaternion_multiply, rotation_vector_error
from .qos import PX4_INPUT_QOS, PX4_OUTPUT_QOS


CONFIRMATION = 'PHOENIX_ATTITUDE_STEP_TEST'
AXES = {'x': 0, 'y': 1, 'z': 2}
CONTROL_DEBUG_INDEX = {name: index for index, name in enumerate(CONTROL_DEBUG_FIELDS)}
ACTUATOR_FEEDBACK_DEBUG_INDEX = {
    name: index for index, name in enumerate(ACTUATOR_FEEDBACK_DEBUG_FIELDS)}
ALPHA_MODEL_DEBUG_INDEX = {
    name: index for index, name in enumerate(ALPHA_MODEL_DEBUG_FIELDS)}


def axis_angle_quaternion(axis_index, angle):
    """Return a Hamilton quaternion for a body-axis rotation."""
    if axis_index not in (0, 1, 2) or not math.isfinite(angle):
        raise ValueError('invalid axis-angle command')
    result = np.zeros(4)
    result[0] = math.cos(0.5 * angle)
    result[axis_index + 1] = math.sin(0.5 * angle)
    return result


def step_phase(elapsed_s, baseline_s, step_s, recovery_s):
    """Return phase name and commanded angle for a +step/-step sequence."""
    boundaries = (
        ('baseline_pre', baseline_s, 0.0),
        ('plus', step_s, 1.0),
        ('baseline_mid', recovery_s, 0.0),
        ('minus', step_s, -1.0),
        ('baseline_post', recovery_s, 0.0),
    )
    cursor = 0.0
    for name, duration, sign in boundaries:
        cursor += duration
        if elapsed_s < cursor:
            return name, sign
    return 'done', 0.0


def summarize_axis_response(records, axis_index, amplitude):
    """Summarize response records collected in Tailsitter-control axes."""
    result = {'sample_count': len(records), 'passed': False}
    if not records:
        result['reason'] = 'no attitude samples'
        return result

    def steady_value(phase):
        samples = [item for item in records if item[0] == phase]
        if not samples:
            return math.nan
        tail = samples[max(0, len(samples) * 3 // 4):]
        return float(np.median([item[1][axis_index] for item in tail]))

    plus_steady = steady_value('plus')
    minus_steady = steady_value('minus')
    angles = np.asarray([item[1] for item in records], dtype=float)
    rates = np.asarray([item[2] for item in records], dtype=float)
    cross_axes = [index for index in range(3) if index != axis_index]
    cross_peak = float(np.max(np.abs(angles[:, cross_axes])))
    angle_peak = float(np.max(np.abs(angles[:, axis_index])))
    rate_peak = float(np.max(np.abs(rates)))
    minimum_response = min(0.01, 0.20 * abs(amplitude))
    direction_ok = plus_steady > minimum_response and minus_steady < -minimum_response
    angle_limit = max(0.10, 3.0 * abs(amplitude))
    cross_axis_limit = max(0.075, 2.0 * abs(amplitude))
    bounded = angle_peak < angle_limit and cross_peak < cross_axis_limit and rate_peak < 1.0
    result.update({
        'plus_steady_rad': plus_steady,
        'minus_steady_rad': minus_steady,
        'axis_peak_abs_rad': angle_peak,
        'cross_axis_peak_abs_rad': cross_peak,
        'angular_rate_peak_abs_rad_s': rate_peak,
        'axis_peak_limit_rad': angle_limit,
        'cross_axis_peak_limit_rad': cross_axis_limit,
        'direction_ok': bool(direction_ok),
        'bounded': bool(bounded),
        'passed': bool(direction_ok and bounded),
    })
    return result


def _median_or_nan(values):
    if not values:
        return math.nan
    return float(np.median(values))


def summarize_control_debug(records, axis_index):
    """Summarize controller internals by step phase."""
    result = {'control_debug_sample_count': len(records)}
    if not records:
        return result

    axis_suffix = ('x', 'y', 'z')[axis_index]
    axis_fields = {
        'attitude_error_axis_rad': f'attitude_error_{axis_suffix}',
        'rate_lpf_axis_rad_s': f'rate_lpf_{axis_suffix}',
        'angular_acceleration_command_axis_rad_s2':
            f'angular_acceleration_command_{axis_suffix}',
        'estimated_moment_axis_nm': f'estimated_moment_lpf_{axis_suffix}',
        'desired_moment_axis_nm': f'desired_moment_{axis_suffix}',
        'allocated_moment_axis_nm': f'allocated_moment_{axis_suffix}',
    }
    scalar_fields = {
        'motor_left_normalized': 'motor_left_normalized',
        'motor_right_normalized': 'motor_right_normalized',
        'flap_left_rad': 'flap_left_rad',
        'flap_right_rad': 'flap_right_rad',
        'total_thrust_n': 'total_thrust_n',
        'output_active': 'output_active',
    }
    fields = {**axis_fields, **scalar_fields}

    for phase in ('baseline_pre', 'plus', 'baseline_mid', 'minus', 'baseline_post'):
        samples = [data for sample_phase, data in records if sample_phase == phase]
        tail = samples[max(0, len(samples) * 3 // 4):]
        for output_name, field_name in fields.items():
            index = CONTROL_DEBUG_INDEX[field_name]
            result[f'{phase}_debug_{output_name}'] = _median_or_nan(
                [sample[index] for sample in tail])
    return result


def summarize_actuator_feedback_debug(records):
    """Summarize actuator feedback estimator state by step phase."""
    result = {'actuator_feedback_debug_sample_count': len(records)}
    if not records:
        return result

    fields = {
        'predicted_flap_left_rad': 'predicted_flap_left_rad',
        'predicted_flap_right_rad': 'predicted_flap_right_rad',
        'feedback_flap_left_rad': 'feedback_flap_left_rad',
        'feedback_flap_right_rad': 'feedback_flap_right_rad',
        'flap_error_left_rad': 'flap_error_left_rad',
        'flap_error_right_rad': 'flap_error_right_rad',
        'feedback_age_s': 'feedback_age_s',
        'feedback_complete': 'feedback_complete',
        'feedback_active': 'feedback_active',
    }
    for phase in ('baseline_pre', 'plus', 'baseline_mid', 'minus', 'baseline_post'):
        samples = [data for sample_phase, data in records if sample_phase == phase]
        tail = samples[max(0, len(samples) * 3 // 4):]
        for output_name, field_name in fields.items():
            index = ACTUATOR_FEEDBACK_DEBUG_INDEX[field_name]
            result[f'{phase}_actuator_debug_{output_name}'] = _median_or_nan(
                [sample[index] for sample in tail])
    return result


def summarize_alpha_model_debug(records, axis_index):
    """Summarize alpha-model estimate state by step phase."""
    result = {'alpha_model_debug_sample_count': len(records)}
    if not records:
        return result

    axis_suffix = ('x', 'y', 'z')[axis_index]
    fields = {
        'velocity_body_axis_m_s': f'velocity_body_ts_{axis_suffix}',
        'force_alpha_x_n': 'force_alpha_x',
        'force_alpha_z_n': 'force_alpha_z',
        'moment_body_axis_nm': f'moment_body_ts_{axis_suffix}',
        'flap_lpf_left_rad': 'flap_lpf_left_rad',
        'flap_lpf_right_rad': 'flap_lpf_right_rad',
        'flap_hpf_left_rad': 'flap_hpf_left_rad',
        'flap_hpf_right_rad': 'flap_hpf_right_rad',
    }
    for phase in ('baseline_pre', 'plus', 'baseline_mid', 'minus', 'baseline_post'):
        samples = [data for sample_phase, data in records if sample_phase == phase]
        tail = samples[max(0, len(samples) * 3 // 4):]
        for output_name, field_name in fields.items():
            index = ALPHA_MODEL_DEBUG_INDEX[field_name]
            result[f'{phase}_alpha_debug_{output_name}'] = _median_or_nan(
                [sample[index] for sample in tail])
    return result


def summarize_land_detected(records):
    """Summarize PX4 land-detector state by step phase."""
    result = {'land_detected_sample_count': len(records)}
    if not records:
        return result
    for phase in ('baseline_pre', 'plus', 'baseline_mid', 'minus', 'baseline_post'):
        samples = [data for sample_phase, data in records if sample_phase == phase]
        result[f'{phase}_landed_fraction'] = _median_or_nan(
            [float(data[0]) for data in samples])
        result[f'{phase}_ground_contact_fraction'] = _median_or_nan(
            [float(data[1]) for data in samples])
    return result


class AttitudeStepTest(Node):
    """Publish one bounded +step/-step sequence after armed/offboard is observed."""

    def __init__(self):
        super().__init__('phoenix_attitude_step_test')
        self.declare_parameter('confirmation', '')
        self.declare_parameter('axis', 'x')
        self.declare_parameter('amplitude_rad', 0.05)
        self.declare_parameter('baseline_s', 0.4)
        self.declare_parameter('step_s', 0.9)
        self.declare_parameter('recovery_s', 0.5)
        self.declare_parameter('hover_thrust_scale', 1.0)
        self.declare_parameter('wait_timeout_s', 45.0)

        confirmation = str(self.get_parameter('confirmation').value)
        self.axis_name = str(self.get_parameter('axis').value).lower()
        self.amplitude = float(self.get_parameter('amplitude_rad').value)
        self.baseline_s = float(self.get_parameter('baseline_s').value)
        self.step_s = float(self.get_parameter('step_s').value)
        self.recovery_s = float(self.get_parameter('recovery_s').value)
        self.hover_thrust_scale = float(self.get_parameter('hover_thrust_scale').value)
        self.wait_timeout_s = float(self.get_parameter('wait_timeout_s').value)
        if confirmation != CONFIRMATION:
            raise ValueError(f'confirmation must equal {CONFIRMATION}')
        if self.axis_name not in AXES:
            raise ValueError('axis must be x, y, or z in Tailsitter-control coordinates')
        amplitude_limit = 0.80 if self.axis_name == 'x' else 0.15
        if not math.isfinite(self.amplitude) or not 0.0 < abs(self.amplitude) <= amplitude_limit:
            raise ValueError(
                f'amplitude_rad must be finite and in (0, {amplitude_limit}] '
                f'for TS {self.axis_name}'
            )
        durations = (self.baseline_s, self.step_s, self.recovery_s, self.wait_timeout_s)
        if not all(math.isfinite(value) and value > 0.0 for value in durations):
            raise ValueError('all durations must be finite and positive')
        if not math.isfinite(self.hover_thrust_scale) or not 0.8 <= self.hover_thrust_scale <= 1.2:
            raise ValueError('hover_thrust_scale must be finite and in [0.8, 1.2]')

        self.axis_index = AXES[self.axis_name]
        self.cfg = PhoenixHoverConfig()
        self.initial_q_ts = None
        self.latest_rates_ts = np.zeros(3)
        self.control_mode = None
        self.started_ns = None
        self.created_ns = self.get_clock().now().nanoseconds
        self.active_once = False
        self.done = False
        self.exit_code = 1
        self.records = []
        self.control_debug_records = []
        self.actuator_feedback_debug_records = []
        self.alpha_model_debug_records = []
        self.land_detected_records = []

        self.publisher = self.create_publisher(
            VehicleAttitudeSetpoint, '/phoenix_tailsitter/attitude_setpoint', PX4_INPUT_QOS)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self._on_attitude, PX4_OUTPUT_QOS)
        self.create_subscription(
            VehicleAngularVelocity, '/fmu/out/vehicle_angular_velocity', self._on_rates,
            PX4_OUTPUT_QOS)
        self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self._on_mode,
            PX4_OUTPUT_QOS)
        self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._on_land_detected, PX4_OUTPUT_QOS)
        self.create_subscription(
            Float64MultiArray, '/phoenix_tailsitter/control_debug',
            self._on_control_debug, 10)
        self.create_subscription(
            Float64MultiArray, '/phoenix_tailsitter/actuator_feedback_debug',
            self._on_actuator_feedback_debug, 10)
        self.create_subscription(
            Float64MultiArray, '/phoenix_tailsitter/alpha_model_debug',
            self._on_alpha_model_debug, 10)
        self.create_timer(0.02, self._tick)
        self.get_logger().warning(
            f'Prepared TS {self.axis_name}-axis +/-{abs(self.amplitude):.3f} rad test; '
            'waiting for operator-controlled armed+offboard state')

    def _on_rates(self, message):
        rates = vector_px4_to_ts(message.xyz)
        if np.all(np.isfinite(rates)):
            self.latest_rates_ts = rates

    def _on_mode(self, message):
        self.control_mode = message

    def _on_land_detected(self, message):
        phase = self._current_phase()
        if phase is None:
            return
        self.land_detected_records.append(
            (phase, (bool(message.landed), bool(message.ground_contact))))

    def _current_phase(self):
        if self.started_ns is None or self.done:
            return None
        elapsed_s = (self.get_clock().now().nanoseconds - self.started_ns) * 1e-9
        phase, _ = step_phase(
            elapsed_s, self.baseline_s, self.step_s, self.recovery_s)
        if phase == 'done':
            return None
        return phase

    def _on_attitude(self, message):
        try:
            current_q_ts = attitude_px4_to_ts(message.q)
        except ValueError:
            return
        # Follow the vehicle while disarmed.  Consecutive tests may start
        # while the model is still settling after landing; freezing the
        # reference before armed+offboard would misreport that motion as
        # cross-axis step response.
        if self.started_ns is None:
            self.initial_q_ts = current_q_ts
        phase = self._current_phase()
        if phase is not None:
            response = rotation_vector_error(self.initial_q_ts, current_q_ts)
            self.records.append((phase, response, self.latest_rates_ts.copy()))

    def _on_control_debug(self, message):
        phase = self._current_phase()
        if phase is None:
            return
        data = np.asarray(message.data, dtype=float)
        if data.size != len(CONTROL_DEBUG_FIELDS) or not np.all(np.isfinite(data)):
            return
        self.control_debug_records.append((phase, data.copy()))

    def _on_actuator_feedback_debug(self, message):
        phase = self._current_phase()
        if phase is None:
            return
        data = np.asarray(message.data, dtype=float)
        if (data.size != len(ACTUATOR_FEEDBACK_DEBUG_FIELDS)
                or not np.all(np.isfinite(data))):
            return
        self.actuator_feedback_debug_records.append((phase, data.copy()))

    def _on_alpha_model_debug(self, message):
        phase = self._current_phase()
        if phase is None:
            return
        data = np.asarray(message.data, dtype=float)
        if data.size != len(ALPHA_MODEL_DEBUG_FIELDS) or not np.all(np.isfinite(data)):
            return
        self.alpha_model_debug_records.append((phase, data.copy()))

    def _publish_setpoint(self, commanded_angle):
        if self.initial_q_ts is None:
            return
        delta = axis_angle_quaternion(self.axis_index, commanded_angle)
        desired = normalize_quaternion(quaternion_multiply(self.initial_q_ts, delta))
        message = VehicleAttitudeSetpoint()
        message.timestamp = self.get_clock().now().nanoseconds // 1000
        message.q_d = desired.tolist()
        hover_normalized = (
            self.hover_thrust_scale
            * self.cfg.hover_total_thrust
            / self.cfg.maximum_total_thrust
        )
        message.thrust_body = [float(hover_normalized), 0.0, 0.0]
        self.publisher.publish(message)

    def _finish(self, aborted=False):
        if self.done:
            return
        result = summarize_axis_response(self.records, self.axis_index, self.amplitude)
        result.update(summarize_control_debug(self.control_debug_records, self.axis_index))
        result.update(summarize_actuator_feedback_debug(
            self.actuator_feedback_debug_records))
        result.update(summarize_alpha_model_debug(
            self.alpha_model_debug_records, self.axis_index))
        result.update(summarize_land_detected(self.land_detected_records))
        result.update({
            'axis_ts': self.axis_name,
            'amplitude_rad': self.amplitude,
            'hover_thrust_scale': self.hover_thrust_scale,
            'aborted': bool(aborted),
        })
        self.done = True
        self.exit_code = 0 if result['passed'] and not aborted else 2
        self.get_logger().info('ATTITUDE_STEP_RESULT ' + json.dumps(result, sort_keys=True))

    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.initial_q_ts is None:
            if (now_ns - self.created_ns) * 1e-9 > self.wait_timeout_s:
                self.get_logger().error('Timed out waiting for vehicle attitude')
                self._finish(aborted=True)
            return

        armed_offboard = bool(
            self.control_mode is not None
            and self.control_mode.flag_armed
            and self.control_mode.flag_control_offboard_enabled
        )
        if self.started_ns is None:
            self._publish_setpoint(0.0)
            if armed_offboard:
                self.started_ns = now_ns
                self.active_once = True
                self.get_logger().info('Armed+offboard observed; starting step sequence')
            elif (now_ns - self.created_ns) * 1e-9 > self.wait_timeout_s:
                self.get_logger().error('Timed out waiting for armed+offboard state')
                self._finish(aborted=True)
            return

        elapsed_s = (now_ns - self.started_ns) * 1e-9
        phase, sign = step_phase(
            elapsed_s, self.baseline_s, self.step_s, self.recovery_s)
        self._publish_setpoint(sign * self.amplitude)
        if self.active_once and not armed_offboard and phase != 'done':
            self.get_logger().error('Vehicle disarmed before the step sequence completed')
            self._finish(aborted=True)
        elif phase == 'done':
            self._finish()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = AttitudeStepTest()
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
