"""Bounded NED position-step publisher and response checker for SITL."""

import json
import math

import numpy as np
import rclpy
from px4_msgs.msg import TrajectorySetpoint, VehicleControlMode, VehicleLocalPosition
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .qos import PX4_INPUT_QOS, PX4_OUTPUT_QOS


CONFIRMATION = 'PHOENIX_POSITION_STEP_TEST'
AXES = {'x': 0, 'y': 1, 'z': 2}


def position_step_phase(elapsed_s, climb_s, baseline_s, step_s, recovery_s):
    phases = (
        ('climb', climb_s, 0.0),
        ('baseline_pre', baseline_s, 0.0),
        ('plus', step_s, 1.0),
        ('baseline_mid', recovery_s, 0.0),
        ('minus', step_s, -1.0),
        ('baseline_post', recovery_s, 0.0),
    )
    cursor = 0.0
    for name, duration, sign in phases:
        cursor += duration
        if elapsed_s < cursor:
            return name, sign
    return 'done', 0.0


def summarize_position_response(records, axis_index, amplitude):
    result = {'sample_count': len(records), 'passed': False}
    if not records:
        result['reason'] = 'no local-position samples'
        return result

    def steady_position(phase):
        samples = [item[1] for item in records if item[0] == phase]
        if not samples:
            return None
        tail = samples[max(0, len(samples) * 3 // 4):]
        return np.median(np.asarray(tail), axis=0)

    baseline_pre = steady_position('baseline_pre')
    baseline_mid = steady_position('baseline_mid')
    plus = steady_position('plus')
    minus = steady_position('minus')
    if any(value is None for value in (baseline_pre, baseline_mid, plus, minus)):
        result['reason'] = 'one or more steady phases have no samples'
        return result

    plus_response = float(plus[axis_index] - baseline_pre[axis_index])
    minus_response = float(minus[axis_index] - baseline_mid[axis_index])
    minimum_response = min(0.02, 0.20 * abs(amplitude))
    direction_ok = plus_response > minimum_response and minus_response < -minimum_response
    velocities = np.asarray([item[2] for item in records], dtype=float)
    velocity_peak = float(np.max(np.abs(velocities)))
    positions = np.asarray([item[1] for item in records], dtype=float)
    position_span = np.ptp(positions, axis=0)
    cross_axes = [index for index in range(3) if index != axis_index]
    axis_span = float(position_span[axis_index])
    cross_span = float(np.max(position_span[cross_axes]))
    axis_limit = max(0.35, 3.0 * abs(amplitude))
    cross_limit = max(0.25, 2.0 * abs(amplitude))
    bounded = axis_span < axis_limit and cross_span < cross_limit and velocity_peak < 1.5
    result.update({
        'plus_steady_response_m': plus_response,
        'minus_steady_response_m': minus_response,
        'axis_span_m': axis_span,
        'cross_axis_span_m': cross_span,
        'velocity_peak_abs_m_s': velocity_peak,
        'direction_ok': bool(direction_ok),
        'bounded': bool(bounded),
        'passed': bool(direction_ok and bounded),
    })
    return result


def summarize_takeoff_response(records, origin, hover_reference,
                               takeoff_height):
    """Require actual climb and a bounded steady hover near the target."""
    result = {'takeoff_passed': False}
    if not records:
        result['takeoff_reason'] = 'no active-flight samples'
        return result
    origin = np.asarray(origin, dtype=float)
    target = np.asarray(hover_reference, dtype=float)
    if (origin.shape != (3,) or target.shape != (3,)
            or not np.all(np.isfinite(np.concatenate((origin, target))))):
        result['takeoff_reason'] = 'invalid origin or hover reference'
        return result

    positions = np.asarray([item[1] for item in records], dtype=float)
    baseline = [item for item in records if item[0] == 'baseline_pre']
    if not baseline:
        result['takeoff_reason'] = 'no baseline hover samples'
        return result
    tail = baseline[max(0, len(baseline) * 3 // 4):]
    steady_position = np.median(
        np.asarray([item[1] for item in tail], dtype=float), axis=0)
    steady_velocity = np.asarray([item[2] for item in tail], dtype=float)
    achieved_climb = float(origin[2] - np.min(positions[:, 2]))
    target_error = float(abs(steady_position[2] - target[2]))
    horizontal_error = float(np.linalg.norm(
        steady_position[:2] - target[:2]))
    vertical_speed_peak = float(np.max(np.abs(steady_velocity[:, 2])))
    required_climb = 0.75 * float(takeoff_height)
    height_tolerance = max(0.08, 0.25 * float(takeoff_height))
    reached = achieved_climb >= required_climb
    stable = (
        target_error <= height_tolerance
        and horizontal_error <= 0.25
        and vertical_speed_peak <= 0.25
    )
    result.update({
        'takeoff_achieved_climb_m': achieved_climb,
        'takeoff_required_climb_m': required_climb,
        'hover_height_error_abs_m': target_error,
        'hover_height_tolerance_m': height_tolerance,
        'hover_horizontal_error_m': horizontal_error,
        'hover_vertical_speed_peak_m_s': vertical_speed_peak,
        'takeoff_reached': bool(reached),
        'hover_stable': bool(stable),
        'takeoff_passed': bool(reached and stable),
    })
    return result


class PositionStepTest(Node):
    """Publish one takeoff and bounded +/- NED position-step sequence."""

    def __init__(self):
        super().__init__('phoenix_position_step_test')
        self.declare_parameter('confirmation', '')
        self.declare_parameter('axis', 'x')
        self.declare_parameter('amplitude_m', 0.10)
        self.declare_parameter('takeoff_height_m', 0.20)
        self.declare_parameter('climb_s', 5.0)
        self.declare_parameter('baseline_s', 2.0)
        self.declare_parameter('step_s', 2.5)
        self.declare_parameter('recovery_s', 2.0)
        self.declare_parameter('wait_timeout_s', 45.0)

        confirmation = str(self.get_parameter('confirmation').value)
        self.axis_name = str(self.get_parameter('axis').value).lower()
        self.amplitude = float(self.get_parameter('amplitude_m').value)
        self.takeoff_height = float(self.get_parameter('takeoff_height_m').value)
        self.climb_s = float(self.get_parameter('climb_s').value)
        self.baseline_s = float(self.get_parameter('baseline_s').value)
        self.step_s = float(self.get_parameter('step_s').value)
        self.recovery_s = float(self.get_parameter('recovery_s').value)
        self.wait_timeout_s = float(self.get_parameter('wait_timeout_s').value)
        if confirmation != CONFIRMATION:
            raise ValueError(f'confirmation must equal {CONFIRMATION}')
        if self.axis_name not in AXES:
            raise ValueError('axis must be x, y, or z in PX4 NED coordinates')
        if not math.isfinite(self.amplitude) or not 0.0 < abs(self.amplitude) <= 0.15:
            raise ValueError('amplitude_m must be finite and in (0, 0.15]')
        if not math.isfinite(self.takeoff_height) or not 0.15 <= self.takeoff_height <= 2.0:
            raise ValueError('takeoff_height_m must be finite and in [0.15, 2.0]')
        durations = (self.climb_s, self.baseline_s, self.step_s,
                     self.recovery_s, self.wait_timeout_s)
        if not all(math.isfinite(value) and value > 0.0 for value in durations):
            raise ValueError('all durations must be finite and positive')

        self.axis_index = AXES[self.axis_name]
        self.local_position = None
        self.control_mode = None
        self.origin = None
        self.hover_reference = None
        self.started_ns = None
        self.created_ns = self.get_clock().now().nanoseconds
        self.active_once = False
        self.done = False
        self.result_reported = False
        self.holding_after_result = False
        self.exit_code = 1
        self.records = []
        self.flight_records = []

        self.publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_INPUT_QOS)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._on_local_position, PX4_OUTPUT_QOS)
        self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode',
            self._on_mode, PX4_OUTPUT_QOS)
        self.create_timer(0.02, self._tick)
        self.get_logger().warning(
            f'Prepared NED {self.axis_name}-axis +/-{abs(self.amplitude):.2f} m test; '
            'waiting for operator-controlled armed+offboard state')

    def _on_mode(self, message):
        self.control_mode = message

    def _on_local_position(self, message):
        position = np.array([message.x, message.y, message.z], dtype=float)
        velocity = np.array([message.vx, message.vy, message.vz], dtype=float)
        if not np.all(np.isfinite(np.concatenate((position, velocity)))):
            return
        self.local_position = (position, velocity)
        if self.started_ns is None:
            self.origin = position.copy()
        elif not self.done:
            horizontal_displacement = float(np.linalg.norm(
                position[:2] - self.origin[:2]))
            velocity_peak = float(np.max(np.abs(velocity)))
            if horizontal_displacement > 1.0 or velocity_peak > 1.5:
                self.get_logger().error(
                    'Safety bound exceeded: horizontal displacement '
                    f'{horizontal_displacement:.3f} m, velocity {velocity_peak:.3f} m/s')
                self._finish(aborted=True)
                return
            elapsed_s = (self.get_clock().now().nanoseconds - self.started_ns) * 1e-9
            phase, _ = position_step_phase(
                elapsed_s, self.climb_s, self.baseline_s,
                self.step_s, self.recovery_s)
            if phase != 'done':
                self.flight_records.append(
                    (phase, position.copy(), velocity.copy()))
            if phase not in ('climb', 'done'):
                self.records.append((phase, position.copy(), velocity.copy()))

    def _publish_setpoint(self, reference):
        message = TrajectorySetpoint()
        message.timestamp = self.get_clock().now().nanoseconds // 1000
        message.position = np.asarray(reference, dtype=float).tolist()
        message.velocity = [math.nan] * 3
        message.acceleration = [math.nan] * 3
        message.jerk = [math.nan] * 3
        message.yaw = math.nan
        message.yawspeed = math.nan
        self.publisher.publish(message)

    def _finish(self, aborted=False):
        if self.done or self.result_reported:
            return
        result = summarize_position_response(self.records, self.axis_index, self.amplitude)
        step_response_passed = bool(result['passed'])
        takeoff_result = summarize_takeoff_response(
            self.flight_records,
            self.origin,
            self.hover_reference,
            self.takeoff_height,
        )
        result.update(takeoff_result)
        result.update({
            'axis_ned': self.axis_name,
            'amplitude_m': self.amplitude,
            'takeoff_height_m': self.takeoff_height,
            'step_response_passed': step_response_passed,
            'aborted': bool(aborted),
        })
        result['passed'] = bool(
            step_response_passed
            and result['takeoff_passed']
            and not aborted)
        self.result_reported = True
        self.exit_code = 0 if result['passed'] else 2
        self.get_logger().info('POSITION_STEP_RESULT ' + json.dumps(result, sort_keys=True))
        armed_offboard = bool(
            self.control_mode is not None
            and self.control_mode.flag_armed
            and self.control_mode.flag_control_offboard_enabled
        )
        if not aborted and armed_offboard and self.hover_reference is not None:
            self.holding_after_result = True
            self.get_logger().warning(
                'Test sequence finished; holding the hover setpoint until '
                'the operator disarms the vehicle')
        else:
            self.done = True

    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.origin is None:
            if (now_ns - self.created_ns) * 1e-9 > self.wait_timeout_s:
                self.get_logger().error('Timed out waiting for local position')
                self._finish(aborted=True)
            return

        armed_offboard = bool(
            self.control_mode is not None
            and self.control_mode.flag_armed
            and self.control_mode.flag_control_offboard_enabled
        )
        if self.holding_after_result:
            self._publish_setpoint(self.hover_reference)
            if not armed_offboard:
                self.holding_after_result = False
                self.done = True
            return
        if self.started_ns is None:
            self._publish_setpoint(self.origin)
            if armed_offboard:
                self.started_ns = now_ns
                self.active_once = True
                self.origin = self.local_position[0].copy()
                self.hover_reference = self.origin.copy()
                self.hover_reference[2] -= self.takeoff_height
                self.get_logger().info(
                    f'Armed+offboard observed; climbing to {self.hover_reference.tolist()}')
            elif (now_ns - self.created_ns) * 1e-9 > self.wait_timeout_s:
                self.get_logger().error('Timed out waiting for armed+offboard state')
                self._finish(aborted=True)
            return

        elapsed_s = (now_ns - self.started_ns) * 1e-9
        phase, sign = position_step_phase(
            elapsed_s, self.climb_s, self.baseline_s,
            self.step_s, self.recovery_s)
        reference = self.hover_reference.copy()
        reference[self.axis_index] += sign * self.amplitude
        self._publish_setpoint(reference)
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
        node = PositionStepTest()
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
