"""Full Tailsitter-control alpha-model controller for PhoenixDrone."""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64MultiArray

from .alpha_aerodynamics import AlphaAerodynamics
from .alpha_allocator import AlphaTheoryAllocator
from .controller_node import TailsitterController
from .debug import (
    pack_alpha_model_debug,
    pack_control_debug,
    pack_position_debug,
)
from .filters import ButterworthHighPass, ButterworthLowPass
from .flatness_control import FlatnessAttitudeController
from .frames import attitude_px4_to_ts, vector_px4_to_ts
from .math_utils import quaternion_to_matrix, rotation_vector_error
from .position_control import (
    degraded_vertical_force,
    resolve_trajectory_reference,
    tailsitter_heading_from_attitude,
)


class AlphaTailsitterController(TailsitterController):
    """Parallel controller path implementing study.md section 11.11."""

    def __init__(self):
        super().__init__()
        self.aerodynamics = AlphaAerodynamics(self.cfg)
        self.allocator = AlphaTheoryAllocator(self.cfg)
        self.flatness_controller = FlatnessAttitudeController(self.cfg)
        self.alpha_motor_filter = None
        self.alpha_flap_filter = None
        self.alpha_flap_hpf_filter = None
        self.motor_speed_lpf = np.zeros(2)
        self.flap_angle_lpf = np.zeros(2)
        self.flap_angle_hpf = np.zeros(2)
        self.flap_angle_without_transient = np.zeros(2)
        self.alpha_debug_publisher = self.create_publisher(
            Float64MultiArray, '/phoenix_tailsitter/alpha_model_debug', 10)
        self.get_logger().warning(
            'Full alpha-theory controller selected; aerodynamic IDENTIFY '
            'defaults are provisional PhoenixDrone starting values.')

    def _reset_alpha_filters(self):
        self.alpha_motor_filter = None
        self.alpha_flap_filter = None
        self.alpha_flap_hpf_filter = None
        self.motor_speed_lpf.fill(0.0)
        self.flap_angle_lpf.fill(0.0)
        self.flap_angle_hpf.fill(0.0)
        self.flap_angle_without_transient.fill(0.0)

    def _reset_dynamic_state(self):
        super()._reset_dynamic_state()
        # During the base constructor this override cannot be reached by the
        # executor, but guard the attributes to keep reset independently safe.
        if hasattr(self, 'motor_speed_lpf'):
            self._reset_alpha_filters()

    def _update_alpha_filters(self):
        if self.alpha_motor_filter is None:
            self.alpha_motor_filter = ButterworthLowPass(
                self.cfg.indi_lpf_cutoff_hz,
                self.cfg.control_rate_hz,
                2,
                self.motor_speed_estimate,
            )
            self.alpha_flap_filter = ButterworthLowPass(
                self.cfg.indi_lpf_cutoff_hz,
                self.cfg.control_rate_hz,
                2,
                self.flap_angle_estimate,
            )
            self.alpha_flap_hpf_filter = ButterworthHighPass(
                self.cfg.flap_hpf_cutoff_hz,
                self.cfg.control_rate_hz,
                2,
                self.flap_angle_estimate,
            )
        self.motor_speed_lpf = np.maximum(
            self.alpha_motor_filter.update(self.motor_speed_estimate), 0.0)
        self.flap_angle_lpf = self.alpha_flap_filter.update(
            self.flap_angle_estimate)
        self.flap_angle_hpf = self.alpha_flap_hpf_filter.update(
            self.flap_angle_lpf)
        self.flap_angle_without_transient = (
            self.flap_angle_lpf - self.flap_angle_hpf)

    def _body_velocity(self, r_ts_to_ned):
        if self.local_position is None:
            return np.zeros(3)
        velocity = np.array([
            self.local_position.vx,
            self.local_position.vy,
            self.local_position.vz,
        ], dtype=float)
        if not np.all(np.isfinite(velocity)):
            return np.zeros(3)
        return r_ts_to_ned.T @ velocity

    def _publish_alpha_debug(self, velocity_body_ts, model_result,
                             flap_transient_force):
        message = Float64MultiArray()
        message.data = pack_alpha_model_debug(
            velocity_body_ts,
            model_result.force_alpha,
            model_result.moment_body_ts,
            flap_transient_force,
            self.motor_speed_lpf,
            self.flap_angle_lpf,
            self.flap_angle_hpf,
        )
        self.alpha_debug_publisher.publish(message)

    def _trajectory_desired_state(self, now_ns, q_current_ts):
        if not self._trajectory_state_is_fresh(now_ns):
            return None
        local = self.local_position
        if not (local.z_valid and local.v_z_valid):
            return None
        if not (local.xy_valid and local.v_xy_valid):
            return self._degraded_vertical_desired_state(q_current_ts)
        position = np.array([local.x, local.y, local.z], dtype=float)
        velocity = np.array([local.vx, local.vy, local.vz], dtype=float)
        acceleration = np.array([local.ax, local.ay, local.az], dtype=float)
        state = np.concatenate((position, velocity, acceleration))
        if not np.all(np.isfinite(state)):
            return None

        r_ts_to_ned = quaternion_to_matrix(q_current_ts)
        r_alpha_to_ned = (
            r_ts_to_ned @ self.aerodynamics.rotation_alpha_to_body)
        velocity_body_ts = r_ts_to_ned.T @ velocity
        if self.trajectory_yaw is None:
            self.trajectory_yaw = tailsitter_heading_from_attitude(q_current_ts)
        try:
            reference = resolve_trajectory_reference(
                self.trajectory_setpoint,
                position,
                velocity,
                self.trajectory_yaw,
            )
        except ValueError as error:
            self.get_logger().error(f'Invalid trajectory setpoint: {error}')
            return None
        self.trajectory_yaw = reference.yaw

        if self.linear_acceleration_filter is None:
            self.linear_acceleration_filter = ButterworthLowPass(
                self.cfg.indi_lpf_cutoff_hz,
                self.cfg.control_rate_hz,
                3,
                acceleration,
            )
        acceleration_lpf = self.linear_acceleration_filter.update(acceleration)

        flap_transient = self.aerodynamics.compute(
            velocity_body_ts,
            self.motor_speed_lpf,
            self.flap_angle_hpf,
        ).flap_force_alpha
        acceleration_without_flap_transient = (
            acceleration_lpf
            - r_alpha_to_ned @ flap_transient / self.cfg.mass)

        acceleration_command = self.position_controller.acceleration_command(
            reference,
            position,
            velocity,
            acceleration_without_flap_transient,
            r_ts_to_ned,
        )
        steady_model = self.aerodynamics.compute(
            velocity_body_ts,
            self.motor_speed_lpf,
            self.flap_angle_without_transient,
        )
        estimated_force_lpf_ned = r_alpha_to_ned @ steady_model.force_alpha
        force_command = (
            estimated_force_lpf_ned
            + self.cfg.mass
            * (acceleration_command - acceleration_without_flap_transient)
        )
        flap_sum = float(np.sum(self.flap_angle_without_transient))
        try:
            desired_q_ts, total_thrust = (
                self.flatness_controller.attitude_and_thrust(
                    force_command,
                    velocity,
                    flap_sum,
                    reference.yaw,
                    q_current_ts,
                ))
            reference_force = self.cfg.mass * (
                reference.acceleration
                - np.array([0.0, 0.0, self.cfg.gravity]))
            _, _, reference_roll, reference_pitch_bar = (
                self.flatness_controller.attitude_and_thrust(
                    reference_force,
                    reference.velocity,
                    flap_sum,
                    reference.yaw,
                    q_current_ts,
                    return_angles=True,
                ))
            desired_rates_ts = self.flatness_controller.feedforward_rates(
                reference.velocity,
                reference.acceleration,
                reference.jerk,
                reference.yaw,
                reference.yawspeed,
                reference_force,
                reference_roll,
                reference_pitch_bar,
                flap_sum,
            )
        except ValueError as error:
            self.get_logger().error(f'Flatness transform failed: {error}')
            return None
        total_thrust = float(np.clip(
            total_thrust, 0.0, self.cfg.maximum_total_thrust))

        debug_message = Float64MultiArray()
        debug_message.data = pack_position_debug(
            position,
            reference.position,
            velocity,
            reference.velocity,
            acceleration_without_flap_transient,
            reference.acceleration,
            acceleration_command,
            estimated_force_lpf_ned,
            force_command,
            force_command,
            reference.yaw,
            total_thrust,
            True,
        )
        self.position_debug_publisher.publish(debug_message)
        return desired_q_ts, total_thrust, desired_rates_ts

    def _degraded_vertical_desired_state(self, q_current_ts):
        """Hold altitude or descend without trusting invalid horizontal EKF state."""
        local = self.local_position
        setpoint = self.trajectory_setpoint
        try:
            z = float(local.z)
            vz = float(local.vz)
            raw_z_reference = float(setpoint.position[2])
            raw_vz_reference = float(setpoint.velocity[2])
            raw_acceleration = float(setpoint.acceleration[2])
            z_reference = raw_z_reference if math.isfinite(raw_z_reference) else z
            vz_reference = (
                raw_vz_reference if math.isfinite(raw_vz_reference)
                else (0.0 if math.isfinite(raw_z_reference) else vz)
            )
            acceleration_feedforward = (
                raw_acceleration if math.isfinite(raw_acceleration) else 0.0)
            force_command = degraded_vertical_force(
                z,
                vz,
                z_reference,
                vz_reference,
                acceleration_feedforward,
                self.cfg,
            )
            if self.trajectory_yaw is None:
                self.trajectory_yaw = tailsitter_heading_from_attitude(
                    q_current_ts)
            flap_sum = float(np.sum(self.flap_angle_without_transient))
            desired_q_ts, total_thrust = (
                self.flatness_controller.attitude_and_thrust(
                    force_command,
                    np.zeros(3),
                    flap_sum,
                    self.trajectory_yaw,
                    q_current_ts,
                ))
        except (ValueError, IndexError) as error:
            self.get_logger().error(
                f'Degraded vertical control failed: {error}',
                throttle_duration_sec=1.0,
            )
            return None
        self.get_logger().warning(
            'Horizontal position invalid; using bounded vertical-only control',
            throttle_duration_sec=1.0,
        )
        return (
            desired_q_ts,
            float(np.clip(total_thrust, 0.0,
                          self.cfg.maximum_total_thrust)),
            np.zeros(3),
        )

    def _control_tick(self):
        now_ns = self._now_ns()
        timestamp_us = now_ns // 1000
        self._publish_mode(timestamp_us)
        dt = 1.0 / self.cfg.control_rate_hz
        self._update_actuator_estimate(dt, now_ns)
        self._publish_actuator_feedback_debug(now_ns)

        if not self._state_is_fresh(now_ns):
            if self.previous_output_active:
                attitude_age = (now_ns - self.attitude_arrival_ns) * 1e-9
                rates_age = (now_ns - self.rates_arrival_ns) * 1e-9
                self.get_logger().error(
                    'Alpha output inhibited by stale attitude/rates: '
                    f'{attitude_age:.3f}/{rates_age:.3f} s')
            self._reset_dynamic_state()
            self._publish_actuators(timestamp_us, np.zeros(2), np.zeros(2))
            return

        try:
            q_current_ts = attitude_px4_to_ts(self.attitude.q)
            rates_ts = vector_px4_to_ts(self.rates.xyz)
            if not np.all(np.isfinite(rates_ts)):
                raise ValueError('angular velocity is not finite')
        except ValueError as error:
            self.get_logger().error(f'Invalid state: {error}')
            self._reset_dynamic_state()
            self._publish_actuators(timestamp_us, np.zeros(2), np.zeros(2))
            return

        mode_ready = (
            self._mode_is_fresh(now_ns)
            and self.control_mode.flag_armed
            and self.control_mode.flag_control_offboard_enabled
        )
        output_active = (
            mode_ready
            and bool(self.get_parameter('output_enabled').value)
            and (self.actuator_feedback_mode != 'required'
                 or self._actuator_feedback_is_fresh(now_ns))
        )
        source = str(self.get_parameter('setpoint_source').value).lower()
        if output_active and not self.previous_output_active:
            if source == 'trajectory':
                hover_speed = math.sqrt(
                    self.cfg.hover_total_thrust
                    / (2.0 * self.cfg.motor_thrust_coefficient))
                self.motor_speed_prediction.fill(hover_speed)
                # Linear INDI is incremental and needs an airborne equilibrium
                # for the first trajectory cycle.  Seed that one cycle even
                # when fresh feedback correctly reports zero speed on the
                # ground; subsequent cycles immediately resume the selected
                # measured/predicted actuator path.
                self.motor_speed_estimate.fill(hover_speed)
                self.linear_acceleration_filter = None
            self._reset_alpha_filters()

        self._update_alpha_filters()
        r_ts_to_ned = quaternion_to_matrix(q_current_ts)
        velocity_body_ts = self._body_velocity(r_ts_to_ned)
        model_result = self.aerodynamics.compute(
            velocity_body_ts,
            self.motor_speed_lpf,
            self.flap_angle_lpf,
        )
        flap_transient = self.aerodynamics.compute(
            velocity_body_ts,
            self.motor_speed_lpf,
            self.flap_angle_hpf,
        ).flap_force_alpha
        self._publish_alpha_debug(
            velocity_body_ts, model_result, flap_transient)

        desired = self._desired_state(now_ns, q_current_ts)
        if desired is None:
            if output_active and self.previous_output_active:
                local_age = (now_ns - self.local_position_arrival_ns) * 1e-9
                trajectory_age = (
                    now_ns - self.trajectory_setpoint_arrival_ns) * 1e-9
                self.get_logger().error(
                    'Alpha output inhibited by unavailable trajectory state: '
                    f'local/setpoint age {local_age:.3f}/{trajectory_age:.3f} s')
            self._reset_dynamic_state()
            self._publish_actuators(timestamp_us, np.zeros(2), np.zeros(2))
            return

        if self.rate_filter is None:
            self.rate_filter = ButterworthLowPass(
                self.cfg.indi_lpf_cutoff_hz,
                self.cfg.control_rate_hz,
                3,
                rates_ts,
            )
            self.moment_filter = ButterworthLowPass(
                self.cfg.indi_lpf_cutoff_hz,
                self.cfg.control_rate_hz,
                3,
                model_result.moment_body_ts,
            )
            self.previous_rate_lpf = rates_ts.copy()

        rates_lpf_ts = self.rate_filter.update(rates_ts)
        acceleration_lpf_ts = (
            rates_lpf_ts - self.previous_rate_lpf) / dt
        self.previous_rate_lpf = rates_lpf_ts.copy()
        estimated_moment_lpf = self.moment_filter.update(
            model_result.moment_body_ts)

        desired_q_ts, total_thrust, desired_rates_ts = desired
        acceleration_command = self.controller.angular_acceleration_command(
            q_current_ts,
            desired_q_ts,
            rates_lpf_ts,
            desired_rates_ts,
        )
        desired_moment = self.controller.moment_command(
            acceleration_command,
            acceleration_lpf_ts,
            estimated_moment_lpf,
        )
        allocation = self.allocator.allocate(
            total_thrust, desired_moment, velocity_body_ts)
        motors_px4, servos_px4 = self.allocator.to_px4_controls(allocation)

        debug_message = Float64MultiArray()
        debug_message.data = pack_control_debug(
            rotation_vector_error(q_current_ts, desired_q_ts),
            rates_lpf_ts,
            acceleration_lpf_ts,
            acceleration_command,
            estimated_moment_lpf,
            desired_moment,
            allocation.achieved_moment_ts,
            allocation.motor_speed_left_right / self.cfg.motor_speed_max,
            allocation.flap_angle_left_right,
            total_thrust,
            output_active,
        )
        self.debug_publisher.publish(debug_message)
        if output_active:
            self.motor_target = allocation.motor_speed_left_right.copy()
            self.flap_target = allocation.flap_angle_left_right.copy()
            self._publish_actuators(
                timestamp_us, motors_px4, servos_px4, total_thrust)
        else:
            self.motor_target.fill(0.0)
            self.flap_target.fill(0.0)
            self._publish_actuators(timestamp_us, np.zeros(2), np.zeros(2))
        self.previous_output_active = output_active


def main(args=None):
    rclpy.init(args=args)
    node = AlphaTailsitterController()
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
