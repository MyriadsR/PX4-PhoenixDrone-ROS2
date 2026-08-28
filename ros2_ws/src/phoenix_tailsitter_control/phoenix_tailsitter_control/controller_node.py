"""ROS 2 direct-actuator node for the low-speed Tailsitter-control inner loop."""

import math
from typing import Optional

import numpy as np
import rclpy
from px4_msgs.msg import (
    ActuatorMotors,
    ActuatorServos,
    OffboardControlMode,
    VehicleAngularVelocity,
    VehicleAttitude,
    VehicleAttitudeSetpoint,
    VehicleControlMode,
    VehicleLocalPosition,
    VehicleThrustSetpoint,
    TrajectorySetpoint,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .actuator_feedback import parse_actuator_joint_state
from .allocator import PhoenixHoverAllocator
from .attitude_control import AttitudeINDIController
from .config import PhoenixHoverConfig
from .debug import (
    pack_actuator_feedback_debug,
    pack_control_debug,
    pack_position_debug,
)
from .filters import ButterworthLowPass
from .frames import attitude_px4_to_ts, vector_px4_to_ts
from .math_utils import normalize_quaternion, quaternion_to_matrix, rotation_vector_error
from .position_control import (
    HoverForceSlewLimiter,
    LinearAccelerationINDIController,
    PositionController,
    force_to_tailsitter_attitude,
    limit_hover_force,
    resolve_trajectory_reference,
    tailsitter_heading_from_attitude,
)
from .qos import PX4_INPUT_QOS, PX4_OUTPUT_QOS


class TailsitterController(Node):
    def __init__(self):
        super().__init__('phoenix_tailsitter_controller')
        self.cfg = PhoenixHoverConfig()
        self.controller = AttitudeINDIController(self.cfg)
        self.allocator = PhoenixHoverAllocator(self.cfg)
        self.position_controller = PositionController(self.cfg)
        self.linear_controller = LinearAccelerationINDIController(self.cfg)
        self.force_slew_limiter = HoverForceSlewLimiter(
            self.cfg.horizontal_force_slew_rate_n_s)

        self.declare_parameter('output_enabled', False)
        self.declare_parameter('setpoint_source', 'latch')
        self.declare_parameter('setpoint_frame', 'px4')
        self.declare_parameter('hover_thrust_scale', 1.0)
        self.declare_parameter('actuator_feedback_mode', 'auto')
        self.declare_parameter(
            'actuator_feedback_topic',
            '/world/stars_ts/model/phoenixdrone_0/joint_state',
        )
        feedback_mode = str(
            self.get_parameter('actuator_feedback_mode').value).lower()
        if feedback_mode not in ('auto', 'estimate', 'required'):
            raise ValueError(
                'actuator_feedback_mode must be auto, estimate, or required')
        self.actuator_feedback_mode = feedback_mode

        self.attitude: Optional[VehicleAttitude] = None
        self.rates: Optional[VehicleAngularVelocity] = None
        self.control_mode: Optional[VehicleControlMode] = None
        self.topic_setpoint: Optional[VehicleAttitudeSetpoint] = None
        self.local_position: Optional[VehicleLocalPosition] = None
        self.trajectory_setpoint: Optional[TrajectorySetpoint] = None
        self.attitude_arrival_ns = 0
        self.rates_arrival_ns = 0
        self.setpoint_arrival_ns = 0
        self.control_mode_arrival_ns = 0
        self.local_position_arrival_ns = 0
        self.trajectory_setpoint_arrival_ns = 0
        self.latched_attitude_ts = None
        self.trajectory_yaw = None

        self.rate_filter = None
        self.moment_filter = None
        self.previous_rate_lpf = None
        self.linear_acceleration_filter = None
        self.force_filter = None
        self.force_slew_limiter.reset()
        self.motor_speed_prediction = np.zeros(2)
        self.flap_angle_prediction = np.zeros(2)
        self.motor_speed_estimate = np.zeros(2)
        self.flap_angle_estimate = np.zeros(2)
        self.motor_speed_feedback = np.zeros(2)
        self.flap_angle_feedback = np.zeros(2)
        self.actuator_feedback_arrival_ns = 0
        self.actuator_feedback_complete = False
        self.actuator_feedback_active = False
        self.motor_target = np.zeros(2)
        self.flap_target = np.zeros(2)
        self.previous_output_active = False

        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self._on_attitude, PX4_OUTPUT_QOS)
        self.create_subscription(
            VehicleAngularVelocity, '/fmu/out/vehicle_angular_velocity', self._on_rates,
            PX4_OUTPUT_QOS)
        self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self._on_control_mode,
            PX4_OUTPUT_QOS)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._on_local_position, PX4_OUTPUT_QOS)
        self.create_subscription(
            VehicleAttitudeSetpoint, '/phoenix_tailsitter/attitude_setpoint',
            self._on_setpoint, PX4_INPUT_QOS)
        self.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
            self._on_trajectory_setpoint, PX4_INPUT_QOS)
        self.create_subscription(
            JointState,
            str(self.get_parameter('actuator_feedback_topic').value),
            self._on_actuator_feedback,
            10,
        )

        self.mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_INPUT_QOS)
        self.motor_publisher = self.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', PX4_INPUT_QOS)
        self.servo_publisher = self.create_publisher(
            ActuatorServos, '/fmu/in/actuator_servos', PX4_INPUT_QOS)
        self.thrust_setpoint_publisher = self.create_publisher(
            VehicleThrustSetpoint,
            '/fmu/in/vehicle_thrust_setpoint',
            PX4_INPUT_QOS,
        )
        self.debug_publisher = self.create_publisher(
            Float64MultiArray, '/phoenix_tailsitter/control_debug', 10)
        self.position_debug_publisher = self.create_publisher(
            Float64MultiArray, '/phoenix_tailsitter/position_debug', 10)
        self.actuator_feedback_debug_publisher = self.create_publisher(
            Float64MultiArray, '/phoenix_tailsitter/actuator_feedback_debug', 10)
        self.create_timer(1.0 / self.cfg.control_rate_hz, self._control_tick)

        if self.get_parameter('output_enabled').value:
            self.get_logger().warning(
                'Direct actuator output requested; armed/offboard/state safety gates remain active.')
        else:
            self.get_logger().warning(
                'Direct actuator output disabled. Validate channels with actuator_probe first.')

    def _now_ns(self):
        return self.get_clock().now().nanoseconds

    def _on_attitude(self, message):
        self.attitude = message
        self.attitude_arrival_ns = self._now_ns()

    def _on_rates(self, message):
        self.rates = message
        self.rates_arrival_ns = self._now_ns()

    def _on_control_mode(self, message):
        self.control_mode = message
        self.control_mode_arrival_ns = self._now_ns()

    def _on_local_position(self, message):
        self.local_position = message
        self.local_position_arrival_ns = self._now_ns()

    def _on_trajectory_setpoint(self, message):
        self.trajectory_setpoint = message
        self.trajectory_setpoint_arrival_ns = self._now_ns()

    def _on_setpoint(self, message):
        try:
            normalize_quaternion(message.q_d)
        except ValueError:
            self.get_logger().error('Rejected non-finite or zero attitude setpoint')
            return
        self.topic_setpoint = message
        self.setpoint_arrival_ns = self._now_ns()

    def _on_actuator_feedback(self, message):
        try:
            feedback = parse_actuator_joint_state(
                message, self.cfg.rotor_velocity_slowdown)
        except ValueError as error:
            self.get_logger().warning(
                f'Rejected actuator joint-state feedback: {error}',
                throttle_duration_sec=2.0,
            )
            return
        self.motor_speed_feedback = np.clip(
            feedback.motor_speed_left_right, 0.0, self.cfg.motor_speed_max)
        self.flap_angle_feedback = np.clip(
            feedback.flap_angle_left_right,
            [-self.cfg.left_flap_limit, -self.cfg.right_flap_limit],
            [self.cfg.left_flap_limit, self.cfg.right_flap_limit],
        )
        self.actuator_feedback_arrival_ns = self._now_ns()
        self.actuator_feedback_complete = True

    def _publish_mode(self, timestamp_us):
        message = OffboardControlMode()
        message.timestamp = timestamp_us
        message.direct_actuator = True
        self.mode_publisher.publish(message)

    def _publish_actuators(self, timestamp_us, motors, servos,
                           total_thrust=0.0):
        sample_timestamp = 0 if self.rates is None else self.rates.timestamp_sample
        motor_message = ActuatorMotors()
        motor_message.timestamp = timestamp_us
        motor_message.timestamp_sample = sample_timestamp
        motor_message.control = list(motors) + [math.nan] * (ActuatorMotors.NUM_CONTROLS - 2)
        self.motor_publisher.publish(motor_message)

        servo_message = ActuatorServos()
        servo_message.timestamp = timestamp_us
        servo_message.timestamp_sample = sample_timestamp
        servo_message.control = list(servos) + [math.nan] * (ActuatorServos.NUM_CONTROLS - 2)
        self.servo_publisher.publish(servo_message)

        # Direct actuator offboard mode bypasses PX4's normal position/rate
        # controllers.  Publish the matching collective-thrust intent so the
        # multicopter land detector can distinguish an airborne vehicle from
        # a low-throttle vehicle on the ground.  TS +x is PX4 body -z.
        thrust_message = VehicleThrustSetpoint()
        thrust_message.timestamp = timestamp_us
        thrust_message.timestamp_sample = sample_timestamp
        collective = float(np.clip(
            total_thrust / self.cfg.maximum_total_thrust, 0.0, 1.0))
        thrust_message.xyz = [0.0, 0.0, -collective]
        self.thrust_setpoint_publisher.publish(thrust_message)

    def _state_is_fresh(self, now_ns):
        timeout_ns = int(self.cfg.state_timeout_s * 1e9)
        return (
            self.attitude is not None
            and self.rates is not None
            and now_ns - self.attitude_arrival_ns <= timeout_ns
            and now_ns - self.rates_arrival_ns <= timeout_ns
        )

    def _mode_is_fresh(self, now_ns):
        return (
            self.control_mode is not None
            and now_ns - self.control_mode_arrival_ns
            <= int(self.cfg.control_mode_timeout_s * 1e9)
        )

    def _actuator_feedback_is_fresh(self, now_ns):
        return (
            self.actuator_feedback_complete
            and now_ns - self.actuator_feedback_arrival_ns
            <= int(self.cfg.actuator_feedback_timeout_s * 1e9)
        )

    def _reset_dynamic_state(self):
        self.motor_target.fill(0.0)
        self.flap_target.fill(0.0)
        self.rate_filter = None
        self.moment_filter = None
        self.previous_rate_lpf = None
        self.linear_acceleration_filter = None
        self.force_filter = None
        self.force_slew_limiter.reset()
        self.trajectory_yaw = None
        self.previous_output_active = False

    def _trajectory_state_is_fresh(self, now_ns):
        timeout_ns = int(self.cfg.state_timeout_s * 1e9)
        return (
            self.local_position is not None
            and now_ns - self.local_position_arrival_ns <= timeout_ns
            and self.trajectory_setpoint is not None
            and now_ns - self.trajectory_setpoint_arrival_ns
            <= int(self.cfg.setpoint_timeout_s * 1e9)
        )

    def _trajectory_desired_state(self, now_ns, q_current_ts):
        if not self._trajectory_state_is_fresh(now_ns):
            return None
        local = self.local_position
        if not (local.xy_valid and local.z_valid and local.v_xy_valid and local.v_z_valid):
            return None
        position = np.array([local.x, local.y, local.z], dtype=float)
        velocity = np.array([local.vx, local.vy, local.vz], dtype=float)
        acceleration = np.array([local.ax, local.ay, local.az], dtype=float)
        if not np.all(np.isfinite(np.concatenate((position, velocity, acceleration)))):
            return None

        r_ts_to_ned = quaternion_to_matrix(q_current_ts)
        if self.trajectory_yaw is None:
            self.trajectory_yaw = tailsitter_heading_from_attitude(q_current_ts)
        try:
            reference = resolve_trajectory_reference(
                self.trajectory_setpoint, position, velocity, self.trajectory_yaw)
        except ValueError as error:
            self.get_logger().error(f'Invalid trajectory setpoint: {error}')
            return None
        self.trajectory_yaw = reference.yaw

        if self.linear_acceleration_filter is None:
            self.linear_acceleration_filter = ButterworthLowPass(
                self.cfg.linear_indi_lpf_cutoff_hz,
                self.cfg.control_rate_hz, 3, acceleration)
            initial_force = r_ts_to_ned @ self.linear_controller.estimate_force_ts(
                self.motor_speed_estimate, self.flap_angle_estimate)
            self.force_filter = ButterworthLowPass(
                self.cfg.linear_indi_lpf_cutoff_hz,
                self.cfg.control_rate_hz, 3, initial_force)
        acceleration_lpf = self.linear_acceleration_filter.update(acceleration)
        estimated_force = r_ts_to_ned @ self.linear_controller.estimate_force_ts(
            self.motor_speed_estimate, self.flap_angle_estimate)
        estimated_force_lpf = self.force_filter.update(estimated_force)
        acceleration_command = self.position_controller.acceleration_command(
            reference, position, velocity, acceleration_lpf, r_ts_to_ned)
        force_command = self.linear_controller.force_command(
            acceleration_command, acceleration_lpf, estimated_force_lpf)
        # The current PhoenixAero elevons generate a sizeable +z_ts force
        # while producing pitch moment.  The attitude map controls propeller
        # thrust direction, so subtract the estimated non-propulsive force to
        # obtain the propeller force that should realize the requested net
        # force.  This is the low-speed incremental compensation; it avoids
        # treating elevon translation as if it were rotor-axis thrust.
        _, aerodynamic_force_ts = self.linear_controller.estimate_force_components_ts(
            self.motor_speed_estimate, self.flap_angle_estimate)
        propulsion_force_command = (
            force_command - r_ts_to_ned @ aerodynamic_force_ts)
        limited_force = limit_hover_force(propulsion_force_command, self.cfg)
        limited_force = self.force_slew_limiter.update(
            limited_force, 1.0 / self.cfg.control_rate_hz)
        desired_q_ts, total_thrust = force_to_tailsitter_attitude(
            limited_force, reference.yaw)
        desired_rates_ts = quaternion_to_matrix(desired_q_ts).T @ np.array(
            [0.0, 0.0, reference.yawspeed])

        debug_message = Float64MultiArray()
        debug_message.data = pack_position_debug(
            position,
            reference.position,
            velocity,
            reference.velocity,
            acceleration_lpf,
            reference.acceleration,
            acceleration_command,
            estimated_force_lpf,
            force_command,
            limited_force,
            reference.yaw,
            total_thrust,
            True,
        )
        self.position_debug_publisher.publish(debug_message)
        return desired_q_ts, total_thrust, desired_rates_ts

    def _desired_state(self, now_ns, q_current_ts):
        source = str(self.get_parameter('setpoint_source').value).lower()
        if source == 'trajectory':
            return self._trajectory_desired_state(now_ns, q_current_ts)
        if source == 'latch':
            if self.latched_attitude_ts is None:
                self.latched_attitude_ts = attitude_px4_to_ts(self.attitude.q)
                self.get_logger().info('Latched the current attitude as the hover reference')
            thrust_scale = float(self.get_parameter('hover_thrust_scale').value)
            if not math.isfinite(thrust_scale):
                self.get_logger().error('Non-finite hover_thrust_scale; outputs inhibited')
                return None
            total_thrust = np.clip(
                thrust_scale * self.cfg.hover_total_thrust,
                0.0,
                self.cfg.maximum_total_thrust,
            )
            return self.latched_attitude_ts, total_thrust, np.zeros(3)

        if source != 'topic':
            self.get_logger().error(f'Unknown setpoint_source={source!r}; outputs inhibited')
            return None
        if (
            self.topic_setpoint is None
            or now_ns - self.setpoint_arrival_ns > int(self.cfg.setpoint_timeout_s * 1e9)
        ):
            return None

        frame = str(self.get_parameter('setpoint_frame').value).lower()
        if frame == 'px4':
            desired_q_ts = attitude_px4_to_ts(self.topic_setpoint.q_d)
            raw_thrust = -float(self.topic_setpoint.thrust_body[2])
        elif frame == 'tailsitter':
            desired_q_ts = normalize_quaternion(self.topic_setpoint.q_d)
            raw_thrust = float(self.topic_setpoint.thrust_body[0])
        else:
            self.get_logger().error(f'Unknown setpoint_frame={frame!r}; outputs inhibited')
            return None
        if not math.isfinite(raw_thrust):
            self.get_logger().error('Non-finite thrust setpoint; outputs inhibited')
            return None
        normalized_thrust = max(0.0, raw_thrust)
        return (desired_q_ts,
                min(1.0, normalized_thrust) * self.cfg.maximum_total_thrust,
                np.zeros(3))

    def _update_actuator_estimate(self, dt, now_ns):
        for index in range(2):
            target = self.motor_target[index]
            current = self.motor_speed_prediction[index]
            tau = (
                self.cfg.motor_time_constant_up
                if target >= current
                else self.cfg.motor_time_constant_down
            )
            alpha = 1.0 - math.exp(-dt / tau)
            self.motor_speed_prediction[index] += alpha * (target - current)
        # PhoenixAero resets the joint directly to the bridge command.
        self.flap_angle_prediction = self.flap_target.copy()

        self.actuator_feedback_active = (
            self.actuator_feedback_mode != 'estimate'
            and self._actuator_feedback_is_fresh(now_ns)
        )
        if self.actuator_feedback_active:
            self.motor_speed_estimate = self.motor_speed_feedback.copy()
            self.flap_angle_estimate = self.flap_angle_feedback.copy()
        else:
            self.motor_speed_estimate = self.motor_speed_prediction.copy()
            self.flap_angle_estimate = self.flap_angle_prediction.copy()

    def _publish_actuator_feedback_debug(self, now_ns):
        if self.actuator_feedback_complete:
            age_s = max(
                0.0,
                (now_ns - self.actuator_feedback_arrival_ns) * 1e-9,
            )
        else:
            age_s = self.cfg.actuator_feedback_timeout_s + 1.0
        message = Float64MultiArray()
        message.data = pack_actuator_feedback_debug(
            self.motor_speed_prediction,
            self.motor_speed_feedback,
            self.flap_angle_prediction,
            self.flap_angle_feedback,
            age_s,
            self.actuator_feedback_complete,
            self.actuator_feedback_active,
        )
        self.actuator_feedback_debug_publisher.publish(message)

    def _control_tick(self):
        now_ns = self._now_ns()
        timestamp_us = now_ns // 1000
        self._publish_mode(timestamp_us)
        dt = 1.0 / self.cfg.control_rate_hz
        self._update_actuator_estimate(dt, now_ns)
        self._publish_actuator_feedback_debug(now_ns)

        ready = self._state_is_fresh(now_ns)
        if not ready:
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
        output_enabled = bool(self.get_parameter('output_enabled').value)
        feedback_ready = (
            self.actuator_feedback_mode != 'required'
            or self._actuator_feedback_is_fresh(now_ns)
        )
        output_active = mode_ready and output_enabled and feedback_ready
        source = str(self.get_parameter('setpoint_source').value).lower()
        if output_active and not self.previous_output_active and source == 'trajectory':
            # Seed the fallback predictor at hover on takeover so incremental
            # force control does not mistake ground contact for a zero-thrust
            # airborne equilibrium when joint feedback is unavailable.
            hover_motor_speed = math.sqrt(
                self.cfg.hover_total_thrust
                / (2.0 * self.cfg.motor_thrust_coefficient))
            self.motor_speed_prediction.fill(hover_motor_speed)
            if not self.actuator_feedback_active:
                self.motor_speed_estimate.fill(hover_motor_speed)
            self.linear_acceleration_filter = None
            self.force_filter = None

        desired = self._desired_state(now_ns, q_current_ts)
        if desired is None:
            self._reset_dynamic_state()
            self._publish_actuators(timestamp_us, np.zeros(2), np.zeros(2))
            return

        if self.rate_filter is None:
            self.rate_filter = ButterworthLowPass(
                self.cfg.indi_lpf_cutoff_hz, self.cfg.control_rate_hz, 3, rates_ts)
            self.moment_filter = ButterworthLowPass(
                self.cfg.indi_lpf_cutoff_hz, self.cfg.control_rate_hz, 3, np.zeros(3))
            self.previous_rate_lpf = rates_ts.copy()

        rates_lpf_ts = self.rate_filter.update(rates_ts)
        acceleration_lpf_ts = (rates_lpf_ts - self.previous_rate_lpf) / dt
        self.previous_rate_lpf = rates_lpf_ts.copy()
        estimated_moment = self.allocator.estimate_moment(
            self.motor_speed_estimate, self.flap_angle_estimate)
        estimated_moment_lpf = self.moment_filter.update(estimated_moment)

        desired_q_ts, total_thrust, desired_rates_ts = desired
        acceleration_command = self.controller.angular_acceleration_command(
            q_current_ts, desired_q_ts, rates_lpf_ts, desired_rates_ts)
        desired_moment = self.controller.moment_command(
            acceleration_command, acceleration_lpf_ts, estimated_moment_lpf)
        allocation = self.allocator.allocate(total_thrust, desired_moment)
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
    node = TailsitterController()
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
