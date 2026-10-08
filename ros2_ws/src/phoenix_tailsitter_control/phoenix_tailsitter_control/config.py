"""Parameters identified from the current PhoenixDrone SDF and airframe."""

from dataclasses import dataclass, field

import numpy as np

from .frames import inertia_px4_to_ts


@dataclass(frozen=True)
class PhoenixHoverConfig:
    # Match the Tailsitter-control reference configuration.
    mass: float = 0.7
    gravity: float = 9.81

    # Tailsitter-control uses TS inertia diag([0.0095, 0.0030, 0.0115]).
    # Express it about PX4 FRD axes using x_ts=-z_px4, y_ts=y_px4, z_ts=x_px4.
    inertia_px4: np.ndarray = field(default_factory=lambda: np.diag([
        0.0115, 0.0030, 0.0095,
    ]))

    motor_thrust_coefficient: float = 1.8e-6
    motor_moment_ratio: float = 2.2e-8 / 1.8e-6
    motor_arm_y: float = 0.15
    motor_speed_max: float = 2500.0
    motor_time_constant_up: float = 0.04
    motor_time_constant_down: float = 0.04
    rotor_velocity_slowdown: float = 10.0
    actuator_feedback_timeout_s: float = 0.05

    # PhoenixAero low-speed model.  Delta is the physical joint command.
    flap_lift_coefficient: float = 3.48e-6
    flap_drag_coefficient: float = 1.75e-6
    flap_pitch_coefficient: float = -3.44e-7
    # PhoenixAero cp.z_GZ=+0.036 becomes x_ts=+0.036 after FLU->FRD and
    # PX4->Tailsitter conversion.
    flap_cp_x_ts: float = 0.036
    left_flap_limit: float = 1.0
    right_flap_limit: float = 1.0

    # Full alpha-theory model (study.md 11.11).  Geometry and propulsion
    # defaults come from the current PhoenixDrone SDF.  Entries explicitly
    # marked IDENTIFY use the Tailsitter-control paper/repository values only
    # as conservative starting points; they are not claimed measurements for
    # PhoenixDrone.
    alpha_zero_lift_rad: float = float(np.deg2rad(-2.0))  # IDENTIFY
    alpha_thrust_installation_rad: float = float(np.deg2rad(-5.0))
    alpha_c_lv: float = 0.29  # IDENTIFY
    alpha_c_dv: float = 0.0  # IDENTIFY
    alpha_c_lt: float = 2.23  # IDENTIFY
    alpha_c_dt: float = 0.0  # IDENTIFY
    alpha_c_lv_delta: float = 0.18  # IDENTIFY
    alpha_c_lt_delta: float = 1.25  # IDENTIFY
    alpha_c_mu_t: float = -0.025
    alpha_motor_torque_coefficient: float = 2.2e-8
    alpha_motor_arm_y: float = 0.15
    alpha_flap_arm_y: float = 0.12
    alpha_flap_arm_x: float = 0.075
    servo_time_constant: float = 0.03  # IDENTIFY
    servo_rate_limit_rad_s: float = 25.0  # IDENTIFY
    flap_hpf_cutoff_hz: float = 1.0

    control_rate_hz: float = 500.0
    indi_lpf_cutoff_hz: float = 15.0
    linear_indi_lpf_cutoff_hz: float = 5.0
    trajectory_prediction_horizon_s: float = 0.10
    maneuver_speed_threshold: float = 0.50
    maneuver_acceleration_threshold: float = 0.50
    maneuver_yaw_rate_threshold: float = 0.30
    maneuver_gain_transition_s: float = 1.0
    state_timeout_s: float = 0.10
    # PX4 publishes vehicle_control_mode every 0.5 s; allow transport jitter
    # without briefly zeroing both motors between otherwise healthy updates.
    control_mode_timeout_s: float = 1.50
    setpoint_timeout_s: float = 0.50

    # PX4/Gazebo inner-loop gains in TS body axes.  The much larger gains from
    # the ideal continuous-time reference model saturate the simulated
    # actuator chain during the ground-to-flight transient.
    attitude_gain: np.ndarray = field(default_factory=lambda: np.array([6.0, 6.0, 6.0]))
    rate_gain: np.ndarray = field(default_factory=lambda: np.array([3.0, 3.0, 3.0]))
    # Alpha trajectory control schedules these gains by requested motion.
    tracking_attitude_gain: np.ndarray = field(
        default_factory=lambda: np.array([78.4, 54.88, 54.88]))
    tracking_rate_gain: np.ndarray = field(
        default_factory=lambda: np.array([14.0, 9.8, 9.8]))
    # Finite stationary trajectory references need enough attitude bandwidth
    # to brake out of a maneuver, without the saturated high-gain hover cycle.
    trajectory_hover_attitude_gain: np.ndarray = field(
        default_factory=lambda: np.array([24.0, 16.0, 16.0]))
    trajectory_hover_rate_gain: np.ndarray = field(
        default_factory=lambda: np.array([8.0, 6.0, 6.0]))
    angular_acceleration_limit: np.ndarray = field(
        default_factory=lambda: np.array([20.0, 15.0, 24.0]))
    moment_limit: np.ndarray = field(default_factory=lambda: np.array([0.25, 0.08, 0.35]))

    # Tailsitter-control position/linear-acceleration loop in TS body axes.
    # Keep the vertical TS-x gains, but damp both horizontal body axes.  The
    # reference gains produced a large, slow limit cycle during staging once
    # horizontal acceleration reached its bound.
    position_gain: np.ndarray = field(
        default_factory=lambda: np.array([4.0, 1.0, 1.5]))
    velocity_gain: np.ndarray = field(
        default_factory=lambda: np.array([3.0, 3.0, 3.0]))
    # Use more velocity damping on moving trajectories; stationary references
    # use the staging gains even if their acceleration fields are finite zeros.
    tracking_position_gain: np.ndarray = field(
        default_factory=lambda: np.array([4.0, 2.0, 3.0]))
    tracking_velocity_gain: np.ndarray = field(
        default_factory=lambda: np.array([4.0, 3.0, 3.0]))
    linear_acceleration_gain: np.ndarray = field(
        default_factory=lambda: np.zeros(3))
    position_error_limit: float = float('inf')
    velocity_error_limit: float = float('inf')
    linear_indi_blend: float = 1.0
    # Limit only position/velocity feedback.  Trajectory acceleration
    # feed-forward bypasses these limits so aggressive reference motion is
    # preserved while takeoff disturbances remain bounded.
    horizontal_acceleration_limit: float = 0.45
    vertical_acceleration_limit: float = 0.80
    # Gazebo cannot tolerate the ideal model's unbounded Eq. 41 feedback.  Keep
    # staging conservative and use the stable maneuver envelope identified in
    # closed-loop SITL testing.
    tracking_horizontal_acceleration_limit: float = 3.00
    tracking_vertical_acceleration_limit: float = 1.50
    trajectory_hover_horizontal_acceleration_limit: float = 1.20
    trajectory_hover_vertical_acceleration_limit: float = 1.00
    position_tilt_limit_rad: float = float(np.deg2rad(89.0))
    horizontal_force_slew_rate_n_s: float = 1.0e6
    minimum_position_thrust_scale: float = 0.0
    maximum_position_thrust_scale: float = float('inf')
    takeoff_force_floor_trigger_scale: float = 0.8
    takeoff_force_floor_max_scale: float = 2.0

    @property
    def inertia_ts(self):
        return inertia_px4_to_ts(self.inertia_px4)

    @property
    def maximum_total_thrust(self):
        return 2.0 * self.motor_thrust_coefficient * self.motor_speed_max ** 2

    @property
    def hover_total_thrust(self):
        return self.mass * self.gravity

    @property
    def minimum_position_thrust(self):
        return self.minimum_position_thrust_scale * self.hover_total_thrust

    @property
    def alpha_identification_defaults(self):
        """Expose every alpha-model entry that should be fitted or verified."""
        return {
            'mass': self.mass,
            'motor_thrust_coefficient': self.motor_thrust_coefficient,
            'alpha_motor_torque_coefficient':
                self.alpha_motor_torque_coefficient,
            'alpha_motor_arm_y': self.alpha_motor_arm_y,
            'alpha_flap_arm_y': self.alpha_flap_arm_y,
            'alpha_flap_arm_x': self.alpha_flap_arm_x,
            'alpha_zero_lift_rad': self.alpha_zero_lift_rad,
            'alpha_thrust_installation_rad':
                self.alpha_thrust_installation_rad,
            'alpha_c_lv': self.alpha_c_lv,
            'alpha_c_dv': self.alpha_c_dv,
            'alpha_c_lt': self.alpha_c_lt,
            'alpha_c_dt': self.alpha_c_dt,
            'alpha_c_lv_delta': self.alpha_c_lv_delta,
            'alpha_c_lt_delta': self.alpha_c_lt_delta,
            'alpha_c_mu_t': self.alpha_c_mu_t,
            'motor_time_constant_up': self.motor_time_constant_up,
            'motor_time_constant_down': self.motor_time_constant_down,
            'servo_time_constant': self.servo_time_constant,
            'servo_rate_limit_rad_s': self.servo_rate_limit_rad_s,
        }

    @property
    def maximum_position_thrust(self):
        return min(
            self.maximum_position_thrust_scale * self.hover_total_thrust,
            self.maximum_total_thrust,
        )
