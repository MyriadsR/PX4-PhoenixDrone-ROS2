"""Parameters identified from the current PhoenixDrone SDF and airframe."""

from dataclasses import dataclass, field

import numpy as np

from .frames import inertia_px4_to_ts


@dataclass(frozen=True)
class PhoenixHoverConfig:
    # SDF masses: base 0.5 kg, IMU 0.015 kg, two 0.005 kg rotors.
    mass: float = 0.525
    gravity: float = 9.81

    # Current controller/SDF values expressed about the PX4 FRD axes.
    inertia_px4: np.ndarray = field(default_factory=lambda: np.diag([
        0.0144, 0.00638929, 0.0176,
    ]))

    motor_thrust_coefficient: float = 7.864e-6
    motor_moment_ratio: float = 0.023
    motor_arm_y: float = 0.195
    motor_speed_max: float = 800.0
    motor_time_constant_up: float = 0.016
    motor_time_constant_down: float = 0.020
    rotor_velocity_slowdown: float = 10.0
    actuator_feedback_timeout_s: float = 0.05

    # PhoenixAero low-speed model.  Delta is the physical joint command.
    flap_lift_coefficient: float = 3.48e-6
    flap_drag_coefficient: float = 1.75e-6
    flap_pitch_coefficient: float = -3.44e-7
    # PhoenixAero cp.z_GZ=+0.036 becomes x_ts=+0.036 after FLU->FRD and
    # PX4->Tailsitter conversion.
    flap_cp_x_ts: float = 0.036
    left_flap_limit: float = float(np.deg2rad(60.0))
    right_flap_limit: float = float(np.deg2rad(30.0))

    # Full alpha-theory model (study.md 11.11).  Geometry and propulsion
    # defaults come from the current PhoenixDrone SDF.  Entries explicitly
    # marked IDENTIFY use the Tailsitter-control paper/repository values only
    # as conservative starting points; they are not claimed measurements for
    # PhoenixDrone.
    alpha_zero_lift_rad: float = float(np.deg2rad(-2.0))  # IDENTIFY
    alpha_thrust_installation_rad: float = 0.0
    alpha_c_lv: float = 0.29  # IDENTIFY
    alpha_c_dv: float = 0.0  # IDENTIFY
    alpha_c_lt: float = 2.23  # IDENTIFY
    alpha_c_dt: float = 0.0  # IDENTIFY
    alpha_c_lv_delta: float = 0.18  # IDENTIFY
    alpha_c_lt_delta: float = 1.25  # IDENTIFY
    alpha_c_mu_t: float = 0.0  # IDENTIFY
    alpha_motor_torque_coefficient: float = 1.80872e-7
    alpha_motor_arm_y: float = 0.195
    alpha_flap_arm_y: float = 0.195
    alpha_flap_arm_x: float = 0.036
    servo_time_constant: float = 0.03  # IDENTIFY
    servo_rate_limit_rad_s: float = 25.0  # IDENTIFY
    flap_hpf_cutoff_hz: float = 1.0

    control_rate_hz: float = 250.0
    indi_lpf_cutoff_hz: float = 15.0
    linear_indi_lpf_cutoff_hz: float = 5.0
    state_timeout_s: float = 0.10
    control_mode_timeout_s: float = 0.50
    setpoint_timeout_s: float = 0.50

    # Conservative hover-only gains in Tailsitter-control body axes.
    # The 2026-08-26 airborne MCAP identification found about 20/30/60 ms
    # plant delay on x/y/z.  The y plant is stronger than the static model,
    # while the differential-thrust z plant is much weaker.  These limits are
    # intentionally sized for the first +/-0.05 rad validation campaign.
    attitude_gain: np.ndarray = field(default_factory=lambda: np.array([6.0, 6.0, 6.0]))
    rate_gain: np.ndarray = field(default_factory=lambda: np.array([3.0, 3.0, 3.0]))
    angular_acceleration_limit: np.ndarray = field(
        default_factory=lambda: np.array([10.0, 6.0, 12.0]))
    moment_limit: np.ndarray = field(default_factory=lambda: np.array([0.04, 0.012, 0.08]))

    # Hover-only position/linear-acceleration loop.  Gains are expressed in
    # TS body axes and rotated into NED each cycle, matching the source
    # Tailsitter-control position law.  Limits deliberately constrain the
    # first validation campaign to small position steps.
    position_gain: np.ndarray = field(
        default_factory=lambda: np.array([2.5, 0.10, 0.10]))
    velocity_gain: np.ndarray = field(
        default_factory=lambda: np.array([1.8, 0.80, 0.80]))
    linear_acceleration_gain: np.ndarray = field(
        default_factory=lambda: np.zeros(3))
    position_error_limit: float = 0.40
    velocity_error_limit: float = 0.80
    # Horizontal force changes are delayed by the attitude loop.  The first
    # airborne identification measured roughly 0.44 s effective horizontal
    # response.  These second-stage limits are still conservative, but allow
    # visible attitude motion while tuning the position loop from step tests.
    linear_indi_blend: float = 1.0
    horizontal_acceleration_limit: float = 0.45
    vertical_acceleration_limit: float = 0.80
    position_tilt_limit_rad: float = float(np.deg2rad(5.0))
    horizontal_force_slew_rate_n_s: float = 0.18
    minimum_position_thrust_scale: float = 0.85
    maximum_position_thrust_scale: float = 1.20

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
