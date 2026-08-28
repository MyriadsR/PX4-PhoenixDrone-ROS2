import numpy as np

from phoenix_tailsitter_control.alpha_aerodynamics import (
    AlphaAerodynamics,
    vector_gz_flu_to_ts,
    vector_ts_to_gz_flu,
)
from phoenix_tailsitter_control.alpha_allocator import AlphaTheoryAllocator
from phoenix_tailsitter_control.config import PhoenixHoverConfig
from phoenix_tailsitter_control.filters import ButterworthHighPass
from phoenix_tailsitter_control.flatness_control import FlatnessAttitudeController
from phoenix_tailsitter_control.frames import attitude_px4_to_ts


def reference_equations(cfg, velocity, motors, flaps):
    """Independent direct transcription of Tailsitter-control equations 5--14."""
    alpha_tilde = cfg.alpha_zero_lift_rad + cfg.alpha_thrust_installation_rad
    ca0, sa0 = np.cos(cfg.alpha_zero_lift_rad), np.sin(cfg.alpha_zero_lift_rad)
    cat, sat = np.cos(alpha_tilde), np.sin(alpha_tilde)
    body_to_alpha = np.array([[ca0, 0.0, sa0], [0.0, 1.0, 0.0],
                              [-sa0, 0.0, ca0]])
    alpha_to_body = body_to_alpha.T
    velocity_alpha = body_to_alpha @ velocity
    speed = max(np.linalg.norm(velocity), 1e-3)
    thrust = cfg.motor_thrust_coefficient * motors ** 2
    direction = np.array([cat * (1.0 - cfg.alpha_c_dt), 0.0,
                          sat * (cfg.alpha_c_lt - 1.0)])
    thrust_forces = thrust[:, None] * direction
    flap_z = -(cfg.alpha_c_lt_delta * cat * thrust
               + cfg.alpha_c_lv_delta * speed * velocity_alpha[0]) * flaps
    force = (np.sum(thrust_forces, axis=0)
             + np.array([0.0, 0.0, np.sum(flap_z)])
             - np.array([cfg.alpha_c_dv * velocity_alpha[0], 0.0,
                         cfg.alpha_c_lv * velocity_alpha[2]]) * speed)
    difference_body = alpha_to_body @ (thrust_forces[1] - thrust_forces[0])
    moment_thrust = np.array([
        cfg.alpha_motor_arm_y * difference_body[2],
        cfg.alpha_c_mu_t * np.sum(thrust),
        -cfg.alpha_motor_arm_y * difference_body[0],
    ])
    reaction = cfg.alpha_motor_torque_coefficient * (motors[0]**2 - motors[1]**2)
    moment_reaction = np.array([
        np.cos(cfg.alpha_thrust_installation_rad) * reaction,
        0.0,
        -np.sin(cfg.alpha_thrust_installation_rad) * reaction,
    ])
    difference_flap = flap_z[1] - flap_z[0]
    moment_flap = np.array([
        cfg.alpha_flap_arm_y * ca0 * difference_flap,
        cfg.alpha_flap_arm_x * np.sum(flap_z),
        cfg.alpha_flap_arm_y * sa0 * difference_flap,
    ])
    return force, moment_thrust + moment_reaction + moment_flap


def test_alpha_theory_matches_reference_equations():
    cfg = PhoenixHoverConfig()
    velocity = np.array([7.0, -0.4, 1.2])
    motors = np.array([620.0, 570.0])
    flaps = np.array([0.18, -0.11])
    expected_force, expected_moment = reference_equations(
        cfg, velocity, motors, flaps)
    result = AlphaAerodynamics(cfg).compute(velocity, motors, flaps)
    np.testing.assert_allclose(result.force_alpha, expected_force, atol=1e-12)
    np.testing.assert_allclose(result.moment_body_ts, expected_moment, atol=1e-12)


def test_equal_motors_and_zero_flaps_have_no_roll_or_yaw_moment():
    cfg = PhoenixHoverConfig()
    result = AlphaAerodynamics(cfg).compute(
        np.zeros(3), np.full(2, 550.0), np.zeros(2))
    np.testing.assert_allclose(result.moment_body_ts[[0, 2]], 0.0, atol=1e-12)


def test_gazebo_flu_tailsitter_vector_conversion_is_invertible():
    vector = np.array([1.0, 2.0, 3.0])
    np.testing.assert_allclose(vector_gz_flu_to_ts(vector), [3.0, -2.0, 1.0])
    np.testing.assert_allclose(
        vector_ts_to_gz_flu(vector_gz_flu_to_ts(vector)), vector)


def test_alpha_allocator_respects_current_actuator_limits():
    cfg = PhoenixHoverConfig()
    allocator = AlphaTheoryAllocator(cfg)
    allocation = allocator.allocate(
        cfg.hover_total_thrust,
        np.array([0.5, 0.5, 0.5]),
        np.array([4.0, 0.0, 0.3]),
    )
    assert np.all(allocation.motor_speed_left_right >= 0.0)
    assert np.all(allocation.motor_speed_left_right <= cfg.motor_speed_max)
    assert abs(allocation.flap_angle_left_right[0]) <= cfg.left_flap_limit
    assert abs(allocation.flap_angle_left_right[1]) <= cfg.right_flap_limit
    assert np.all(np.isfinite(allocation.achieved_moment_ts))


def test_parameters_requiring_identification_are_explicit():
    defaults = PhoenixHoverConfig().alpha_identification_defaults
    assert 'alpha_zero_lift_rad' in defaults
    assert 'alpha_c_lt_delta' in defaults
    assert 'servo_time_constant' in defaults
    assert all(np.isfinite(list(defaults.values())))


def test_high_pass_has_zero_steady_output_and_detects_a_step():
    high_pass = ButterworthHighPass(1.0, 250.0, 2, np.array([0.2, -0.1]))
    np.testing.assert_allclose(high_pass.update([0.2, -0.1]), 0.0, atol=1e-15)
    assert np.linalg.norm(high_pass.update([0.3, -0.2])) > 0.1


def test_flatness_port_matches_reference_regression_values():
    cfg = PhoenixHoverConfig()
    controller = FlatnessAttitudeController(cfg)
    q_current = attitude_px4_to_ts([1.0, 0.0, 0.0, 0.0])
    result = controller.attitude_and_thrust(
        np.array([1.2, -0.3, -4.7]),
        np.array([3.0, 0.2, -0.4]),
        0.13,
        0.2,
        q_current,
        return_angles=True,
    )
    np.testing.assert_allclose(
        result[0],
        [-0.918018434787, 0.090324592505, -0.379685604015,
         -0.070160269979],
        atol=1e-11,
    )
    np.testing.assert_allclose(
        result[1:], [4.247891588531, 6.170384653873, 0.830574127239],
        atol=1e-11,
    )
    rates = controller.feedforward_rates(
        np.array([2.0, 0.1, -0.2]),
        np.array([0.3, -0.1, 0.05]),
        np.array([0.02, 0.03, -0.01]),
        0.2,
        0.07,
        np.array([1.2, -0.3, -4.7]),
        result[2],
        result[3],
        0.13,
    )
    np.testing.assert_allclose(
        rates, [-0.059124639249, -0.053191753453, 0.039040096428],
        atol=1e-11,
    )
