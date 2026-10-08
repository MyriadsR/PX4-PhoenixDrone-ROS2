from types import SimpleNamespace

import numpy as np

from phoenix_tailsitter_control.allocator import PhoenixHoverAllocator
from phoenix_tailsitter_control.attitude_control import AttitudeINDIController
from phoenix_tailsitter_control.attitude_step_test import (
    axis_angle_quaternion,
    step_phase,
    summarize_axis_response,
)
from phoenix_tailsitter_control.config import PhoenixHoverConfig
from phoenix_tailsitter_control.controller_node import TailsitterController
from phoenix_tailsitter_control.debug import (
    ACTUATOR_FEEDBACK_DEBUG_FIELDS,
    CONTROL_DEBUG_FIELDS,
    POSITION_DEBUG_FIELDS,
    pack_actuator_feedback_debug,
    pack_control_debug,
    pack_position_debug,
)
from phoenix_tailsitter_control.filters import ButterworthLowPass


def test_control_mode_remains_fresh_across_normal_px4_publish_interval():
    cfg = PhoenixHoverConfig()
    controller = SimpleNamespace(
        cfg=cfg,
        control_mode=object(),
        control_mode_arrival_ns=1_000_000_000,
    )
    assert TailsitterController._mode_is_fresh(controller, 1_600_000_000)
    assert not TailsitterController._mode_is_fresh(controller, 2_600_000_000)


def test_filter_starts_at_steady_state():
    initial = np.array([1.0, -2.0, 3.0])
    low_pass = ButterworthLowPass(15.0, 250.0, 3, initial)
    np.testing.assert_allclose(low_pass.update(initial), initial, atol=1e-12)


def test_zero_attitude_error_produces_zero_angular_acceleration():
    cfg = PhoenixHoverConfig()
    controller = AttitudeINDIController(cfg)
    q = np.array([1.0, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(
        controller.angular_acceleration_command(q, q, np.zeros(3)),
        np.zeros(3),
    )


def test_maneuver_gains_blend_without_changing_default_hover_control():
    controller = AttitudeINDIController(PhoenixHoverConfig())
    q = np.array([1.0, 0.0, 0.0, 0.0])
    desired = axis_angle_quaternion(2, 0.1)
    rates = np.array([0.1, 0.1, 0.1])
    hover = controller.angular_acceleration_command(q, desired, rates)
    tracking = controller.angular_acceleration_command(
        q, desired, rates, tracking_blend=1.0)
    halfway = controller.angular_acceleration_command(
        q, desired, rates, tracking_blend=0.5)
    np.testing.assert_allclose(hover, [-0.3, -0.3, 0.3], atol=1e-12)
    np.testing.assert_allclose(halfway, 0.5 * (hover + tracking), atol=1e-12)
    assert tracking[2] > hover[2]


def test_hover_allocation_uses_equal_motors_and_zero_flaps():
    cfg = PhoenixHoverConfig()
    allocator = PhoenixHoverAllocator(cfg)
    result = allocator.allocate(cfg.hover_total_thrust, np.zeros(3))
    np.testing.assert_allclose(result.motor_speed_left_right[0], result.motor_speed_left_right[1])
    np.testing.assert_allclose(result.flap_angle_left_right, np.zeros(2), atol=1e-12)
    np.testing.assert_allclose(result.achieved_moment_ts, np.zeros(3), atol=1e-12)


def test_positive_tailsitter_yaw_uses_more_left_motor():
    cfg = PhoenixHoverConfig()
    allocator = PhoenixHoverAllocator(cfg)
    desired = np.array([0.0, 0.0, 0.10])
    result = allocator.allocate(cfg.hover_total_thrust, desired)
    assert result.motor_speed_left_right[0] > result.motor_speed_left_right[1]
    np.testing.assert_allclose(result.achieved_moment_ts, desired, atol=2e-5)
    motors_px4, _ = allocator.to_px4_controls(result)
    # FLU->FRD makes motorNumber 0 left and motorNumber 1 right.
    assert motors_px4[0] > motors_px4[1]


def test_motor_reaction_and_flap_moment_signs_include_flu_to_frd():
    cfg = PhoenixHoverConfig()
    allocator = PhoenixHoverAllocator(cfg)
    omega = np.array([600.0, 500.0])
    motor_only = allocator.estimate_moment(omega, np.zeros(2))
    assert motor_only[0] < 0.0
    assert motor_only[2] > 0.0

    common_positive_flap = allocator.estimate_moment(omega, np.array([0.1, 0.1]))
    assert common_positive_flap[1] > 0.0
    left_positive_only = allocator.estimate_moment(
        np.array([600.0, 600.0]), np.array([0.1, 0.0]))
    assert left_positive_only[0] < 0.0


def test_px4_servo_mapping_reverses_and_swaps_physical_flaps():
    cfg = PhoenixHoverConfig()
    allocator = PhoenixHoverAllocator(cfg)
    result = allocator.allocate(cfg.hover_total_thrust, np.zeros(3))
    result.flap_angle_left_right[:] = [0.25 * cfg.left_flap_limit,
                                       -0.20 * cfg.right_flap_limit]
    _, servos_px4 = allocator.to_px4_controls(result)
    # servo_0=right and servo_1=left; both SIM_GZ angle ranges are reversed.
    np.testing.assert_allclose(servos_px4, [0.20, -0.25])


def test_allocator_respects_asymmetric_physical_flap_limits():
    cfg = PhoenixHoverConfig()
    allocator = PhoenixHoverAllocator(cfg)
    result = allocator.allocate(cfg.hover_total_thrust, np.array([10.0, 10.0, 0.0]))
    assert abs(result.flap_angle_left_right[0]) <= cfg.left_flap_limit
    assert abs(result.flap_angle_left_right[1]) <= cfg.right_flap_limit


def test_attitude_step_profile_has_positive_and_negative_phases():
    assert step_phase(0.1, 0.4, 0.9, 0.5) == ('baseline_pre', 0.0)
    assert step_phase(0.5, 0.4, 0.9, 0.5) == ('plus', 1.0)
    assert step_phase(1.4, 0.4, 0.9, 0.5) == ('baseline_mid', 0.0)
    assert step_phase(2.0, 0.4, 0.9, 0.5) == ('minus', -1.0)
    assert step_phase(3.3, 0.4, 0.9, 0.5) == ('done', 0.0)


def test_axis_angle_quaternion_matches_requested_tailsitter_axis():
    angle = 0.05
    q = axis_angle_quaternion(2, angle)
    np.testing.assert_allclose(q, [np.cos(angle / 2.0), 0.0, 0.0,
                                   np.sin(angle / 2.0)])


def test_attitude_step_summary_requires_both_directions_and_bounded_response():
    records = []
    for value in np.linspace(0.02, 0.045, 20):
        records.append(('plus', np.array([value, 0.002, 0.0]), np.array([0.2, 0.0, 0.0])))
    for value in np.linspace(-0.02, -0.044, 20):
        records.append(('minus', np.array([value, -0.002, 0.0]), np.array([-0.2, 0.0, 0.0])))
    result = summarize_axis_response(records, 0, 0.05)
    assert result['direction_ok']
    assert result['bounded']
    assert result['passed']


def test_attitude_step_summary_rejects_large_overshoot():
    records = [
        ('plus', np.array([0.20, 0.0, 0.0]), np.array([0.2, 0.0, 0.0]))
        for _ in range(20)
    ]
    records += [
        ('minus', np.array([-0.05, 0.0, 0.0]), np.array([-0.2, 0.0, 0.0]))
        for _ in range(20)
    ]
    result = summarize_axis_response(records, 0, 0.05)
    assert result['direction_ok']
    assert not result['bounded']
    assert not result['passed']


def test_control_debug_vector_has_stable_finite_field_count():
    vector = pack_control_debug(
        np.zeros(3), np.ones(3), np.full(3, 2.0), np.full(3, 3.0),
        np.full(3, 4.0), np.full(3, 5.0), np.full(3, 6.0),
        np.array([0.7, 0.8]), np.array([0.1, -0.1]), 5.15, True)
    assert len(vector) == len(CONTROL_DEBUG_FIELDS) == 27
    assert vector[-1] == 1.0


def test_position_debug_vector_has_stable_finite_field_count():
    vector = pack_position_debug(
        np.zeros(3), np.ones(3), np.full(3, 2.0), np.full(3, 3.0),
        np.full(3, 4.0), np.full(3, 5.0), np.full(3, 6.0),
        np.full(3, 7.0), np.full(3, 8.0), np.full(3, 9.0),
        0.2, 5.1, True)
    assert len(vector) == len(POSITION_DEBUG_FIELDS) == 33
    assert vector[-1] == 1.0


def test_actuator_feedback_debug_vector_has_stable_finite_field_count():
    vector = pack_actuator_feedback_debug(
        np.array([500.0, 510.0]),
        np.array([505.0, 508.0]),
        np.array([0.1, -0.1]),
        np.array([0.11, -0.09]),
        0.004,
        True,
        True,
    )
    assert len(vector) == len(ACTUATOR_FEEDBACK_DEBUG_FIELDS) == 15
    np.testing.assert_allclose(vector[4:6], [5.0, -2.0])
    assert vector[-2:] == [1.0, 1.0]
