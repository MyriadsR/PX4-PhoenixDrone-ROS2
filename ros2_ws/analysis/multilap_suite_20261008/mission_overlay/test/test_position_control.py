import math
from types import SimpleNamespace

import numpy as np
import pytest

from phoenix_tailsitter_control.config import PhoenixHoverConfig
from phoenix_tailsitter_control.frames import C_PX4_TS
from phoenix_tailsitter_control.math_utils import quaternion_to_matrix
from phoenix_tailsitter_control.position_control import (
    HoverForceSlewLimiter,
    LinearAccelerationINDIController,
    PositionController,
    apply_takeoff_force_floor,
    degraded_vertical_force,
    force_to_tailsitter_attitude,
    limit_hover_force,
    resolve_trajectory_reference,
)


def test_hover_force_slew_limiter_bounds_horizontal_vector_change():
    limiter = HoverForceSlewLimiter(0.5)
    first = limiter.update([1.0, 0.0, -5.0], 0.1)
    second = limiter.update([-1.0, 0.0, -4.0], 0.1)

    assert np.allclose(first, [0.05, 0.0, -5.0])
    assert np.allclose(second, [0.0, 0.0, -4.0])


def test_hover_force_slew_limiter_reset_and_validation():
    limiter = HoverForceSlewLimiter(1.0)
    limiter.update([0.5, 0.0, -5.0], 0.1)
    limiter.reset()
    assert np.allclose(limiter.update([0.0, 0.2, -4.0], 0.1), [0.0, 0.1, -4.0])
    with pytest.raises(ValueError):
        limiter.update([np.nan, 0.0, -4.0], 0.1)


def test_degraded_vertical_force_holds_climbs_and_descends_in_ned():
    config = PhoenixHoverConfig()
    hover = degraded_vertical_force(
        -2.0, 0.0, -2.0, 0.0, 0.0, config)
    climb = degraded_vertical_force(
        -1.0, 0.0, -2.0, 0.0, 0.0, config)
    descend = degraded_vertical_force(
        -2.0, 0.0, 0.0, 0.0, 0.0, config)

    np.testing.assert_allclose(hover, [0.0, 0.0, -config.hover_total_thrust])
    assert climb[2] < hover[2]
    assert descend[2] > hover[2]
    assert -climb[2] <= config.maximum_position_thrust
    assert -descend[2] >= config.minimum_position_thrust


def test_degraded_vertical_force_rejects_nonfinite_state():
    with pytest.raises(ValueError):
        degraded_vertical_force(
            math.nan, 0.0, -2.0, 0.0, 0.0, PhoenixHoverConfig())
from phoenix_tailsitter_control.position_step_test import (
    position_step_phase,
    summarize_position_response,
    summarize_takeoff_response,
)


def trajectory(position, velocity, acceleration, jerk=None, yaw=math.nan,
               yawspeed=math.nan):
    return SimpleNamespace(
        position=position,
        velocity=velocity,
        acceleration=acceleration,
        jerk=[math.nan] * 3 if jerk is None else jerk,
        yaw=yaw,
        yawspeed=yawspeed,
    )


def test_trajectory_nan_semantics_hold_finite_position_with_zero_velocity():
    reference = resolve_trajectory_reference(
        trajectory([1.0, math.nan, -0.5], [math.nan, 0.2, math.nan],
                   [math.nan, math.nan, math.nan]),
        np.array([0.1, 0.2, 0.3]),
        np.array([0.4, 0.5, 0.6]),
        0.7,
    )
    np.testing.assert_allclose(reference.position, [1.0, 0.2, -0.5])
    np.testing.assert_allclose(reference.velocity, [0.0, 0.2, 0.0])
    np.testing.assert_array_equal(reference.position_mask, [True, False, True])
    np.testing.assert_array_equal(reference.velocity_mask, [True, True, True])
    np.testing.assert_array_equal(reference.acceleration_mask, [False] * 3)
    np.testing.assert_allclose(reference.acceleration, np.zeros(3))
    assert reference.yaw == 0.7
    assert reference.yawspeed == 0.0


def test_position_feedback_moves_toward_positive_north_reference():
    cfg = PhoenixHoverConfig()
    controller = PositionController(cfg)
    reference = resolve_trajectory_reference(
        trajectory([0.1, 0.0, -0.2], [math.nan] * 3, [math.nan] * 3),
        np.array([0.0, 0.0, -0.2]), np.zeros(3), 0.0)
    command = controller.acceleration_command(
        reference, np.array([0.0, 0.0, -0.2]), np.zeros(3), np.zeros(3),
        C_PX4_TS)
    assert command[0] > 0.0
    np.testing.assert_allclose(command[1:], np.zeros(2), atol=1e-12)


def test_position_feedback_limits_do_not_clip_trajectory_feedforward():
    cfg = PhoenixHoverConfig()
    controller = PositionController(cfg)
    acceleration = np.array([2.0, -3.0, 4.0])
    reference = resolve_trajectory_reference(
        trajectory([math.nan] * 3, [math.nan] * 3, acceleration),
        np.zeros(3), np.zeros(3), 0.0)

    command = controller.acceleration_command(
        reference, np.zeros(3), np.zeros(3), np.zeros(3), C_PX4_TS)

    np.testing.assert_allclose(command, acceleration)


def test_tracking_reference_uses_separate_maneuver_gains():
    cfg = PhoenixHoverConfig()
    controller = PositionController(cfg)
    position = np.zeros(3)
    velocity = np.zeros(3)
    position_only = resolve_trajectory_reference(
        trajectory([0.01, 0.0, 0.0], [0.01, 0.0, 0.0], [math.nan] * 3),
        position, velocity, 0.0)
    tracking = resolve_trajectory_reference(
        trajectory([0.01, 0.0, 0.0], [0.01, 0.0, 0.0], [0.0] * 3),
        position, velocity, 0.0)

    position_command = controller.acceleration_command(
        position_only, position, velocity, np.zeros(3), np.eye(3))
    tracking_command = controller.acceleration_command(
        tracking, position, velocity, np.zeros(3), np.eye(3))

    np.testing.assert_allclose(
        position_command[0],
        0.01 * (cfg.position_gain[0] + cfg.velocity_gain[0]),
    )
    np.testing.assert_allclose(
        tracking_command[0],
        0.01 * (
            cfg.tracking_position_gain[0] + cfg.tracking_velocity_gain[0]),
    )
    assert tracking_command[0] != position_command[0]


def test_linear_indi_preserves_estimated_hover_force_at_zero_error():
    cfg = PhoenixHoverConfig()
    controller = LinearAccelerationINDIController(cfg)
    hover_force = np.array([0.0, 0.0, -cfg.hover_total_thrust])
    np.testing.assert_allclose(
        controller.force_command(np.zeros(3), np.zeros(3), hover_force),
        hover_force,
    )


def test_linear_indi_blends_nominal_and_incremental_force():
    cfg = PhoenixHoverConfig()
    controller = LinearAccelerationINDIController(cfg)
    result = controller.force_command(np.ones(3), np.zeros(3), np.zeros(3))
    gravity = np.array([0.0, 0.0, cfg.gravity])
    nominal = cfg.mass * (np.ones(3) - gravity)
    indi = cfg.mass * np.ones(3)
    np.testing.assert_allclose(result, nominal + cfg.linear_indi_blend * (indi - nominal))
    assert cfg.linear_indi_blend == 1.0


def test_takeoff_force_floor_restores_gravity_compensation_when_ground_started():
    cfg = PhoenixHoverConfig()
    reference = resolve_trajectory_reference(
        trajectory([0.0, 0.0, -1.0], [0.0, 0.0, -0.3], [0.0, 0.0, -1.0]),
        np.zeros(3), np.zeros(3), 0.0)
    force = np.array([0.4, -0.2, -3.4])
    nominal = np.array([0.0, 0.0, -cfg.hover_total_thrust - cfg.mass])
    estimated = np.array([0.0, 0.0, -3.0])

    result = apply_takeoff_force_floor(
        force, nominal, estimated, np.zeros(3), reference, cfg)

    np.testing.assert_allclose(result, nominal)


def test_takeoff_force_floor_keeps_airborne_indi_force_when_estimate_is_valid():
    cfg = PhoenixHoverConfig()
    reference = resolve_trajectory_reference(
        trajectory([0.0, 0.0, -1.0], [0.0, 0.0, -0.3], [0.0, 0.0, -1.0]),
        np.array([0.0, 0.0, -0.9]), np.zeros(3), 0.0)
    force = np.array([0.2, -0.1, -6.9])
    nominal = np.array([0.0, 0.0, -8.0])
    estimated = np.array([0.0, 0.0, -0.9 * cfg.hover_total_thrust])

    result = apply_takeoff_force_floor(
        force, nominal, estimated, np.array([0.0, 0.0, -0.9]), reference, cfg)

    np.testing.assert_allclose(result, force)


def test_ground_only_takeoff_floor_preserves_airborne_maneuver_force_with_low_lift():
    cfg = PhoenixHoverConfig()
    position = np.array([0.0, 0.0, -9.5])
    reference = resolve_trajectory_reference(
        trajectory([0.0, 0.0, -10.0], [0.0, 6.0, 0.0], [0.0, -12.0, 0.0]),
        position, np.zeros(3), 0.0)
    force = np.array([2.0, -8.4, -4.0])
    result = apply_takeoff_force_floor(
        force, [0.0, -8.4, -8.0], [0.0, 0.0, -3.0], position,
        reference, cfg, ground_only=True)
    np.testing.assert_allclose(result, force)


def test_force_estimate_uses_ts_axes_and_phoenix_aero_force_directions():
    cfg = PhoenixHoverConfig()
    controller = LinearAccelerationINDIController(cfg)
    propulsion, aerodynamic = controller.estimate_force_components_ts(
        [500.0, 500.0], [0.1, 0.1])
    force = propulsion + aerodynamic
    assert force[0] > 0.0
    assert force[1] == 0.0
    assert force[2] > 0.0
    assert aerodynamic[0] < 0.0
    assert aerodynamic[2] > 0.0
    expected_thrust = 2.0 * cfg.motor_thrust_coefficient * 500.0 ** 2
    np.testing.assert_allclose(propulsion[0], expected_thrust)


def test_hover_force_maps_to_requested_ts_body_basis():
    q_ts, thrust = force_to_tailsitter_attitude([0.0, 0.0, -5.0], yaw=0.0)
    np.testing.assert_allclose(quaternion_to_matrix(q_ts), C_PX4_TS, atol=1e-12)
    assert thrust == 5.0


def test_hover_force_limits_tilt_and_total_thrust():
    cfg = PhoenixHoverConfig()
    limited = limit_hover_force([100.0, 0.0, -100.0], cfg)
    assert np.linalg.norm(limited) <= cfg.maximum_position_thrust + 1e-12
    tilt = math.atan2(np.linalg.norm(limited[:2]), -limited[2])
    assert tilt <= cfg.position_tilt_limit_rad + 1e-12
    assert -limited[2] >= cfg.minimum_position_thrust - 1e-12


def test_position_step_profile_contains_takeoff_and_both_directions():
    assert position_step_phase(0.1, 1.0, 0.5, 0.7, 0.4) == ('climb', 0.0)
    assert position_step_phase(1.2, 1.0, 0.5, 0.7, 0.4) == ('baseline_pre', 0.0)
    assert position_step_phase(1.6, 1.0, 0.5, 0.7, 0.4) == ('plus', 1.0)
    assert position_step_phase(2.5, 1.0, 0.5, 0.7, 0.4) == ('baseline_mid', 0.0)
    assert position_step_phase(2.8, 1.0, 0.5, 0.7, 0.4) == ('minus', -1.0)


def test_position_step_summary_checks_direction_and_bounds():
    records = []
    for _ in range(20):
        records.append(('baseline_pre', np.zeros(3), np.zeros(3)))
        records.append(('plus', np.array([0.08, 0.0, 0.0]), np.array([0.1, 0.0, 0.0])))
        records.append(('baseline_mid', np.zeros(3), np.zeros(3)))
        records.append(('minus', np.array([-0.07, 0.0, 0.0]), np.array([-0.1, 0.0, 0.0])))
    result = summarize_position_response(records, 0, 0.1)
    assert result['direction_ok']
    assert result['bounded']
    assert result['passed']


def test_takeoff_summary_requires_absolute_climb_and_steady_hover():
    origin = np.array([0.0, 0.0, 0.0])
    target = np.array([0.0, 0.0, -0.30])
    records = []
    for z in np.linspace(0.0, -0.30, 100):
        records.append(('climb', np.array([0.0, 0.0, z]),
                        np.array([0.0, 0.0, -0.1])))
    for _ in range(100):
        records.append(('baseline_pre', np.array([0.01, -0.01, -0.29]),
                        np.array([0.01, 0.0, 0.02])))
    result = summarize_takeoff_response(records, origin, target, 0.30)
    assert result['takeoff_reached']
    assert result['hover_stable']
    assert result['takeoff_passed']


def test_takeoff_summary_rejects_estimator_drift_false_positive():
    origin = np.array([0.0, 0.0, -0.04])
    target = np.array([0.0, 0.0, -0.34])
    records = []
    for z in np.linspace(-0.04, -0.11, 100):
        records.append(('climb', np.array([0.0, 0.0, z]), np.zeros(3)))
    for _ in range(100):
        records.append(('baseline_pre', np.array([0.0, 0.0, -0.09]),
                        np.zeros(3)))
    result = summarize_takeoff_response(records, origin, target, 0.30)
    assert not result['takeoff_reached']
    assert not result['hover_stable']
    assert not result['takeoff_passed']
