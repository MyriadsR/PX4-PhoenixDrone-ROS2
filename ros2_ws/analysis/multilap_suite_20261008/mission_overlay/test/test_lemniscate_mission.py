from types import SimpleNamespace

import numpy as np
import pytest

from phoenix_tailsitter_control.lemniscate_mission_test import (
    BernoulliLemniscateTrajectory,
    LemniscateMissionTest,
    abort_brake_sample,
    landing_descent_allowed,
    landing_disarm_allowed,
    landed_state_confirmed,
    near_ground_runaway,
    smooth_stop_sample,
)


def test_landing_waits_for_horizontal_error_and_drift_to_settle():
    setpoint = np.array([-5.0, -1.0, -2.0])
    assert not landing_descent_allowed(
        [-7.0, 1.0, -1.7], [0.1, 0.0, 0.0], setpoint)
    assert not landing_descent_allowed(
        [-5.2, -1.1, -1.7], [0.8, 0.0, 0.0], setpoint)
    assert landing_descent_allowed(
        [-5.2, -1.1, -1.7], [0.3, 0.1, 0.0], setpoint)


def test_landing_can_disarm_after_confirmed_touchdown_with_height_bias():
    origin = np.zeros(3)
    assert landing_disarm_allowed([0, 0, -0.1], [0.1, 0, 0], origin, False)
    assert landing_disarm_allowed([0, 0, -0.39], [0.1, 0, 0], origin, True)
    assert not landing_disarm_allowed([0, 0, -0.39], [0.1, 0, 0], origin, False)
    assert not landing_disarm_allowed([0, 0, -0.8], [0.1, 0, 0], origin, True)
    assert not landing_disarm_allowed([0, 0, -0.39], [1, 0, 0], origin, True)


def test_landed_state_requires_two_spaced_fresh_reports():
    first = 1_000_000_000
    second = 1_600_000_000
    confirmed = landed_state_confirmed
    assert not confirmed(1, first, first, second)
    assert not confirmed(2, first, first + 100_000_000, second)
    assert confirmed(2, first, second, second)
    assert not confirmed(2, first, second, second + 2_000_000_000)


def test_runaway_disarm_is_near_ground_and_outside_boundary():
    origin = np.zeros(3)
    assert not near_ground_runaway([15, 0, -1.0], origin, 14.0)
    assert not near_ground_runaway([13, 0, -0.2], origin, 14.0)
    assert near_ground_runaway([15, 0, -0.2], origin, 14.0)


def test_lemniscate_starts_above_origin_and_closes_one_lap():
    trajectory = BernoulliLemniscateTrajectory(
        speed=0.15, lap_time=48.0, laps=1, takeoff_height=2.0)
    start = trajectory.sample(0.0)
    end = trajectory.sample(trajectory.total_duration)
    np.testing.assert_allclose(start.position, [0.0, 0.0, -2.0], atol=1e-12)
    np.testing.assert_allclose(end.position, start.position, atol=2e-4)


def test_lemniscate_velocity_has_constant_requested_speed():
    trajectory = BernoulliLemniscateTrajectory(
        speed=0.2, lap_time=40.0, laps=1, takeoff_height=1.5)
    for t in np.linspace(0.0, trajectory.total_duration, 25, endpoint=False):
        sample = trajectory.sample(float(t))
        assert np.linalg.norm(sample.velocity) == pytest.approx(0.2, rel=1e-4)
        assert np.all(np.isfinite(np.concatenate((
            sample.position, sample.velocity, sample.acceleration, sample.jerk,
        ))))
        assert np.isfinite(sample.yaw)
        assert np.isfinite(sample.yawspeed)


def test_lemniscate_horizontal_bound_is_finite_and_visible():
    trajectory = BernoulliLemniscateTrajectory(
        speed=0.15, lap_time=48.0, laps=1, takeoff_height=2.0)
    assert 1.0 < trajectory.horizontal_radius_bound() < 3.0


def test_lemniscate_entry_ramps_from_rest_and_joins_constant_speed_curve():
    trajectory = BernoulliLemniscateTrajectory(
        speed=2.0, lap_time=10.0, laps=1, takeoff_height=1.0)
    duration = 4.0

    start = trajectory.entry_sample(0.0, duration)
    midpoint = trajectory.entry_sample(0.5 * duration, duration)
    end = trajectory.entry_sample(duration, duration)
    joined = trajectory.sample(0.5 * duration)

    np.testing.assert_allclose(start.position, trajectory.sample(0.0).position)
    np.testing.assert_allclose(start.velocity, np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(start.acceleration, np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(start.jerk, np.zeros(3), atol=1e-12)
    assert np.linalg.norm(midpoint.velocity) == pytest.approx(1.0, rel=1e-4)
    np.testing.assert_allclose(end.position, joined.position, atol=1e-12)
    np.testing.assert_allclose(end.velocity, joined.velocity, atol=1e-12)
    np.testing.assert_allclose(end.acceleration, joined.acceleration, atol=1e-12)
    np.testing.assert_allclose(end.jerk, joined.jerk, atol=1e-12)


def test_lemniscate_lap_closes_from_shifted_entry_phase():
    trajectory = BernoulliLemniscateTrajectory(
        speed=2.0, lap_time=10.0, laps=1, takeoff_height=1.0)
    phase_offset = 2.0
    np.testing.assert_allclose(
        trajectory.sample(phase_offset + trajectory.total_duration).position,
        trajectory.sample(phase_offset).position,
        atol=2e-4,
    )


def test_lemniscate_exit_leaves_constant_speed_curve_and_stops_smoothly():
    trajectory = BernoulliLemniscateTrajectory(
        speed=2.0, lap_time=10.0, laps=1, takeoff_height=1.0)
    start_time = 12.0
    duration = 4.0

    start = trajectory.exit_sample(start_time, 0.0, duration)
    joined = trajectory.sample(start_time)
    midpoint = trajectory.exit_sample(start_time, 0.5 * duration, duration)
    end = trajectory.exit_sample(start_time, duration, duration)
    stopped = trajectory.sample(start_time + 0.5 * duration)

    np.testing.assert_allclose(start.position, joined.position, atol=1e-12)
    np.testing.assert_allclose(start.velocity, joined.velocity, atol=1e-12)
    np.testing.assert_allclose(start.acceleration, joined.acceleration, atol=1e-12)
    np.testing.assert_allclose(start.jerk, joined.jerk, atol=1e-12)
    assert np.linalg.norm(midpoint.velocity) == pytest.approx(1.0, rel=1e-4)
    np.testing.assert_allclose(end.position, stopped.position, atol=1e-12)
    np.testing.assert_allclose(end.velocity, np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(end.acceleration, np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(end.jerk, np.zeros(3), atol=1e-12)


def test_smooth_stop_preserves_endpoints_and_stops_in_a_straight_line():
    position = np.array([1.0, -2.0, -0.8])
    velocity = np.array([2.0, -1.0, 0.3])
    duration = 2.0

    start = smooth_stop_sample(position, velocity, 0.0, duration, -1.0)
    midpoint = smooth_stop_sample(
        position, velocity, 0.5 * duration, duration, -1.0)
    end = smooth_stop_sample(position, velocity, duration, duration, -1.0)

    np.testing.assert_allclose(start.position, [1.0, -2.0, -1.0])
    np.testing.assert_allclose(start.velocity, [2.0, -1.0, 0.0])
    np.testing.assert_allclose(start.acceleration, np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(midpoint.velocity, [1.0, -0.5, 0.0])
    np.testing.assert_allclose(end.position, [3.0, -3.0, -1.0])
    np.testing.assert_allclose(end.velocity, np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(end.acceleration, np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(end.jerk, np.zeros(3), atol=1e-12)


def test_abort_brake_stops_horizontal_motion_and_climbs_smoothly():
    position = np.array([2.0, -1.0, -0.6])
    velocity = np.array([4.0, 1.0, 0.4])
    start = abort_brake_sample(position, velocity, 0.0, 4.0, -2.0)
    middle = abort_brake_sample(position, velocity, 2.0, 4.0, -2.0)
    end = abort_brake_sample(position, velocity, 4.0, 4.0, -2.0)
    np.testing.assert_allclose(start.position, position)
    np.testing.assert_allclose(start.velocity, [4.0, 1.0, 0.0])
    assert -2.0 < middle.position[2] < -0.6
    np.testing.assert_allclose(end.position, [10.0, 1.0, -2.0])
    np.testing.assert_allclose(end.velocity, np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(end.acceleration, np.zeros(3), atol=1e-12)


def test_high_speed_exit_abort_enters_measured_braking_phase():
    mission = SimpleNamespace(
        done=False, aborted=False, phase='exit',
        local_position=(
            np.array([1.0, 2.0, -0.6]), np.array([4.0, 1.0, 0.2])),
        setpoint=np.zeros(3), setpoint_velocity=np.zeros(3),
        origin=np.zeros(3), takeoff_height=2.0,
        posttrack_braking_acceleration=2.0,
        get_logger=lambda: SimpleNamespace(error=lambda _: None),
        _is_armed_offboard=lambda: True,
    )
    mission._set_phase = lambda phase, _: setattr(mission, 'phase', phase)
    LemniscateMissionTest._abort_to_land(mission, 'test', 0)
    assert mission.aborted
    assert mission.phase == 'abort_brake'
    assert mission.abort_brake_duration_s >= (
        1.875 * np.linalg.norm([4.0, 1.0]) / 2.0)
    assert mission.abort_brake_target_z == -2.0
    np.testing.assert_allclose(mission.origin[:2], [1.0, 2.0])
