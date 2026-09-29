import numpy as np
import pytest

from phoenix_tailsitter_control.lemniscate_mission_test import (
    BernoulliLemniscateTrajectory,
    smooth_stop_sample,
)


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
