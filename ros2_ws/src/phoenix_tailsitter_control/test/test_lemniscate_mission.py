import numpy as np
import pytest

from phoenix_tailsitter_control.lemniscate_mission_test import (
    BernoulliLemniscateTrajectory,
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
