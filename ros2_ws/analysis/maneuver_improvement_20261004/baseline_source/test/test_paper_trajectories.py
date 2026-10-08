"""Preserve original flight profiles and verify physical derivative continuity."""
import hashlib
from pathlib import Path

import numpy as np
import pytest
from phoenix_tailsitter_control import reference_trajectories
from phoenix_tailsitter_control.paper_trajectory_mission import NAMES, PaperTrajectoryAdapter


def test_reference_snapshot_is_unmodified():
    assert hashlib.sha256(Path(reference_trajectories.__file__).read_bytes()).hexdigest() == 'b2a2b64d5316a579ef5ae263c5b8d75af7568359f4c7a554b0cec97202831786'


@pytest.mark.parametrize('name', NAMES)
def test_translation_preserves_all_reference_derivatives_and_yaw(name):
    trajectory = PaperTrajectoryAdapter(name)
    for t in np.linspace(0, trajectory.total_duration, 137):
        actual = trajectory.sample(t)
        original = trajectory.reference.sample(t)
        np.testing.assert_allclose(actual.position, original[0] + trajectory.offset, atol=1e-12)
        np.testing.assert_allclose(np.r_[actual.velocity, actual.acceleration, actual.jerk, actual.yaw, actual.yawspeed], np.r_[original[1], original[2], original[3], original[4], original[5]], atol=1e-12)


@pytest.mark.parametrize('name', NAMES)
def test_entry_and_exit_match_reference_derivatives(name):
    trajectory = PaperTrajectoryAdapter(name)
    start = trajectory.entry_sample(0, 4)
    np.testing.assert_allclose(start.position, trajectory.staging_position, atol=1e-9)
    np.testing.assert_allclose(np.r_[start.velocity, start.acceleration, start.jerk], 0, atol=1e-9)
    entry_end = trajectory.entry_sample(4, 4)
    reference_start = trajectory.sample(0)
    np.testing.assert_allclose(np.r_[entry_end.position, entry_end.velocity, entry_end.acceleration, entry_end.jerk, entry_end.yaw, entry_end.yawspeed], np.r_[reference_start.position, reference_start.velocity, reference_start.acceleration, reference_start.jerk, reference_start.yaw, reference_start.yawspeed], atol=1e-8)
    exit_start = trajectory.exit_sample(trajectory.total_duration, 0, 4)
    reference_end = trajectory.sample(trajectory.total_duration)
    np.testing.assert_allclose(np.r_[exit_start.position, exit_start.velocity, exit_start.acceleration, exit_start.jerk], np.r_[reference_end.position, reference_end.velocity, reference_end.acceleration, reference_end.jerk], atol=1e-8)
    stopped = trajectory.exit_sample(trajectory.total_duration, 4, 4)
    np.testing.assert_allclose(np.r_[stopped.velocity, stopped.acceleration, stopped.jerk, stopped.yawspeed], 0, atol=1e-8)


@pytest.mark.parametrize('name,t', [('segments', 14), ('lemniscate', 1.2), ('knife-edge-transition', 1.8), ('circle-coordinated', .7), ('circular-knife-edge', .7), ('transition-to-forward', 1.5), ('transition-to-hover', 1.5), ('differential-turn', 1.5)])
def test_reference_position_derivatives_are_consistent(name, t):
    trajectory = PaperTrajectoryAdapter(name)
    h = 1e-4
    before, middle, after = [trajectory.sample(t + dt) for dt in (-h, 0, h)]
    np.testing.assert_allclose((after.position - before.position) / (2*h), middle.velocity, atol=2e-3)
    np.testing.assert_allclose((after.velocity - before.velocity) / (2*h), middle.acceleration, atol=2e-3)
    np.testing.assert_allclose((after.acceleration - before.acceleration) / (2*h), middle.jerk, atol=2e-3)
