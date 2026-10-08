import numpy as np
import pytest

from phoenix_tailsitter_control.position_mission_test import (
    build_cross_mission,
    build_mission,
    build_rectangle_mission,
    slew_setpoint,
    slew_setpoint_with_velocity,
    target_is_stable,
)


def test_cross_mission_uses_px4_ned_forward_left_and_returns_home():
    phases = build_cross_mission(2.0, 2.0, 3.0)
    assert [phase.name for phase in phases] == [
        'hover', 'forward', 'backward', 'left', 'right', 'land']
    np.testing.assert_allclose(phases[0].offset_ned, [0.0, 0.0, -2.0])
    np.testing.assert_allclose(phases[1].offset_ned, [2.0, 0.0, -2.0])
    np.testing.assert_allclose(phases[2].offset_ned, [0.0, 0.0, -2.0])
    np.testing.assert_allclose(phases[3].offset_ned, [0.0, -2.0, -2.0])
    np.testing.assert_allclose(phases[4].offset_ned, [0.0, 0.0, -2.0])
    np.testing.assert_allclose(phases[5].offset_ned, [0.0, 0.0, 0.0])


def test_cross_mission_rejects_out_of_campaign_distance():
    with pytest.raises(ValueError):
        build_cross_mission(2.01, 2.0, 3.0)


def test_rectangle_mission_flies_four_equal_horizontal_legs():
    phases = build_rectangle_mission(2.0, 2.0, 3.0)
    assert [phase.name for phase in phases] == [
        'hover', 'forward', 'forward_left', 'left', 'home', 'land']
    offsets = [phase.offset_ned for phase in phases]
    np.testing.assert_allclose(offsets[0], [0.0, 0.0, -2.0])
    np.testing.assert_allclose(offsets[1], [2.0, 0.0, -2.0])
    np.testing.assert_allclose(offsets[2], [2.0, -2.0, -2.0])
    np.testing.assert_allclose(offsets[3], [0.0, -2.0, -2.0])
    np.testing.assert_allclose(offsets[4], [0.0, 0.0, -2.0])
    np.testing.assert_allclose(offsets[5], [0.0, 0.0, 0.0])
    for start, end in zip(offsets[0:4], offsets[1:5]):
        assert np.linalg.norm((end - start)[:2]) == pytest.approx(2.0)
    assert max(np.linalg.norm(offset[:2]) for offset in offsets) == pytest.approx(
        2.0 * np.sqrt(2.0))


def test_build_mission_accepts_rectangle_alias():
    rectangle = build_mission('rectangle', 1.0, 1.0, 0.5)
    square = build_mission('square', 1.0, 1.0, 0.5)
    assert [phase.offset_ned.tolist() for phase in rectangle] == [
        phase.offset_ned.tolist() for phase in square]


def test_setpoint_slew_limits_horizontal_norm_and_vertical_rate():
    result = slew_setpoint(
        [0.0, 0.0, 0.0], [3.0, 4.0, -2.0], 0.5, 0.2, 0.3)
    np.testing.assert_allclose(result, [0.06, 0.08, -0.15])
    reached = slew_setpoint(result, [0.061, 0.081, -0.151], 0.5, 0.2, 0.3)
    np.testing.assert_allclose(reached, [0.061, 0.081, -0.151])


def test_slewed_setpoint_publishes_matching_feedforward_velocity():
    position, velocity = slew_setpoint_with_velocity(
        [0.0, 0.0, 0.0], [3.0, 4.0, -2.0], 0.5, 0.2, 0.3)
    np.testing.assert_allclose(position, [0.06, 0.08, -0.15])
    np.testing.assert_allclose(velocity, [0.12, 0.16, -0.30])


def test_target_stability_requires_setpoint_position_and_low_speed():
    target = np.array([2.0, 0.0, -2.0])
    assert target_is_stable(
        [1.85, 0.02, -1.90], [0.05, 0.02, 0.01],
        target, target, 0.30, 0.20, 0.30)
    assert not target_is_stable(
        [1.85, 0.02, -1.90], [0.31, 0.0, 0.0],
        target, target, 0.30, 0.20, 0.30)
    assert not target_is_stable(
        target, [0.0, 0.0, 0.0], [1.9, 0.0, -2.0],
        target, 0.30, 0.20, 0.30)
