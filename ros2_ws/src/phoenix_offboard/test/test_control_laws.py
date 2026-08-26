import math

import pytest

from phoenix_offboard.phoenix_controller import PhoenixController
from phoenix_offboard.phoenix_position_controller import PhoenixPositionController


def test_reduced_attitude_identity_has_zero_error():
    controller = PhoenixController.__new__(PhoenixController)
    error = controller._attitude_error([1.0, 0.0, 0.0, 0.0],
                                       [1.0, 0.0, 0.0, 0.0])
    assert error == pytest.approx([0.0, 0.0, 0.0])


def test_nonlinear_mixer_is_symmetric_at_zero_moment():
    controller = PhoenixController.__new__(PhoenixController)
    motors, servos = controller._mix(0.4, [0.0, 0.0, 0.0])
    expected_motor = math.sqrt(0.4 / PhoenixController.KT) / PhoenixController.MAX_OMEGA
    assert motors == pytest.approx([expected_motor, expected_motor])
    assert servos == pytest.approx([0.0, 0.0])


def test_position_hover_force_and_bug_compatible_heading():
    controller = PhoenixPositionController.__new__(PhoenixPositionController)
    force = controller._limit_force(
        [0.0, 0.0, -controller.MASS * controller.GRAVITY])
    quaternion, force_per_motor = controller._force_to_attitude(force)
    sqrt_half = math.sqrt(0.5)
    assert quaternion == pytest.approx([sqrt_half, 0.0, 0.0, sqrt_half])
    assert force_per_motor == pytest.approx(
        controller.MASS * controller.GRAVITY / 2.0)


def test_position_force_limits_match_original_offboard_branch():
    controller = PhoenixPositionController.__new__(PhoenixPositionController)
    assert controller._limit_force([0.0, 0.0, -20.0]) == pytest.approx(
        [0.0, 0.0, -controller.THR_MAX])
    limited = controller._limit_force([20.0, 0.0, -1.0])
    assert math.atan2(abs(limited[0]), -limited[2]) == pytest.approx(
        controller.TILT_MAX)
