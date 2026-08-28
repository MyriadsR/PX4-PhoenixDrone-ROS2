from types import SimpleNamespace

import numpy as np
import pytest

from phoenix_tailsitter_control.actuator_feedback import parse_actuator_joint_state


def joint_state(names, position, velocity):
    return SimpleNamespace(name=names, position=position, velocity=velocity)


def test_joint_feedback_maps_gazebo_actuators_to_tailsitter_left_right():
    message = joint_state(
        ['rotor_left_joint', 'rotor_right_joint',
         'left_elevon_joint', 'right_elevon_joint'],
        [0.0, 0.0, 0.2, -0.1],
        [-55.0, 60.0, 0.0, 0.0],
    )
    feedback = parse_actuator_joint_state(message, 10.0)
    # TS left is Gazebo rotor_right after the required +90 deg body-frame map.
    np.testing.assert_allclose(feedback.motor_speed_left_right, [600.0, 550.0])
    np.testing.assert_allclose(feedback.flap_angle_left_right, [0.2, -0.1])


@pytest.mark.parametrize(
    'message',
    [
        joint_state(
            ['rotor_left_joint', 'rotor_right_joint', 'left_elevon_joint'],
            [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        joint_state(
            ['rotor_left_joint', 'rotor_left_joint',
             'left_elevon_joint', 'right_elevon_joint'],
            [0.0] * 4, [0.0] * 4),
        joint_state(
            ['rotor_left_joint', 'rotor_right_joint',
             'left_elevon_joint', 'right_elevon_joint'],
            [0.0] * 3, [0.0] * 4),
        joint_state(
            ['rotor_left_joint', 'rotor_right_joint',
             'left_elevon_joint', 'right_elevon_joint'],
            [0.0, 0.0, np.nan, 0.0], [0.0] * 4),
    ],
)
def test_joint_feedback_rejects_incomplete_or_invalid_messages(message):
    with pytest.raises(ValueError):
        parse_actuator_joint_state(message, 10.0)


def test_joint_feedback_rejects_invalid_slowdown():
    message = joint_state(
        ['rotor_left_joint', 'rotor_right_joint',
         'left_elevon_joint', 'right_elevon_joint'],
        [0.0] * 4, [0.0] * 4,
    )
    with pytest.raises(ValueError):
        parse_actuator_joint_state(message, 0.0)
