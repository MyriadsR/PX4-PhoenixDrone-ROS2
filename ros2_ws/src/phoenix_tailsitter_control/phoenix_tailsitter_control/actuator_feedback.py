"""Gazebo joint-state parsing for PhoenixDrone actuator feedback."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ActuatorJointFeedback:
    motor_speed_left_right: np.ndarray
    flap_angle_left_right: np.ndarray


def parse_actuator_joint_state(message, rotor_velocity_slowdown):
    """Return actual actuator state in the controller's ``[left, right]`` order.

    Gazebo simulates rotor joint velocity at a reduced speed.  PhoenixAero and
    the motor plugin both recover physical rotor speed with the configured
    slowdown factor.  The SDF joint names are geometrical FLU names: after the
    required body-axis conversion, ``rotor_right_joint`` is TS left.
    """
    slowdown = float(rotor_velocity_slowdown)
    if not np.isfinite(slowdown) or slowdown <= 0.0:
        raise ValueError('rotor velocity slowdown must be positive')
    names = list(message.name)
    if len(names) != len(set(names)):
        raise ValueError('joint-state names must be unique')
    position = np.asarray(message.position, dtype=float)
    velocity = np.asarray(message.velocity, dtype=float)
    if position.shape != (len(names),) or velocity.shape != (len(names),):
        raise ValueError('joint-state arrays must match the name array')
    if not np.all(np.isfinite(np.concatenate((position, velocity)))):
        raise ValueError('joint-state values must be finite')
    indices = {name: index for index, name in enumerate(names)}
    required = (
        'rotor_left_joint', 'rotor_right_joint',
        'left_elevon_joint', 'right_elevon_joint',
    )
    if any(name not in indices for name in required):
        raise ValueError('joint state does not contain all PhoenixDrone actuators')

    motor_speed = slowdown * np.abs(np.array([
        velocity[indices['rotor_right_joint']],
        velocity[indices['rotor_left_joint']],
    ]))
    flap_angle = np.array([
        position[indices['left_elevon_joint']],
        position[indices['right_elevon_joint']],
    ])
    return ActuatorJointFeedback(motor_speed, flap_angle)
