"""Attitude PD and angular-acceleration INDI in Tailsitter-control axes."""

import numpy as np

from .math_utils import rotation_vector_error


class AttitudeINDIController:
    def __init__(self, config):
        self.attitude_gain = np.asarray(config.attitude_gain, dtype=float)
        self.rate_gain = np.asarray(config.rate_gain, dtype=float)
        self.acceleration_limit = np.asarray(config.angular_acceleration_limit, dtype=float)
        self.moment_limit = np.asarray(config.moment_limit, dtype=float)
        self.inertia = np.asarray(config.inertia_ts, dtype=float)

    def angular_acceleration_command(self, q_current_ts, q_desired_ts,
                                     omega_lpf_ts, omega_reference_ts=None):
        if omega_reference_ts is None:
            omega_reference_ts = np.zeros(3)
        attitude_error = rotation_vector_error(q_current_ts, q_desired_ts)
        command = (
            self.attitude_gain * attitude_error
            + self.rate_gain * (np.asarray(omega_reference_ts) - np.asarray(omega_lpf_ts))
        )
        return np.clip(command, -self.acceleration_limit, self.acceleration_limit)

    def moment_command(self, acceleration_command_ts, acceleration_lpf_ts,
                       estimated_moment_lpf_ts):
        moment = (
            self.inertia
            @ (np.asarray(acceleration_command_ts) - np.asarray(acceleration_lpf_ts))
            + np.asarray(estimated_moment_lpf_ts)
        )
        return np.clip(moment, -self.moment_limit, self.moment_limit)
