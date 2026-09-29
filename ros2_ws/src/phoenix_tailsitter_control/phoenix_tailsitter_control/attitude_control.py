"""Attitude PD and angular-acceleration INDI in Tailsitter-control axes."""

import numpy as np

from .math_utils import rotation_vector_error


class AttitudeINDIController:
    def __init__(self, config):
        self.attitude_gain = np.asarray(config.attitude_gain, dtype=float)
        self.rate_gain = np.asarray(config.rate_gain, dtype=float)
        self.tracking_attitude_gain = np.asarray(config.tracking_attitude_gain, dtype=float)
        self.tracking_rate_gain = np.asarray(config.tracking_rate_gain, dtype=float)
        self.acceleration_limit = np.asarray(config.angular_acceleration_limit, dtype=float)
        self.moment_limit = np.asarray(config.moment_limit, dtype=float)
        self.inertia = np.asarray(config.inertia_ts, dtype=float)

    def angular_acceleration_command(self, q_current_ts, q_desired_ts,
                                     omega_lpf_ts, omega_reference_ts=None,
                                     tracking_blend=0.0):
        if omega_reference_ts is None:
            omega_reference_ts = np.zeros(3)
        attitude_error = rotation_vector_error(q_current_ts, q_desired_ts)
        if not np.isfinite(tracking_blend) or not 0.0 <= tracking_blend <= 1.0:
            raise ValueError('tracking blend must be between zero and one')
        attitude_gain = self.attitude_gain + tracking_blend * (
            self.tracking_attitude_gain - self.attitude_gain)
        rate_gain = self.rate_gain + tracking_blend * (
            self.tracking_rate_gain - self.rate_gain)
        command = (
            attitude_gain * attitude_error
            + rate_gain * (np.asarray(omega_reference_ts) - np.asarray(omega_lpf_ts))
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
