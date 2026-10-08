"""Attitude PD and angular-acceleration INDI in Tailsitter-control axes."""

import numpy as np

from .math_utils import quaternion_to_matrix, rotation_vector_error


def transport_reference_rates(q_current_ts, q_reference_ts, omega_reference_ts):
    """Express reference-body angular velocity in the measured body basis."""
    omega = np.asarray(omega_reference_ts, dtype=float)
    if omega.shape != (3,) or not np.all(np.isfinite(omega)):
        raise ValueError('reference angular velocity must be a finite 3-vector')
    return (quaternion_to_matrix(q_current_ts).T
            @ quaternion_to_matrix(q_reference_ts) @ omega)


class AttitudeINDIController:
    def __init__(self, config):
        self.attitude_gain = np.asarray(config.attitude_gain, dtype=float)
        self.rate_gain = np.asarray(config.rate_gain, dtype=float)
        self.tracking_attitude_gain = np.asarray(config.tracking_attitude_gain, dtype=float)
        self.tracking_rate_gain = np.asarray(config.tracking_rate_gain, dtype=float)
        self.trajectory_hover_attitude_gain = np.asarray(config.trajectory_hover_attitude_gain, dtype=float)
        self.trajectory_hover_rate_gain = np.asarray(config.trajectory_hover_rate_gain, dtype=float)
        self.acceleration_limit = np.asarray(config.angular_acceleration_limit, dtype=float)
        self.moment_limit = np.asarray(config.moment_limit, dtype=float)
        self.inertia = np.asarray(config.inertia_ts, dtype=float)

    def angular_acceleration_command(self, q_current_ts, q_desired_ts,
                                     omega_lpf_ts, omega_reference_ts=None,
                                     tracking_blend=0.0, trajectory_mode=False):
        if omega_reference_ts is None:
            omega_reference_ts = np.zeros(3)
        attitude_error = rotation_vector_error(q_current_ts, q_desired_ts)
        if not np.isfinite(tracking_blend) or not 0.0 <= tracking_blend <= 1.0:
            raise ValueError('tracking blend must be between zero and one')
        low_attitude_gain = (self.trajectory_hover_attitude_gain
                             if trajectory_mode else self.attitude_gain)
        low_rate_gain = (self.trajectory_hover_rate_gain
                        if trajectory_mode else self.rate_gain)
        attitude_gain = low_attitude_gain + tracking_blend * (
            self.tracking_attitude_gain - low_attitude_gain)
        rate_gain = low_rate_gain + tracking_blend * (
            self.tracking_rate_gain - low_rate_gain)
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
