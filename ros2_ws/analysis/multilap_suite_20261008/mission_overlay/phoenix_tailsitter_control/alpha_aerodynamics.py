"""Complete Tailsitter-control alpha-theory force and moment equations."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class AlphaAerodynamicResult:
    force_alpha: np.ndarray
    moment_body_ts: np.ndarray
    flap_force_alpha: np.ndarray


class AlphaAerodynamics:
    """Evaluate Tailsitter-control equations 5--14 in TS body axes."""

    def __init__(self, config):
        self.cfg = config
        self.alpha_zero = float(config.alpha_zero_lift_rad)
        self.alpha_thrust = float(config.alpha_thrust_installation_rad)
        self.alpha_tilde = self.alpha_zero + self.alpha_thrust
        ca0 = math.cos(self.alpha_zero)
        sa0 = math.sin(self.alpha_zero)
        self.rotation_body_to_alpha = np.array([
            [ca0, 0.0, sa0],
            [0.0, 1.0, 0.0],
            [-sa0, 0.0, ca0],
        ])
        self.rotation_alpha_to_body = self.rotation_body_to_alpha.T

    @staticmethod
    def _vector(value, size, name):
        result = np.asarray(value, dtype=float)
        if result.shape != (size,) or not np.all(np.isfinite(result)):
            raise ValueError(f'{name} must contain {size} finite values')
        return result

    def compute(self, velocity_body_ts, motor_speed_left_right,
                flap_angle_left_right):
        velocity = self._vector(velocity_body_ts, 3, 'body velocity')
        motor_speed = self._vector(
            motor_speed_left_right, 2, 'motor speed')
        flap = self._vector(flap_angle_left_right, 2, 'flap angle')
        if np.any(motor_speed < 0.0):
            raise ValueError('motor speed cannot be negative')

        speed = max(float(np.linalg.norm(velocity)), 1e-3)
        velocity_alpha = self.rotation_body_to_alpha @ velocity
        vx_alpha, _, vz_alpha = velocity_alpha
        thrust = self.cfg.motor_thrust_coefficient * motor_speed ** 2

        ca_tilde = math.cos(self.alpha_tilde)
        sa_tilde = math.sin(self.alpha_tilde)
        thrust_direction_alpha = np.array([
            ca_tilde * (1.0 - self.cfg.alpha_c_dt),
            0.0,
            sa_tilde * (self.cfg.alpha_c_lt - 1.0),
        ])
        thrust_force_each = thrust[:, None] * thrust_direction_alpha
        thrust_force = np.sum(thrust_force_each, axis=0)

        flap_force_z = -(
            self.cfg.alpha_c_lt_delta * ca_tilde * thrust
            + self.cfg.alpha_c_lv_delta * speed * vx_alpha
        ) * flap
        flap_force = np.array([0.0, 0.0, float(np.sum(flap_force_z))])
        wing_force = -np.array([
            self.cfg.alpha_c_dv * vx_alpha,
            0.0,
            self.cfg.alpha_c_lv * vz_alpha,
        ]) * speed
        force_alpha = thrust_force + flap_force + wing_force

        difference_force_body = (
            self.rotation_alpha_to_body
            @ (thrust_force_each[1] - thrust_force_each[0])
        )
        thrust_moment = np.array([
            self.cfg.alpha_motor_arm_y * difference_force_body[2],
            self.cfg.alpha_c_mu_t * float(np.sum(thrust)),
            -self.cfg.alpha_motor_arm_y * difference_force_body[0],
        ])

        motor_reaction = self.cfg.alpha_motor_torque_coefficient * np.array([
            motor_speed[0] ** 2,
            -motor_speed[1] ** 2,
        ])
        reaction_sum = float(np.sum(motor_reaction))
        reaction_moment = np.array([
            math.cos(self.alpha_thrust) * reaction_sum,
            0.0,
            -math.sin(self.alpha_thrust) * reaction_sum,
        ])

        flap_difference = flap_force_z[1] - flap_force_z[0]
        flap_moment = np.array([
            self.cfg.alpha_flap_arm_y * math.cos(self.alpha_zero)
            * flap_difference,
            self.cfg.alpha_flap_arm_x * float(np.sum(flap_force_z)),
            self.cfg.alpha_flap_arm_y * math.sin(self.alpha_zero)
            * flap_difference,
        ])
        moment = thrust_moment + reaction_moment + flap_moment
        values = np.concatenate((force_alpha, moment, flap_force))
        if not np.all(np.isfinite(values)):
            raise ValueError('alpha-theory result is not finite')
        return AlphaAerodynamicResult(force_alpha, moment, flap_force)

    def force_body_ts(self, result):
        return self.rotation_alpha_to_body @ np.asarray(
            result.force_alpha, dtype=float)


def vector_gz_flu_to_ts(vector_gz):
    """Convert a body vector from Gazebo FLU to the fixed TS basis."""
    x_gz, y_gz, z_gz = np.asarray(vector_gz, dtype=float)
    return np.array([z_gz, -y_gz, x_gz])


def vector_ts_to_gz_flu(vector_ts):
    """Inverse of :func:`vector_gz_flu_to_ts`."""
    x_ts, y_ts, z_ts = np.asarray(vector_ts, dtype=float)
    return np.array([z_ts, -y_ts, x_ts])
