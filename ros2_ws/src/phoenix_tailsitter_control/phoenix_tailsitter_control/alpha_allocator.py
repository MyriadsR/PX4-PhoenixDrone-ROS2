"""Velocity-dependent Tailsitter-control allocation, equations 37--40."""

import math

import numpy as np

from .allocator import AllocationResult, PhoenixHoverAllocator
from .alpha_aerodynamics import AlphaAerodynamics


class AlphaTheoryAllocator:
    def __init__(self, config):
        self.cfg = config
        self.aerodynamics = AlphaAerodynamics(config)
        self.alpha_tilde = (
            config.alpha_zero_lift_rad
            + config.alpha_thrust_installation_rad
        )

    def allocate(self, total_thrust, desired_moment_ts, velocity_body_ts):
        cfg = self.cfg
        desired = np.asarray(desired_moment_ts, dtype=float)
        velocity = np.asarray(velocity_body_ts, dtype=float)
        if (desired.shape != (3,) or velocity.shape != (3,)
                or not np.all(np.isfinite(np.concatenate((desired, velocity))))):
            raise ValueError('alpha allocation inputs must be finite 3-vectors')

        ca0 = math.cos(cfg.alpha_zero_lift_rad)
        sa0 = math.sin(cfg.alpha_zero_lift_rad)
        cat = math.cos(self.alpha_tilde)
        sat = math.sin(self.alpha_tilde)
        ca_t = math.cos(cfg.alpha_thrust_installation_rad)
        sa_t = math.sin(cfg.alpha_thrust_installation_rad)

        differential_denominator = (
            cfg.alpha_motor_arm_y
            * (ca0 * cat * (1.0 - cfg.alpha_c_dt)
               - sa0 * sat * (cfg.alpha_c_lt - 1.0))
            - sa_t * cfg.alpha_motor_torque_coefficient
            / cfg.motor_thrust_coefficient
        )
        differential_thrust = (
            0.0 if abs(differential_denominator) < 1e-8
            else desired[2] / differential_denominator
        )

        thrust_max = cfg.motor_thrust_coefficient * cfg.motor_speed_max ** 2
        total = float(np.clip(total_thrust, 0.0, 2.0 * thrust_max))
        differential_lower = max(-total, total - 2.0 * thrust_max)
        differential_upper = min(total, 2.0 * thrust_max - total)
        differential_thrust = float(np.clip(
            differential_thrust, differential_lower, differential_upper))
        thrust = np.array([
            0.5 * (total + differential_thrust),
            0.5 * (total - differential_thrust),
        ])
        motor_speed = np.sqrt(
            np.maximum(thrust, 0.0) / cfg.motor_thrust_coefficient)

        force_direction_alpha = np.array([
            cat * (1.0 - cfg.alpha_c_dt),
            0.0,
            sat * (cfg.alpha_c_lt - 1.0),
        ])
        force_direction_body = (
            self.aerodynamics.rotation_alpha_to_body
            @ force_direction_alpha
        )
        thrust_moment = np.array([
            cfg.alpha_motor_arm_y * force_direction_body[2]
            * (thrust[1] - thrust[0]),
            cfg.alpha_c_mu_t * total,
            cfg.alpha_motor_arm_y * force_direction_body[0]
            * (thrust[0] - thrust[1]),
        ])
        reaction_sum = (
            cfg.alpha_motor_torque_coefficient
            / cfg.motor_thrust_coefficient * differential_thrust
        )
        reaction_moment = np.array([
            ca_t * reaction_sum,
            0.0,
            -sa_t * reaction_sum,
        ])
        flap_target = desired[:2] - (thrust_moment + reaction_moment)[:2]

        speed = max(float(np.linalg.norm(velocity)), 1e-3)
        velocity_alpha_x = (
            ca0 * velocity[0] + sa0 * velocity[2]
        )
        effectiveness = -(
            cfg.alpha_c_lt_delta * cat * thrust
            + cfg.alpha_c_lv_delta * speed * velocity_alpha_x
        )
        flap_matrix = np.array([
            [-cfg.alpha_flap_arm_y * ca0 * effectiveness[0],
             cfg.alpha_flap_arm_y * ca0 * effectiveness[1]],
            [cfg.alpha_flap_arm_x * effectiveness[0],
             cfg.alpha_flap_arm_x * effectiveness[1]],
        ])
        lower = -np.array([cfg.left_flap_limit, cfg.right_flap_limit])
        upper = np.array([cfg.left_flap_limit, cfg.right_flap_limit])
        flap = PhoenixHoverAllocator._bounded_two_by_two(
            flap_matrix, flap_target, lower, upper)
        achieved = self.aerodynamics.compute(
            velocity, motor_speed, flap).moment_body_ts
        return AllocationResult(motor_speed, flap, achieved)

    def to_px4_controls(self, allocation):
        omega_left, omega_right = allocation.motor_speed_left_right
        delta_left, delta_right = allocation.flap_angle_left_right
        motors = np.array([omega_left, omega_right]) / self.cfg.motor_speed_max
        servos = np.array([
            -delta_right / self.cfg.right_flap_limit,
            -delta_left / self.cfg.left_flap_limit,
        ])
        return np.clip(motors, 0.0, 1.0), np.clip(servos, -1.0, 1.0)
