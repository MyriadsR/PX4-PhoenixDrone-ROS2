"""Hover allocator fitted to the current simplified PhoenixAero plugin.

Internal ordering is always ``[left, right]`` in Tailsitter-control axes.
After Gazebo FLU -> PX4 FRD conversion, direct motor channels are
``[left, right]`` even though the SDF joint names suggest the opposite.
Direct servo channels are ``[right, left]``.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class AllocationResult:
    motor_speed_left_right: np.ndarray
    flap_angle_left_right: np.ndarray
    achieved_moment_ts: np.ndarray


class PhoenixHoverAllocator:
    def __init__(self, config):
        self.cfg = config

    @staticmethod
    def _bounded_two_by_two(matrix, target, lower, upper):
        candidates = []
        try:
            candidate = np.linalg.solve(matrix, target)
        except np.linalg.LinAlgError:
            candidate = np.linalg.pinv(matrix) @ target
        if np.all(candidate >= lower) and np.all(candidate <= upper):
            candidates.append(candidate)

        # A convex two-variable least-squares optimum is either interior or
        # on one of the four box edges.
        for fixed_index in (0, 1):
            free_index = 1 - fixed_index
            free_column = matrix[:, free_index]
            denominator = float(free_column @ free_column)
            for fixed_value in (lower[fixed_index], upper[fixed_index]):
                edge = np.zeros(2)
                edge[fixed_index] = fixed_value
                residual = target - matrix[:, fixed_index] * fixed_value
                free_value = (
                    0.0
                    if denominator < 1e-12
                    else float(free_column @ residual) / denominator
                )
                edge[free_index] = np.clip(free_value, lower[free_index], upper[free_index])
                candidates.append(edge)
        return min(candidates, key=lambda value: np.linalg.norm(matrix @ value - target))

    def _flap_matrix(self, motor_speed_left_right):
        omega_left, omega_right = np.asarray(motor_speed_left_right, dtype=float)
        lift_left = self.cfg.flap_lift_coefficient * omega_left ** 2
        lift_right = self.cfg.flap_lift_coefficient * omega_right ** 2
        pitch_arm = (
            -self.cfg.flap_cp_x_ts * self.cfg.flap_lift_coefficient
            - self.cfg.flap_pitch_coefficient
        )
        return np.array([
            [-self.cfg.motor_arm_y * lift_left, self.cfg.motor_arm_y * lift_right],
            [pitch_arm * omega_left ** 2, pitch_arm * omega_right ** 2],
        ])

    def estimate_moment(self, motor_speed_left_right, flap_angle_left_right):
        omega = np.asarray(motor_speed_left_right, dtype=float)
        thrust = self.cfg.motor_thrust_coefficient * omega ** 2
        thrust_difference = thrust[0] - thrust[1]
        motor_moment = np.array([
            -self.cfg.motor_moment_ratio * thrust_difference,
            0.0,
            self.cfg.motor_arm_y * thrust_difference,
        ])
        flap_xy = self._flap_matrix(omega) @ np.asarray(flap_angle_left_right, dtype=float)
        return motor_moment + np.array([flap_xy[0], flap_xy[1], 0.0])

    def allocate(self, total_thrust, desired_moment_ts):
        cfg = self.cfg
        total_thrust = float(np.clip(total_thrust, 0.0, cfg.maximum_total_thrust))
        desired = np.asarray(desired_moment_ts, dtype=float)

        thrust_max = cfg.motor_thrust_coefficient * cfg.motor_speed_max ** 2
        requested_difference = desired[2] / cfg.motor_arm_y
        difference_lower = max(-total_thrust, total_thrust - 2.0 * thrust_max)
        difference_upper = min(total_thrust, 2.0 * thrust_max - total_thrust)
        thrust_difference = float(np.clip(
            requested_difference, difference_lower, difference_upper))
        thrust_left = 0.5 * (total_thrust + thrust_difference)
        thrust_right = 0.5 * (total_thrust - thrust_difference)
        omega = np.sqrt(np.maximum(0.0, np.array([thrust_left, thrust_right]))
                        / cfg.motor_thrust_coefficient)

        motor_roll = -cfg.motor_moment_ratio * thrust_difference
        flap_target = np.array([desired[0] - motor_roll, desired[1]])
        flap_matrix = self._flap_matrix(omega)
        flap_lower = -np.array([cfg.left_flap_limit, cfg.right_flap_limit])
        flap_upper = np.array([cfg.left_flap_limit, cfg.right_flap_limit])
        flap = self._bounded_two_by_two(flap_matrix, flap_target, flap_lower, flap_upper)
        achieved = self.estimate_moment(omega, flap)
        return AllocationResult(omega, flap, achieved)

    def to_px4_controls(self, allocation):
        omega_left, omega_right = allocation.motor_speed_left_right
        delta_left, delta_right = allocation.flap_angle_left_right
        # motorNumber 0 is at y_ts<0 (left); motorNumber 1 is at y_ts>0
        # (right).  The rotor_right/rotor_left SDF joint names are misleading
        # because their Gazebo FLU y component flips on entry to PX4 FRD.
        motors_px4 = np.array([omega_left, omega_right]) / self.cfg.motor_speed_max
        # SIM_GZ_SV_MINA is greater than MAXA: normalized command is opposite
        # to the physical angle consumed by PhoenixAero.
        servos_px4 = np.array([
            -delta_right / self.cfg.right_flap_limit,
            -delta_left / self.cfg.left_flap_limit,
        ])
        return np.clip(motors_px4, 0.0, 1.0), np.clip(servos_px4, -1.0, 1.0)
