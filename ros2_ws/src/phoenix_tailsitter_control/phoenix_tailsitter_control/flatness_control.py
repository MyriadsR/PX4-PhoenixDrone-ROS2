"""Tailsitter-control differential-flatness equations 17--35."""

import math

import numpy as np

from .math_utils import normalize_quaternion, quaternion_to_matrix


def zxy_euler_to_quaternion(yaw, roll, pitch):
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    return normalize_quaternion(np.array([
        cy * cr * cp - sy * sr * sp,
        cy * sr * cp - sy * cr * sp,
        cy * cr * sp + sy * sr * cp,
        sy * cr * cp + cy * sr * sp,
    ]))


class FlatnessAttitudeController:
    def __init__(self, config):
        self.cfg = config
        self.alpha_zero = config.alpha_zero_lift_rad
        self.alpha_tilde = (
            config.alpha_zero_lift_rad
            + config.alpha_thrust_installation_rad
        )

    def attitude_and_thrust(self, force_ned, velocity_ned, flap_sum,
                            yaw, q_current_ts, return_angles=False):
        force = np.asarray(force_ned, dtype=float)
        velocity = np.asarray(velocity_ned, dtype=float)
        if (force.shape != (3,) or velocity.shape != (3,)
                or not np.all(np.isfinite(np.concatenate((force, velocity))))):
            raise ValueError('flatness inputs must be finite 3-vectors')
        speed = float(np.linalg.norm(velocity))
        cy, sy = math.cos(yaw), math.sin(yaw)
        inertial_to_yaw = np.array([
            [cy, sy, 0.0], [-sy, cy, 0.0], [0.0, 0.0, 1.0],
        ])
        force_yaw = inertial_to_yaw @ force
        body_y_current = quaternion_to_matrix(q_current_ts)[:, 1]
        if abs(force_yaw[1]) < 1e-4 and abs(force[2]) < 1e-4:
            body_y_yaw = inertial_to_yaw @ body_y_current
            roll = math.atan2(body_y_yaw[2], body_y_yaw[1])
        else:
            roll_candidate = -math.atan2(force_yaw[1], force[2])
            cr0, sr0 = math.cos(roll_candidate), math.sin(roll_candidate)
            yaw_to_roll = np.array([
                [1.0, 0.0, 0.0],
                [0.0, cr0, sr0],
                [0.0, -sr0, cr0],
            ])
            new_y = (yaw_to_roll @ inertial_to_yaw).T[:, 1]
            roll = (roll_candidate if np.dot(body_y_current, new_y) > 0.0
                    else roll_candidate + math.pi)

        cr, sr = math.cos(roll), math.sin(roll)
        yaw_to_roll = np.array([
            [1.0, 0.0, 0.0],
            [0.0, cr, sr],
            [0.0, -sr, cr],
        ])
        inertial_to_roll = yaw_to_roll @ inertial_to_yaw
        force_roll = inertial_to_roll @ force
        velocity_roll = inertial_to_roll @ velocity
        fx, fz = force_roll[0], force_roll[2]
        vx, vz = velocity_roll[0], velocity_roll[2]
        ca = math.cos(self.alpha_tilde)
        sa = math.sin(self.alpha_tilde)
        denominator = ca * (1.0 - self.cfg.alpha_c_dt)
        if abs(denominator) < 1e-6:
            raise ValueError('flatness thrust denominator is singular')
        eta = (
            sa * (self.cfg.alpha_c_lt - 1.0)
            - ca * self.cfg.alpha_c_lt_delta * flap_sum / 2.0
        ) / denominator
        y_value = (
            eta * (fx + self.cfg.alpha_c_dv * speed * vx)
            - self.cfg.alpha_c_lv_delta * flap_sum * speed * vx
            - self.cfg.alpha_c_lv * speed * vz - fz
        )
        x_value = (
            eta * (fz + self.cfg.alpha_c_dv * speed * vz)
            - self.cfg.alpha_c_lv_delta * flap_sum * speed * vz
            + self.cfg.alpha_c_lv * speed * vx + fx
        )
        if abs(x_value) < 1e-4 and abs(y_value) < 1e-4:
            rotation = quaternion_to_matrix(q_current_ts)
            alpha_x = (math.cos(self.alpha_zero) * rotation[:, 0]
                       + math.sin(self.alpha_zero) * rotation[:, 2])
            alpha_x_roll = inertial_to_roll @ alpha_x
            pitch_bar = math.atan2(-alpha_x_roll[2], alpha_x_roll[0])
        else:
            pitch_bar = math.atan2(y_value, x_value)
        cp, sp = math.cos(pitch_bar), math.sin(pitch_bar)
        thrust = (
            cp * fx - sp * fz
            + self.cfg.alpha_c_dv * speed * (cp * vx - sp * vz)
        ) / denominator
        if thrust < 0.0:
            pitch_bar += math.pi
            cp, sp = math.cos(pitch_bar), math.sin(pitch_bar)
            thrust = (
                cp * fx - sp * fz
                + self.cfg.alpha_c_dv * speed * (cp * vx - sp * vz)
            ) / denominator
        q_desired = zxy_euler_to_quaternion(
            yaw, roll, pitch_bar + self.alpha_zero)
        if return_angles:
            return q_desired, thrust, roll, pitch_bar
        return q_desired, thrust

    def feedforward_rates(self, velocity_reference, acceleration_reference,
                          jerk_reference, yaw, yawspeed, reference_force,
                          roll, pitch_bar, flap_sum):
        velocity = np.asarray(velocity_reference, dtype=float)
        acceleration = np.asarray(acceleration_reference, dtype=float)
        jerk = np.asarray(jerk_reference, dtype=float)
        force = np.asarray(reference_force, dtype=float)
        speed = max(float(np.linalg.norm(velocity)), 1e-3)
        speed_dot = float(velocity @ acceleration) / speed
        force_dot = self.cfg.mass * jerk
        cy, sy = math.cos(yaw), math.sin(yaw)
        cr, sr = math.cos(roll), math.sin(roll)
        inertial_to_yaw = np.array([
            [cy, sy, 0.0], [-sy, cy, 0.0], [0.0, 0.0, 1.0],
        ])
        yaw_to_roll = np.array([
            [1.0, 0.0, 0.0], [0.0, cr, sr], [0.0, -sr, cr],
        ])
        inertial_to_roll = yaw_to_roll @ inertial_to_yaw
        beta_x = -sy * force[0] + cy * force[1]
        beta_z = force[2]
        beta_x_dot = (-yawspeed * (cy * force[0] + sy * force[1])
                      - sy * force_dot[0] + cy * force_dot[1])
        beta_z_dot = force_dot[2]
        roll_denominator = beta_x**2 + beta_z**2
        roll_rate = (0.0 if roll_denominator < 1e-6 else
                     -(beta_x_dot * beta_z - beta_x * beta_z_dot)
                     / roll_denominator)
        frame_rate = (inertial_to_roll @ np.array([0.0, 0.0, yawspeed])
                      + np.array([roll_rate, 0.0, 0.0]))
        velocity_roll = inertial_to_roll @ velocity
        force_roll = inertial_to_roll @ force
        velocity_dot_roll = (inertial_to_roll @ acceleration
                             - np.cross(frame_rate, velocity_roll))
        force_dot_roll = (inertial_to_roll @ force_dot
                          - np.cross(frame_rate, force_roll))
        fx, fz = force_roll[0], force_roll[2]
        vx, vz = velocity_roll[0], velocity_roll[2]
        fx_dot, fz_dot = force_dot_roll[0], force_dot_roll[2]
        vx_dot, vz_dot = velocity_dot_roll[0], velocity_dot_roll[2]
        tau_x = speed_dot * vx + speed * vx_dot
        tau_z = speed_dot * vz + speed * vz_dot
        ca = math.cos(self.alpha_tilde)
        sa = math.sin(self.alpha_tilde)
        denominator = ca * (1.0 - self.cfg.alpha_c_dt)
        eta = (
            sa * (self.cfg.alpha_c_lt - 1.0)
            - ca * self.cfg.alpha_c_lt_delta * flap_sum / 2.0
        ) / denominator
        y_value = (eta * (fx + self.cfg.alpha_c_dv * speed * vx)
                   - self.cfg.alpha_c_lv_delta * flap_sum * speed * vx
                   - self.cfg.alpha_c_lv * speed * vz - fz)
        x_value = (eta * (fz + self.cfg.alpha_c_dv * speed * vz)
                   - self.cfg.alpha_c_lv_delta * flap_sum * speed * vz
                   + self.cfg.alpha_c_lv * speed * vx + fx)
        y_dot = (eta * (fx_dot + self.cfg.alpha_c_dv * tau_x)
                 - self.cfg.alpha_c_lv_delta * flap_sum * tau_x
                 - self.cfg.alpha_c_lv * tau_z - fz_dot)
        x_dot = (eta * (fz_dot + self.cfg.alpha_c_dv * tau_z)
                 - self.cfg.alpha_c_lv_delta * flap_sum * tau_z
                 + self.cfg.alpha_c_lv * tau_x + fx_dot)
        pitch_denominator = x_value**2 + y_value**2
        pitch_rate = (0.0 if pitch_denominator < 1e-6 else
                      (x_value * y_dot - y_value * x_dot)
                      / pitch_denominator)
        pitch = pitch_bar + self.alpha_zero
        cp, sp = math.cos(pitch), math.sin(pitch)
        roll_to_body = np.array([
            [cp, 0.0, -sp], [0.0, 1.0, 0.0], [sp, 0.0, cp],
        ])
        inertial_to_body = roll_to_body @ inertial_to_roll
        return (inertial_to_body @ np.array([0.0, 0.0, yawspeed])
                + roll_to_body @ np.array([roll_rate, 0.0, 0.0])
                + np.array([0.0, pitch_rate, 0.0]))
