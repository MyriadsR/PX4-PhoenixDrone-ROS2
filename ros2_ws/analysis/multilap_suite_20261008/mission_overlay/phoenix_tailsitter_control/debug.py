"""Stable field ordering for the low-speed controller debug topic."""

import numpy as np


CONTROL_DEBUG_FIELDS = (
    'attitude_error_x', 'attitude_error_y', 'attitude_error_z',
    'rate_lpf_x', 'rate_lpf_y', 'rate_lpf_z',
    'angular_acceleration_lpf_x', 'angular_acceleration_lpf_y',
    'angular_acceleration_lpf_z',
    'angular_acceleration_command_x', 'angular_acceleration_command_y',
    'angular_acceleration_command_z',
    'estimated_moment_lpf_x', 'estimated_moment_lpf_y', 'estimated_moment_lpf_z',
    'desired_moment_x', 'desired_moment_y', 'desired_moment_z',
    'allocated_moment_x', 'allocated_moment_y', 'allocated_moment_z',
    'motor_left_normalized', 'motor_right_normalized',
    'flap_left_rad', 'flap_right_rad',
    'total_thrust_n', 'output_active',
)

POSITION_DEBUG_FIELDS = (
    'position_n', 'position_e', 'position_d',
    'position_reference_n', 'position_reference_e', 'position_reference_d',
    'velocity_n', 'velocity_e', 'velocity_d',
    'velocity_reference_n', 'velocity_reference_e', 'velocity_reference_d',
    'acceleration_lpf_n', 'acceleration_lpf_e', 'acceleration_lpf_d',
    'acceleration_reference_n', 'acceleration_reference_e', 'acceleration_reference_d',
    'acceleration_command_n', 'acceleration_command_e', 'acceleration_command_d',
    'estimated_force_lpf_n', 'estimated_force_lpf_e', 'estimated_force_lpf_d',
    'force_command_n', 'force_command_e', 'force_command_d',
    'limited_force_n', 'limited_force_e', 'limited_force_d',
    'yaw_reference_rad', 'total_thrust_n', 'trajectory_active',
)

ACTUATOR_FEEDBACK_DEBUG_FIELDS = (
    'predicted_motor_left_rad_s', 'predicted_motor_right_rad_s',
    'feedback_motor_left_rad_s', 'feedback_motor_right_rad_s',
    'motor_error_left_rad_s', 'motor_error_right_rad_s',
    'predicted_flap_left_rad', 'predicted_flap_right_rad',
    'feedback_flap_left_rad', 'feedback_flap_right_rad',
    'flap_error_left_rad', 'flap_error_right_rad',
    'feedback_age_s', 'feedback_complete', 'feedback_active',
)

ALPHA_MODEL_DEBUG_FIELDS = (
    'velocity_body_ts_x', 'velocity_body_ts_y', 'velocity_body_ts_z',
    'force_alpha_x', 'force_alpha_y', 'force_alpha_z',
    'moment_body_ts_x', 'moment_body_ts_y', 'moment_body_ts_z',
    'flap_transient_force_alpha_x', 'flap_transient_force_alpha_y',
    'flap_transient_force_alpha_z',
    'motor_lpf_left_rad_s', 'motor_lpf_right_rad_s',
    'flap_lpf_left_rad', 'flap_lpf_right_rad',
    'flap_hpf_left_rad', 'flap_hpf_right_rad',
)


def pack_control_debug(attitude_error, rates_lpf, acceleration_lpf,
                       acceleration_command, estimated_moment_lpf,
                       desired_moment, allocated_moment, motors_normalized,
                       flaps_rad, total_thrust, output_active):
    """Pack finite controller state in ``CONTROL_DEBUG_FIELDS`` order."""
    groups = (
        attitude_error,
        rates_lpf,
        acceleration_lpf,
        acceleration_command,
        estimated_moment_lpf,
        desired_moment,
        allocated_moment,
        motors_normalized,
        flaps_rad,
        [total_thrust, float(bool(output_active))],
    )
    result = np.concatenate([np.asarray(group, dtype=float).reshape(-1) for group in groups])
    if result.size != len(CONTROL_DEBUG_FIELDS) or not np.all(np.isfinite(result)):
        raise ValueError('invalid controller debug vector')
    return result.tolist()


def pack_position_debug(position, position_reference, velocity, velocity_reference,
                        acceleration_lpf, acceleration_reference,
                        acceleration_command, estimated_force_lpf,
                        force_command, limited_force, yaw_reference,
                        total_thrust, trajectory_active):
    """Pack finite position-loop state in ``POSITION_DEBUG_FIELDS`` order."""
    groups = (
        position,
        position_reference,
        velocity,
        velocity_reference,
        acceleration_lpf,
        acceleration_reference,
        acceleration_command,
        estimated_force_lpf,
        force_command,
        limited_force,
        [yaw_reference, total_thrust, float(bool(trajectory_active))],
    )
    result = np.concatenate([np.asarray(group, dtype=float).reshape(-1) for group in groups])
    if result.size != len(POSITION_DEBUG_FIELDS) or not np.all(np.isfinite(result)):
        raise ValueError('invalid position debug vector')
    return result.tolist()


def pack_actuator_feedback_debug(predicted_motors, feedback_motors,
                                 predicted_flaps, feedback_flaps,
                                 feedback_age_s, feedback_complete,
                                 feedback_active):
    """Pack estimator-versus-measurement diagnostics in stable field order."""
    predicted_motors = np.asarray(predicted_motors, dtype=float)
    feedback_motors = np.asarray(feedback_motors, dtype=float)
    predicted_flaps = np.asarray(predicted_flaps, dtype=float)
    feedback_flaps = np.asarray(feedback_flaps, dtype=float)
    groups = (
        predicted_motors,
        feedback_motors,
        feedback_motors - predicted_motors,
        predicted_flaps,
        feedback_flaps,
        feedback_flaps - predicted_flaps,
        [feedback_age_s, float(bool(feedback_complete)),
         float(bool(feedback_active))],
    )
    result = np.concatenate([np.asarray(group, dtype=float).reshape(-1) for group in groups])
    if result.size != len(ACTUATOR_FEEDBACK_DEBUG_FIELDS) or not np.all(np.isfinite(result)):
        raise ValueError('invalid actuator-feedback debug vector')
    return result.tolist()


def pack_alpha_model_debug(velocity_body_ts, force_alpha, moment_body_ts,
                           flap_transient_force_alpha, motor_lpf, flap_lpf,
                           flap_hpf):
    """Pack alpha-theory states used for parameter identification."""
    groups = (
        velocity_body_ts,
        force_alpha,
        moment_body_ts,
        flap_transient_force_alpha,
        motor_lpf,
        flap_lpf,
        flap_hpf,
    )
    result = np.concatenate([
        np.asarray(group, dtype=float).reshape(-1) for group in groups
    ])
    if (result.size != len(ALPHA_MODEL_DEBUG_FIELDS)
            or not np.all(np.isfinite(result))):
        raise ValueError('invalid alpha-model debug vector')
    return result.tolist()
