import math

import numpy as np

from phoenix_tailsitter_control.attitude_step_test import (
    summarize_actuator_feedback_debug,
    summarize_alpha_model_debug,
    summarize_control_debug,
    summarize_land_detected,
)
from phoenix_tailsitter_control.debug import (
    ACTUATOR_FEEDBACK_DEBUG_FIELDS,
    ALPHA_MODEL_DEBUG_FIELDS,
    CONTROL_DEBUG_FIELDS,
)


def _debug_vector(**overrides):
    values = np.zeros(len(CONTROL_DEBUG_FIELDS), dtype=float)
    for name, value in overrides.items():
        values[CONTROL_DEBUG_FIELDS.index(name)] = value
    return values


def _actuator_vector(**overrides):
    values = np.zeros(len(ACTUATOR_FEEDBACK_DEBUG_FIELDS), dtype=float)
    for name, value in overrides.items():
        values[ACTUATOR_FEEDBACK_DEBUG_FIELDS.index(name)] = value
    return values


def _alpha_vector(**overrides):
    values = np.zeros(len(ALPHA_MODEL_DEBUG_FIELDS), dtype=float)
    for name, value in overrides.items():
        values[ALPHA_MODEL_DEBUG_FIELDS.index(name)] = value
    return values


def test_summarize_control_debug_reports_axis_and_actuator_medians():
    records = [
        ('plus', _debug_vector(
            attitude_error_y=0.02,
            desired_moment_y=0.004,
            allocated_moment_y=0.003,
            flap_left_rad=-0.01,
            flap_right_rad=-0.02,
            output_active=1.0,
        )),
        ('plus', _debug_vector(
            attitude_error_y=0.04,
            desired_moment_y=0.008,
            allocated_moment_y=0.006,
            flap_left_rad=-0.03,
            flap_right_rad=-0.04,
            output_active=1.0,
        )),
        ('minus', _debug_vector(
            attitude_error_y=-0.03,
            desired_moment_y=-0.006,
            allocated_moment_y=-0.005,
            flap_left_rad=0.02,
            flap_right_rad=0.01,
            output_active=1.0,
        )),
    ]

    summary = summarize_control_debug(records, axis_index=1)

    assert summary['control_debug_sample_count'] == 3
    assert math.isclose(summary['plus_debug_attitude_error_axis_rad'], 0.04)
    assert math.isclose(summary['plus_debug_desired_moment_axis_nm'], 0.008)
    assert math.isclose(summary['plus_debug_allocated_moment_axis_nm'], 0.006)
    assert math.isclose(summary['plus_debug_flap_left_rad'], -0.03)
    assert math.isclose(summary['minus_debug_desired_moment_axis_nm'], -0.006)
    assert math.isclose(summary['minus_debug_flap_right_rad'], 0.01)


def test_summarize_actuator_feedback_debug_reports_feedback_flaps():
    summary = summarize_actuator_feedback_debug([
        ('minus', _actuator_vector(
            predicted_flap_left_rad=0.04,
            feedback_flap_left_rad=0.03,
            flap_error_left_rad=-0.01,
            feedback_complete=1.0,
            feedback_active=1.0,
        )),
    ])

    assert summary['actuator_feedback_debug_sample_count'] == 1
    assert math.isclose(
        summary['minus_actuator_debug_predicted_flap_left_rad'], 0.04)
    assert math.isclose(
        summary['minus_actuator_debug_feedback_flap_left_rad'], 0.03)
    assert math.isclose(summary['minus_actuator_debug_flap_error_left_rad'], -0.01)
    assert math.isclose(summary['minus_actuator_debug_feedback_complete'], 1.0)


def test_summarize_alpha_model_debug_reports_axis_moment():
    summary = summarize_alpha_model_debug([
        ('plus', _alpha_vector(
            velocity_body_ts_y=0.2,
            moment_body_ts_y=-0.005,
            flap_lpf_left_rad=0.02,
            flap_lpf_right_rad=0.03,
        )),
    ], axis_index=1)

    assert summary['alpha_model_debug_sample_count'] == 1
    assert math.isclose(summary['plus_alpha_debug_velocity_body_axis_m_s'], 0.2)
    assert math.isclose(summary['plus_alpha_debug_moment_body_axis_nm'], -0.005)
    assert math.isclose(summary['plus_alpha_debug_flap_lpf_right_rad'], 0.03)


def test_summarize_land_detected_reports_phase_contact_state():
    summary = summarize_land_detected([
        ('plus', (False, False)),
        ('minus', (True, True)),
    ])

    assert summary['land_detected_sample_count'] == 2
    assert math.isclose(summary['plus_landed_fraction'], 0.0)
    assert math.isclose(summary['plus_ground_contact_fraction'], 0.0)
    assert math.isclose(summary['minus_landed_fraction'], 1.0)
    assert math.isclose(summary['minus_ground_contact_fraction'], 1.0)
