import math

import numpy as np

from phoenix_tailsitter_control.frames import (
    C_PX4_TS,
    attitude_px4_to_ts,
    attitude_ts_to_px4,
    inertia_px4_to_ts,
    vector_px4_to_ts,
    vector_ts_to_px4,
)
from phoenix_tailsitter_control.math_utils import quaternion_to_matrix, rotation_vector_error


def test_requested_basis_mapping():
    # x_ts=-z_px4, y_ts=y_px4, z_ts=x_px4.
    np.testing.assert_allclose(vector_ts_to_px4([1.0, 0.0, 0.0]), [0.0, 0.0, -1.0])
    np.testing.assert_allclose(vector_ts_to_px4([0.0, 1.0, 0.0]), [0.0, 1.0, 0.0])
    np.testing.assert_allclose(vector_ts_to_px4([0.0, 0.0, 1.0]), [1.0, 0.0, 0.0])
    np.testing.assert_allclose(vector_px4_to_ts([1.0, 2.0, 3.0]), [-3.0, 2.0, 1.0])


def test_mapping_is_a_proper_rotation_and_round_trips_vectors():
    np.testing.assert_allclose(C_PX4_TS.T @ C_PX4_TS, np.eye(3), atol=1e-12)
    assert np.linalg.det(C_PX4_TS) == 1.0
    vector = np.array([1.2, -3.4, 5.6])
    np.testing.assert_allclose(vector_px4_to_ts(vector_ts_to_px4(vector)), vector)


def test_attitude_conversion_changes_body_basis_only():
    q_px4 = np.array([1.0, 0.0, 0.0, 0.0])
    q_ts = attitude_px4_to_ts(q_px4)
    np.testing.assert_allclose(quaternion_to_matrix(q_ts), C_PX4_TS, atol=1e-12)
    q_round_trip = attitude_ts_to_px4(q_ts)
    np.testing.assert_allclose(q_round_trip, q_px4, atol=1e-12)


def test_px4_x_rotation_becomes_tailsitter_z_rotation_error():
    angle = 0.2
    q_current_ts = attitude_px4_to_ts([1.0, 0.0, 0.0, 0.0])
    q_desired_px4 = [math.cos(angle / 2.0), math.sin(angle / 2.0), 0.0, 0.0]
    q_desired_ts = attitude_px4_to_ts(q_desired_px4)
    np.testing.assert_allclose(
        rotation_vector_error(q_current_ts, q_desired_ts),
        [0.0, 0.0, angle],
        atol=1e-10,
    )


def test_inertia_is_rotated_not_relabelled_ad_hoc():
    inertia_px4 = np.diag([1.0, 2.0, 3.0])
    np.testing.assert_allclose(inertia_px4_to_ts(inertia_px4), np.diag([3.0, 2.0, 1.0]))
