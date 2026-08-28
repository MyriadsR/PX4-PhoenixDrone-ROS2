"""One authoritative conversion between Tailsitter-control and PX4 body axes.

The basis relation requested for this vehicle is::

    x_ts = -z_px4,  y_ts = y_px4,  z_ts = x_px4

Consequently, components convert as ``v_px4 = C_PX4_TS @ v_ts`` with
``v_px4 = [v_ts.z, v_ts.y, -v_ts.x]``.  Do not duplicate sign changes outside
this module.
"""

import numpy as np

from .math_utils import matrix_to_quaternion, quaternion_to_matrix


C_PX4_TS = np.array([
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
])
C_TS_PX4 = C_PX4_TS.T


def vector_ts_to_px4(vector):
    return C_PX4_TS @ np.asarray(vector, dtype=float)


def vector_px4_to_ts(vector):
    return C_TS_PX4 @ np.asarray(vector, dtype=float)


def inertia_px4_to_ts(inertia_px4):
    matrix = np.asarray(inertia_px4, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError('inertia must be 3x3')
    return C_TS_PX4 @ matrix @ C_PX4_TS


def attitude_px4_to_ts(q_px4):
    """Convert body-to-NED attitude after changing only the body basis."""
    r_px4_to_ned = quaternion_to_matrix(q_px4)
    return matrix_to_quaternion(r_px4_to_ned @ C_PX4_TS)


def attitude_ts_to_px4(q_ts):
    """Convert a Tailsitter body-to-NED attitude into PX4 body-to-NED."""
    r_ts_to_ned = quaternion_to_matrix(q_ts)
    return matrix_to_quaternion(r_ts_to_ned @ C_TS_PX4)
