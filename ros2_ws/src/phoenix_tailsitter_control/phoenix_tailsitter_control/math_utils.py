"""Small quaternion and matrix helpers using Hamilton [w, x, y, z]."""

import math

import numpy as np


def normalize_quaternion(q):
    values = np.asarray(q, dtype=float)
    if values.shape != (4,) or not np.all(np.isfinite(values)):
        raise ValueError('quaternion must contain four finite values')
    magnitude = np.linalg.norm(values)
    if magnitude < 1e-9:
        raise ValueError('quaternion magnitude is zero')
    return values / magnitude


def quaternion_inverse(q):
    q = normalize_quaternion(q)
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = np.asarray(q1, dtype=float)
    w2, x2, y2, z2 = np.asarray(q2, dtype=float)
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quaternion_to_matrix(q):
    w, x, y, z = normalize_quaternion(q)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ])


def matrix_to_quaternion(matrix):
    m = np.asarray(matrix, dtype=float)
    if m.shape != (3, 3) or not np.all(np.isfinite(m)):
        raise ValueError('rotation matrix must be finite and 3x3')
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = np.array([
            0.25 * s,
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
        ])
    else:
        axis = int(np.argmax(np.diag(m)))
        if axis == 0:
            s = math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) * 2.0
            q = np.array([(m[2, 1] - m[1, 2]) / s, 0.25 * s,
                          (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s])
        elif axis == 1:
            s = math.sqrt(max(0.0, 1.0 + m[1, 1] - m[0, 0] - m[2, 2])) * 2.0
            q = np.array([(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s,
                          0.25 * s, (m[1, 2] + m[2, 1]) / s])
        else:
            s = math.sqrt(max(0.0, 1.0 + m[2, 2] - m[0, 0] - m[1, 1])) * 2.0
            q = np.array([(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s,
                          (m[1, 2] + m[2, 1]) / s, 0.25 * s])
    q = normalize_quaternion(q)
    return q if q[0] >= 0.0 else -q


def rotation_vector_error(q_current, q_desired):
    """Return the shortest body-frame rotation from current to desired."""
    error = quaternion_multiply(quaternion_inverse(q_current), normalize_quaternion(q_desired))
    error = normalize_quaternion(error)
    if error[0] < 0.0:
        error = -error
    vector_norm = np.linalg.norm(error[1:])
    if vector_norm < 1e-8:
        return 2.0 * error[1:]
    angle = 2.0 * math.atan2(vector_norm, min(1.0, max(-1.0, error[0])))
    return angle * error[1:] / vector_norm
