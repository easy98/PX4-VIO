import math
import numpy as np
from scipy.linalg import expm



def quat_magnitude(q):
    """
    Calculate the magnitude (or norm) of the quaternion.
    """
    return np.linalg.norm(q)

def quat_normalize(q):
    """
    Normalize the quaternion to have a magnitude of 1. This is often useful for rotations.
    """
    mag = quat_magnitude(q)
    return np.array(q / mag)

def quant_inverse(q):
    """
    Calculate the inverse of the quaternion. The inverse is given by conjugate(q) / magnitude(q)^2.
    """
    mag_squared = quat_magnitude(q) ** 2
    # Conjugate of a quaternion flips the signs of the imaginary components.
    return np.array([-q[0] / mag_squared, -q[1] / mag_squared, -q[2] / mag_squared, q[3] / mag_squared])

def quattorot(q):
    """
    Convert this quaternion to a 3x3 rotation matrix.
    """
    # Pre-compute squared components for efficiency
    q1q1 = q[0] * q[0]
    q2q2 = q[1] * q[1]
    q3q3 = q[2] * q[2]
    q1q2 = q[0] * q[1]
    q1q3 = q[0] * q[2]
    q1q4 = q[0] * q[3]
    q2q3 = q[1] * q[2]
    q2q4 = q[1] * q[3]
    q3q4 = q[2] * q[3]

    # Construct the rotation matrix
    R = [
        [1 - 2 * (q2q2 + q3q3), 2 * (q1q2 - q3q4), 2 * (q1q3 + q2q4)],
        [2 * (q1q2 + q3q4), 1 - 2 * (q1q1 + q3q3), 2 * (q2q3 - q1q4)],
        [2 * (q1q3 - q2q4), 2 * (q2q3 + q1q4), 1 - 2 * (q1q1 + q2q2)]
    ]

    return np.array(R)

def skew(v):
    """
    Returns the skew-symmetric matrix of a vector.

    :param v: The input vector as a numpy array.
    :return: A skew-symmetric matrix as a numpy array.
    """
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

def quat_integral(omega_m, dt):
    """
    Integrates a quaternion using the matrix exponential method.

    :param omega_m: The angular velocity as a 3-element numpy array.
    :param dt: The time step in seconds.
    :return: The integrated quaternion as a 4x4 matrix.
    """
    # Create the skew-symmetric matrix for omega_m
    M_skew = skew(omega_m)
    Omega_mat_top = np.column_stack((-M_skew, omega_m))
    Omega_mat_bot = np.hstack((-omega_m, 0))
    Omega_mat = np.vstack((Omega_mat_top, Omega_mat_bot))

    # Compute the matrix exponential
    dy = expm(0.5 * Omega_mat * dt)

    return dy


