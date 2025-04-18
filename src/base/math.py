import math
import numpy as np


def quatern2euler(q):
    # Separate Components
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]

    # Precompute elements of the rotation matrix
    R11 = 2 * w**2 - 1 + 2 * x**2
    R21 = 2 * (x * y - w * z)
    R31 = 2 * (x * z + w * y)
    R32 = 2 * (y * z - w * x)
    R33 = 2 * w**2 - 1 + 2 * z**2

    # Compute Euler angles
    phi = np.arctan2(R32, R33)
    theta = -np.arctan(R31 / np.sqrt(1 - R31**2))
    psi = np.arctan2(R21, R11)

    return (psi, theta, phi)


def zenith_azimuth_to_quaternion(
        zenith: float, azimuth: float) -> tuple[float, float, float, float]:
    # input in degrees

    zenith = np.radians(zenith)
    azimuth = np.radians(azimuth)

    w = np.cos(zenith / 2)
    x = np.sin(zenith / 2) * np.sin(azimuth)
    y = np.sin(zenith / 2) * np.cos(azimuth)
    z = 0

    return (w, x, y, z)


def quatern_prod(a, b) -> tuple[float, float, float, float]:

    # Calculates the quaternion product of quaternion a and b. Not communative.
    q1 = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3]
    q2 = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2]
    q3 = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1]
    q4 = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]

    return (q1, q2, q3, q4)


def quatern_conj(q) -> tuple[float, float, float, float]:

    # Converts a quaternion to its conjugate
    return (-q[0], -q[1], -q[2], q[3])


def euler_to_zyx_rotmat(e):

    alpha = math.radians(e[0])
    beta = math.radians(e[1])
    gamma = math.radians(e[2])

    R11 = math.cos(alpha) * math.cos(beta)
    R12 = math.cos(alpha) * math.sin(beta) * math.sin(gamma) - math.cos(
        gamma) * math.sin(alpha)
    R13 = math.sin(alpha) * math.sin(gamma) + math.cos(alpha) * math.cos(
        gamma) * math.sin(beta)
    R21 = math.cos(beta) * math.sin(alpha)
    R22 = math.cos(alpha) * math.cos(gamma) + math.sin(alpha) * math.sin(
        beta) * math.sin(gamma)
    R23 = math.cos(gamma) * math.sin(alpha) * math.sin(beta) - math.cos(
        alpha) * math.sin(gamma)
    R31 = -math.sin(beta)
    R32 = math.cos(beta) * math.sin(gamma)
    R33 = math.cos(beta) * math.cos(gamma)

    return np.array([[R11, R12, R13], [R21, R22, R23], [R31, R32, R33]])
