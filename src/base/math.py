import math
import numpy as np


def quatern2euler(w, x, y, z):
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

    return (phi, theta, psi)
