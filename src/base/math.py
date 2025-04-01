import math
import numpy as np


def quatern2euler(x, y, z, w):
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
    
    q1 = a(1)*b(1) - a(2)*b(2)
    #bruh discrepency brb


    return (q1, q2, q3, q4)




