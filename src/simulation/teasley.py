from dataclasses import dataclass

import numpy as np

@dataclass
class TeasleyState:
    time = 0
    x_earth = (0, 0, 0)
    vel_earth = (0, 0, 0)
    phi = 0
    theta = 0
    psi = 0
    p = 0
    q = 0
    r = 0
    flap_angle = 0


def Tb2eTe2b(phi, theta, psi):
    cphi = np.cos(phi)
    sphi = np.sin(phi)
    ctheta = np.cos(theta)
    stheta = np.sin(theta)
    cpsi = np.cos(psi)
    spsi = np.sin(psi)

    R_rot = [
        [ctheta * cpsi, ctheta * spsi, -stheta],
        [
            sphi * stheta * cpsi - cphi * spsi,
            sphi * stheta * spsi + cphi * cpsi, sphi * ctheta
        ],
        [
            cphi * stheta * cpsi + sphi * spsi,
            cphi * stheta * spsi - sphi * cpsi, cphi * ctheta
        ],
    ]

    R_init = [
        [0, 0, 1],
        [0, 1, 0],
        [-1, 0, 0]
    ]

    Te2b = R_rot @ R_init
    Tb2e = Te2b.T

    return (Tb2e, Te2b)


def calculate_derivatives_teasley(vehicle, motor, environment, time, x_earth,
                                  vel_earth, phi, theta, psi, p, q, r,
                                  flap_angle):
    Tb2e, Te2b = Tb2eTe2b(phi, theta, psi)
