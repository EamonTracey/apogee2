from dataclasses import dataclass

import numpy as np

from base.constants import EARTH_GRAVITY_ACCELERATION
from simulation.constants import CFD_AIR_DENSITY


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

    R_init = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]

    Te2b = R_rot @ R_init
    Tb2e = Te2b.T

    return (Tb2e, Te2b)


def calculate_derivatives_teasley(vehicle, motor, environment, time, x_earth,
                                  vel_earth, phi, theta, psi, p, q, r,
                                  flap_angle):
    Tb2e, Te2b = Tb2eTe2b(phi, theta, psi)

    # Vehicle.
    Ix = vehicle.intertia[2]
    Iy = vehicle.inertia[1]
    Iz = vehicle.inertia[0]
    cm = vehicle.center_of_mass
    cp = vehicle.center_of_pressure
    r_body = np.array([[cm - cp], [0], [0]])
    bodyAxisVec_body = np.array([[1], [0], [0]])
    bodyAxisVec_earth = Tb2e @ bodyAxisVec_body

    # Motor.
    thrust = motor.calculate_thrust(time)
    m = vehicle.mass + motor.calculate_mass(time)

    # Environment.
    # TODO: Incorporate wind.
    # TODO: Check density calculation.
    altitude = x_earth[2]
    wind_earth = np.array([[0], [0], [0]])
    temperature = environment.ground_temperature - 0.00356 * x_earth[2]
    pressure = environment.ground_pressure * (temperature / 518.6)**5.256
    density = pressure / 1718 / temperature
    density_SL = CFD_AIR_DENSITY
    a = np.sqrt(1.4 * pressure / density)
    velocityVec_earth = vel_earth + wind_earth
    velocity_mag = np.linalg.norm(velocityVec_earth)
    mach_number = velocity_mag / a

    if np.linalg.norm(vel_earth) < 1e-6 or np.linalg.norm(wind_earth) < 1e-6:
        aoa = 0
        sign_alpha = 1
        wind_dir_earth = 0
        perp_wind_dir_earth = 0
        F_normal_dir = np.array([[0], [0], [0]])
    else:
        wind_dir_earth = wind_earth / np.linalg.norm(wind_earth)
        perp_wind_dir_earth = np.array([[-wind_dir_earth[1]],
                                        [wind_dir_earth[0]], [0]])

        v_hat = velocityVec_earth / np.linalg.norm(velocityVec_earth)
        b_hat = bodyAxisVec_earth / np.linalg.norm(bodyAxisVec_earth)

        dot_val = np.sum(v_hat * b_hat)
        dot_val = max(min(dot_val, 1), -1)
        angle_mag = np.degrees(np.acos(dot_val))

        cross_prod = np.cross(v_hat, b_hat)

        dot_val = np.sum(cross_prod * perp_wind_dir_earth)
        if abs(dot_val) < 1e-6:
            sign_alpha = 1
        else:
            sign_alpha = np.sign(dot_val)

        aoa = sign_alpha * angle_mag

        if abs(aoa) < 1e-6:
            F_normal_dir = np.array([[0], [0], [0]])
        else:
            F_normal_dir = Te2b * np.cross(b_hat, np.cross(b_hat, v_hat))

        if sign_alpha < 0:
            F_axial = vehicle.calculate_axial(flap_angle, -aoa, mach_number)
            F_normal = vehicle.calculate_normal(flap_angle, -aoa, mach_number)
            F_axial = F_axial * density / density_SL
            F_normal = F_normal * density / density_SL
        else:
            F_axial = vehicle.calculate_axial(flap_angle, aoa, mach_number)
            F_normal = vehicle.calculate_normal(flap_angle, aoa, mach_number)
            F_axial = F_axial * density / density_SL
            F_normal = F_normal * density / density_SL

        F_axial_body = np.array([[-F_axial], [0], [0]])
        F_normal_body = F_normal * F_normal_dir
        F_A_body = F_axial_body + F_normal_body

        g_earth = np.array([[0], [0], [-EARTH_GRAVITY_ACCELERATION]])
        g_body = Te2b @ g_earth
        F_g_body = m * g_body

        F_P_body = np.array([[thrust], [0], [0]])

        F_body = F_A_body + F_P_body + F_g_body

        torque_body = np.cross(r_body, F_normal_body)

        pdot = (torque_body[0] / Ix) - q * r * (Iz - Iy) / Ix
        qdot = (torque_body[1] / Iy) - r * p * (Ix - Iz) / Iy
        rdot = (torque_body[2] / Iz) - p * q * (Iy - Ix) / Iz

        accel_body = F_body / m
        accel_earth = Tb2e @ accel_body

        phi_dot = p + (q * np.sin(phi) + r * np.cos(phi)) * np.tan(theta)
        theta_dot = q * np.cos(phi) - r * np.sin(phi)
        psi_dot = (q * np.sin(phi) + r * np.cos(phi)) / np.cos(theta)

        return (vel_earth, accel_earth, phi_dot, theta_dot, psi_dot, pdot,
                qdot, rdot)
