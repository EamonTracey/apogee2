import numpy as np

from simulation.environment import Environment
from simulation.motor import Motor
from simulation.vehicle import Vehicle
from simulation.teasley import TeasleyState, calculate_derivatives_teasley

motor = Motor.from_json("data/motors/aerotech_l1940x.json")
vehicle = Vehicle.from_json("data/vehicles/fullscale25.json")
environment = Environment.from_json("data/environments/threeoaks_basic.json")

state = TeasleyState()

dt = 0.025
while state.vel_earth[2] >= -5:
    flap_angle = state.flap_angle
    time = state.time

    x_earth = np.array(state.x_earth, dtype=float).reshape(3, 1)
    vel_earth = np.array(state.vel_earth, dtype=float).reshape(3, 1)
    phi = state.phi
    theta = state.theta
    psi = state.psi
    p = state.p
    q = state.q
    r = state.r

    d = calculate_derivatives_teasley(vehicle, motor, environment, flap_angle,
                                      time, x_earth, vel_earth, phi, theta,
                                      psi, p, q, r)
    x_earth += d[0] * dt
    vel_earth += d[1] * dt
    phi += d[2] * dt
    theta += d[3] * dt
    psi += d[4] * dt
    p += d[5] * dt
    q += d[6] * dt
    r += d[7] * dt

    state.time = time + dt
    state.x_earth = tuple(x_earth)
    state.vel_earth = tuple(vel_earth)
    state.phi = float(phi)
    state.theta = float(theta)
    state.psi = float(psi)
    state.p = float(p[0])
    state.q = float(q[0])
    state.r = float(r[0])

    print(x_earth[2])
