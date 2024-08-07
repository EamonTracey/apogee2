from dataclasses import dataclass
from typing import Optional

import numpy as np

from base.constants import EARTH_GRAVITY_ACCELERATION, SPEED_OF_SOUND
from simulation.motor import Motor
from simulation.vehicle import Vehicle


@dataclass
class SimulationState:
    time: float = 0.0

    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)


class DynamicsSimulation:

    def __init__(self, vehicle: Vehicle, motor: Motor):
        self._vehicle = vehicle
        self._motor = motor

        self._state = SimulationState()

    @property
    def state(self):
        return self._state

    def step(self, time_delta: float):
        assert time_delta > 0

        # Unpack the simulation state.
        time = self.state.time
        position = np.array(self.state.position)
        velocity = np.array(self.state.velocity)

        # Derive mach number.
        mach = velocity[2] / SPEED_OF_SOUND

        # Calculate the forces acting on the rocket.
        mass_total = self._vehicle.mass + self._motor.calculate_mass(time)
        force_gravity = mass_total * -EARTH_GRAVITY_ACCELERATION
        force_thrust = self._motor.calculate_thrust(time)
        force_drag = -self._vehicle.calculate_drag(mach)
        force_total = force_gravity + force_thrust + force_drag

        # Advance Newtonian physics.
        acceleration = force_total / mass_total
        velocity[2] += acceleration * time_delta
        position[2] += velocity[2] * time_delta

        # Repack the simulation state.
        self._state.time = time + time_delta
        self._state.position = tuple(position)
        self._state.velocity = tuple(velocity)
