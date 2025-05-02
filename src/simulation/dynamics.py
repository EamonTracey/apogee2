from dataclasses import dataclass
import logging
from typing import Optional

import numpy as np

from base.component import Component
from simulation.eamonteasley import calculate_derivatives_eamonteasley
from simulation.environment import Environment
from simulation.motor import Motor
from simulation.vehicle import Vehicle

logger = logging.getLogger(__name__)


@dataclass
class DynamicsState:
    time: float = 0.0
    mass: float = 0

    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: tuple[float, float, float,
                       float] = (0.70710678, 0.0, -0.70710678, 0.0)
    angular_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)


class DynamicsComponent(Component):

    def __init__(self, vehicle: Vehicle, motor: Motor,
                 environment: Environment):
        self._state = DynamicsState()

        self._vehicle = vehicle
        self._motor = motor
        self._environment = environment

        self._state.mass = vehicle.mass + motor.calculate_mass(0)

        self._previous_time = 0

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        self._step(time - self._previous_time)
        self._previous_time = time

    def _step(self, time_delta: float):
        assert time_delta >= 0
        if (time_delta == 0):
            return

        # Unpack the simulation state into numpy objects.
        vehicle = self._vehicle
        motor = self._motor
        environment = self._environment
        time = self._state.time
        position = np.array(self._state.position, dtype=float)
        velocity = np.array(self._state.velocity, dtype=float)
        orientation = np.array(self._state.orientation, dtype=float)
        angular_velocity = np.array(self._state.angular_velocity, dtype=float)

        # Perform RK4.
        k1p, k1l, k1o, k1a = calculate_derivatives_eamonteasley(
            vehicle, motor, environment, 0, time, position, velocity,
            orientation, angular_velocity)
        k2p, k2l, k2o, k2a = calculate_derivatives_eamonteasley(
            vehicle, motor, environment, 0, time + time_delta / 2,
            position + time_delta * k1p / 2, velocity + time_delta * k1l / 2,
            orientation + time_delta * k1o / 2,
            angular_velocity + time_delta * k1a / 2)
        k3p, k3l, k3o, k3a = calculate_derivatives_eamonteasley(
            vehicle, motor, environment, 0, time + time_delta / 2,
            position + time_delta * k2p / 2, velocity + time_delta * k2l / 2,
            orientation + time_delta * k2o / 2,
            angular_velocity + time_delta * k2a / 2)
        k4p, k4l, k4o, k4a = calculate_derivatives_eamonteasley(
            vehicle, motor, environment, 0, time + time_delta,
            position + time_delta * k3p, velocity + time_delta * k3l,
            orientation + time_delta * k3o,
            angular_velocity + time_delta * k3a)
        position += time_delta / 6 * (k1p + 2 * k2p + 2 * k3p + k4p)
        velocity += time_delta / 6 * (k1l + 2 * k2l + 2 * k3l + k4l)
        orientation += time_delta / 6 * (k1o + 2 * k2o + 2 * k3o + k4o)
        orientation /= np.linalg.norm(orientation)
        angular_velocity += time_delta / 6 * (k1a + 2 * k2a + 2 * k3a + k4a)

        # Repack the simulation state.
        self._state.position = tuple(position)
        self._state.velocity = tuple(velocity)
        self._state.orientation = tuple(orientation)
        self._state.angular_velocity = tuple(angular_velocity)

        # Update other simulation state items.
        self._state.time += time_delta
        self._state.mass = self._vehicle.mass + self._motor.calculate_mass(
            self._state.time)
