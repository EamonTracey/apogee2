from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

from base.constants import EARTH_GRAVITY_ACCELERATION, SPEED_OF_SOUND
from base.constants import PITCH, ROLL, YAW
from simulation.motor import Motor
from simulation.vehicle import Vehicle


@dataclass
class SimulationState:
    time: float = 0.0

    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    linear_momentum: tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_momentum: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


class DynamicsSimulation:

    def __init__(self, vehicle: Vehicle, motor: Motor):
        self._vehicle = vehicle
        self._motor = motor

        self._state = SimulationState()

    @property
    def state(self):
        return self._state

    def _calculate_derivatives(self):
        """Calculates the derivatives of the current position, linear momentum, angular momentum, and orientation."""

        # Unpack the current simulation state.
        time = self.state.time
        position = np.array(self.state.position)
        linear_momentum = np.array(self.state.linear_momentum)
        angular_momentum = np.array(self.state.angular_momentum)
        orientation = np.array(self.state.orientation)

        # The total mass equals the sum of the mass of the vehicle and mass of the motor. The mass of the motor is a function of time since its mass decreases as it burns.
        mass_total = self._vehicle.mass + self._motor.calculate_mass(time)

        # The derivative of the position vector equals the linear momentum vector divided by the mass.
        derivative_position = linear_momentum / mass_total

        # Compute the angular velocity.
        # TODO: explain the calculation.
        # TODO: implement an inertia model
        rotation = Rotation.from_quat(orientation).as_matrix()
        inertia_inverse = np.diag([1, 1, 1])
        angular_velocity = rotation @ inertia_inverse @ rotation.T @ angular_momentum

        # TODO: understand and explain.
        s = orientation[0]
        v = orientation[1:]
        s_dot = 0.5 * np.dot(angular_velocity, v)
        v_dot = 0.5 * (s * angular_velocity + np.cross(angular_velocity, v))
        derivative_orientation = np.array([s_dot, *v_dot])

        # TODO: force???
        Ra = rotation @ ROLL
        force_thrust = -1 * self._motor.calculate_thrust(time) * Ra
        force_gravity = mass_total * EARTH_GRAVITY_ACCELERATION * Ra
        force = force_thrust + force_gravity

        # TODO: torque???
        torque = np.array([0.0, 0.0, 0.0])

        return derivative_position, force, torque, derivative_orientation

    def step(self, time_delta: float):
        assert time_delta > 0

        # Calculate the derivatives at the current state
        derivative_position, force, torque, derivative_orientation = self._calculate_derivatives(
        )

        # Update the state in place
        self._state.position = tuple(
            np.array(self._state.position) + derivative_position * time_delta)
        self._state.linear_momentum = tuple(
            np.array(self._state.linear_momentum) + force * time_delta)
        self._state.angular_momentum = tuple(
            np.array(self._state.angular_momentum) + torque * time_delta)
        self._state.orientation = tuple(
            np.array(self._state.orientation) +
            derivative_orientation * time_delta)

        # Update the simulation time
        self._state.time += time_delta
