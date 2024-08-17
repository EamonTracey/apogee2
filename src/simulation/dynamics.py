from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

from base.constants import EARTH_GRAVITY_ACCELERATION
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
        """Calculate the derivatives of the current position, linear momentum,
        orientation, and angular momentum."""

        # Unpack the current simulation state.
        time = self.state.time
        position = np.array(self.state.position)
        linear_momentum = np.array(self.state.linear_momentum)
        orientation = np.array(self.state.orientation)
        angular_momentum = np.array(self.state.angular_momentum)

        # The total mass equals the sum of the mass of the vehicle and mass of
        # the motor. The mass of the motor is a function of time since its
        # mass decreases as it burns.
        mass_total = self._vehicle.mass + self._motor.calculate_mass(time)

        # The derivative of the position vector equals the linear momentum
        # divided by the mass.
        linear_velocity = linear_momentum / mass_total

        # Compute the rotation matrix and yaw, pitch, and roll axes of the
        # vehicle.
        rotation = Rotation.from_quat(orientation).as_matrix()
        yaw_vehicle = rotation @ YAW
        pitch_vehicle = rotation @ PITCH
        roll_vehicle = rotation @ ROLL

        # Compute the angular velocity using the rotation matrix and reference
        # inertia tensor.
        inertia = np.diag(self._vehicle.inertia)
        inertia_inverse = np.linalg.inv(inertia)
        angular_velocity = rotation @ inertia_inverse @ rotation.T @ angular_momentum

        # Compute the derivative of the orientation using the angular velocity
        # and orientation (quaternion).
        scalar = orientation[0]
        vector = orientation[1:]
        scalar_derivative = 0.5 * np.dot(angular_velocity, vector)
        vector_dot = 0.5 * (scalar * angular_velocity +
                            np.cross(angular_velocity, vector))
        orientation_derivative = np.array(
            [scalar_derivative, *vector_derivative])

        # Approximate the apparent velocity vector.
        # V_cm

        # TODO: force???
        Ra = rotation @ ROLL
        force_thrust = -1 * self._motor.calculate_thrust(time) * Ra
        force_gravity = mass_total * EARTH_GRAVITY_ACCELERATION * Ra
        force = force_thrust + force_gravity

        # TODO: torque???
        torque = np.array([0.0, 0.0, 0.0])

        return linear_velocity, force, orientation_derivative, torque

    def step(self, time_delta: float):
        self._vehicle.calculate_axial(0.3, 0)
        assert time_delta > 0

        # Calculate the derivatives at the current state
        linear_velocity, force, orientation_derivative, torque = self._calculate_derivatives(
        )

        # Update the state in place
        self._state.position = tuple(
            np.array(self._state.position) + linear_velocity * time_delta)
        self._state.linear_momentum = tuple(
            np.array(self._state.linear_momentum) + force * time_delta)
        self._state.angular_momentum = tuple(
            np.array(self._state.angular_momentum) + torque * time_delta)
        self._state.orientation = tuple(
            np.array(self._state.orientation) +
            orientation_derivative * time_delta)

        # Update the simulation time
        self._state.time += time_delta
