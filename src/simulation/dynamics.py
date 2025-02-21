from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

from base.component import Component
from base.constants import EARTH_GRAVITY_ACCELERATION
from base.constants import PITCH, ROLL, YAW
from base.stage import Stage
from simulation.environment import Environment
from simulation.motor import Motor
from simulation.vehicle import Vehicle


@dataclass
class DynamicsState:
    time: float = 0.0

    # SI units.

    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    linear_momentum: tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_momentum: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    mass: float = 0

    stage: Stage = Stage.GROUND


class DynamicsComponent(Component):

    def __init__(self, vehicle: Vehicle, motor: Motor,
                 environment: Environment):
        self._state = DynamicsState()
        self._state.mass = vehicle.calculate_mass(0)

        self._vehicle = vehicle
        self._motor = motor
        self._environment = environment

        self._previous_time = 0

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        self._step(time - self._previous_time)

    def _calculate_derivatives(self, time, position, linear_momentum,
                               orientation, angular_momentum):
        """Calculate the derivatives of the current position, linear momentum,
        orientation, and angular momentum."""

        # The total mass equals the sum of the mass of the vehicle and mass of
        # the motor. The mass of the motor is a function of time since its
        # mass decreases as it burns.
        vehicle_mass = self._vehicle.mass
        motor_mass = self._motor.calculate_mass(time)
        mass_total = vehicle_mass + motor_mass

        # The derivative of the position vector equals the linear momentum
        # divided by the mass.
        linear_velocity = linear_momentum / mass_total

        # Compute the rotation matrix and yaw, pitch, and roll axes of the
        # vehicle.
        rotation = Rotation.from_quat(orientation,
                                      scalar_first=True).as_matrix()
        vehicle_yaw = rotation @ YAW
        vehicle_pitch = rotation @ PITCH
        vehicle_roll = rotation @ ROLL

        # Compute the angular velocity using the rotation matrix and reference
        # inertia tensor.
        inertia = np.diag(self._vehicle.inertia)
        inertia_inverse = np.linalg.inv(inertia)
        angular_velocity = rotation @ inertia_inverse @ rotation.T @ angular_momentum

        # Compute the derivative of the orientation using the angular velocity
        # and orientation (quaternion).
        orientation_scalar = orientation[0]
        orientation_vector = orientation[1:]
        orientation_scalar_derivative = 0.5 * np.dot(angular_velocity,
                                                     orientation_vector)
        orientation_vector_derivative = 0.5 * (
            orientation_scalar * angular_velocity +
            np.cross(angular_velocity, orientation_vector))
        orientation_derivative = np.array(
            (orientation_scalar_derivative, *orientation_vector_derivative))

        # Compute the center of mass and center of pressure.
        center_of_mass = (
            vehicle_mass * self._vehicle.center_of_mass + motor_mass *
            (self._vehicle.length - self._motor.length / 2)) / mass_total
        center_of_pressure = self._vehicle.center_of_pressure

        # Approximate the apparent velocity vector.
        distance_cm_cp = abs(center_of_mass - center_of_pressure)
        velocity_cm = linear_velocity + self._environment.calculate_wind(
            position[2])
        velocity_cp = np.array((0, 0, 0))
        if not np.all(angular_velocity == 0):
            velocity_cp = distance_cm_cp * np.sin(
                np.arccos(
                    np.dot(vehicle_roll, angular_velocity /
                           np.linalg.norm(angular_velocity)))) * np.cross(
                               vehicle_roll, angular_velocity)
        velocity_apparent = velocity_cm + velocity_cp
        velocity_apparent_magnitude = 0
        velocity_apparent_direction = None
        if not np.all(velocity_apparent == 0):
            velocity_apparent_magnitude = np.linalg.norm(velocity_apparent)
            velocity_apparent_direction = velocity_apparent / velocity_apparent_magnitude

        # Compute the angle of attack.
        angle_of_attack = 0
        if velocity_apparent_magnitude != 0:
            angle_of_attack = np.arccos(
                np.dot(velocity_apparent_direction, vehicle_roll))

        # Compute the mach number.
        mach_number = 0
        # TODO: speed of sound using environment
        speed_of_sound = 343
        mach_number = velocity_apparent_magnitude / speed_of_sound

        # Compute the force.
        force_thrust = self._motor.calculate_thrust(time) * vehicle_roll
        force_gravity = np.array(
            (0, 0, -mass_total * EARTH_GRAVITY_ACCELERATION))
        force_axial = np.array((0, 0, 0))
        force_normal = np.array((0, 0, 0))
        if velocity_apparent_magnitude != 0:
            force_axial = -1 * self._vehicle.calculate_axial(
                angle_of_attack, mach_number) * vehicle_roll
            force_normal = -1 * self._vehicle.calculate_normal(
                angle_of_attack, mach_number) * np.cross(
                    vehicle_roll,
                    np.cross(vehicle_roll, velocity_apparent_direction))
        force = force_thrust + force_gravity + force_axial + force_normal

        # Compute the torque.
        torque_normal = np.array((0, 0, 0))
        if velocity_apparent_magnitude != 0:
            torque_normal = force_normal * distance_cm_cp * np.cross(
                vehicle_roll, velocity_apparent_direction)
        # TODO: torque roll from cfd
        torque_roll = np.array((0, 0, 0))
        torque = torque_normal + torque_roll

        return linear_velocity, force, orientation_derivative, torque

    def _update_stage(self):
        # TODO: implement
        if self._state.stage == Stage.GROUND:
            self._state.stage = Stage.RAIL
        elif self._state.stage == Stage.RAIL:
            self._state.stage = Stage.BURN
        elif self._state.stage == Stage.BURN:
            self._state.stage = Stage.COAST
        elif self._state.stage == Stage.COAST:
            if self._state.linear_momentum[2] < 0:
                self._state.stage = Stage.DESCENT
        elif self._state.stage == Stage.DESCENT:
            ...

    def _step(self, time_delta: float):
        assert time_delta >= 0
        if (time_delta == 0):
            return

        # Unpack the simulation state into numpy objects.
        time = self._state.time
        position = np.array(self._state.position)
        linear_momentum = np.array(self._state.linear_momentum)
        orientation = np.array(self._state.orientation)
        angular_momentum = np.array(self._state.angular_momentum)

        # Perform RK4.
        k1p, k1l, k1o, k1a = self._calculate_derivatives(
            time, position, linear_momentum, orientation, angular_momentum)
        k2p, k2l, k2o, k2a = self._calculate_derivatives(
            time + time_delta / 2, position + time_delta * k1p / 2,
            linear_momentum + time_delta * k1l / 2,
            orientation + time_delta * k1o / 2,
            angular_momentum + time_delta * k1a / 2)
        k3p, k3l, k3o, k3a = self._calculate_derivatives(
            time + time_delta / 2, position + time_delta * k2p / 2,
            linear_momentum + time_delta * k2l / 2,
            orientation + time_delta * k2o / 2,
            angular_momentum + time_delta * k2a / 2)
        k4p, k4l, k4o, k4a = self._calculate_derivatives(
            time + time_delta, position + time_delta * k3p,
            linear_momentum + time_delta * k3l, orientation + time_delta * k3o,
            angular_momentum + time_delta * k3a)
        position += time_delta / 6 * (k1p + 2 * k2p + 2 * k3p + k4p)
        linear_momentum += time_delta / 6 * (k1l + 2 * k2l + 2 * k3l + k4l)
        orientation += time_delta / 6 * (k1o + 2 * k2o + 2 * k3o + k4o)
        angular_momentum += time_delta / 6 * (k1a + 2 * k2a + 2 * k3a + k4a)

        # Repack the simulation state.
        self._state.position = tuple(position)
        self._state.linear_momentum = tuple(linear_momentum)
        self._state.orientation = tuple(orientation)
        self._state.angular_momentum = tuple(angular_momentum)

        # Update the flight stage.
        self._update_stage()

        # Update other simulation state items.
        self._state.time += time_delta
        self._state.mass = self._motor.calculate_mass(self._state.time)
