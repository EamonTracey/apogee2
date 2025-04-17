from dataclasses import dataclass
import logging
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

from base.component import Component
from base.constants import EARTH_GRAVITY_ACCELERATION
from base.constants import PITCH, ROLL, YAW
from base.math import zenith_azimuth_to_quaternion
from base.stage import Stage
from simulation.constants import CFD_AIR_DENSITY
from simulation.environment import Environment
from simulation.motor import Motor
from simulation.vehicle import Vehicle

logger = logging.getLogger(__name__)


@dataclass
class DynamicsState:
    time: float = 0.0

    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    linear_momentum: tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_momentum: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    mass: float = 0

    stage: Stage = Stage.GROUND


class DynamicsComponent(Component):

    def __init__(self,
                 vehicle: Vehicle,
                 motor: Motor,
                 environment: Environment,
                 rail_zenith: float = 5,
                 rail_azimuth: float = 0):
        self._state = DynamicsState()

        self._vehicle = vehicle
        self._motor = motor
        self._environment = environment

        self._state.orientation = zenith_azimuth_to_quaternion(
            rail_zenith, rail_azimuth)
        self._state.mass = vehicle.mass + motor.calculate_mass(0)

        self._previous_time = 0

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        self._step(time - self._previous_time)
        self._previous_time = time

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
        vehicle = self._vehicle
        motor = self._motor
        environment = self._environment
        time = self._state.time
        position = np.array(self._state.position)
        linear_momentum = np.array(self._state.linear_momentum)
        orientation = np.array(self._state.orientation)
        angular_momentum = np.array(self._state.angular_momentum)

        # Perform RK4.
        k1p, k1l, k1o, k1a = calculate_derivatives(vehicle, motor, environment,
                                                   0, time, position,
                                                   linear_momentum,
                                                   orientation,
                                                   angular_momentum)
        k2p, k2l, k2o, k2a = calculate_derivatives(
            vehicle, motor, environment, 0, time + time_delta / 2,
            position + time_delta * k1p / 2,
            linear_momentum + time_delta * k1l / 2,
            orientation + time_delta * k1o / 2,
            angular_momentum + time_delta * k1a / 2)
        k3p, k3l, k3o, k3a = calculate_derivatives(
            vehicle, motor, environment, 0, time + time_delta / 2,
            position + time_delta * k2p / 2,
            linear_momentum + time_delta * k2l / 2,
            orientation + time_delta * k2o / 2,
            angular_momentum + time_delta * k2a / 2)
        k4p, k4l, k4o, k4a = calculate_derivatives(
            vehicle, motor, environment, 0, time + time_delta,
            position + time_delta * k3p, linear_momentum + time_delta * k3l,
            orientation + time_delta * k3o,
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
        self._state.mass = self._vehicle.mass + self._motor.calculate_mass(
            self._state.time)


def calculate_derivatives(vehicle, motor, environment, angle_of_actuation,
                          time, position, linear_momentum, orientation,
                          angular_momentum):
    """Calculate the derivatives of the current position, linear momentum,
        orientation, and angular momentum."""

    # Quaternion into normalized quaternion.
    orientation = orientation / np.linalg.norm(orientation) if np.linalg.norm(
        orientation) != 0 else orientation

    angle_of_actuation = 0
    orientation = orientation / np.linalg.norm(orientation) if np.linalg.norm(
        orientation) != 0 else orientation

    # The total mass equals the sum of the mass of the vehicle and mass of
    # the motor. The mass of the motor is a function of time since its
    # mass decreases as it burns.
    vehicle_mass = vehicle.mass
    motor_mass = motor.calculate_mass(time)
    mass_total = vehicle_mass + motor_mass

    # The derivative of the position vector equals the linear momentum
    # divided by the mass.
    linear_velocity = linear_momentum / mass_total

    # Compute the rotation matrix and yaw, pitch, and roll axes of the
    # vehicle.
    rotation = Rotation.from_quat(
        (*orientation[1:4], orientation[0])).as_matrix()

    vehicle_yaw = rotation @ YAW
    vehicle_pitch = rotation @ PITCH
    vehicle_roll = rotation @ ROLL

    # Compute the angular velocity using the rotation matrix and reference
    # inertia tensor.
    inertia = np.diag(vehicle.inertia)
    inertia_inverse = np.linalg.inv(inertia)
    angular_velocity = rotation @ inertia_inverse @ rotation.T @ angular_momentum.T

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
    center_of_mass = (vehicle_mass * vehicle.center_of_mass + motor_mass *
                      (vehicle.length - motor.length / 2)) / mass_total
    center_of_pressure = vehicle.center_of_pressure

    # Approximate the apparent velocity vector.
    distance_cm_cp = abs(center_of_mass - center_of_pressure)
    velocity_cm = linear_velocity + environment.calculate_wind(position[2])
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
        angle_of_attack = np.rad2deg(
            np.arccos(np.dot(velocity_apparent_direction, vehicle_roll)))

    # Earth atmosphere model.
    # https://www.grc.nasa.gov/www/k-12/airplane/atmos.html.
    air_temperature = environment.ground_temperature - 0.00356 * position[2]
    air_pressure = environment.ground_pressure * (air_temperature /
                                                  518.6)**5.256
    air_density = air_pressure / 1718 / air_temperature

    # Compute the mach number.
    # https://www.grc.nasa.gov/www/k-12/VirtualAero/BottleRocket/airplane/sound.html.
    speed_of_sound = np.sqrt(1.4 * air_pressure / air_density)
    mach_number = velocity_apparent_magnitude / speed_of_sound

    # Compute the force.
    #force_thrust = 0
    force_thrust = motor.calculate_thrust(time) * vehicle_roll
    force_gravity = np.array((0, 0, -mass_total * EARTH_GRAVITY_ACCELERATION))
    force_axial = np.array((0, 0, 0))
    force_normal = np.array((0, 0, 0))
    if velocity_apparent_magnitude != 0:
        aerodynamic_multiplier = air_density / CFD_AIR_DENSITY
        force_axial = -aerodynamic_multiplier * vehicle.calculate_axial_force(
            angle_of_actuation, angle_of_attack, mach_number) * vehicle_roll
        force_normal = aerodynamic_multiplier * vehicle.calculate_normal_force(
            angle_of_actuation, angle_of_attack, mach_number) * np.cross(
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
