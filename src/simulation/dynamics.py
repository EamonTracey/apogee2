from base.constants import EARTH_GRAVITY_ACCELERATION, SPEED_OF_SOUND
from simulation.motor import Motor
from simulation.vehicle import Vehicle


class DynamicsSimulation:

    def __init__(self, vehicle: Vehicle, motor: Motor):
        self._vehicle = vehicle
        self._motor = motor

        self._time = 0
        self._vehicle_position = 0
        self._vehicle_velocity = 0
        self._vehicle_acceleration = 0

    @property
    def time(self):
        return self._time

    @property
    def vehicle_position(self):
        return self._vehicle_position

    @property
    def vehicle_velocity(self):
        return self._vehicle_velocity

    @property
    def vehicle_acceleration(self):
        return self._vehicle_acceleration

    def initialize_conditions(self, vehicle_position: float = 0, self._vehicle_velocity: float = 0):
        self._vehicle_position = vehicle_position
        self._vehicle_velocity = vehicle_velocity

        self._time = 0
        self._vehicle_acceleration = 0

    def step(self, time_delta: float):
        assert time_delta > 0

        # Derive mach number.
        mach = self._vehicle_velocity / SPEED_OF_SOUND

        # Calculate the forces acting on the rocket.
        mass_total = self._vehicle.mass + self._motor.calculate_mass(
            self._time)
        force_gravity = mass_total * -EARTH_GRAVITY_ACCELERATION
        force_thrust = self._motor.calculate_thrust(self._time)
        force_drag = -self._vehicle.calculate_drag(mach)
        force_total = force_gravity + force_thrust + force_drag

        # Advance Newtonian physics.
        self._vehicle_acceleration = force_total / mass_total
        self._vehicle_velocity += self._vehicle_acceleration * time_delta
        self._vehicle_position += self._vehicle_velocity * time_delta

        # Update time.
        self._time += time_delta
