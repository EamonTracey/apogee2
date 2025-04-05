import json

import numpy as np
from scipy.interpolate import RegularGridInterpolator


class Vehicle:

    def __init__(self, mass: float, length: float, center_of_mass: float,
                 center_of_pressure: float, inertia: tuple[float, float,
                                                           float],
                 axial_force: dict, normal_force: dict):
        self._mass = mass
        self._length = length
        self._center_of_mass = center_of_mass
        self._center_of_pressure = center_of_pressure
        self._inertia = tuple(inertia)
        self._axial_force = axial_force
        self._normal_force = normal_force

        # Craft axial and normal force interpolator objects.
        x = self._axial_force["angles_of_actuation"]
        y = self._axial_force["angles_of_attack"]
        z = self._axial_force["mach_numbers"]
        w = self._axial_force["forces"]
        self._axial_interpolator = RegularGridInterpolator((x, y, z),
                                                           w,
                                                           bounds_error=False,
                                                           fill_value=None)
        x = self._normal_force["angles_of_actuation"]
        y = self._normal_force["angles_of_attack"]
        z = self._normal_force["mach_numbers"]
        w = self._normal_force["forces"]
        self._normal_interpolator = RegularGridInterpolator((x, y, z),
                                                            w,
                                                            bounds_error=False,
                                                            fill_value=None)

        self._validate_vehicle()

    @property
    def mass(self):
        return self._mass

    @property
    def length(self):
        return self._length

    @property
    def center_of_mass(self):
        return self._center_of_mass

    @property
    def center_of_pressure(self):
        return self._center_of_pressure

    @property
    def inertia(self):
        return self._inertia

    def _validate_vehicle(self):
        # TODO: implement
        ...

    @classmethod
    def from_json(cls, file_path: str):
        with open(file_path, "r") as file:
            vehicle_json = json.load(file)

        vehicle = cls(**vehicle_json)
        return vehicle

    def calculate_axial_force(self, angle_of_actuation: float,
                              angle_of_attack: float,
                              mach_number: float) -> float:
        axial_force = self._axial_interpolator(
            (angle_of_actuation, angle_of_attack, mach_number))
        if axial_force < 0:
            axial_force = 0

        return axial_force

    def calculate_normal_force(self, angle_of_actuation: float,
                               angle_of_attack: float,
                               mach_number: float) -> float:
        normal_force = self._normal_interpolator(
            (angle_of_actuation, angle_of_attack, mach_number))
        if normal_force < 0:
            normal_force = 0

        return normal_force
