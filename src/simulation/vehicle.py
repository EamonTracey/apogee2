import json
from typing import Self

import numpy as np
from scipy.interpolate import LinearNDInterpolator


class Vehicle:

    def __init__(self, mass: float, inertia: tuple[float, float, float],
                 attacks: tuple[float, ...], machs: tuple[float, ...],
                 axials: tuple[float, ...], normals: tuple[float, ...]):
        self._mass = float(mass)
        self._inertia = tuple(inertia)

        self._attacks = tuple(attacks)
        self._machs = tuple(machs)
        self._axials = tuple(axials)
        self._normals = tuple(normals)

        self._validate_vehicle()

    @property
    def mass(self):
        return self._mass

    @property
    def inertia(self):
        return self._inertia

    def _validate_vehicle(self):
        assert len(self._attacks) > 0
        assert len(self._attacks) == len(self._axials)
        assert all(a >= 0 for a in self._attacks)

        assert len(self._machs) > 0
        assert len(self._machs) == len(self._axials)
        assert all(m >= 0 for m in self._machs)

        assert len(self._axials) == len(self._normals)
        assert all(a >= 0 for a in self._axials)
        assert all(n >= 0 for n in self._normals)

        assert self._mass > 0

    @classmethod
    def from_json(cls, file_path: str) -> Self:
        with open(file_path, "r") as file:
            vehicle_json = json.load(file)

        vehicle = cls(**vehicle_json)
        return vehicle

    def calculate_axial(self, attack: float, mach: float) -> float:
        grid = np.array(tuple(zip(self._attacks, self._machs)))
        interpolator = LinearNDInterpolator(grid, self._axials)
        axial = interpolator(np.array([mach, attack]))

        return axial

    def calculate_normal(self, attack: float, mach: float) -> float:
        grid = np.array(tuple(zip(self._attacks, self._machs)))
        interpolator = LinearNDInterpolator(grid, self._normals)
        normal = interpolator(np.array([mach, attack]))

        return normal
