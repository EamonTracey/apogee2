import json
from typing import Self

import numpy as np
from scipy.interpolate import interpn


class Vehicle:

    def __init__(self, mass: float, length: float, center_of_mass: float,
                 center_of_pressure: float, inertia: tuple[float, float,
                                                           float],
                 attacks: tuple[float, ...], machs: tuple[float, ...],
                 axials: tuple[tuple[float, ...],
                               ...], normals: tuple[tuple[float, ...], ...]):
        self._mass = float(mass)
        self._length = float(length)
        self._center_of_mass = float(center_of_mass)
        self._center_of_pressure = float(center_of_pressure)
        self._inertia = tuple(inertia)

        self._attacks = tuple(map(float, attacks))
        self._machs = tuple(map(float, machs))

        self._axials = tuple(map(tuple, axials))
        self._normals = tuple(map(tuple, normals))

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
        assert len(self._attacks) > 0
        assert len(self._attacks) == len(self._axials)
        assert self._attacks == tuple(sorted(self._attacks))
        assert all(a >= 0 for a in self._attacks)

        assert len(self._machs) > 0
        assert all(len(self._machs) == len(ax) for ax in self._axials)
        assert self._machs == tuple(sorted(self._machs))
        assert all(m >= 0 for m in self._machs)

        assert len(self._axials) == len(self._normals)
        assert all(a >= 0 for ax in self._axials for a in ax)
        assert all(n >= 0 for no in self._normals for n in no)

        assert self._mass > 0

    @classmethod
    def from_json(cls, file_path: str) -> Self:
        with open(file_path, "r") as file:
            vehicle_json = json.load(file)

        vehicle = cls(**vehicle_json)
        return vehicle

    def calculate_axial(self, attack: float, mach: float) -> float:
        if attack < self._attacks[0]:
            attack = self._attacks[0]
        elif attack > self._attacks[-1]:
            attack = self._attacks[-1]

        if mach < self._machs[0]:
            mach = self._machs[0]
        elif mach > self._machs[-1]:
            mach = self._machs[-1]

        grid = (self._attacks, self._machs)
        axial = interpn(grid, self._axials, np.array((attack, mach)))

        return float(axial)

    def calculate_normal(self, attack: float, mach: float) -> float:
        if attack < self._attacks[0]:
            attack = self._attacks[0]
        elif attack > self._attacks[-1]:
            attack = self._attacks[-1]

        if mach < self._machs[0]:
            mach = self._machs[0]
        elif mach > self._machs[-1]:
            mach = self._machs[-1]

        grid = (self._attacks, self._machs)
        normal = interpn(grid, self._normals, np.array((attack, mach)))

        return float(normal)
