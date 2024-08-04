import json
from typing import Self

import numpy as np


class Vehicle:

    def __init__(self, mass: float, machs: tuple[float], drags: tuple[float]):
        self._mass = float(mass)
        self._machs = tuple(machs)
        self._drags = tuple(drags)

        self._validate_vehicle()

    @property
    def mass(self):
        return self._mass

    def _validate_vehicle(self):
        assert len(self._machs) > 0
        assert len(self._machs) == len(self._drags)
        assert self._machs == tuple(sorted(self._machs))
        assert all(v >= 0 for v in self._machs)
        assert self._mass > 0

    @classmethod
    def from_json(cls, file_path: str) -> Self:
        with open(file_path, "r") as file:
            vehicle_json = json.load(file)

        vehicle = cls(**vehicle_json)
        return vehicle

    def calculate_drag(self, mach):
        drag = np.interp(mach, self._machs, self._drags, 0, self._drags[-1])
        return drag
