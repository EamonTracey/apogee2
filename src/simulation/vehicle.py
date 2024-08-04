import json
from typing import Self

class Vehicle:
    def __init__(self, mass: float):
        self._mass = mass

        self._validate_vehicle()

    @property
    def mass(self):
        return self._mass

    def _validate_vehicle(self):
        assert self._mass > 0

    @classmethod
    def from_json(cls, file_path: str) -> Self:
        with open(file_path, "r") as file:
            vehicle_json = json.load(file)

        mass = float(vehicle_json["mass"])
        vehicle = cls(mass)
        return vehicle

    def calculate_drag(self, velocity):
        # TODO: Refine drag calculation.

        drag = 0.5 * 1.225 * (velocity ** 2) * 0.39 * 0.018232222
        return drag
