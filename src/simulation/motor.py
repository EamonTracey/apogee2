import json
from typing import Self

import numpy as np


class Motor:

    def __init__(self, times: tuple[float], forces: tuple[float],
                 dry_mass: float, wet_mass: float):
        self._times = times
        self._forces = forces
        self._dry_mass = dry_mass
        self._wet_mass = wet_mass

        self._validate_motor()

    @classmethod
    def from_json(cls, file_path: str) -> Self:
        with open(file_path, "r") as file:
            motor_json = json.load(file)

        times = tuple(motor_json["times"])
        forces = tuple(motor_json["forces"])
        dry_mass = float(motor_json["dry_mass"])
        wet_mass = float(motor_json["wet_mass"])
        motor = cls(times, forces, dry_mass, wet_mass)
        return motor

    def _validate_motor(self):
        assert len(self._times) > 0
        assert len(self._times) == len(self._forces)
        assert self._times == tuple(sorted(self._times))
        assert self._wet_mass >= self._dry_mass >= 0

    def calculate_thrust(self, time: float) -> float:
        thrust = np.interp(time, self._times, self._forces, 0, 0)
        return thrust

    def calculate_mass(self, time: float) -> float:
        mass = np.interp(time, (0, self._times[-1]),
                         (self._wet_mass, self._dry_mass))
        return mass
