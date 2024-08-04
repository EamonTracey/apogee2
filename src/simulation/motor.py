import json
from typing import Self

import numpy as np


class Motor:

    def __init__(self, times: tuple[float], forces: tuple[float],
                 dry_mass: float, wet_mass: float):
        self._times = tuple(times)
        self._forces = tuple(forces)
        self._dry_mass = float(dry_mass)
        self._wet_mass = float(wet_mass)

        self._validate_motor()

    @classmethod
    def from_json(cls, file_path: str) -> Self:
        with open(file_path, "r") as file:
            motor_json = json.load(file)

        motor = cls(**motor_json)
        return motor

    def _validate_motor(self):
        assert len(self._times) > 0
        assert len(self._times) == len(self._forces)
        assert self._times == tuple(sorted(self._times))
        assert all(t >= 0 for t in self._times)
        assert self._wet_mass >= self._dry_mass >= 0

    def calculate_thrust(self, time: float) -> float:
        thrust = np.interp(time, self._times, self._forces, 0, 0)
        return thrust

    def calculate_mass(self, time: float) -> float:
        # TODO: The motor does not expel propellant mass linearly.

        mass = np.interp(time, (0, self._times[-1]),
                         (self._wet_mass, self._dry_mass))
        return mass
