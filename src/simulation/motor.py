import json

import numpy as np


class Motor:

    def __init__(self, length: float, dry_mass: float, wet_mass: float,
                 times: tuple[float, ...], forces: tuple[float, ...]):
        self._length = length
        self._dry_mass = dry_mass
        self._wet_mass = wet_mass
        self._times = times
        self._forces = forces

        self._validate_motor()

    @classmethod
    def from_json(cls, file_path: str):
        with open(file_path, "r") as file:
            motor_json = json.load(file)
        motor_json = {
            k: tuple(v) if isinstance(v, list) else v
            for k, v in motor_json.items()
        }
        motor = cls(**motor_json)
        return motor

    @property
    def length(self):
        return self._length

    @property
    def dry_mass(self):
        return self._dry_mass

    @property
    def wet_mass(self):
        return self._wet_mass

    def _validate_motor(self):
        assert len(self._times) > 0
        assert len(self._times) == len(self._forces)
        assert self._times == tuple(sorted(self._times))
        assert all(t >= 0 for t in self._times)
        assert self._wet_mass >= self._dry_mass >= 0

    def calculate_thrust(self, time: float) -> float:
        thrust = np.interp(time, self._times, self._forces, 0, 0)
        return float(thrust)

    def calculate_mass(self, time: float) -> float:
        # TODO: The motor does not expel propellant mass linearly.
        mass = np.interp(time, (0, self._times[-1]),
                         (self._wet_mass, self._dry_mass))
        return float(mass)
