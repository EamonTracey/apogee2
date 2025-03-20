import json
from typing import Self

import numpy as np


class Environment:

    def __init__(self, ground_temperature: float, ground_pressure: float, ground_wind: tuple[float, float, float]):
        self._ground_temperature = ground_temperature
        self._ground_pressure = ground_pressure
        self._ground_wind = ground_wind

        self._validate_environment()

    def _validate_environment(self):
        ...

    @classmethod
    def from_json(cls, file_path: str) -> Self:
        with open(file_path, "r") as file:
            environment_json = json.load(file)
            environment = cls(**environment_json)
            return environment

    @property
    def ground_temperature(self):
        return self._ground_temperature

    @property
    def ground_pressure(self):
        return self._ground_pressure
    
    @property
    def ground_wind(self):
        return self._ground_wind

    def calculate_wind(self, altitude: float) -> float:
        # TODO: implement
        ...
        return 0
