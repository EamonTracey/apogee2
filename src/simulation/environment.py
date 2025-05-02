import json

import numpy as np

from base.constants import SPECIFIC_GAS_CONSTANT_AIR


class Environment:

    def __init__(self, ground_temperature: float, ground_pressure: float,
                 ground_wind: tuple[float, float, float]):
        self._ground_temperature = ground_temperature
        self._ground_pressure = ground_pressure
        self._ground_wind = ground_wind

        self._validate_environment()

    def _validate_environment(self):
        ...

    @classmethod
    def from_json(cls, file_path: str):
        with open(file_path, "r") as file:
            environment_json = json.load(file)
        environment_json = {
            k: tuple(v) if isinstance(v, list) else v
            for k, v in environment_json.items()
        }
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

    def calculate_wind(self, altitude: float):
        # TODO?: Implement a model.
        return self.ground_wind

    def calculate_temperature(self, altitude: float):
        # NASA Earth Atmosphere Model in the troposphere.
        # https://www.grc.nasa.gov/www/k-12/airplane/atmos.html.
        temperature = self.ground_temperature - 0.00356 * altitude
        return temperature

    def calculate_pressure(self, altitude: float):
        # NASA Earth Atmosphere Model in the troposphere.
        # https://www.grc.nasa.gov/www/k-12/airplane/atmos.html.
        temperature = self.calculate_temperature(altitude)
        pressure = self.ground_pressure * (temperature / 518.6)**5.256
        return pressure

    def calculate_density(self, altitude: float):
        temperature = self.calculate_temperature(altitude)
        pressure = self.calculate_pressure(altitude)
        density = pressure / SPECIFIC_GAS_CONSTANT_AIR / temperature
        return density
