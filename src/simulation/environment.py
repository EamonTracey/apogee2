import json
from typing import Self

import numpy as np


class Environment:

    def __init__(
        self,
        altitudes: tuple[float, ...],
        winds: tuple[tuple[float, float, float], ...],
    ):
        self._altitudes = tuple(altitudes)
        self._winds = winds

        self._validate_environment()

    def _validate_environment(self):
        ...

    @classmethod
    def from_json(cls, file_path: str) -> Self:
        with open(file_path, "r") as file:
            environment_json = json.load(file)

        environment = cls(**environment_json)
        return environment
