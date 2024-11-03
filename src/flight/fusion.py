from dataclasses import dataclass

import imufusion

from base.component import Component
from bmp390 import BMP390State
from icm20948 import ICM20948State


@dataclass
class FusionState:
    quaternion: tuple[float, float, float, float] = 0
    euler: tuple[float, float, float] = 0


class FusionComponent(Component):

    def __init__(self, imu_state: ICM20948State):
        self._state = FusionState()

    @property
    def state(self):
        return self._state

    def dispatch(self):
        ...
