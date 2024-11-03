from dataclasses import dataclass

import busio

from base.component import Component


@dataclass
class ICM20948State:
    acceleration: tuple[float, float, float] = (0, 0, 0)
    magnetic: tuple[float, float, float] = (0, 0, 0)
    gyro: tuple[float, float, float] = (0, 0, 0)


class ICM20948Component(Component):

    def __init__(self, i2c: busio.I2C, address: int = 0x68):
        self._state = ICM20948State()

    @property
    def state(self):
        return self._state

    def dispatch(self):
        ...