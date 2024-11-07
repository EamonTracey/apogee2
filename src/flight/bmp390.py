from dataclasses import dataclass
import board

from base.component import Component

@dataclass
class BMP390State:
    pressure: float = 0
    altitude: float = 0
    temperature: float = 0

class BMP390Component(Component):
    def __init__(self):
        self._state = BMP390State()

    @property
    def state(self):
        return self._state

    def dispatch(self):
        i2c = board.I2C()
        altimeter = BMP390State(altimeter)
