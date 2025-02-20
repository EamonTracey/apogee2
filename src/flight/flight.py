import logging
from typing import Optional

import board
import busio

from base.component import Component
from base.loop import Loop
from flight.bmp390 import BMP390Component
from flight.bno085 import BNO085Component
from flight.icm20649 import ICM20649Component
from flight.phase import PhaseComponent
from flight.filter import FilterComponent
from flight.log import LogComponent
from flight.active_states import ActiveStateComponent

logger = logging.getLogger(__name__)


class Flight:

    def __init__(self, name: str, results: int):
        self.loop = Loop(30)
        loop_state = self.loop.state

        # Connect to the I2C bus.
        i2c = busio.I2C(board.SCL, board.SDA)

        # BMP390.
        bmp390_component = BMP390Component(i2c)
        bmp390_state = bmp390_component.state
        self.loop.add_component(bmp390_component, 30)

        # BNO085.
        bno085_component = BNO085Component(i2c)
        bno085_state = bno085_component.state
        self.loop.add_component(bno085_component, 30)

        # ICM20649.
        icm20649_component = ICM20649Component(i2c)
        icm20649_state = icm20649_component.state
        self.loop.add_component(icm20649_component, 30)

        # Kalman Filter and Phase States
        active_states_component = ActiveStateComponent()
        phase_state = active_states_component.get_phase_state
        filter_state = active_states_component.get_filter_state

        # Kalman Filter.
        filter_component = FilterComponent(loop_state, bmp390_state,
                                            icm20649_state, phase_state, vertical = True)
        filter_state = filter_component.state
        active_states_component.filter_state = filter_state
        self.loop.add_component(filter_component, 30)

        # Phase Determination.
        phase_component = PhaseComponent(filter_state)
        phase_state = phase_component.state
        active_states_component.phase_state = phase_state
        self.loop.add_component(phase_component, 30)


        # Log.
        log_path = f"{name}.csv"
        log_component = LogComponent(log_path, results, loop_state, bmp390_state,
                                     bno085_state, icm20649_state, filter_state, phase_state) 
        self.loop.add_component(log_component, 30)

    def run(self):
        self.loop.run(0)
