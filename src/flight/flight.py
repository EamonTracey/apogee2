import datetime
from typing import Optional

import board
import busio

from base.component import Component
from base.loop import Loop
from flight.bmp390 import BMP390Component
from flight.bno085 import BNO085Component
from flight.icm20649 import ICM20649Component
from flight.log import LogComponent


class Flight:

    def __init__(self, log_file: Optional[str] = None):
        self.loop = Loop(30)

        # Naming is hard.
        utc_date = datetime.datetime.now(datetime.UTC)
        utc_date_string = strftime("%Y%m%d%H%M%S")
        name = f"ACS {utc_date_string}"

        # Handle optional configurations.
        if log_file is None:
            log_file = f"{name}.log"

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

        # Log.
        log_component = LogComponent(log_file, bmp390_state, bno085_state,
                                     icm20649_state)
        self.loop.add_component(log_component, 30)

    def run(self):
        self.loop.run(0)
