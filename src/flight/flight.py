import datetime

import board
import busio

from base.component import Component
from base.loop import Loop
from flight.bmp390 import BMP390Component
from flight.icm20649 import ICM20649Component
from flight.log import LogComponent

class Flight:
    def __init__(self):
        self.loop = Loop(30)
        utc_date = datetime.datetime.now(datetime.UTC)
        utc_date_string = strftime("%Y%m%d%H%M%S")
        name = f"ACS {utc_date_string}"

        # Connect to the I2C bus.
        i2c = busio.I2C(board.SCL, board.SDA)

        # BMP390.
        bmp390_component = BMP390Component(i2c)
        bmp390_state = bmp390_component.state
        self.loop.add_component(bmp390_component, 30)

        # BNO055.
        bno055_component = BNO055Component(i2c)
        bno055_state = bno055_component.state
        self.loop.add_component(bno055_component, 30)

        # ICM20649.
        icm20649_component = ICM20649Component(i2c)
        icm20649_state = icm20649_component.state
        self.loop.add_component(icm20649_component, 30)

        # Log.
        log_component = LogComponent(f"{name}.log", bmp390_state, bno055_state, icm20649_state)
        self.loop.add_component(log_component, 30)

    def fly(self):
        self.loop.run(0)
