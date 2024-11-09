from dataclasses import dataclass
import logging
import traceback

import adafruit_bmp3xx
import busio

from base.component import Component

logger = logging.getLogger(__name__)


@dataclass
class BMP390State:
    # Altitude in meters.
    altitude: float = 0

    # Temperature in Celsius.
    temperature: float = 0

    # Count the number of times each reading fails.
    altitude_errors: int = 0
    temperature_errors: int = 0


class BMP390Component(Component):

    def __init__(self, i2c: busio.I2C, address: int = 0x77):
        self._state = BMP390State()

        self._bmp390 = adafruit_bmp3xx.BMP3XX_I2C(i2c, address=address)

        logger.info("BMP390 initialized.")
        logger.info(f"{self._bmp390.pressure_oversampling=}")
        logger.info(f"{self._bmp390.temperature_oversampling=}")
        logger.info(f"{self._bmp390.filter_coefficient=}")

    @property
    def state(self):
        return self._state

    def dispatch(self):
        altitude = None
        try:
            altitude = self._bmp390.altitude
        except Exception as exception:
            logger.exception(
                f"Exception when reading BMP390 altitude: {traceback.format_exc()}"
            )
        if altitude is not None:
            self._state.altitude = altitude
        else:
            self._state.altitude_errors += 1

        temperature = None
        try:
            temperature = self._bmp390.temperature
        except Exception as exception:
            logger.exception(
                f"Exception when reading BMP390 temperature: {traceback.format_exc()}"
            )
        if temperature is not None:
            self._state.temperature = temperature
        else:
            self._state.temperature_errors += 1
