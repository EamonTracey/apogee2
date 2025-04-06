from dataclasses import dataclass
import logging
import traceback

import adafruit_icm20x
import busio

from base.component import Component
from flight.blackboard import ICM20649State

logger = logging.getLogger(__name__)


class ICM20649Component(Component):

    def __init__(self, i2c: busio.I2C, address: int = 0x68):
        self._state = ICM20649State()

        self._icm20649 = adafruit_icm20x.ICM20649(i2c, address=address)
        self._icm20649.accelerometer_range = adafruit_icm20x.AccelRange.RANGE_30G

        logger.info("ICM20649 initialized.")
        logger.info(f"{self._icm20649.accelerometer_range=}")
        logger.info(f"{self._icm20649.accelerometer_data_rate_divisor=}")
        logger.info(f"{self._icm20649.accelerometer_data_rate=}")
        logger.info(f"{self._icm20649.accel_dlpf_cutoff=}")
        logger.info(f"{self._icm20649.gyro_range=}")
        logger.info(f"{self._icm20649.gyro_data_rate_divisor=}")
        logger.info(f"{self._icm20649.gyro_data_rate=}")
        logger.info(f"{self._icm20649.gyro_dlpf_cutoff=}")

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        # Read the raw acceleration.
        acceleration = None
        try:
            acceleration = self._icm20649.acceleration
        except Exception as exception:
            logger.exception(
                f"Exception when reading ICM20649 acceleration: {traceback.format_exc()}"
            )
        if acceleration is not None and abs(acceleration[2]) < 200:
            self._state.acceleration = acceleration
        else:
            self._state.acceleration_errors += 1

        # Read the raw angular velocity.
        gyro = None
        try:
            gyro = self._icm20649.gyro
        except Exception as exception:
            logger.exception(
                f"Exception when reading ICM20649 gyro: {traceback.format_exc()}"
            )
        if gyro is not None:
            self._state.gyro = gyro
        else:
            self._state.gyro_errors += 1
