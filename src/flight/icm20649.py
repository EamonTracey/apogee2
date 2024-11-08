from dataclasses import dataclass

import adafruit_icm20x
import busio

from base.component import Component


@dataclass
class ICM20649State:
    # The acceleration in meters per second squared.
    acceleration: tuple[float, float, float] = (0, 0, 0)

    # The magnetic field in microteslas.
    magnetic: tuple[float, float, float] = (0, 0, 0)

    # The angular velocity in degrees per second.
    gyro: tuple[float, float, float] = (0, 0, 0)

    # Count the number of times each reading fails.
    acceleration_errors: int = 0
    magnetic_errors: int = 0
    gyro_errors: int = 0


class ICM20649Component(Component):

    def __init__(self, i2c: busio.I2C, address: int = 0x68):
        self._state = ICM20649State()

        self._icm20649 = adafruit_icm20649.ICM20649(i2c, address)
        self._icm20649.initialize()
        self._icm20649.accelerometer_range = adafruit_icm20649.AccelRange.RANGE_30G

    @property
    def state(self):
        return self._state

    def dispatch(self):
        # Read the raw acceleration.
        acceleration = None
        try:
            acceleration = self._icm20649.acceleration
        except Exception as exception:
            # TODO: Implement logging.
            ...
        if acceleration is not None:
            self._state.acceleration = acceleration
        else:
            self._state.acceleration_errors += 1

        # Read the raw magnetic field.
        magnetic = None
        try:
            magnetic = self._icm20649.magnetic
        except Exception as exception:
            # TODO: Implement logging.
            ...
        if magnetic is not None:
            self._state.magnetic = magnetic
        else:
            self._state.magnetic_errors += 1

        # Read the raw angular velocity.
        gyro = None
        try:
            gyro = self._icm20649.gyro
        except Exception as exception:
            # TODO: Implement logging.
            ...
        if gyro is not None:
            self._state.gyro = gyro
        else:
            self._state.gyro_errors += 1
