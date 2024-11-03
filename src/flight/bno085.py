from dataclasses import dataclass

import adafruit_bno08x
import adafruit_bno08x.i2c
import busio

from base.component import Component
from datetime import datetime

import numpy as np


@dataclass
class BNO085State:
    # Acceleration in meters per second squared.
    acceleration: tuple[float, float, float] = (0, 0, 0)

    # Magnetic field in microteslas.
    magnetic: tuple[float, float, float] = (0, 0, 0)

    # Angular velocity radians per second.
    gyro: tuple[float, float, float] = (0, 0, 0)

    # Orientation as a (w, x, y, z) quaternion.
    quaternion: tuple[float, float, float, float] = (0, 0, 0, 0)

    # Count the number of times each reading fails.
    acceleration_errors: int = 0
    magnetic_errors: int = 0
    gyro_errors: int = 0
    quaternion_errors: int = 0


class BNO085Component(Component):

    def __init__(self, i2c: busio.I2C, address: int = 0x4a):
        self._state = BNO085State()

        self._bno085 = adafruit_bno08x.i2c.BNO08X_I2C(i2c, address)
        self._bno085.initialize()
        self._bno085.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
        self._bno085.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
        self._bno085.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
        self._bno085.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
        self._bno085.begin_calibration()

    @property
    def state(self):
        return self._state

    def dispatch(self):
        # Read the raw acceleration.
        acceleration = None
        try:
            acceleration = self._bno085.acceleration
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
            magnetic = self._bno085.magnetic
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
            gyro = self._bno085.gyro
        except Exception as exception:
            # TODO: Implement logging.
            ...
        if gyro is not None:
            self._state.gyro = gyro
        else:
            self._state.gyro_errors += 1

        # Read the fused orientation.
        quaternion = None
        try:
            quaternion = self._bno085.quaternion
        except Exception as exception:
            # TODO: Implement logging.
            ...
        if quaternion is not None:
            self._state.quaternion = quaternion
        else:
            self._state.quaternion_errors += 1
