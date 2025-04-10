from dataclasses import dataclass
import logging
import traceback

import adafruit_bno08x
import adafruit_bno08x.i2c
import busio

from base.component import Component
from flight.blackboard import BNO085State

logger = logging.getLogger(__name__)


class BNO085Component(Component):

    def __init__(self, i2c: busio.I2C, address: int = 0x4a):
        self._state = BNO085State()

        self._bno085 = adafruit_bno08x.i2c.BNO08X_I2C(i2c, address=address)
        self._bno085.initialize()
        self._bno085.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
        self._bno085.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
        self._bno085.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
        self._bno085.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
        self._bno085.begin_calibration()

        logger.info("BNO085 initialized.")
        logger.info(f"{self._bno085.calibration_status=}")

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        # Read the raw acceleration.
        acceleration = None
        try:
            acceleration = self._bno085.acceleration
        except Exception as exception:
            logger.exception(
                f"Exception when reading BNO085 acceleration: {traceback.format_exc()}"
            )
        if acceleration is not None and acceleration[
                0] is not None and acceleration[
                    1] is not None and acceleration[2] is not None:
            self._state.acceleration = acceleration
        else:
            self._state.acceleration_errors += 1

        # Read the raw magnetic field.
        magnetic = None
        try:
            magnetic = self._bno085.magnetic
        except Exception as exception:
            logger.exception(
                f"Exception when reading BNO085 magnetic: {traceback.format_exc()}"
            )
        if magnetic is not None and magnetic[0] is not None and magnetic[
                1] is not None and magnetic[2] is not None:
            self._state.magnetic = magnetic
        else:
            self._state.magnetic_errors += 1

        # Read the raw angular velocity.
        gyro = None
        try:
            gyro = self._bno085.gyro
        except Exception as exception:
            logger.exception(
                f"Exception when reading BNO085 gyro: {traceback.format_exc()}"
            )
        if gyro is not None and gyro[0] is not None and gyro[
                1] is not None and gyro[2] is not None:
            self._state.gyro = gyro
        else:
            self._state.gyro_errors += 1

        # Read the fused orientation.
        quaternion = None
        try:
            quaternion = self._bno085.quaternion
        except Exception as exception:
            logger.exception(
                f"Exception when reading BNO085 quaternion: {traceback.format_exc()}"
            )
        if quaternion is not None and quaternion[0] is not None and quaternion[
                1] is not None and quaternion[2] and quaternion[3] is not None:
            self._state.quaternion = quaternion
        else:
            self._state.quaternion_errors += 1
