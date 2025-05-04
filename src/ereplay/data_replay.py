import logging
from dataclasses import dataclass

from base.component import Component
from base.stage import Stage
from flight.blackboard import BMP390State, BNO085State, ControlState, ICM20649State

import pandas as pd
import numpy as np


class DataReplayComponent(Component):

    def __init__(self, path):
        self._bmp390_state = BMP390State()
        self._bno085_state = BNO085State()
        self._icm20649_state = ICM20649State()
        self._control_state = ControlState()

        self._i = 0
        self._df = pd.read_csv(path)

    @property
    def bmp390_state(self):
        return self._bmp390_state

    @property
    def bno085_state(self):
        return self._bno085_state

    @property
    def icm20649_state(self):
        return self._icm20649_state

    @property
    def control_state(self):
        return self._control_state

    def dispatch(self, time: float):
        self._bmp390_state.altitude = self._df.iloc[self._i]["Altitude_BMP390"]
        self._bmp390_state.temperature = self._df.iloc[
            self._i]["Temperature_BMP390"]

        self._bno085_state.acceleration = (
            self._df.iloc[self._i]["Acceleration_X_BNO085"],
            self._df.iloc[self._i]["Acceleration_Y_BNO085"],
            self._df.iloc[self._i]["Acceleration_Z_BNO085"])
        self._bno085_state.magnetic = (
            self._df.iloc[self._i]["Magnetic_X_BNO085"],
            self._df.iloc[self._i]["Magnetic_Y_BNO085"],
            self._df.iloc[self._i]["Magnetic_Z_BNO085"])
        self._bno085_state.gyro = (self._df.iloc[self._i]["Gyro_X_BNO085"],
                                   self._df.iloc[self._i]["Gyro_Y_BNO085"],
                                   self._df.iloc[self._i]["Gyro_Z_BNO085"])
        self._bno085_state.quaternion = (
            self._df.iloc[self._i]["Quaternion_X_BNO085"],
            self._df.iloc[self._i]["Quaternion_Y_BNO085"],
            self._df.iloc[self._i]["Quaternion_Z_BNO085"],
            self._df.iloc[self._i]["Quaternion_W_BNO085"])

        self._icm20649_state.acceleration = (
            self._df.iloc[self._i]["Acceleration_X_ICM20649"],
            self._df.iloc[self._i]["Acceleration_Y_ICM20649"],
            self._df.iloc[self._i]["Acceleration_Z_ICM20649"])
        self._icm20649_state.gyro = (self._df.iloc[self._i]["Gyro_X_ICM20649"],
                                     self._df.iloc[self._i]["Gyro_Y_ICM20649"],
                                     self._df.iloc[self._i]["Gyro_Z_ICM20649"])

        self._control_state.servo_angle = self._df.iloc[self._i]["Servo_Angle"]

        self._i += 1
