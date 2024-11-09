import csv
from dataclasses import dataclass

from base.component import Component
from base.loop import LoopState
from flight.bmp390 import BMP390State
from flight.bno085 import BNO085State
from flight.icm20649 import ICM20649State

HEADERS = [
    "Time",
    "Loop_Slip_Count",
    "Altitude_BMP390",
    "Pressure_BMP390",
    "Temperature_BMP390",
    "Acceleration_X_BNO085",
    "Acceleration_Y_BNO085",
    "Acceleration_Z_BNO085",
    "Gyro_X_BNO085",
    "Gyro_Y_BNO085",
    "Gyro_Z_BNO085",
    "Magnetic_X_BNO085",
    "Magnetic_Y_BNO085",
    "Magnetic_Z_BNO085",
    "Quaternion_W_BNO085",
    "Quaternion_X_BNO085",
    "Quaternion_Y_BNO085",
    "Quaternion_Z_BNO085",
    "Acceleration_X_ICM20649",
    "Acceleration_Y_ICM20649",
    "Acceleration_Z_ICM20649",
    "Gyro_X_ICM20649",
    "Gyro_Y_ICM20649",
    "Gyro_Z_ICM20649",
    "Magnetic_X_ICM20649",
    "Magnetic_Y_ICM20649",
    "Magnetic_Z_ICM20649",
]


@dataclass
class LogState:
    ...


class LogComponent(Component):

    def __init__(self, path: str, loop_state: LoopState,
                 bmp390_state: BMP390State, bno085_state: BNO085State,
                 icm20649_state: ICM20649State):
        self._state = LogState()

        self._path = path
        self._loop_state = loop_state
        self._bmp390_state = bmp390_state
        self._bno085_state = bno085_state
        self._icm20649_state = icm20649_state

        self._file = open(self._path, "w")
        self._writer = csv.writer(self._file)
        self._writer.writerow(HEADERS)

    def dispatch(self):
        log = [
            self._loop_state.time,
            self._loop_state.slip_count,
            self._bmp390_state.altitude,
            self._bmp390_state.pressure,
            self._bmp390_state.temperature,
            *self._bno085_state.acceleration,
            *self._bno085_state.magnetic,
            *self._bno085_state.gyro,
            *self._bno085_state.quaternion,
            *self._icm20649_state.acceleration,
            *self._icm20649_state.magnetic,
            *self._icm20649_state.gyro,
        ]
        csv.writerow(log)
