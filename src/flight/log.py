import csv
from dataclasses import dataclass

from base.component import Component
from base.loop import LoopState
from flight.bmp390 import BMP390State
from flight.bno085 import BNO085State
from flight.icm20649 import ICM20649State
from flight.filter import FilterState

HEADERS = [
    "Time",
    "Loop_Slip_Count",
    "Phase",
    "Filtered_Altitude",
    "Filtered_Z_Velocity",
    "Filtered_Z_Acceleration",
    "Altitude_BMP390",
    "Temperature_BMP390",
    "Acceleration_X_BNO085",
    "Acceleration_Y_BNO085",
    "Acceleration_Z_BNO085",
    "Magnetic_X_BNO085",
    "Magnetic_Y_BNO085",
    "Magnetic_Z_BNO085",
    "Gyro_X_BNO085",
    "Gyro_Y_BNO085",
    "Gyro_Z_BNO085",
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
    "Altitude_Errors_BMP390",
    "Temperature_Errors_BMP390",
    "Acceleration_Errors_BNO085",
    "Magnetic_Errors_BNO085",
    "Gyro_Errors_BNO085",
    "Acceleration_Errors_ICM20649",
    "Gyro_Errors_ICM20649",
]


@dataclass
class LogState:
    ...


class LogComponent(Component):

    def __init__(self, path: str, loop_state: LoopState,
                 bmp390_state: BMP390State, bno085_state: BNO085State,
                 icm20649_state: ICM20649State, z_filter_state: FilterState, phase_state: PhaseState):
        self._state = LogState()

        self._path = path
        self._loop_state = loop_state
        self._bmp390_state = bmp390_state
        self._bno085_state = bno085_state
        self._icm20649_state = icm20649_state
        self._phase_state = phase_state
        self._z_filter_state = z_filter_state

        self._file = open(self._path, "w")
        self._writer = csv.writer(self._file)
        self._writer.writerow(HEADERS)

    def dispatch(self):
        log = [
            self._loop_state.time - self._loop_state.first_time,
            self._loop_state.slip_count,
            self._phase_state.phase
            self._z_filter_state.altitude_filtered,
            self._z_filter_state.velocity_filtered,
            self._z_filter_state.acceleration_filtered,
            self._bmp390_state.altitude,
            self._bmp390_state.temperature,
            *self._bno085_state.acceleration,
            *self._bno085_state.magnetic,
            *self._bno085_state.gyro,
            *self._bno085_state.quaternion,
            *self._icm20649_state.acceleration,
            *self._icm20649_state.gyro,
            self._bmp390_state.altitude_errors,
            self._bmp390_state.temperature_errors,
            self._bno085_state.acceleration_errors,
            self._bno085_state.magnetic_errors,
            self._bno085_state.gyro_errors,
            self._icm20649_state.acceleration_errors,
            self._icm20649_state.gyro_errors,
        ]
        self._writer.writerow(log)
