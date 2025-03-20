import csv
from dataclasses import dataclass

from base.component import Component
from base.loop import LoopState
from flight.bmp390 import BMP390State
from flight.bno085 import BNO085State
from flight.icm20649 import ICM20649State
from flight.filter import FilterState
from flight.stage import StageState
from flight.control import ControlState

HEADERS = [
    "Time",
    "Loop_Slip_Count",
    "Stage",
    "Servo_Angle",
    "Altitude",
    "Velocity_X",
    "Velocity_Y",
    "Velocity_Z",
    "Acceleration_X",
    "Acceleration_Y",
    "Acceleration_Z",
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

    def __init__(self, path: str, results: int, loop_state: LoopState,
                 bmp390_state: BMP390State, bno085_state: BNO085State,
                 icm20649_state: ICM20649State, filter_state: FilterState,
                 control_state: ControlState, stage_state: StageState):
        self._state = LogState()

        self._path = path
        self._loop_state = loop_state
        self._bmp390_state = bmp390_state
        self._bno085_state = bno085_state
        self._icm20649_state = icm20649_state
        self._stage_state = stage_state
        self._filter_state = filter_state
        self._control_state = control_state

        self._file = open(self._path, "w")
        self._writer = csv.writer(self._file)
        self._writer.writerow(HEADERS)
        self._results = results

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        log = [
            time,
            self._loop_state.slip_count,
            self._stage_state.stage,
            self._control_state.servo_angle,
            self._filter_state.altitude,
            *self._filter_state.velocity,
            *self._filter_state.acceleration,
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

        # If console output is selected.
        if self._results == 1:
            print(log[0], log[1], log[2], log[3], log[4], log[5], log[6], log[7], log[8], log[9], log[10])
        elif self._results == 2:
            print(log)
