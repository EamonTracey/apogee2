import csv
from dataclasses import dataclass

from base.component import Component
from base.loop import LoopState
from flight.blackboard import (BMP390State, BNO085State, ControlState,
                               FilterState, FusionState, ICM20649State,
                               LogState, PredictState, StageState)

HEADERS = [
    "Time",
    "Loop_Slip_Count",
    "Stage",
    "Servo_Angle",
    "Predicted_Apogee",
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
    "Quaternion_X_BNO085",
    "Quaternion_Y_BNO085",
    "Quaternion_Z_BNO085",
    "Quaternion_W_BNO085",
    "Acceleration_X_ICM20649",
    "Acceleration_Y_ICM20649",
    "Acceleration_Z_ICM20649",
    "Gyro_X_ICM20649",
    "Gyro_Y_ICM20649",
    "Gyro_Z_ICM20649",
    "Quaternion_Fused_X",
    "Quaternion_Fused_Y",
    "Quaternion_Fused_Z",
    "Quaternion_Fused_W",
    "Euler_Fused_Roll",
    "Euler_Fused_Yaw",
    "Euler_Fused_Pitch",
    "Euler_Fused_Zenith",
    "Altitude_Errors_BMP390",
    "Temperature_Errors_BMP390",
    "Acceleration_Errors_BNO085",
    "Magnetic_Errors_BNO085",
    "Gyro_Errors_BNO085",
    "Acceleration_Errors_ICM20649",
    "Gyro_Errors_ICM20649",
]


class LogComponent(Component):

    def __init__(self, path: str, loop_state: LoopState,
                 bmp390_state: BMP390State, bno085_state: BNO085State,
                 icm20649_state: ICM20649State, filter_state: FilterState,
                 control_state: ControlState, stage_state: StageState,
                 predict_state: PredictState, fusion_state: FusionState):
        self._state = LogState()

        self._path = path
        self._loop_state = loop_state
        self._bmp390_state = bmp390_state
        self._bno085_state = bno085_state
        self._icm20649_state = icm20649_state
        self._stage_state = stage_state
        self._filter_state = filter_state
        self._control_state = control_state
        self._predict_state = predict_state
        self._fusion_state = fusion_state

        self._file = open(self._path, "w")
        self._writer = csv.writer(self._file)
        self._writer.writerow(HEADERS)

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        log = [
            time,
            self._loop_state.slip_count,
            self._stage_state.stage,
            self._control_state.servo_angle,
            self._predict_state.apogee_prediction,
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
            *self._fusion_state.quaternion,
            *self._fusion_state.euler,
            self._fusion_state.zenith,
            self._bmp390_state.altitude_errors,
            self._bmp390_state.temperature_errors,
            self._bno085_state.acceleration_errors,
            self._bno085_state.magnetic_errors,
            self._bno085_state.gyro_errors,
            self._icm20649_state.acceleration_errors,
            self._icm20649_state.gyro_errors,
        ]
        self._writer.writerow(log)

        # TEMPORARY: Console prints for testing.
        print(time, self._loop_state.slip_count, self._stage_state.stage,
              self._control_state.servo_angle,
              self._predict_state.apogee_prediction,
              self._filter_state.altitude, self._filter_state.velocity[2],
              self._filter_state.acceleration[2],
              self._fusion_state.zenith)
