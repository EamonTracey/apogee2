import csv
from dataclasses import dataclass

from base.component import Component
from base.loop import LoopState

from flight.blackboard import (ControlState, FilterState,
                               FusionState, PredictState, 
                               StageState, LogState)

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
        "Quaternion_X_Fused",
        "Quaternion_Y_Fused",
        "Quaternion_Z_Fused",
        "Quaternion_W_Fused",
        "Euler_Fused_Roll",
        "Euler_Fused_Yaw",
        "Euler_Fused_Pitch",
        "Euler_Fused_Zenith"
        ]

class LogReplayComponent(Component):
    
    def __init__(self, path: str, loop_state: LoopState,
                filter_state: FilterState, control_state: ControlState,
                fusion_state: FusionState, predict_state: PredictState,
                stage_state: StageState):
        
        self._state = LogState()

        self._path = path
        self._fusion_state = fusion_state
        self._filter_state = filter_state
        self._predict_state = predict_state
        self._control_state = control_state
        self._stage_state = stage_state
        self._loop_state = loop_state

        self._file = open(self._path, 'w')
        self._writer = csv.writer(self._file)
        self._writer.writerow(HEADERS)

    @property
    def state(self):
        return self._state

    def dispatch(self, time:float):
        log = [
                time,
                self._loop_state.slip_count,
                self._stage_state.stage,
                self._control_state.servo_angle,
                self._predict_state.apogee_prediction,
                self._filter_state.altitude,
                *self._filter_state.velocity,
                *self._filter_state.acceleration,
                *self._fusion_state.quaternion,
                *self._fusion_state.euler,
                self._fusion_state.zenith
                ]
        self._writer.writerow(log)

        print(time, self._loop_state.slip_count,
              self._stage_state.stage,
              self._control_state.servo_angle,
              self._predict_state.apogee_prediction,
              self._filter_state.altitude,
              self._filter_state.velocity[2],
              self._filter_state.acceleration[2],
              self._fusion_state.zenith)

                
    
