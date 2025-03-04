import csv
from dataclasses import dataclass

from base.component import Component
from simulation.dynamics import DynamicsState

HEADERS = [
    "Time",
    "Stage",
    "Mass",
    "Position_X",
    "Position_Y",
    "Position_Z",
    "Linear_Momentum_X",
    "Linear_Momentum_Y",
    "Linear_Momentum_Z",
    "Orientation_W",
    "Orientation_X",
    "Orientation_Y",
    "Orientation_Z",
    "Angular_Momentum_X",
    "Angular_Momentum_Y",
    "Angular_Momentum_Z",
    "Acceleration_X",
    "Acceleration_Y",
    "Acceleration_Z",
]


@dataclass
class LogState:
    ...


class LogComponent(Component):

    def __init__(self, path: str, dynamics_state: DynamicsState):
        self._state = LogState()

        self._path = path
        self._dynamics_state = dynamics_state

        self._file = open(self._path, "w")
        self._writer = csv.writer(self._file)
        self._writer.writerow(HEADERS)

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        log = [
            self._dynamics_state.time,
            self._dynamics_state.stage,
            self._dynamics_state.mass,
            self._dynamics_state.position[0],
            self._dynamics_state.position[1],
            self._dynamics_state.position[2],
            self._dynamics_state.linear_momentum[0],
            self._dynamics_state.linear_momentum[1],
            self._dynamics_state.linear_momentum[2],
            self._dynamics_state.orientation[0],
            self._dynamics_state.orientation[1],
            self._dynamics_state.orientation[2],
            self._dynamics_state.orientation[3],
            self._dynamics_state.angular_momentum[0],
            self._dynamics_state.angular_momentum[1],
            self._dynamics_state.angular_momentum[2],
            self._dynamics_state.acceleration[0],
            self._dynamics_state.acceleration[1],
            self._dynamics_state.acceleration[2],
        ]
        self._writer.writerow(log)
