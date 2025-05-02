from dataclasses import dataclass
import logging
import traceback

import numpy as np

from base.component import Component
from base.stage import Stage
from flight.blackboard import FilterState, FusionState, PredictState, StageState, ControlState
from simulation.dynamics import DynamicsComponent
from simulation.environment import Environment
from simulation.motor import Motor
from simulation.vehicle import Vehicle

logger = logging.getLogger(__name__)


class PredictComponent(Component):

    def __init__(self, filter_state: FilterState, stage_state: StageState,
                 fusion_state: FusionState, control_state: ControlState,
                 vehicle: Vehicle, motor: Motor, environment: Environment):
        self._state = PredictState()

        self._filter_state = filter_state
        self._stage_state = stage_state
        self._fusion_state = fusion_state
        self._control_state = control_state

        self._vehicle = vehicle
        self._motor = motor
        self._environment = environment

        logger.info("Prediction Component Initialized.")

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        if self._stage_state.stage not in [
                Stage.COAST,
                Stage.OVERSHOOT,
        ]:
            return

        # Hack the dynamics component.
        DT = 0.5
        dynamics = DynamicsComponent(self._vehicle, self._motor,
                                     self._environment)
        dynamics.state.time = 1000
        dynamics.state.position = (0, 0, self._filter_state.altitude)
        dynamics.state.velocity = self._filter_state.velocity
        dynamics.state.orientation = self._fusion_state.quaternion[
            3], self._fusion_state.quaternion[
                0], self._fusion_state.quaternion[
                    1], self._fusion_state.quaternion[2]
        dynamics.state.angular_velocity = (0, 0, 0)
        while dynamics.state.velocity[2] > 0:
            dynamics._step(DT)

        self._state.apogee_prediction = dynamics.state.position[2]
        print(self._state.apogee_prediction)
