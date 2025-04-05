from dataclasses import dataclass
import logging

from base.component import Component
from flight.filter import FilterState
from flight.stage import StageState
from flight.fusion import FusionState

from flight.constants import SIM_APOGEE

from simulation.vehicle import Vehicle
from simulation.motor import Motor
from simulation.environment import Environment

logger = logging.getLogger(__name__)


class PredictComponent(Component):

    def __init__(self, filter_state: FilterState, stage_state: StageState,
                 fusion_state: FusionState, vehicle: Vehicle,
                 environment: Environment, motor: Motor):
        self._state = PredictState

        self._filter_state = filter_state
        self._stage_state = stage_state
        self._fusion_state = fusion_state

        self._vehicle = vehicle
        self._environment = environment
        self._motor = motor

        logger.info("Prediction Component Initialized.")

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        pass
