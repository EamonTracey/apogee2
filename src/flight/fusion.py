from dataclasses import dataclass

import numpy as np

from base.component import Component
from flight.filter import FilterState
from flight.stage import StageState

from base.math import quatern2euler

import logging
logger = logging.getLogger(__name__)


@dataclass
class FusionState:

    # Outputs euler in Pitch, Yaw, Roll. 
    euler: tuple[float, float, float] = 0


class FusionComponent(Component):

    def __init__(self, filter_state: FilterState, stage_state = StageState):
        self._state = FusionState()

        self._filter_state = filter_state
        self._stage_state = stage_state
       
    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        pass 
