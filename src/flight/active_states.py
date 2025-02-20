from dataclasses import dataclass

from base.stage import Stage
from base.component import Component

import logging

logger = logging.getLogger(__name__)

@dataclass
class PhaseState:
    phase: Stage = Stage.GROUND

@dataclass
class FilterState:

    # filtered altitude in feet.
    altitude_filtered: float = 0

    # filtered vertical velocity in feet/s.
    velocity_filtered: float = 0

    # filtered vertical acceleration in feet/s^2.
    acceleration_filtered: float = 0


class ActiveStateComponent(Component):
    def __init__(self, ):
        self.phase_state = PhaseState()
        self.filter_state = FilterState()

        logger.info("Initialized Active State Component")
    
    @property
    def get_phase_state(self):
        return self.phase_state

    @property
    def get_filter_state(self):
        return self.filter_state

    def dispatch(self):
        pass

        

    

