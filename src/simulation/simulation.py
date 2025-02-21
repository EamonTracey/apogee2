import logging
from typing import Optional

import board
import busio

from base.loop import Loop
from flight.stage import StageComponent

logger = logging.getLogger(__name__)


class Simulation:

    def __init__(self, name: str):
        self.loop = Loop(30)
        loop_state = self.loop.state

        # Simulated filter component.
        simulated_filter_component = SimulatedFilterComponent(loop_state)
        simulated_filter_state = simulated_filter_component.state
        self.loop.add_component(simulated_filter_component, 30)

        # Stage Determination.
        stage_component = StageComponent(filter_state)
        stage_state = stage_component.state
        self.loop.add_component(stage_component, 30)

    def run(self):
        self.loop.run(0)
