from dataclasses import dataclass
import logging

from flight.filter import FilterState

logger = logging.getLogger(__name__)


@dataclass
class PredictState:
    # Predicted apogee in feet.
    apogee_prediction: float = 0


class PredictComponent(Component):

    def __init__(self, filter_state: FilterState):
        self._state = PredictState

        self._filter_state = filter_state

        logger.info("Prediction Component Initialized.")
    
    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):

        pass



