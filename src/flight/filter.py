from dataclasses import dataclass

from filterpy import KalmanFilter
import numpy as np

from base.component import Component 

logger = logging.getLogger(__name__)

@dataclass
class FilterState:

    # Filtered altitude in meters.
    filtered_altitude: float = 0

    # Filtered vertical velocity in meters.
    filtered_z_velocity: float = 0

    # Filtered vertical acceleration in meters.
    filtered_z_acceleration: float = 0

    # Errors.
    filter_errors: int = 0

class FilterComponent(Component):

    def __init__(self):

        self._state = FilterState()

        self._dt = 0
        self._t_prev = None
        self._initialize_filter()

        logger.info("Kalman Filter Initialized.")

    @property
    def state(self):
        return self._state

    def dispatch(self):
        pass

