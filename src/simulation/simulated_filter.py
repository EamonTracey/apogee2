from dataclasses import dataclass

from base.component import Component
from base.stage import Stage

from base.loop import LoopState
from simulation.dynamcis import DynamicsState

logger = logging.getLogger(__name__)


@dataclass
class SimulatedFilterState:
    # Filtered altitude in feet.
    altitude: float = 0

    # Filtered vertical velocity in feet / second.
    velocity: tuple[float, float, float] = (0, 0, 0)

    # Filtered vertical acceleration in feet / second^2.
    acceleration: tuple[float, float, float] = (0, 0, 0)


class SimulatedFilterComponent(Component):

    def __init__(self, dynamics_state: DynamicsState):
        self._state = SimulatedFilterState()

        self._dynamics_state = DynamicsState()

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        self._state.altitude = METERS_TO_FEET * self._dynamics_state.altitude
        self._state.velocity = tuple(
            METERS_TO_FEET * lm / self._dynamics_state.mass
            for lm in self._dynamics_state.linear_momentum)
        # self._state.acceleration = (acceleration[0], acceleration[1], self.filter.x[2])
