from dataclasses import dataclass

from base.component import Component
from base.stage import Stage
import logging

from flight.filter import FilterState
from flight.constants import (
    LAUNCH_ALTITUDE,
    LAUNCH_ACCELERATION,
    LAUNCH_ALTITUDE_CRITICAL,
    BURNOUT_ALTITUDE,
    BURNOUT_ACCELERATION,
    BURNOUT_ALTITUDE_CRITICAL,
    APOGEE_ALTITUDE,
    DESCENT_VELOCITY,
)

logger = logging.getLogger(__name__)


@dataclass
class StageState:
    stage: Stage = Stage.GROUND


class StageComponent(Component):

    def __init__(self, filter_state: FilterState):
        self._state = StageState()
        self._filter_state = filter_state

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        altitude = self._filter_state.altitude
        velocity = self._filter_state.velocity
        acceleration = self._filter_state.acceleration

        # GROUND -> BURN.
        if self._state.stage == Stage.GROUND:
            if (altitude > LAUNCH_ALTITUDE
                    and acceleration > LAUNCH_ACCELERATION
                ) or altitude > LAUNCH_ALTITUDE_CRITICAL:
                self._state.stage = Stage.BURN

        # BURN -> COAST.
        elif self._state.stage == Stage.BURN:
            if (BURNOUT_ALTITUDE < altitude
                    and acceleration < BURNOUT_ACCELERATION
                ) or altitude > BURNOUT_ALTITUDE_CRITICAL:
                self._state.stage = Stage.COAST

        # COAST -> OVERSHOOT or DESCENT.
        elif self._state.stage == Stage.COAST:
            if altitude > APOGEE_ALTITUDE:
                self._state.stage = Stage.OVERSHOOT
            elif velocity < DESCENT_VELOCITY:
                self._state.stage = Stage.DESCENT

        # OVERSHOOT -> DESCENT
        elif self._state.stage == Stage.DESCENT:
            if velocity < DESCENT_VELOCITY:
                self._state.stage = Stage.DESCENT
