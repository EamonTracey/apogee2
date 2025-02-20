from dataclasses import dataclass

from base.component import Component
from base.stage import Stage
import logging

from flight.active_states import FilterState
from flight.active_states import PhaseState

logger = logging.getLogger(__name__)

   
class PhaseComponent(Component):

    def __init__(self, filter_state: FilterState): 

        self._state = PhaseState()

        self._filter_state = filter_state

        self._RAIL_ORIENTATION = 5              # degrees (not implemented)
        self._LAUNCH_ALTITUDE = 100             # feet
        self._LAUNCH_ACCELERATION = 200         # feet / second ^ 2
        self._LAUNCH_ALTITUDE_CRITICAL = 300    # feet
        self._BURNOUT_ALTITUDE = 800            # feet
        self._BURNOUT_ACCELERATION = 0          # feet / second ^ 2
        self._BURNOUT_ALTITUDE_CRITICAL = 1200  # feet
        self._APOGEE_ALTITUDE = 5100            # feet
        self._DESCENT_VELOCITY = 0              # feet / second

        logger.info("Phase Determination Initialized.")

    @property
    def state(self):
        return self._state

    def dispatch(self):

        altitude_filtered = self._filter_state.altitude_filtered
        velocity_filtered = self._filter_state.velocity_filtered
        acceleration_filtered = self._filter_state.acceleration_filtered

        # GROUND -> BURN (rail takes orientation which uh isn't ready yet teehee)
        if self._state.phase == Stage.GROUND:
            if (altitude_filtered > self._LAUNCH_ALTITUDE and acceleration_filtered > self._LAUNCH_ACCELERATION) or altitude_filtered > self._LAUNCH_ALTITUDE_CRITICAL:
                self._state.phase = Stage.BURN

        # BURN -> COAST
        elif self._state.phase == Stage.BURN:
            if (self._BURNOUT_ALTITUDE < altitude_filtered and acceleration_filtered < self._BURNOUT_ACCELERATION) or altitude_filtered > self._BURNOUT_ALTITUDE_CRITICAL:
                self._state.phase = Stage.COAST

#       # COAST -> OVERSHOOT / DESCENT
        elif self._state.phase == Stage.COAST:
            if altitude_filtered > self._APOGEE_ALTITUDE:
                self._state.phase = Stage.OVERSHOOT
            elif velocity_filtered < self._DESCENT_VELOCITY:
                self._state.phase = Stage.DESCENT

        # OVERSHOOT -> DESCENT
        elif self._state.phase == Stage.DESCENT:
            if velocity_filtered < self._DESCENT_VELOCITY:
                self._state.phase = Stage.DESCENT


        
