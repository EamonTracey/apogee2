from dataclass import dataclass

from base.component import Component
from base.stage import Stage

from flight.filter import FilterState

@dataclass
class PhaseState

    # Rocket Phase
    phase: Stage = Stage.GROUND
   
class PhaseComponent(Component):

    def __init__(self, filter_state: FilterState):

        self._state = PhaseState()

        self._filter_state = filter_state

        self._RAIL_ACCELERATION = 25            # feet / second ^ 2
        self._LAUNCH_ALTITUDE = 100             # feet
        self._LAUNCH_ACCELERATION = 200         # feet / second ^ 2
        self._LAUNCH_ALTITUDE_CRITICAL = 300    # feet
        self._BURNOUT_ALTITUDE = 800            # feet
        self._BURNOUT_ACCELERATION = 0          # feet / second ^ 2
        self._BURNOUT_ALTITUDE_CRITICAL = 1200  # feet
        self._APOGEE_ALTITUDE = 5100            # feet
        self._DESCENT_VELOCITY = 0              # feet / second

        logger.info("Phase Determination Initialize")

    def dispatch(self):

        altitude_filtered = self._filter_state.altitude_filtered
        velocity_filtered = self._filter_state.velocity_filtered
        acceleration_filtered = self._filter_state.acceleration_filtered

        # GROUND -> RAIL
        if self._state.phase == State.GROUND:
            if acceleration_filtered > self._RAIL_ACCELERATION:
                self._state.phase = State.RAIL

        # RAIL -> BURN or RAIL -> GROUND
        elif self._state.phase == State.RAIL:
            if (altitude_filtered > self._LAUNCH_ALTITUDE and acceleration_filtered > self._LAUNCH_ACCELERATION) or altitude_filtered > self._LAUNCH_ALTITUDE_CRITICAL:
                self._state.phase = State.BURN
            elif acceleration_filtered < self._RAIL_ACCELERATION:
                self._state.phase = State.GROUND

        # BURN -> COAST
        elif self._state.phase == State.BURN:
            if (self._BURNOUT_ALTITUDE < altitude_filtered and acceleration_filtered < self._BURNOUT_ACCELERATION) or altitude_filtered > self._BURNOUT_ALTITUDE_CRITICAL:
                self._state.phase = State.COAST

#       # COAST -> OVERSHOOT / DESCENT
        elif self._state.phase == State.COAST:
            if altitude_filtered > self._APOGEE_ALTITUDE:
                self._state.phase = State.OVERSHOOT
            elif velocity_filtered < self._DESCENT_VELOCITY:
                self._state.phase = State.DESCENT

        # OVERSHOOT -> DESCENT
        elif self._state.phase == State.DESCENT:
            if velocity_filtered < self._DESCENT_VELOCITY:
                self._state.phase = State.DESCENT


        
