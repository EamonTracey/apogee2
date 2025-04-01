from dataclasses import dataclass

import numpy as np

from base.component import Component
from base.stage import Stage
from flight.filter import FilterState
from flight.stage import StageState
from flight.bno085 import BNO085State

from base.math import quatern2euler
from base.constants import EARTH_GRAVITY_ACCELERATION

import logging
logger = logging.getLogger(__name__)


@dataclass
class FusionState:

    # Outputs quaternion in (x, y, z, w)
    quaternion: tuple[float, float, float, float] = (0, 0, 0, 0)

    # Outputs euler in (Pitch, Yaw, Roll)
    euler: tuple[float, float, float] = (0, 0, 0)


class FusionComponent(Component):

    def __init__(self, bno085_state: BNO085State, filter_state: FilterState, stage_state: StageState):
        self._state = FusionState()

        self._filter_state = filter_state
        self._stage_state = stage_state
        self._bno085_state = bno085_state

        self._first = True

        self._previous_time = 0
       
    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        
        # Use BNO085 onboard fusion to gather quaternion data for when not launched 
        # and descending.
        if self._stage_state == Stage.GROUND or self._stage_state == Stage.DESCENT:

            self._state.quaternion = self._bno085_state.quaternion

            eulers = quatern2euler(self._bno085_state.quaternion[0],
                                   self._bno085_state.quaternion[1],
                                   self._bno085_state.quaternion[2],
                                   self._bno085_state.quaternion[3])

            self._previous_time = time


        # Use Gyro interpolation for orientation when burn or coast. NOTE: Gyro drift starts to kick
        # in pretty heavily after around 15-20 seconds. Gotta switch back to bno when descening... it's 
        # gonna be unreliable but no other option xd

        # Teasley Algorithm
        else: 
            quat = self._state.quaternion
            gyro = self._filter_state.gyro
            accel = self._filter_state.acceleration
            dt = time - self._previous_time

            quat_calc = madgwickFilter(quat, gyro, accel, dt, 0.025)

            #bruhhhhhhhh

    def madgwickFilter(quat, gyro, accel, dt, beta):

        g = EARTH_GRAVITY_ACCELERATION

        if np.linalg.norm(accel) > 1.15*g:
            gyro_corrected = gyro
            

           

