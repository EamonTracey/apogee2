from collections import deque
from dataclasses import dataclass
import logging

from filterpy.kalman import KalmanFilter
import numpy as np
import math

from base.component import Component
from base.constants import EARTH_GRAVITY_ACCELERATION, METERS_TO_FEET
from base.loop import LoopState
from base.stage import Stage
from base.math import euler_to_zyx_rotmat
from flight.blackboard import BMP390State, FilterState, ICM20649State, StageState, BNO085State, FusionState

logger = logging.getLogger(__name__)


class FilterComponent(Component):

    def __init__(self, loop_state: LoopState, bmp390_state: BMP390State,
                 icm20649_state: ICM20649State, bno085_state: BNO085State,
                 stage_state: StageState, fusion_state: FusionState):
        self._state = FilterState()

        self._loop_state = loop_state
        self._bmp390_state = bmp390_state
        self._icm20649_state = icm20649_state
        self._bno085_state = bno085_state
        self._stage_state = stage_state
        self._fusion_state = fusion_state

        self._initialize_filters()

        self._previous_time = 0
        self._ground_altitudes = deque()

        logger.info("Kalman Filter Initialized.")

    def _initialize_filters(self):
        self.filter_matrix_list = {}

        self.filter_matrix_list["Zdir"] = [[1, 0, 0], [0, 0, 1]]
        self.filter_matrix_list["Ydir"] = [[0, 0, 1]]
        self.filter_matrix_list["Xdir"] = [[0, 0, 1]]

        self.filter_list = {}

        # Z direction filter
        self.filter_list["Zdir"] = KalmanFilter(
            dim_x=3, dim_z=len(self.filter_matrix_list["Zdir"]))

        # Y direction filter
        self.filter_list["Ydir"] = KalmanFilter(
            dim_x=3, dim_z=len(self.filter_matrix_list["Ydir"]))

        # X direction filter
        self.filter_list["Xdir"] = KalmanFilter(
            dim_x=3, dim_z=len(self.filter_matrix_list["Xdir"]))

        for unit in self.filter_list:
            self.filter_list[unit].H = np.array(self.filter_matrix_list[unit])
            self.filter_list[unit].P *= 1
            self.filter_list[unit].R *= 1
            self.filter_list[unit].Q *= 1
            self.filter_list[unit].x = np.array([0, 0, 0])

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        altitude = METERS_TO_FEET * self._bmp390_state.altitude

        # We keep the 300 previous altitude readings to have a zero offset.
        if self._stage_state.stage == Stage.GROUND:
            self._ground_altitudes.append(altitude)
            if len(self._ground_altitudes) > 300:
                self._ground_altitudes.popleft()

        # Acceleration vector. 
        acceleration = [
            METERS_TO_FEET * a for a in
            (self._icm20649_state.acceleration if self._stage_state.stage in
             [Stage.GROUND, Stage.BURN] else self._bno085_state.acceleration)
        ]

        # Calculate gravity vector with world reference frame.
        g_w = np.array([[0], [0], [-EARTH_GRAVITY_ACCELERATION]])

        # Generate rotation matrix.
        r = euler_to_zyx_rotmat(self._fusion_state.euler)

        # Calculate gravity vector with body reference frame and offset.
        g_b = r @ g_w
        acceleration[0] -= g_b[1]
        acceleration[1] += g_b[0]
        acceleration[2] += g_b[2] 


        params_list = {}
        params_list["Zdir"] = np.array(
            [float(altitude), float(acceleration[2])])
        params_list["Ydir"] = np.array([float(acceleration[1])])
        params_list["Xdir"] = np.array([float(acceleration[0])])

        for unit in self.filter_list:
            self.filter_list[unit].F = self._generate_phi(time, unit)
            self.filter_list[unit].predict()
            self.filter_list[unit].update(params_list[unit])

        self._previous_time = time

        # X/Y Velo derivation - currently buggy and bad so no use
        #self._state.altitude = self.filter_list["Zdir"].x[0]
        #self._state.velocity = (self.filter_list["Xdir"].x[1],
        #                        self.filter_list["Ydir"].x[1],
        #                        self.filter_list["Zdir"].x[1])

        self._state.altitude = self.filter_list["Zdir"].x[0]
        self._state.altitude -= self._ground_altitudes[0]
        self._state.velocity = (0, 0, self.filter_list["Zdir"].x[1])
        self._state.acceleration = (self.filter_list["Xdir"].x[2],
                                    self.filter_list["Ydir"].x[2],
                                    self.filter_list["Zdir"].x[2])

    def _generate_phi(self, time: float, unit):
        dt = time - self._previous_time

        dp = 1
        ds = 0
        di = dt**2 / 2
        phi = np.array([[dp, dt, di], [ds, dp, dt], [ds, ds, dp]])

        return phi
