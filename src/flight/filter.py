from collections import deque
from dataclasses import dataclass
import logging

from filterpy.kalman import KalmanFilter
import numpy as np

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

        self.filter_matrix = [[1, 0, 0], [0, 0, 1]]


        self.filter = KalmanFilter(dim_x=3, dim_z=len(self.filter_matrix))

        self.filter.H = np.array(self.filter_matrix)
        self.filter.P *= 1
        self.filter.R *= 1
        self.filter.Q *= 1
        self.filter.x = np.array([0, 0, 0])

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
        if self._stage_state.stage in [Stage.GROUND, Stage.BURN]:
            acceleration = [METERS_TO_FEET * a for a in self._icm20649_state.acceleration]
        else:
            acceleration = [METERS_TO_FEET * a for a in self._bno085_state.acceleration]
            x, y, z = acceleration[0], acceleration[1], acceleration[2]
            acceleration[0] = -y
            acceleration[1] = x
            acceleration[2] = z

        # Calculate gravity vector with world reference frame.
        g_w = np.array([[0], [0], [-EARTH_GRAVITY_ACCELERATION]])

        # Generate rotation matrix.
        r = euler_to_zyx_rotmat(self._fusion_state.euler)

        # Calculate gravity vector with body reference frame and offset.
        g_b = r @ g_w
        acceleration[0] += g_b[0]
        acceleration[1] += g_b[1]
        acceleration[2] += g_b[2]

        params_list = np.array([float(altitude), float(acceleration[2])])

        self.filter.F = self._generate_phi(time)
        self.filter.predict()
        self.filter.update(params_list)

        self._previous_time = time

        self._state.altitude = self.filter.x[0]
        self._state.altitude -= self._ground_altitudes[0]
        self._state.velocity = (0, 0, self.filter.x[1])
        self._state.acceleration = (acceleration[0],
                                    acceleration[1],
                                    self.filter.x[2])

    def _generate_phi(self, time: float):
        dt = time - self._previous_time

        dp = 1
        ds = 0
        di = dt**2 / 2
        phi = np.array([[dp, dt, di], [ds, dp, dt], [ds, ds, dp]])

        return phi
