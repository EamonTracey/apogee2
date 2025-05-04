from collections import deque
from dataclasses import dataclass
import logging

from filterpy.kalman import KalmanFilter
import numpy as np
from scipy.spatial.transform import Rotation

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

        # CONVERT SENSOR FRAME TO BODY FRAME
        x, y, z = acceleration[0], acceleration[1], acceleration[2]
        acceleration[0] = z
        acceleration[1] = y
        acceleration[2] = x

        # CONVERT BODY FRAME TO GLOBAL FRAME
        zenith = self._fusion_state.zenith
        r = Rotation.from_euler("zyz", [0, zenith, 0],
                                degrees=True) * Rotation.from_euler(
                                    "y", -90, degrees=True)
        acceleration = r.apply(acceleration)

        # REMOVE GRAVITY
        acceleration[2] -= EARTH_GRAVITY_ACCELERATION

        # print(float(acceleration[2]))

        params = np.array([float(altitude), float(acceleration[2])])

        self.filter.F = self._generate_phi(time)
        self.filter.predict()
        self.filter.update(params)


        self._state.altitude = self.filter.x[0]
        self._state.altitude -= self._ground_altitudes[0]
        self._state.velocity = (0, 0, self.filter.x[1])
        self._state.acceleration = (acceleration[0],
                                    acceleration[1],
                                    self.filter.x[2])
        #print(self._state.altitude)
        # print(self._state.velocity[2])

        # SNAG x,y VELOCITY GLOBAL
        dt = time - self._previous_time
        if self._stage_state.stage in [
                Stage.COAST,
                Stage.OVERSHOOT,
        ]:
            vx = self._state.velocity[0] + acceleration[0] * dt
            vy = self._state.velocity[1] + acceleration[1] * dt
            self._state.velocity = (vx, vy, self._state.velocity[2])

        self._previous_time = time

    def _generate_phi(self, time: float):
        dt = time - self._previous_time

        dp = 1
        ds = 0
        di = dt**2 / 2
        phi = np.array([[dp, dt, di], [ds, dp, dt], [ds, ds, dp]])

        return phi
