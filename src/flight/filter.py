from dataclasses import dataclass
import logging

from filterpy.kalman import KalmanFilter
import numpy as np

from base.component import Component
from base.constants import EARTH_GRAVITY_ACCELERATION
from base.loop import LoopState
from base.stage import Stage
from flight.bmp390 import BMP390State
from flight.icm20649 import ICM20649State

logger = logging.getLogger(__name__)

# TODO: Implement filtration for X,Y Accel and 3DOF Mag & Gyro.
# TODO: Continuously zero the altitude at ground.


@dataclass
class FilterState:
    # Filtered altitude in feet.
    altitude: float = 0

    # Filtered vertical velocity in feet / second.
    velocity: tuple[float, float, float] = (0, 0, 0)

    # Filtered vertical acceleration in feet / second^2.
    acceleration: tuple[float, float, float] = (0, 0, 0)


class FilterComponent(Component):

    def __init__(self, loop_state: LoopState, bmp390_state: BMP390State,
                 icm_state: ICM20649State):
        self._state = FilterState()

        self._loop_state = loop_state
        self._bmp390_state = bmp390_state
        self._icm_state = icm_state

        self._initialize_filter()

        self._previous_time = 0

        logger.info("Kalman Filter Initialized.")

    def _initialize_filter(self):
        sensor_matrix = [[1, 0, 0], [0, 0, 1]]

        self.filter = KalmanFilter(dim_x=3, dim_z=len(sensor_matrix))
        self.filter.H = np.array(sensor_matrix)

        self.filter.P *= 1
        self.filter.R *= 1
        self.filter.Q *= 1
        self.filter.x = np.array([0, 0, 0])

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        altitude = METERS_TO_FEET * self._bmp390_state.altitude

        # Offset the Z acceleration of the accelerometer by gravity.
        acceleration = tuple(METERS_TO_FEET * a for a in acceleration)
        acceleration[2] -= METERS_TO_FEET * EARTH_GRAVITY_ACCELERATION

        measurements = [float(altitude), float(acceleration[2])]
        params = np.array(measurements)

        self.filter.F = self._generate_phi(time)
        self.filter.predict()
        self.filter.update(params)

        self._state.altitude = self.filter.x[0]
        self._state.velocity = (0, 0, self.filter.x[1])
        self._state.acceleration = (acceleration[0], acceleration[1], self.filter.x[2])

    def _generate_phi(self, time: float):
        dt = time - self._previous_time
        self._previous_time = time

        dp = 1
        ds = 0
        di = dt**2 / 2
        phi = np.array([[dp, dt, di], [ds, dp, dt], [ds, ds, dp]])

        return phi
