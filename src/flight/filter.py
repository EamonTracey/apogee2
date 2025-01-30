from dataclasses import dataclass

from filterpy.kalman import KalmanFilter
import numpy as np
import time

from base.component import Component 

from base.loop import LoopState
from flight.bmp390 import BMP390State
from flight.icm20649 import ICM20649State

logger = logging.getLogger(__name__)

##TODO: Implement filtration for X,Y Accel and 3DOF Mag & Gyro

@dataclass
class FilterState:

    # Filtered altitude in meters.
    altitude_filtered: float = 0

    # Filtered vertical velocity in meters.
    velocity_filtered: float = 0

    # Filtered vertical acceleration in meters.
    acceleration_filtered: float = 0

class FilterComponent(Component):

    def __init__(self, loop_state: LoopState, bmp390_state: BMP390State,
                 icm_state: ICM20649State, vertical):

        self._state = FilterState()

        self._loop_state = loop_state
        self._bmp390_state = bmp390_state
        self._icm_state = icm_state

        self._dt = 0

        logger.info("Kalman Filter Initialized.")

    def _initialize_filter(self):
        sensor_matrix = [
                [1, 0, 0],
                [0, 0, 1]]

        self.filter = KalmanFilter(dim_x=3, dim_z=len(sensor_matrix))
        self.filter.H = np.array(sensor_matrix)

        self.filter.P *= 1
        self.filter.R *= 1
        self.filter.Q *= 1
        self.filter.x = np.array([0, 0, 0])

    @property
    def state(self):
        return self._state

    def dispatch(self):
        altitude = self._bmp390_state.altitude
        acceleration = self._icm_state.acceleration

        measurements = [float(altitude), float(acceleration)]

        params = np.array(measurements)

        self.filter.F = self._generate_phi()

        self.filter.predict()
        self.filter.update(params)

        self._state.filtered_altitude, self._state.filtered_velocity, self._state.filtered_acceleration = self.filter.x

    def _generate_phi(self):
        self._dt = self._loop_state.time - self._loop_state.first_time

        dp = 1
        ds = 0
        di = (self._dt ** 2) / 2

        phi = np.array([
            [dp, self._dt, di],
            [ds, dp, self._dt],
            [ds, ds, dp]])
    
        return phi

