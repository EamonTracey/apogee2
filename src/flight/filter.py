from dataclasses import dataclass

from filterpy.kalman import KalmanFilter
import numpy as np
import logging
import time

from base.component import Component 
from base.stage import Stage

from base.loop import LoopState
from flight.bmp390 import BMP390State
from flight.icm20649 import ICM20649State
from flight.active_states import PhaseState
from flight.active_states import FilterState

logger = logging.getLogger(__name__)


##TODO: Implement filtration for X,Y Accel and 3DOF Mag & Gyro

class FilterComponent(Component):

    def __init__(self, loop_state: LoopState, bmp390_state: BMP390State,
                 icm_state: ICM20649State, phase_state: PhaseState,
                 vertical):

        self._state = FilterState()

        self._loop_state = loop_state
        self._bmp390_state = bmp390_state
        self._icm_state = icm_state
        self._phase_state = phase_state

        self._dt = 0
        self._prev_time = -0.05
        self._altitude_offset = 0

        self._initialize_filter()

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

        # Offset due to initial height and perioditcally zero altimeter
        if (self._phase_state.phase == Stage.GROUND) and ((self._loop_state.time - self._loop_state.first_time) % self._loop_state.zero_period < 2 / self._loop_state.frequency):
            self._altitude_offset = self._bmp390_state.altitude

        altitude = self._meters_to_feet(self._bmp390_state.altitude - self._altitude_offset)
    
        # Offset due to gravity
        acceleration = self._meters_to_feet(self._icm_state.acceleration[2] - 9.80665)

        measurements = [float(altitude), float(acceleration)]

        params = np.array(measurements)

        self.filter.F = self._generate_phi()

        self.filter.predict()
        self.filter.update(params)

        self._state.altitude_filtered, self._state.velocity_filtered, self._state.acceleration_filtered = self.filter.x

    def _generate_phi(self):
        self._dt = self._loop_state.time - self._prev_time

        dp = 1
        ds = 0
        di = (self._dt ** 2) / 2

        phi = np.array([
            [dp, self._dt, di],
            [ds, dp, self._dt],
            [ds, ds, dp]])

        self._prev_time = self._loop_state.time

        return phi

    def _meters_to_feet(self, val):
        return val * 3.28084

