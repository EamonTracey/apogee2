from dataclasses import dataclass
import logging

from filterpy.kalman import KalmanFilter
import numpy as np

from base.component import Component
from base.constants import EARTH_GRAVITY_ACCELERATION, METERS_TO_FEET
from base.loop import LoopState
from base.stage import Stage
from flight.bmp390 import BMP390State
from flight.icm20649 import ICM20649State

logger = logging.getLogger(__name__)

# TODO: Implement filtration for 3DOF Mag & ICMGyro/BNOGyro.
# TODO: Continuously zero the altitude at ground.
# TODO: Fix error with x/y accel not being zero at init causing velo to be goofy

@dataclass
class FilterState:
    # Filtered altitude in feet.
    altitude: float = 0

    # Filtered vertical velocity in feet / second.
    velocity: tuple[float, float, float] = (0, 0, 0)

    # Filtered vertical acceleration in feet / second^2.
    acceleration: tuple[float, float, float] = (0, 0, 0)

    # Filtered gyroscope data in radians / second
    gyro: tuple[float, float, float] = (0, 0, 0)

    # Filtered magnetometer data in microteslas
    mag: tuple[float, float, float] = (0, 0, 0)


class FilterComponent(Component):

    def __init__(self, loop_state: LoopState, bmp390_state: BMP390State,
                 icm20649_state: ICM20649State):
        self._state = FilterState()

        self._loop_state = loop_state
        self._bmp390_state = bmp390_state
        self._icm20649_state = icm20649_state

        self._initialize_filters()

        self._previous_time = 0

        self._zero_offset = 0

        logger.info("Kalman Filter Initialized.")

    def _initialize_filters(self):

        self.filter_matrix_list = {}

        self.filter_matrix_list['Zdir'] = [[1, 0, 0], [0, 0, 1]]
        self.filter_matrix_list['Ydir'] = [[0, 0, 1]]
        self.filter_matrix_list['Xdir'] = [[0, 0, 1]]
        self.filter_matrix_list['Gyro'] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

        self.filter_list = {}

        # Z direction filter
        self.filter_list['Zdir'] = KalmanFilter(dim_x=3,
                                                dim_z=len(self.filter_matrix_list['Zdir']))

        # Y direction filter
        self.filter_list['Ydir'] = KalmanFilter(dim_x=3,
                                                dim_z=len(self.filter_matrix_list['Ydir']))

        # X direction filter
        self.filter_list['Xdir'] = KalmanFilter(dim_x=3,
                                                dim_z=len(self.filter_matrix_list['Xdir']))

        # Gyro filter
        self.filter_list['Gyro'] = KalmanFilter(dim_x=3,
                                                dim_z=len(self.filter_matrix_list['Gyro']))

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
     
        # temporary:  zeroing when acs turns on
        #if self._previous_time <= 0.02:
        #     self._zero_offset = METERS_TO_FEET * self._bmp390_state.altitude

        altitude = METERS_TO_FEET * self._bmp390_state.altitude - self._zero_offset

        gyro_x = self._icm20649_state.gyro[0]
        gyro_y = self._icm20649_state.gyro[1]
        gyro_z = self._icm20649_state.gyro[2]

        # Offset the Z acceleration of the accelerometer by gravity.
        acceleration = [
            METERS_TO_FEET * a for a in self._icm20649_state.acceleration
        ]
        acceleration[2] += EARTH_GRAVITY_ACCELERATION

        # TODO: Continuously Zero Altimeter at ground - on init and every minute before burn detected
        # How to do this when cannot access state? good question i dont know

        params_list = {}
        params_list['Zdir'] = np.array([float(altitude), float(acceleration[2])])
        params_list['Ydir'] = np.array([float(acceleration[1])])
        params_list['Xdir'] = np.array([float(acceleration[0])])
        params_list['Gyro'] = np.array([float(gyro_x), float(gyro_y), float(gyro_z)])

        for unit in self.filter_list:
            self.filter_list[unit].F = self._generate_phi(time, unit)
            self.filter_list[unit].predict()
            self.filter_list[unit].update(params_list[unit])

        self._previous_time = time

        # X/Y Velo derivation - currently buggy and bad so no use
        #self._state.altitude = self.filter_list['Zdir'].x[0]
        #self._state.velocity = (self.filter_list['Xdir'].x[1],
        #                        self.filter_list['Ydir'].x[1],
        #                        self.filter_list['Zdir'].x[1])


        self._state.altitude = self.filter_list['Zdir'].x[0]
        self._state.velocity = (0, 0, self.filter_list['Zdir'].x[1])
        self._state.acceleration = (self.filter_list['Xdir'].x[2],
                                    self.filter_list['Ydir'].x[2],
                                    self.filter_list['Zdir'].x[2])        
        self._state.gyro = (self.filter_list['Gyro'].x[0], 
                            self.filter_list['Gyro'].x[1],
                            self.filter_list['Gyro'].x[2])
        
        
    def _generate_phi(self, time: float, unit):
        dt = time - self._previous_time

        if (unit == 'Xdir') or (unit == 'Ydir') or (unit == 'Zdir'):
            dp = 1
            ds = 0
            di = dt**2 / 2       
            phi = np.array([[dp, dt, di], [ds, dp, dt], [ds, ds, dp]])

        elif unit == 'Gyro':
            phi = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

        return phi
