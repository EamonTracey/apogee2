from dataclasses import dataclass

from math import pi
import numpy as np

from base.component import Component
from base.stage import Stage
from flight.filter import FilterState
from flight.stage import StageState
from flight.bno085 import BNO085State
from flight.icm20649 import ICM20649State

from base.math import quatern2euler
from base.math import quatern_prod
from base.math import quatern_conj
from base.constants import EARTH_GRAVITY_ACCELERATION

import logging
logger = logging.getLogger(__name__)


@dataclass
class FusionState:

    # Outputs quaternion in (x, y, z, w)
    quaternion: tuple[float, float, float, float] = (0, 0, 0, 0)

    # Outputs euler in (Roll, Yaw, Pitch)
    euler: tuple[float, float, float] = (0, 0, 0)

    # Outputs zenith angle
    zenith: float = 0


class FusionComponent(Component):

    def __init__(self, bno085_state: BNO085State, icm20649_state: ICM20649State, filter_state: FilterState, stage_state: StageState):
        self._state = FusionState()

        self._filter_state = filter_state
        self._stage_state = stage_state
        self._bno085_state = bno085_state
        self._icm20649_state = icm20649_state
        
        # remove after testing - ensures a few seconds when ACS turns on of bno085 quaternions so it can normalize.
        # won't be necessary in flight.
        self._time_threshhold = 4 

        self._previous_time = 0
       
    @property
    def state(self):
        return self._state


    # Sensor fusion.
    def teasleyFilter(self, quat, gyro, accel, dt, beta):

        g = EARTH_GRAVITY_ACCELERATION

        # Temporary restructuring of quaternion tuple for use in this 
        quat = (quat[3], quat[0], quat[1], quat[2])

        # Add back gravity into filtered acceleration (needed for quat)
        accel = np.array([accel[0], accel[1], accel[2]])
        
        # Gyroscope interpolation (angular velocity -> change in angular positon
        # If acceleration is 1.15*g <- hard to extrapolate velocity vector
        if np.linalg.norm(accel) > 1.15*g:
            gyro_corrected = gyro
            omega = (0, gyro_corrected[0], gyro_corrected[1], gyro_corrected[2])
            dq = 0.5 * np.array(quatern_prod(quat, omega))

        else:
            # Normalize acceleration
            accel = accel / np.linalg.norm(accel) if np.linalg.norm(accel) != 0 else accel

            # Quaternion elements
            qw = quat[0]
            qx = quat[1]
            qy = quat[2]
            qz = quat[3]
 
            # Estimate gravity direction from quaternion
            g_dir = (2*(qx*qz - qw*qy), 2*(qw*qx + qy*qz), qw**2 - qx**2 - qy**2 + qz**2)
            
            # Gradient descent error (cross measured and estimated gravity)
            error = np.cross(np.array(g_dir), accel)

            # Convert error into quaternion derivative (scaled)
            dq = 0.5 * np.array(quatern_prod(quat, (0, gyro[0], gyro[1], gyro[2])))
            correction = np.array([0, error[0], error[1], error[2]])
            dq = dq - beta * correction
       
        qnew = np.array(quat) + dq * dt
        quatnorm = np.linalg.norm(quat)
        qnewer = tuple(x / quatnorm for x in qnew) if quatnorm != 0 else qnew

        # Restructure quat 
        qnewest = (qnewer[1], qnewer[2], qnewer[3], qnewer[0])
        return qnewest

    def dispatch(self, time: float):
        
        # Use BNO085 onboard fusion to gather quaternion data for when not launched 
        # and descending.
        if self._time_threshhold > time or self._stage_state.stage == Stage.DESCENT:
            
            self._state.quaternion = self._bno085_state.quaternion
            
            # Calculate Euler Angles
            euler_calculated = tuple(x * (180/pi) for x in quatern2euler(self._bno085_state.quaternion))
                
            self._previous_time = time

        # At high accelerations, the BNO085's orientation determination is unreliable.
        # For this reason, manual sensor fusion is used.
        # NOTE: Gyro drift starts to kick in pretty heavily after around 15-20 seconds.

        elif self._stage_state.stage == Stage.GROUND or self._stage_state.stage == Stage.BURN or self._stage_state.stage == Stage.COAST: 
            quat = self._state.quaternion
            gyro = self._filter_state.gyro
            accel = self._icm20649_state.acceleration
            dt = time - self._previous_time

            # Calculate quaternions manually using gyro and acceleration (fusion)
            quat_calc = self.teasleyFilter(quat, gyro, accel, dt, 0.025)

            self._state.quaternion = quat_calc
            
            # Calculate euler angles
            euler_calculated = tuple(x * (180/pi) for x in quatern2euler(quat_calc))

            self._previous_time = time


        self._state.euler = euler_calculated

        # Calculate Zenith angle.
        v_up_earth = (0, 0, 1, 0)
        v_rot_earth_calculated = quatern_prod(self._state.quaternion, 
                                              quatern_prod(v_up_earth, 
                                                           quatern_conj(self._state.quaternion)))
        
        zenith_calculated = np.degrees(np.arccos(v_rot_earth_calculated[2]))

        self._state.zenith = zenith_calculated

        # If pitch angle is between -1 and 1 degrees or btwn -179 and 179 the zenith angle nan's, likely
        # due to gimbal lock-y stuff.. This is a manual override for those circumstances
        if (self._state.euler[2] <= 1 and self._state.euler[2] >= -1):
            self._state.zenith = 0

        elif self._state.euler[2] > 179 or self._state.euler[2] < -179:
            self._state.zenith = 179
            


            

           

