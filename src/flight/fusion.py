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

    # Outputs euler in (Yaw, Pitch, Roll)
    euler: tuple[float, float, float] = (0, 0, 0)

    # Outputs zenith angle
    zenith: float = 0


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
            
            # Calculate Euler Angles
            euler_calculated = quatern2euler(self._bno085_state.quaternion)

            self._previous_time = time

        # At high accelerations, the BNO085's orientation determination is unreliable.
        # For this reason, manual sensor fusion is used.
        # NOTE: Gyro drift starts to kick in pretty heavily after around 15-20 seconds.

        else: 
            quat = self._state.quaternion
            gyro = self._filter_state.gyro
            accel = self._filter_state.acceleration
            dt = time - self._previous_time

            # Calculate quaternions manually using gyro and acceleration (fusion)
            quat_calc = madgwickFilter(quat, gyro, accel, dt, 0.025)

            self._state.quaternion = quat_calc
            
            # Calculate euler angles
            euler_calculated = tuple(x * (180/pi) for x in quatern2euler(quat_calc))

        self._state.euler = euler_calculated

        # Calculate Zenith angle.
        v_up_earth = (0, 0, 0, 1)
        v_rot_earth_calculated = quatern_prod(self._state.quaternion, 
                                              quatern_prod(v_up_earth, 
                                                           quatern_conj(self._state.quaternion)))
        zenith_calculated = np.degrees(np.arccos(v_rot_earth_calculated[3]))

        self._state.zenith = zenith_calculated

    # Sensor fusion.
    def madgwickFilter(quat, gyro, accel, dt, beta):

        g = EARTH_GRAVITY_ACCELERATION

        # Add back gravity into filtered acceleration (needed for quat)
        accel = np.array([accel[0], accel[1], accel[2] + g])
        
        # Gyroscope interpolation (angular velocity -> change in angular positon
        # If acceleration is 1.15*g <- hard to extrapolate velocity vector
        if np.linalg.norm(accel) > 1.15*g:
            gyro_corrected = gyro
            omega = (gyro_corrected[0], gyro_corrected[1], gyro_corrected[2], 0)
            dq = 0.5 * np.array(quatern_prod(quat, omega))

        else:
            # Normalize acceleration
            accel = accel / np.linalg.norm(accel)

            # Quaternion elements
            qx = q[0]
            qy = q[1]
            qz = q[2]
            qw = q[3]

            # Estimate gravity direction from quaternion
            g_dir = (2*(qx*qz - qw*qy), 2*(qw*qx + qy*qz), qw**2 - qx**2 - qy**2 + qz**2)
            
            # Gradient descent error (cross measured and estimated gravity)
            error = np.cross(np.array(g_dir), np.array(accel))

            # Convert error into quaternion derivative (scaled)
            dq = 0.5 * np.array(quatern_prod(quat, (gyro[0], gyro[1], gyro[2], 0)))
            correction = np.array([error[0], error[1], error[2], 0])
            dq = dq - beta * correction

        qnew = np.array(q) + dq * dt
        qnewer = qnew / norm(q)

        return tuple(qnewer)
            
            


            

           

