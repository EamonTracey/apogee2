from dataclasses import dataclass
from math import pi

import numpy as np

from base.component import Component
from base.math import quatern2euler, quatern_prod, quatern_conj
from base.stage import Stage
from flight.blackboard import BNO085State, FusionState, StageState

import logging

logger = logging.getLogger(__name__)


class FusionComponent(Component):

    def __init__(self, bno085_state: BNO085State, stage_state: StageState):
        self._state = FusionState()

        self._stage_state = stage_state
        self._bno085_state = bno085_state

        self._previous_time = 0

    @property
    def state(self):
        return self._state

    # Sensor fusion.
    def teasleyFilter(self, quat, gyro, dt):

        # Temporary restructuring of quaternion tuple to (w,x,y,z)
        quat = (quat[3], quat[0], quat[1], quat[2])

        # Gyroscope interpolation (angular velocity -> change in angular positon)
        omega = (0, gyro[0], gyro[1], gyro[2])
        dq = 0.5 * np.array(quatern_prod(quat, omega))

        # Quaternion with corrected gyro
        qnew = np.array(quat) + dq * dt

        # Normalize
        qnewer = tuple(x / np.linalg.norm(quat)
                       for x in qnew) if np.linalg.norm(quat) != 0 else qnew

        # Restructure quat back to (x,y,z,w)
        qnewest = (qnewer[1], qnewer[2], qnewer[3], qnewer[0])
        return qnewest

    def dispatch(self, time: float):

        # Use BNO085 onboard fusion to gather quaternion data for when not launched
        # and descending.
        if self._stage_state.stage == Stage.GROUND or self._stage_state.stage == Stage.DESCENT:

            # Utilize BNO085 quaternions
            self._state.quaternion = self._bno085_state.quaternion

            # Calculate euler Angles
            self._state.euler = tuple(
                x * (180 / pi)
                for x in quatern2euler(self._bno085_state.quaternion))

            self._previous_time = time

        # At high accelerations, the BNO085's orientation determination is unreliable.
        # For this reason, manual sensor fusion is used.
        # NOTE: Gyro drift starts to kick in pretty heavily after around 15-20 seconds.

        elif self._stage_state.stage == Stage.BURN or self._stage_state.stage == Stage.COAST:

            # Calculate quaternions manually using gyro (fusion)
            self._state.quaternion = self.teasleyFilter(
                self._state.quaternion, self._bno085_state.gyro,
                time - self._previous_time)

            # Calculate euler angles
            self._state.euler = tuple(
                x * (180 / pi) for x in quatern2euler(self._state.quaternion))

            self._previous_time = time

        # Calculate Zenith angle.
        v_up_earth = (0, 0, 1, 0)
        v_rot_earth_calculated = quatern_prod(
            self._state.quaternion,
            quatern_prod(v_up_earth, quatern_conj(self._state.quaternion)))
        # Avoid arccos > 1
        if v_rot_earth_calculated[2] > 1:
            v_rot_earth_calculated = (v_rot_earth_calculated[0],
                                      v_rot_earth_calculated[1], 1,
                                      v_rot_earth_calculated[3])
        if v_rot_earth_calculated[2] < -1:
            v_rot_earth_calculated = (v_rot_earth_calculated[0],
                                      v_rot_earth_calculated[1], -1,
                                      v_rot_earth_calculated[3])

        self._state.zenith = np.degrees(np.arccos(v_rot_earth_calculated[2]))

