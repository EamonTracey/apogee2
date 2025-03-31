from dataclasses import dataclass

import numpy as np

from base.component import Component
from bmp390 import BMP390State
from icm20948 import ICM20948State


@dataclass
class FusionState:
    quaternion: tuple[float, float, float, float] = 0
    euler: tuple[float, float, float] = 0


class FusionComponent(Component):

    def __init__(self, imu_state: ICM20948State):
        self._state = FusionState()

        # Set up AHRS for imufusion
        self.samplerate = 60  # Hz

        # Instantiate Algorithms
        self.offset = imufusion.Offset(samplerate)
        self.ahrs = imufusion.Ahrs()

        # Will need to be fucked with depending on what is needed for the imu we use
        self.ahrs.settings = imufusion.Settings(
            imufusion.CONVENTION_NWU,
            0.5,  # gain
            2000,  # gyroscope range
            10,  # acceleration rejection
            10,  # magnetic rejection
            5 * samplerate  # recovery trigger period = 5 seconds
        )

        self.delta_time = 1 / 60
        self.euler = np.empty(1, 3)
        self.internal_state = np.empty(1, 6)
        self.flags = np.empty(1, 4)

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):

        accel = list(imu_state.acceleration) / 9.807
        gyro = list(imu_state.gyro)
        mag = list(imu_state.magnetic)

        gyro = self.offset.update(gyro)

        self.ahrs.update(gyro, accel, mag, delta_time)

        thiseuler = self.ahrs.quaternion.to_euler()
        self._state.euler = thiseuler

        self.euler = np.append(self.euler, thiseuler, axis=0)

        ahrsinternalstates = self.ahrs.internal_states
        self.internal_state = np.append(
            self.internal_state,
            np.array([
                ahrsinternalstates.acceleration_error,
                ahrsinternalstates.accelerometer_ignored,
                ahrsinternalstates.acceleration_recovery_trigger,
                ahrsinternalstates.magnetic_error,
                ahrsinternalstates.magnetometer_ignored,
                ahrsinternalstates.magnetic_recovery_trigger,
            ]),
            axis=0)

        ahrsflags = self.ahrs.flags
        self.flags = np.append(self.flags,
                               np.array([
                                   ahrsflags.initialising,
                                   ahrsflags.angular_rate_recovery,
                                   ahrsflags.acceleration_recovery,
                                   ahrsflags.magnetic_recovery,
                               ]),
                               axis=0)
