from dataclasses import dataclass

import adafruit_bno08x
import adafruit_bno08x.i2c
import busio

from base.component import Component

import numpy as np


@dataclass
class BNO085State:
    acceleration: tuple[float, float, float] = (0, 0, 0)
    magnetic: tuple[float, float, float] = (0, 0, 0)
    euler: tuple[float, float, float] = (0, 0, 0)
    gyro: tuple[float, float, float] = (0, 0, 0)
    quaternion: tuple[float, float, float, float] = (0, 0, 0, 0)

    acceleration_readings: int = 0
    magnetic_readings: int = 0
    euler_readings: int = 0
    gyro_readings: int = 0
    quaternion_readings: int = 0


class BNO085Component(Component):

    def __init__(self, i2c: busio.I2C, address: int = 0x4a):
        self._state = BNO085State()

        self._bno085 = adafruit_bno08x.i2c.BNO08X_I2C(i2c, address)
        self._bno085.initialize()
        self._bno085.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
        self._bno085.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
        self._bno085.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
        self._bno085.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
        self._bno085.begin_calibration()

    @property
    def state(self):
        return self._state

    def dispatch(self):
        #getting data
        try:
            (acc_x, acc_y, acc_z) = self.sensor.acceleration()
            (gyro_x, gyro_y, gyro_z) = self.sensor.gyro()
            #(euler_x, euler_y, euler_z) = self.sensor.euler()
            (mag_x, mag_y, mag_z) = self.sensor.magnetic()
            (quaternion_w, quaternion_x, quaternion_y, quaternion_z) = self.sensor.quaternion()

        #writing if valid
        if (acc_x and acc_y and acc_z):
            self._state.acceleration = (acc_x, acc_y, acc_z)
            self._state.acceleration_readings += 1

        if (gyro_x and gyro_y and gyro_z):
            self._state.gyro = (gyro_x, gyro_y, gyro_z)
            self._state.gyro_readings += 1

        #not reading euler because can calculate it from quaternion
        """
        if (euler_x and euler_y and euler_z):
            self._state.euler = (euler_x, euler_y, euler_z)
            self._state.euler += 1
        """

        if (mag_x and mag_y and mag_z):
            self._state.magnetic = (mag_x, mag_y, mag_z)
            self._state.magnetic_readings += 1

        if (quaternion_x and quaternion_y and quaternion_z):
            self._state.quaternion = (quaternion_w, quaternion_x, quaternion_y, quaternion_z)
            #euler angle from quaternion!
            self._state.euler = quatern2euler((quaternion_w, quaternion_x, quaternion_y, quaternion_z))
            self._state.quaternion_readings += 1

        except:



def quatern2euler(quaternion_w, quaternion_x, quaternion_y, quaternion_z):
    """
    Converts a quaternion orientation to ZYX Euler angles.
    
    Parameters:
    q : numpy array of shape (N, 4)
        Array of quaternions, where each quaternion is represented as [qw, qx, qy, qz].

    Returns:
    euler : numpy array of shape (N, 3)
        Array of Euler angles [phi, theta, psi] for each quaternion.

    COPIED FROM ZEBNERS MATLAB CODE WHICH IS BASED OFF WIKI for rotation matrix
    """

    # Precompute elements of the rotation matrix
    R11 = 2 * quaternion_w**2 - 1 + 2 * quaternion_x**2
    R21 = 2 * (quaternion_x * quaternion_y - quaternion_w * quaternion_z)
    R31 = 2 * (quaternion_x * quaternion_z + quaternion_w * quaternion_y)
    R32 = 2 * (quaternion_y * quaternion_z - quaternion_w * quaternion_x)
    R33 = 2 * quaternion_w**2 - 1 + 2 * quaternion_z**2

    # Compute Euler angles
    phi = np.arctan2(R32, R33)
    theta = -np.arctan(R31 / np.sqrt(1 - R31**2))
    psi = np.arctan2(R21, R11)

    return (phi, theta, psi)