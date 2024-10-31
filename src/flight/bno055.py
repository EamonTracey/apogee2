from dataclasses import dataclass


from base.component import Component
import adafruit_bno055
import busio

@dataclass
class BNO055State:
    acceleration: Tuple[float, float, float]
    temperature: float = 0

class BNO055Component(Component):
    def __init__(self, i2c: busio.I2C, address: int = ):
        self._state = BNO055State()
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)

    @property
    def state(self):
        return self._state

    def dispatch(self):
        #getting values
        temperature = self.sensor.temperature()
        (acc_x, acc_y, acc_z) = self.sensor.acceleration()
        (gyro_x, gyro_y, gyro_z) = self.sensor.gyro()
        (mag_x, mag_y, mag_z) = self.sensor.magnetic()
        (euler_x, euler_y, euler_z) = self.sensor.euler()

        #writing if valid
        if temperature:
            self._state.temperature = temperature

        if (acc_x and acc_y and acc_z):
            self._state.acceleration = (acc_x, acc_y, acc_z)

        if (gyro_x and gyro_y and gyro_z):
            self._state.acceleration = (gyro_x, gyro_y, gyro_z)

        if (mag_x and mag_y and mag_z):
            self._state.acceleration = (mag_x, mag_y, mag_z)

        if (euler_x and euler_y and euler_z):
            self._state.acceleration = (euler_x, euler_y, euler_z)
        
  
        
