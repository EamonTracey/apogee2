#=====================================================================
# Code to read AS5600.  9 lines of python is all it takes.
import smbus
DEVICE_AS5600 = 0x36 # Default device I2C address
bus = smbus.SMBus(1)

def read_raw_angle(): # Read angle (0-360 represented as 0-4096)
  read_bytes = bus.read_i2c_block_data(DEVICE_AS5600, 0x0C, 2)
  return (read_bytes[0]<<8) | read_bytes[1];

def read_magnitude(): # Read magnetism magnitude
  read_bytes = bus.read_i2c_block_data(DEVICE_AS5600, 0x1B, 2)
  return (read_bytes[0]<<8) | read_bytes[1];

#=====================================================================

import time

import board

from flight.servo_motor import ServoMotor

servo_motor = ServoMotor(board.D12)

time.sleep(3)
for angle in [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 40, 35, 30, 25, 20, 15, 10, 5, 0]:
    servo_motor.rotate(angle)
    time.sleep(3)

    magnetic_angle = read_raw_angle() / 4096 * 360
    print(f"{angle=} {magnetic_angle=}")
