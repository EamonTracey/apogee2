import sys

import board
import microcontroller
import pwmio


class ServoMotor:
    ON = 2**16
    MOTOR_MIN = 0.140
    MOTOR_MAX = 0.810

    def __init__(self, pin: microcontroller.Pin, **kwargs):
        self.motor = pwmio.PWMOut(pin, variable_frequency=False, **kwargs)
        self.motor.frequency = 330

        self.degrees = None

    def rotate(self, degrees: float):
        degrees = 135 - degrees
        if not (135 >=  degrees >= 90):
            print("Servo motor rotation must be in the range 135-90 degrees.",
                  file=sys.stderr)
            return

        percentage = degrees / 270
        duty_cycle = ServoMotor.MOTOR_MIN + (ServoMotor.MOTOR_MAX -
                                             ServoMotor.MOTOR_MIN) * percentage
        duty_cycle *= ServoMotor.ON
        self.motor.duty_cycle = duty_cycle

        self.degrees = degrees


servo_motor = ServoMotor(board.D12)
while True:
    degrees = float(input("Rotation (°): "))
    servo_motor.rotate(degrees)
