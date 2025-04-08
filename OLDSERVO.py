import logging
import sys

import board
import pwmio

class ServoMotor:
    ON = 2 ** 16
    MOTOR_MIN = 0.15
    MOTOR_MAX = 0.80

    def __init__(self, pin, **kwargs):
        self.motor = pwmio.PWMOut(pin, variable_frequency=False, **kwargs)
        self.motor.frequency = 330
        self.percentage = 0

    def rotate(self, n):
        """
        Rotate the servo motor by the percentage indicates by n.
            n = 0 -> 0% = 0 degrees
            n = 50 -> 50% = 90 degrees
            n = 100 -> 100% = 180 degrees
        The servo motor 
        """
        try:
            n = float(n)
        except ValueError:
            logging.warning(f"SERVO ERROR: must pass a number, not {n} of type {type(n)}.")
            return
#        if n < 0 or n > 100:
#            logging.warning(f"SERVO ERROR: the percentage value ({n}) must be in [0, 100].")
#            return
#        if n < 0 or n > 40:
#            logging.warning(f"SERVO ERROR: unsafe actuation percentage ({n}), stay in [0, 40].")
#            return

        logging.debug(f"Actuating servo motor to {n}% = {n} degrees.")
        delta = ServoMotor.MOTOR_MAX - ServoMotor.MOTOR_MIN
        duty = ServoMotor.MOTOR_MIN + delta * n / 100
        self.motor.duty_cycle = duty * ServoMotor.ON
        self.percentage = n


servo_motor = ServoMotor(board.D12)
while True:
    degrees = input("Rotation (°): ")
    if degrees.lower() in ["q", "quit", "exit"]:
        break

    try:
        degrees = float(degrees)
    except ValueError:
        print(f"Invalid number {degrees}.", file=sys.stderr)
        continue

    if not (-0 <= degrees <= 100):
        print(f"{degrees} not in the range 0-100.", file=sys.stderr)
        continue

    servo_motor.rotate(degrees)
