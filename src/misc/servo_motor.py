import sys

import board

from flight.servo_motor import ServoMotor

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

    if not (0 <= degrees <= 45):
        print(f"{degrees} not in the range 0-45.", file=sys.stderr)
        continue

    servo_motor.rotate(degrees)
