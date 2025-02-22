from dataclasses import dataclass
import logging

import sys
import board
import microcontroller
import pwmio

from flight.filter import FilterState
from flight.stage import StageState
from base.component import Component

logger = logging.getLogger(__name__)

@dataclass
class ControlState:
    # Current Servo Angle.
    servo_angle: float = 0 

class ControlComponent(Component):
    def __init__(self, filter_state: FilterState, stage_state: StageState):

        self._state = ControlState()

        self._filter_state = filter_state
        self._stage_state = stage_state

        self._servo_motor = ServoMotor(board.D12)
        
        # First iteration in burnout.
        self._first = True

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        
        # TEMPORARY: Actuate motor to max starting 2 seconds after burnout for 2 seconds, then close.
        if self._first and self._stage_state.stage == Stage.COAST:
            self._first = False

            first_time = time

        if self._stage_state.stage == Stage.COAST:
            if time - first_time > 2:
                self._state.servo_angle = 45

            elif time - first_time > 4:
                self._state.servo_angle = 0

        # Rotate Servo
        self._servo_motor.rotate(self._state.servo_angle)


class ServoMotor:
    ON = 2**16
    MOTOR_MIN = 0.140
    MOTOR_MAX = 0.810

    def __init__(self, pin: microcontroller.Pin, **kwargs):
        self.motor = pwmio.PWMOut(pin, variable_frequency=False, **kwargs)
        self.motor.frequency = 330

        self.degrees = None

    # Maps the angle of the acs flaps (0-45) to the angle of the servo (135-90)
    def map(self, n: float):
        return 135 - n

    def rotate(self, degrees: float):
        # Motor degrees of 135 is fully closed (0 degree flaps), 
        # Motor degrees of 90 is fully open. (45 degree flaps)

        if not (0 <= degrees <= 270):
            logging.warning(f"ERROR: Servo motor rotation of {degrees} must be in the range 0-270 degrees.")
            return
        if not (0 <= degrees <= 45):
            logging.warning(f"ERROR: Unsafe servo actuation percentage ({degrees}), stay in [0, 45].")
            return

        logging.debug(f"Actuating servo to {degrees} degrees")

        degrees = map(degrees)

        percentage = degrees / 270
        duty_cycle = ServoMotor.MOTOR_MIN + (ServoMotor.MOTOR_MAX -
                                             ServoMotor.MOTOR_MIN) * percentage
        duty_cycle *= ServoMotor.ON
        self.motor.duty_cycle = duty_cycle

        self.degrees = degrees

