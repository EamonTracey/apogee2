from dataclasses import dataclass
import logging
import traceback
import sys

import board
import microcontroller
import pwmio

from base.component import Component
from base.stage import Stage
from flight.blackboard import ControlState, FilterState, StageState, PredictState

from flight.constants import APOGEE_ALTITUDE

logger = logging.getLogger(__name__)


class ControlComponent(Component):

    def __init__(self, filter_state: FilterState, stage_state: StageState,
                 predict_state: PredictState):

        self._state = ControlState()

        self._filter_state = filter_state
        self._stage_state = stage_state
        self._predict_state = predict_state

        self._servo_motor = ServoMotor(board.D12)

        self._error_previous = None
        self._time_previous = None
        self._integral_previous = None
        self._pi_previous = None

        self._first = True

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):
        try:
            # Motor Control Algorithm.
            if self._stage_state.stage == Stage.GROUND or self._stage_state.stage == Stage.BURN:
                self._state.servo_angle = 0

            elif self._stage_state.stage == Stage.OVERSHOOT:
                self._state.servo_angle = 40

            elif self._stage_state.stage == Stage.DESCENT:
                self._state.servo_angle = 0

            elif self._stage_state.stage == Stage.COAST:
                # First iteration in coast?
                if self._time_previous is None:
                    self._time_previous = time
                    self._error_previous = self._predict_state.apogee_prediction - APOGEE_ALTITUDE
                    self._integral_previous = 0
                    self._pi_previous = 0
                    self._state.servo_angle = 0

                else:
                    # Error Calculation.
                    apogee_error = self._predict_state.apogee_prediction - APOGEE_ALTITUDE

                    dt = time - self._time_previous

                    # PI Terms.
                    proportional = apogee_error
                    integral = self._integral_previous + (
                        (apogee_error + self._error_previous) * dt / 2)

                    # Predetermined Proportional Constants.
                    Kp = 0.025
                    Ki = 0.005
                    Kg = 1
                    pi0 = 10

                    if self._predict_state.apogee_prediction < 4300:
                        Ki = 0.0

                    # Max Servo Speed - Test 10.1.4 DT.1
                    max_servo_delta = dt * 45 / 0.35

                    # PI Control.
                    pi = pi0 + (Kp * proportional + Ki * integral)
                    pi_delta = pi - self._pi_previous
                    if pi_delta >= max_servo_delta:
                        pi = self._pi_previous + max_servo_delta
                    elif pi_delta <= -max_servo_delta:
                        pi = self._pi_previous - max_servo_delta

                    if pi >= 40:
                        pi = 40
                    elif pi <= 0:
                        pi = 0

                    self._error_previous = apogee_error
                    self._time_previous = time
                    self._integral_previous = integral
                    self._pi_previous = pi

                    self._state.servo_angle = pi

            # Rotate Servo
            self._servo_motor.rotate(self._state.servo_angle)
        except Exception as e:
            logger.info(
                f"Exception when controlling: {traceback.format_exc()}")


class ServoMotor:
    ON = 2**16
    MOTOR_MIN = 0.140
    MOTOR_MAX = 0.810

    def __init__(self, pin: microcontroller.Pin, **kwargs):
        self.motor = pwmio.PWMOut(pin, variable_frequency=False, **kwargs)
        self.motor.frequency = 330

        self.degrees = None

    # Maps the angle of the acs flaps (0-40) to the angle of the servo (135-90)
    def map(self, n: float):
        return 140 - n

    def rotate(self, degrees: float):
        # Motor degrees of 135 is fully closed (0 degree flaps),
        # Motor degrees of 90 is fully open. (40 degree flaps)

        if not (-10 <= degrees <= 40):
            logging.warning(
                f"ERROR: Unsafe servo actuation percentage ({degrees}), stay in [-10, 40]."
            )
            return

        logging.debug(f"Actuating servo to {degrees} degrees")

        degrees = self.map(degrees)

        percentage = degrees / 270
        duty_cycle = ServoMotor.MOTOR_MIN + (ServoMotor.MOTOR_MAX -
                                             ServoMotor.MOTOR_MIN) * percentage
        duty_cycle *= ServoMotor.ON
        self.motor.duty_cycle = duty_cycle

        self.degrees = degrees
