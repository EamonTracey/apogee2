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

    def map(self, degrees: float):
        # Motor degrees of 140 is fully closed (0 degree flaps),
        # Motor degrees of 95 is fully open. (45 degree flaps)
        return 140 - degrees

    def rotate(self, degrees: float):
        if not (0 <= degrees <= 45):
            logging.warning(
                f"ERROR: Unsafe servo actuation percentage ({degrees}), stay in [0, 45]."
            )
            return

        degrees = self.map(degrees)

        percentage = degrees / 270
        duty_cycle = ServoMotor.MOTOR_MIN + (ServoMotor.MOTOR_MAX -
                                             ServoMotor.MOTOR_MIN) * percentage
        duty_cycle *= ServoMotor.ON
        self.motor.duty_cycle = duty_cycle

        self.degrees = degrees
