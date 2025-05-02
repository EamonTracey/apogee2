from dataclasses import dataclass

from base.stage import Stage


@dataclass
class BMP390State:
    # Altitude in meters.
    altitude: float = 0

    # Temperature in Celsius.
    temperature: float = 0

    # Count the number of times each reading fails.
    altitude_errors: int = 0
    temperature_errors: int = 0


@dataclass
class BNO085State:
    # Acceleration in meters per second squared.
    acceleration: tuple[float, float, float] = (0, 0, 0)

    # Magnetic field in microteslas.
    magnetic: tuple[float, float, float] = (0, 0, 0)

    # Angular velocity in radians per second.
    gyro: tuple[float, float, float] = (0, 0, 0)

    # Orientation as a (x, y, z, w) quaternion.
    quaternion: tuple[float, float, float, float] = (0, 0, 0, 0)

    # Count the number of times each reading fails.
    acceleration_errors: int = 0
    magnetic_errors: int = 0
    gyro_errors: int = 0
    quaternion_errors: int = 0


@dataclass
class ControlState:
    # Current Servo Angle.
    servo_angle: float = 0


@dataclass
class FilterState:
    # Filtered altitude in feet.
    altitude: float = 0

    # Filtered vertical velocity in feet / second.
    velocity: tuple[float, float, float] = (0, 0, 0)

    # Filtered vertical acceleration in feet / second^2.
    acceleration: tuple[float, float, float] = (0, 0, 0)


@dataclass
class FusionState:
    # Outputs quaternion in (w, x, y, z)
    quaternion: tuple[float, float, float, float] = (0, 0, 0, 0)

    # Outputs euler in (Roll, Yaw, Pitch)
    euler: tuple[float, float, float] = (0, 0, 0)

    # Outputs zenith angle
    zenith: float = 0


@dataclass
class ICM20649State:
    # The acceleration in meters per second squared.
    acceleration: tuple[float, float, float] = (0, 0, 0)

    # The angular velocity in degrees per second.
    gyro: tuple[float, float, float] = (0, 0, 0)

    # Count the number of times each reading fails.
    acceleration_errors: int = 0
    gyro_errors: int = 0


@dataclass
class LogState:
    ...


@dataclass
class PredictState:
    # Predicted apogee in feet.
    apogee_prediction: float = 5100


@dataclass
class StageState:
    stage: Stage = Stage.GROUND
