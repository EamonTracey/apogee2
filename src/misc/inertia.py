import json
import sys


def inertia_cylinder(mass: float, radius: float,
                     length: float) -> tuple[float, float, float]:
    pitch_yaw = 1 / 4 * mass * radius**2 + 1 / 12 * mass * length**2
    roll = 1 / 2 * mass * radius**2
    return (pitch_yaw, pitch_yaw, roll)


values = tuple(map(float, sys.argv[1:4]))
print(inertia_cylinder(*values))
