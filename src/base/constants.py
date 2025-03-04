import numpy as np

# All values use SI units.

# The mangitude of  gravitational acceleration experienced by an object near Earth's surface.
EARTH_GRAVITY_ACCELERATION = 32.17  # ft/s^2

# Axes.
YAW = np.array([1, 0, 0])
PITCH = np.array([0, 1, 0])
ROLL = np.array([0, 0, 1])

# Unit conversions.
METERS_TO_FEET = 3.28084
