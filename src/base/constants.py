import numpy as np

# American units everywhere:
#     time -> seconds
#     length -> feet
#     mass -> slugs
#     temperature -> rankine
#     force -> lbf

# The mangitude of  gravitational acceleration experienced by an object near Earth's surface.
EARTH_GRAVITY_ACCELERATION = 32.17404  # ft/s^2

# Axes.
YAW = np.array([1, 0, 0])
PITCH = np.array([0, 1, 0])
ROLL = np.array([0, 0, 1])

# Unit conversions.
METERS_TO_FEET = 3.28084
