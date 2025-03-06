import os
import statistics
import sys

import numpy as np
from scipy.interpolate import LinearNDInterpolator, RBFInterpolator

from typing import Union

axial = []
normal = []

# Collect data.
cfd_directory = sys.argv[1]
for file in os.listdir(cfd_directory):
    path = os.path.join(cfd_directory, file)
    angle_of_actuation = float(file.split("_")[1][:-7])
    angle_of_attack = float(file.split("_")[2])
    mach_number = float(file.split("_")[3])

    forces = None
    with open(path, "r") as fp:
        forces = fp.readlines()
    qforces = [
        float(forces[-i].strip().split(" ")[1]) for i in range(10, 0, -1)
    ]
    forces = qforces

    if file.endswith("axial"):
        axial.append(
            [angle_of_actuation, angle_of_attack, mach_number, forces])
    elif file.endswith("normal"):
        normal.append(
            [angle_of_actuation, angle_of_attack, mach_number, forces])

assert len(axial) == len(normal)


# Prune nonconverged values.
def valid_point(point):
    forces = point[3]
    if statistics.stdev(forces) > 1:
        return False
    return True


axial = list(filter(valid_point, axial))
axial = [(a, b, c, d[-1]) for (a, b, c, d) in axial]
normal = list(filter(valid_point, normal))
normal = [(a, b, c, d[-1]) for (a, b, c, d) in normal]

missing_axial = []
missing_normal = []
for a in [0, 5, 10, 15, 20, 25, 30, 35, 40, 45]:
    for b in [0, 1.25, 2.5, 3.75, 5, 6.25, 7.5, 8.75, 10]:
        for c in [
                0.025, 0.05, 0.075, 0.1, 0.125, 0.15, 0.175, 0.2, 0.225, 0.25,
                0.275, 0.3, 0.325, 0.35, 0.375, 0.4, 0.425, 0.45, 0.475, 0.5,
                0.525, 0.55, 0.575, 0.6, 0.625, 0.65, 0.675, 0.7
        ]:
            r = False
            for q in axial:
                if a == q[0] and b == q[1] and c == q[2]:
                    r = True
            if not r:
                missing_axial.append((a, b, c))
for a in [0, 5, 10, 15, 20, 25, 30, 35, 40, 45]:
    for b in [0, 1.25, 2.5, 3.75, 5, 6.25, 7.5, 8.75, 10]:
        for c in [
                0.025, 0.05, 0.075, 0.1, 0.125, 0.15, 0.175, 0.2, 0.225, 0.25,
                0.275, 0.3, 0.325, 0.35, 0.375, 0.4, 0.425, 0.45, 0.475, 0.5,
                0.525, 0.55, 0.575, 0.6, 0.625, 0.65, 0.675, 0.7
        ]:
            r = False
            for q in normal:
                if a == q[0] and b == q[1] and c == q[2]:
                    r = True
            if not r:
                missing_normal.append((a, b, c))

missing_axial_input = np.array(missing_axial)
missing_normal_input = np.array(missing_normal)

##### INTERPOLATION TO BACKFILL AXIAL #####
# linear interpolation for values inside convex hull

axial_input = np.array([q[:3] for q in axial])
axial_output = np.array([q[3] for q in axial])

axial_interpolator = LinearNDInterpolator(axial_input,
                                          axial_output,
                                          rescale=True)
missing_axial_output = axial_interpolator(missing_axial_input)

filled_axial_input, filled_axial_output = zip(
    *[(x, y) for (x, y) in zip(missing_axial_input, missing_axial_output)
      if not np.isnan(y)])
filled_axial_input = np.array(filled_axial_input)
filled_axial_output = np.array(filled_axial_output)
axial_input = np.concatenate((axial_input, filled_axial_input), axis=0)
axial_output = np.concatenate((axial_output, filled_axial_output), axis=0)
missing_axial_input = [
    q for q in missing_axial_input
    if not np.any(np.all(filled_axial_input == q, axis=1))
]

# rbf extrapolation for values outside convex hull
axial_interpolator = RBFInterpolator(axial_input, axial_output)
missing_axial_output = axial_interpolator(missing_axial_input)

axial_input = np.concatenate((axial_input, missing_axial_input), axis=0)
axial_output = np.concatenate((axial_output, missing_axial_output), axis=0)

##### INTERPOLATION TO BACKFILL NORMAL #####
# linear interpolation for values inside convex hull

normal_input = np.array([q[:3] for q in normal])
normal_output = np.array([q[3] for q in normal])

normal_interpolator = LinearNDInterpolator(normal_input,
                                           normal_output,
                                           rescale=True)
missing_normal_output = normal_interpolator(missing_normal_input)

filled_normal_input, filled_normal_output = zip(
    *[(x, y) for (x, y) in zip(missing_normal_input, missing_normal_output)
      if not np.isnan(y)])
filled_normal_input = np.array(filled_normal_input)
filled_normal_output = np.array(filled_normal_output)
normal_input = np.concatenate((normal_input, filled_normal_input), axis=0)
normal_output = np.concatenate((normal_output, filled_normal_output), axis=0)
missing_normal_input = [
    q for q in missing_normal_input
    if not np.any(np.all(filled_normal_input == q, axis=1))
]

# rbf extrapolation for values outside convex hull
normal_interpolator = RBFInterpolator(normal_input, normal_output)
missing_normal_output = normal_interpolator(missing_normal_input)

normal_input = np.concatenate((normal_input, missing_normal_input), axis=0)
normal_output = np.concatenate((normal_output, missing_normal_output), axis=0)

##### INTERPOLATION DONE #####
axial_input = [tuple(a) for a in axial_input]
axial_output = [a for a in axial_output]
normal_input = [tuple(a) for a in normal_input]
normal_output = [a for a in normal_output]

axial_input, axial_output = zip(*sorted(zip(axial_input, axial_output)))
normal_input, normal_output = zip(*sorted(zip(normal_input, normal_output)))

# convert negative -> zero ... should we do this?
axial_output = [q if q >= 0 else 0 for q in axial_output]
normal_output = [q if q >= 0 else 0 for q in normal_output]

##### CSV WRITING #####

import csv

headers = ["angle of actuation", "angle of attack", "mach number", "axial force", "normal force"]
fp = open("cfd.csv", "w")
writer = csv.writer(fp)
writer.writerow(headers)
assert len(axial_input) == len(axial_output) == len(normal_input) == len(normal_output)
for i in range(len(axial_input)):
    assert axial_input[i] == normal_input[i]
    writer.writerow([*axial_input[i], axial_output[i], normal_output[i]])
fp.close()
