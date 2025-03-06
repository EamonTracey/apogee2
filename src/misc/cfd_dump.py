import json
import sys

import numpy as np
from scipy.interpolate import RegularGridInterpolator

NEWTONS_TO_LBF = 0.2248089431

cfd = None
with open(sys.argv[1], "r") as fp:
    cfd = fp.readlines()

cfd = cfd[1:]
cfd = [tuple(map(float, q.split(","))) for q in cfd]
cfd = np.array(cfd)

angles_of_actuation = np.unique(cfd[:, 0])
angles_of_attack = np.unique(cfd[:, 1])
mach_numbers = z_vals = np.unique(cfd[:, 2])
axials = cfd[:, 3].reshape(len(angles_of_actuation), len(angles_of_attack),
                           len(mach_numbers)) * 0.2248089431
normals = cfd[:, 4].reshape(len(angles_of_actuation), len(angles_of_attack),
                            len(mach_numbers)) * 0.2248089431

axial = dict()
axial["angles_of_actuation"] = angles_of_actuation.tolist()
axial["angles_of_attack"] = angles_of_attack.tolist()
axial["mach_numbers"] = mach_numbers.tolist()
axial["forces"] = axials.tolist()

normal = dict()
normal["angles_of_actuation"] = angles_of_actuation.tolist()
normal["angles_of_attack"] = angles_of_attack.tolist()
normal["mach_numbers"] = mach_numbers.tolist()
normal["forces"] = normals.tolist()

final = {"axial_force": axial, "normal_force": normal}

print(json.dumps(final))
