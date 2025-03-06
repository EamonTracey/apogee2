import sys

import numpy as np
from scipy.interpolate import RegularGridInterpolator

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
                           len(mach_numbers))
normals = cfd[:, 4].reshape(len(angles_of_actuation), len(angles_of_attack),
                            len(mach_numbers))

axial_interpolator = RegularGridInterpolator(
    (angles_of_actuation, angles_of_attack, mach_numbers), axials)
normal_interpolator = RegularGridInterpolator(
    (angles_of_actuation, angles_of_attack, mach_numbers), normals)

##### VISUALIZE FIXED ANGLE OF ATTACK #####

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Fix angle of attack at 0
angle_of_attack_fixed = 0

# Generate a fine grid of actuation angles and Mach numbers
X, Y = np.meshgrid(
    np.linspace(angles_of_actuation.min(), angles_of_actuation.max(), 50),
    np.linspace(mach_numbers.min(), mach_numbers.max(), 50))
Z = np.full_like(X, angle_of_attack_fixed)  # Keep angle of attack constant

# Flatten grid and interpolate axial force
points = np.c_[X.ravel(), Z.ravel(), Y.ravel()]  # (actuation, attack=0, mach)
W = axial_interpolator(points).reshape(X.shape)  # Reshape back to grid

# Create a 3D surface plot
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")

surf = ax.plot_surface(X, Y, W, cmap="magma", edgecolor="k")

# Add color bar
fig.colorbar(surf, label="Interpolated Axial Force")

# Labels
ax.set_xlabel("Angle of Actuation")
ax.set_ylabel("Mach Number")
ax.set_zlabel("Axial Force")
ax.set_title(
    f"Interpolated Axial Force at Angle of Attack = {angle_of_attack_fixed}")

plt.show()

##### VISUALIZE ANIMATED ANGLE OF ATTACK #####

import plotly.graph_objects as go

# Set up the meshgrid for actuation angle and Mach number
X, Y = np.meshgrid(
    np.linspace(angles_of_actuation.min(), angles_of_actuation.max(), 50),
    np.linspace(mach_numbers.min(), mach_numbers.max(), 50))

# Create an empty list to hold frames for animation
frames = []

# Loop through the angles of attack to create frames
for angle_of_attack_fixed in angles_of_attack:
    Z = np.full_like(
        X, angle_of_attack_fixed)  # Create a grid for this angle of attack
    points = np.c_[X.ravel(), Z.ravel(),
                   Y.ravel()]  # Prepare the points for interpolation
    W = normal_interpolator(points).reshape(
        X.shape)  # Interpolate the normal force

    # Create a frame with the data for the current angle of attack
    frames.append(
        go.Frame(
            data=[
                go.Surface(
                    z=W,
                    x=X,
                    y=Y,
                    colorscale='Viridis',
                    showscale=
                    False,  # Color scale is handled globally, not per-frame
                    opacity=0.8)
            ],
            name=str(angle_of_attack_fixed)))

# Create the initial figure
fig = go.Figure(
    data=[
        go.Surface(
            z=np.zeros_like(X),  # Initial Z values
            x=X,
            y=Y,
            colorscale='Viridis',
            opacity=0.8,
            showscale=True)
    ],
    layout=go.Layout(title="Interpolated Axial Force",
                     scene=dict(xaxis_title="Angle of Actuation",
                                yaxis_title="Mach Number",
                                zaxis_title="Axial Force"),
                     updatemenus=[{
                         'buttons': [{
                             'args': [
                                 None, {
                                     'frame': {
                                         'duration': 500,
                                         'redraw': True
                                     },
                                     'fromcurrent': True
                                 }
                             ],
                             'label':
                             'Play',
                             'method':
                             'animate'
                         }, {
                             'args': [[None], {
                                 'frame': {
                                     'duration': 0,
                                     'redraw': True
                                 },
                                 'mode': 'immediate',
                                 'transition': {
                                     'duration': 0
                                 }
                             }],
                             'label':
                             'Pause',
                             'method':
                             'animate'
                         }],
                         'direction':
                         'left',
                         'pad': {
                             'r': 10,
                             't': 87
                         },
                         'showactive':
                         False,
                         'type':
                         'buttons',
                         'x':
                         0.1,
                         'xanchor':
                         'right',
                         'y':
                         0,
                         'yanchor':
                         'top'
                     }],
                     sliders=[{
                         'currentvalue': {
                             'font': {
                                 'size': 20
                             },
                             'prefix': 'Angle of Attack: ',
                             'visible': True
                         },
                         'pad': {
                             'b': 10
                         },
                         'len':
                         0.9,
                         'x':
                         0.1,
                         'xanchor':
                         'center',
                         'y':
                         0,
                         'yanchor':
                         'top',
                         'steps': [{
                             'args': [[str(angle_of_attack)], {
                                 'frame': {
                                     'duration': 500,
                                     'redraw': True
                                 },
                                 'mode': 'immediate',
                                 'transition': {
                                     'duration': 300
                                 }
                             }],
                             'label':
                             f"{angle_of_attack}",
                             'method':
                             'animate'
                         } for angle_of_attack in angles_of_attack]
                     }]),
    frames=frames)

# Show the animation
fig.show()
