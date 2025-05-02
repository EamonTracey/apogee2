import numpy as np
from scipy.spatial.transform import Rotation

from base.constants import EARTH_GRAVITY_ACCELERATION, SPECIFIC_GAS_CONSTANT_AIR
from simulation.eamonteasley import calculate_derivatives_eamonteasley
from simulation.environment import Environment
from simulation.motor import Motor
from simulation.vehicle import Vehicle

np.set_printoptions(suppress=True)

DT = 0.025
ROLL = 0
PITCH = 0
YAW = 0

vehicle = Vehicle.from_json("data/vehicles/fullscale25.json")
motor = Motor.from_json("data/motors/aerotech_l1940x.json")
environment = Environment.from_json("data/environments/threeoaks_basic.json")

time = np.float64(0)
position = np.array([0, 0, 0], dtype=np.float64)
velocity = np.array([0, 0, 0], dtype=np.float64)
orientation = np.roll(
    (Rotation.from_euler("zyz", [ROLL, PITCH, YAW], degrees=True) *
     Rotation.from_euler("y", -90, degrees=True)).as_quat(), 1)
print(orientation)
angular_velocity = np.array([0, 0, 0], dtype=np.float64)
q = []
while velocity[2] > -5:
    q.append(orientation.copy())
    dposition, dvelocity, dorientation, dangular_velocity = calculate_derivatives_eamonteasley(
        vehicle, motor, environment, time, position, velocity, orientation,
        angular_velocity)
    time += DT
    position += dposition * DT
    velocity += dvelocity * DT
    orientation += dorientation * DT
    orientation /= np.linalg.norm(orientation)
    angular_velocity += dangular_velocity * DT
    #print(position)

q = np.array(q)
rotations = Rotation.from_quat(q, scalar_first=True)
vectors = rotations.apply(np.array([1, 0, 0]))
print(vectors)

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import numpy as np

origin = np.array([0, 0, 0])
max_val = np.max(np.abs(vectors)) + 0.5

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-max_val, max_val])
ax.set_ylim([-max_val, max_val])
ax.set_zlim([-max_val, max_val])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Rotating Vector Animation")

# Initial vector
quiver = ax.quiver(*origin, *vectors[0], color='r', arrow_length_ratio=0.1)


# Update function
def update_quiver(frame):
    global quiver
    # Remove the previous arrow
    quiver.remove()
    # Draw a new arrow
    vec = vectors[frame]
    quiver = ax.quiver(*origin, *vec, color='r', arrow_length_ratio=0.1)
    return quiver,


# Animate
ani = animation.FuncAnimation(fig,
                              update_quiver,
                              frames=len(vectors),
                              interval=25,
                              blit=False,
                              repeat=True)
plt.show()
