import numpy as np
from scipy.spatial.transform import Rotation

from base.constants import EARTH_GRAVITY_ACCELERATION, SPECIFIC_GAS_CONSTANT_AIR
from simulation.environment import Environment
from simulation.motor import Motor
from simulation.vehicle import Vehicle
"""
Notes

There are two reference frames, global and body. The global frame is attached
to the Earth. The body frame is attached to the vehicle. We will use the ENU
(East-North-Up) convention in the global frame and the RPY (roll-pitch-yaw)
convention in the body frame.

The Earth is assumed to be an inertial frame of reference. This means the
Coriolis effect caused by Earth's rotation is ignored. This is a fine
assumption on our scale (though this would not work for ICBMs!).

The state of the vehicle at any instant is a 4-tuple that contains the
position, velocity, orientation, and angular velocity. Position and velocity
are in global coordinates (otherwise they equal 0). Orientation is a quaternion
that maps any point in the body frame to the global frame. Angular velocity is
defined about the body axes. Note that it is simple to define the pre-launch
state of the vehicle. Further, these values can be obtained (possibly with
finagling/filtering) from sensors.

Simulation involves the integration of the state over time. If we know the
current state and state derivative, we can step forward in time (assuming the
derivative remains constant throughout the timestep). This is a good
approximation when we use a small timestamp.

How can we compute the state derivative of the vehicle? Let's handle each component separately:
    1) position: The derivative of position is velocity. Velocity is already
    part of the state, so no computation is necessary.
    2) velocity: The derivative of velocity is acceleration. Newton's Second
    Law states that F=ma. The mass of the vehicle should be easily known at all
    times, so the difficulty is to compute the force. Assume that the vehicle
    experiences three forces: thrust, gravity, and aerodynamic. Thrust is
    simple to compute given the motor's thrust curve. Gravity is simple to
    compute knowing the mass and Earth's gravitational acceleration. The
    aerodynamic force is more complicated. The aerodynamic force depends on the
    Mach number and angle of attack. On a vehicle with aerodynamic control
    surfaces, the force also depends on the actuation of these surfaces. The
    recommended approach is to interpolate CFD results to compute the
    aerodynamic force from these inputs.
    3) orientation: The derivative of orientation is the quaternion derivative.
    The quaternion derivative depends on the quaternion and the angular
    velocity which are both part of the state, so computation is simple.
    4) angular velocity: The derivative of angular velocity is angular
    acceleration. Newton's Second Law states that T=Ia. The moment of inertia
    of the vehicle should be easily known at all times, so the difficulty is to
    compute the torque. Assume that the vehicle experiences one torque from the
    normal component of the aerodynamic force. Then, we can use the same
    aerodynamic normal force calculated prior.
"""


def calculate_derivatives(vehicle, motor, environment, time, position,
                          velocity, orientation, angular_velocity):
    # Package the raw quaternion into a scipy Rotation object. This lets us
    # convert any vector in the body frame to the global frame.
    body_to_earth = Rotation.from_quat(orientation, scalar_first=True)
    earth_to_body = body_to_earth.inv()

    # Calculate the total mass of the vehicle with the motor.
    mass = vehicle.mass + motor.calculate_mass(time)

    # Calculate environment values at the current altitude.
    temperature = environment.calculate_temperature(position[2])
    pressure = environment.calculate_pressure(position[2])
    density = environment.calculate_density(position[2])
    wind = np.array(environment.calculate_wind(position[2]))

    # Calculate the speed of sound and mach number.
    # NASA Speed of Sound Model.
    # https://www.grc.nasa.gov/www/k-12/VirtualAero/BottleRocket/airplane/sound.html.
    speed_of_sound = np.sqrt(1.4*SPECIFIC_GAS_CONSTANT_AIR*temperature)
    velocity_relative_air_earth = velocity - wind
    velocity_relative_air_magnitude = np.round(np.linalg.norm(velocity_relative_air_earth), decimals=8)
    mach_number = velocity_relative_air_magnitude/speed_of_sound

    # The angle of attack is the angle between the roll axis and the velocity
    # relative to the air.
    if velocity_relative_air_magnitude != 0:
        roll_earth = body_to_earth.apply(np.array([1, 0, 0]))
        velocity_relative_air_direction_earth = velocity_relative_air_earth/velocity_relative_air_magnitude
        cosine_angle_of_attack = np.dot(roll_earth, velocity_relative_air_direction_earth)
        angle_of_attack = np.round(np.arccos(np.clip(cosine_angle_of_attack, -1, 1)), decimals=8)
    else:
        angle_of_attack = 0
    # TODO? #
    angle_of_attack = np.degrees(angle_of_attack)
    # TODO? #

    # The thrust force acts along the roll axis of the vehicle.
    force_thrust_body = np.array([motor.calculate_thrust(time), 0, 0])
    force_thrust_earth = body_to_earth.apply(force_thrust_body)

    # The gravity force acts downward.
    force_gravity_earth = np.array([0, 0, -mass*EARTH_GRAVITY_ACCELERATION])

    # The aerodynamic axial force acts along the roll axis of the vehicle.
    force_axial_body = np.array(
        [-vehicle.calculate_axial_force(0, abs(angle_of_attack), mach_number, density), 0, 0])
    force_axial_earth = body_to_earth.apply(force_axial_body)

    # The aerodynamic normal force acts in the plane perpdendicular to the roll
    # axis of the vehicle.
    if angle_of_attack != 0:
        force_normal_magnitude = vehicle.calculate_normal_force(0, abs(angle_of_attack), mach_number, density)
        velocity_relative_air_body = earth_to_body.apply(velocity_relative_air_earth)
        velocity_relative_air_normal_body = np.insert(velocity_relative_air_body[1:], 0, 0)
        force_normal_direction_body = -velocity_relative_air_normal_body/np.linalg.norm(velocity_relative_air_normal_body)
    else:
        force_normal_magnitude = 0
        force_normal_direction_body = np.array([0, 0, 0])
    force_normal_body = force_normal_magnitude * force_normal_direction_body
    force_normal_earth = body_to_earth.apply(force_normal_body)

    # Newton's Second Law F=ma.
    force_earth = force_thrust_earth + force_gravity_earth + force_axial_earth + force_normal_earth
    acceleration = force_earth/mass

    # The torque on the ...
    moment_arm_body = np.array([vehicle.center_of_mass - vehicle.center_of_pressure, 0, 0])
    torque_body = np.cross(moment_arm_body, force_normal_body)

    # Newton's Second Law T=Ia
    angular_acceleration = 0.2 * torque_body / vehicle.inertia
    print(angle_of_attack)

    ####### TODO? #######
    def quaternion_multiply(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
    omega_quat = np.insert(angular_velocity, 0, 0)
    dorientation = 0.5 * quaternion_multiply(orientation, omega_quat)
    ####### TODO? #######

    return velocity, acceleration, dorientation, angular_acceleration


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
orientation = (Rotation.from_euler("zyz", [ROLL, PITCH, YAW], degrees=True) *
               Rotation.from_euler("y", -90, degrees=True)).as_quat(
                   scalar_first=True)
angular_velocity = np.array([0, 0, 0], dtype=np.float64)
q=[]
while velocity[2] > -5:
    q.append(orientation.copy())
    dposition, dvelocity, dorientation, dangular_velocity = calculate_derivatives(vehicle, motor, environment,
                                                 time, position, velocity,
                                                 orientation, angular_velocity)
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
ani = animation.FuncAnimation(fig, update_quiver, frames=len(vectors), interval=25, blit=False, repeat=True)
plt.show()


