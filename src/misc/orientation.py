import numpy as np


def matrix_x(theta):
    return np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta), np.cos(theta)]])


def matrix_y(theta):
    return np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])


def matrix_z(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])


def matrix_to_quaternion(matrix):
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    q0 = np.sqrt((trace + 1) / 4)
    q1 = np.sqrt(matrix[0][0] / 2 + (1 - trace) / 4)
    q2 = np.sqrt(matrix[1][1] / 2 + (1 - trace) / 4)
    q3 = np.sqrt(matrix[2][2] / 2 + (1 - trace) / 4)
    return np.array([q0, q1, q2, q3])


def matrix_to_quaternion(R):
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]

    trace = m00 + m11 + m22

    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2  # S = 4 * qw
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = np.sqrt(1.0 + m00 - m11 - m22) * 2  # S = 4 * qx
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = np.sqrt(1.0 + m11 - m00 - m22) * 2  # S = 4 * qy
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = np.sqrt(1.0 + m22 - m00 - m11) * 2  # S = 4 * qz
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S

    return np.array([qw, qx, qy, qz])


def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])


def quaternion_conjugate(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def rotate_with_matrix(vector, matrix):
    return matrix @ vector


def rotate_with_quaternion(vector, quaternion):
    return quaternion_multiply(
        quaternion_multiply(quaternion, np.insert(vector, 0, 0)),
        quaternion_conjugate(quaternion))[1:]


np.set_printoptions(suppress=True)


def prove_to_myself():
    xdegrees = -45
    ydegrees = 30
    zdegrees = 10

    vector = np.array([1, 0, 0])
    matrix = matrix_z(np.deg2rad(zdegrees)) @ matrix_x(
        np.deg2rad(xdegrees)) @ matrix_y(np.deg2rad(ydegrees))
    quaternion = matrix_to_quaternion(matrix)

    print(rotate_with_matrix(vector, matrix))
    print(rotate_with_quaternion(vector, quaternion))

    matrix1 = matrix_y(np.deg2rad(ydegrees))
    matrix2 = matrix_x(np.deg2rad(xdegrees))
    matrix3 = matrix_z(np.deg2rad(zdegrees))
    quaternion1 = matrix_to_quaternion(matrix1)
    quaternion2 = matrix_to_quaternion(matrix2)
    quaternion3 = matrix_to_quaternion(matrix3)

    print(
        rotate_with_matrix(
            rotate_with_matrix(rotate_with_matrix(vector, matrix1), matrix2),
            matrix3))
    print(
        rotate_with_quaternion(
            rotate_with_quaternion(rotate_with_quaternion(vector, quaternion1),
                                   quaternion2), quaternion3))

    quaternion = quaternion_multiply(
        quaternion3, quaternion_multiply(quaternion2, quaternion1))
    print(rotate_with_quaternion(vector, quaternion))


def body_to_earth():
    matrix = matrix_y(np.deg2rad(-90))
    vector = np.array([1, 0, 0])
    quaternion = matrix_to_quaternion(matrix)
    print(quaternion)
    print(rotate_with_quaternion(vector, quaternion))


def complexify():
    # The [1 0 0] vector in the body frame points in the positive direction of the roll axis.
    # Let's use the ENU earth frame.

    body_to_earth = matrix_to_quaternion(matrix_y(np.deg2rad(-90)))

    pitch = matrix_to_quaternion(matrix_y(10))
    yaw = matrix_to_quaternion(matrix_z(10))
    roll = matrix_to_quaternion(matrix_x(10))

    orientation_body = quaternion_multiply(roll,
                                           quaternion_multiply(yaw, pitch))
    orientation_earth = quaternion_multiply(body_to_earth, orientation_body)

    vector = np.array([1, 0, 0])
    print(rotate_with_quaternion(vector, orientation_earth))


def angularize():

    def quat_derivative(q, omega_body):
        omega_quat = np.insert(omega_body, 0, 0)
        return 0.5 * quaternion_multiply(q, omega_quat)

    def rk4_quaternion_step(q, omega_body, dt):
        k1 = quat_derivative(q, omega_body)
        k2 = quat_derivative(q + 0.5 * dt * k1, omega_body)
        k3 = quat_derivative(q + 0.5 * dt * k2, omega_body)
        k4 = quat_derivative(q + dt * k3, omega_body)
        q_next = q + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return q_next / np.linalg.norm(q_next)

    def integrate_quaternion(q, omega, dt, steps):
        for _ in range(steps):
            q = rk4_quaternion_step(q, omega, dt)
        return q

    roll = matrix_to_quaternion(matrix_x(0))
    pitch = matrix_to_quaternion(matrix_y(0))
    yaw = matrix_to_quaternion(matrix_z(0))
    q = quaternion_multiply(yaw, quaternion_multiply(pitch, roll))

    # Angular velocity is 0.1*pi radians per second on each axis.
    # This means that after 20 seconds, there should be no rotation.
    angular_velocity = np.array([0.1 * np.pi, 0, 0])

    q_new = integrate_quaternion(q, angular_velocity, 0.01, 1000)

    vector = np.array([1, 0, 0])
    print(rotate_with_quaternion(vector, q))
    print(rotate_with_quaternion(vector, q_new))


body_to_earth()
