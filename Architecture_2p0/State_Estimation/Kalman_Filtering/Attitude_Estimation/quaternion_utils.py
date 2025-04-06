"""
Referencing from: https://cookierobotics.com/073/
"""

import numpy as np
from quaternion import Quaternion
from ekf import EKF

g = 9.8
random_generator = np.random.default_rng(0)

def axisang_to_quaternion(unit_axis, angle_rad):
    ux, uy, uz = unit_axis.flat
    sin_half = np.sin(angle_rad / 2)
    qw = np.cos(angle_rad / 2)
    qx = ux * sin_half
    qy = uy * sin_half
    qz = uz * sin_half
    return Quaternion(qw, qx, qy, qz)
    
def quaternion_from_rotation_vector(v, eps=0):
    angle_rad = np.linalg.norm(v)

    # Guard against division by zero
    if angle_rad > eps:
        unit_axis = v / angle_rad
        q_unit = axisang_to_quaternion(unit_axis, angle_rad)
    else:
        q_unit = Quaternion(1.0, 0.0, 0.0, 0.0)

    return q_unit

def quaternion_from_euler(roll, pitch, yaw):
    c_roll   = np.cos(roll / 2)
    s_roll   = np.sin(roll / 2)
    c_pitch = np.cos(pitch / 2)
    s_pitch = np.sin(pitch / 2)
    c_yaw   = np.cos(yaw / 2)
    s_yaw   = np.sin(yaw / 2)

    qw = c_roll * c_pitch * c_yaw + s_roll * s_pitch * s_yaw
    qx = s_roll * c_pitch * c_yaw - c_roll * s_pitch * s_yaw
    qy = c_roll * s_pitch * c_yaw + s_roll * c_pitch * s_yaw
    qz = c_roll * c_pitch * s_yaw - s_roll * s_pitch * c_yaw
    return Quaternion(qw, qx, qy, qz)

def get_gyro_data(q1, q2, bias, dt, noise):

    mean = np.array([
        2/dt * (q1.qw*q2.qx - q1.qx*q2.qw - q1.qy*q2.qz + q1.qz*q2.qy),
        2/dt * (q1.qw*q2.qy + q1.qx*q2.qz - q1.qy*q2.qw - q1.qz*q2.qx),
        2/dt * (q1.qw*q2.qz - q1.qx*q2.qy + q1.qy*q2.qx - q1.qz*q2.qw)
    ])

    w = random_generator.normal(mean + bias, noise)
    return w

def get_accelerometer_data(q, noise):
    q_to_body = q.invertq()
    up_world = np.c_[[0, 0, -g]]
    up_body = q_to_body.rotate_vector(up_world)
    a = random_generator.normal(up_body, noise)
    return a

def generate_accelerometer_data_array(q, acc_noise):
    N = len(q)
    ax, ay, az = [None] * N, [None] * N, [None] * N

    for i in range(N):
        a = get_accelerometer_data(q[i], acc_noise)
        ax[i], ay[i], az[i] = a.flat
    
    return ax, ay, az

def generate_gyro_data_array(t, q, b, gyro_noise):
    N = len(t)
    wx, wy, wz = [None] * N, [None] * N, [None] * N

    for i in range(1, N):
        dt = t[i] - t[i-1]
        bi = np.array([b[0][i], b[1][i], b[2][i]])
        w = get_gyro_data(q[i-1], q[i], bi, dt, gyro_noise)
        wx[i], wy[i], wz[i] = w.flat
    
    return wx, wy, wz

####
def estimate(t, w, a):
    N = len(t)
    q = [None] * N
    bx, by, bz = [None] * N, [None] * N, [None] * N
    bx_err, by_err, bz_err = [None] * N, [None] * N, [None] * N
    ekf = EKF()

    q[0] = ekf.x[0:4]
    bx[0], by[0], bz[0] = ekf.x.flat[4:7]
    bx_err[0] = np.sqrt(ekf.P[4, 4])
    by_err[0] = np.sqrt(ekf.P[5, 5])
    bz_err[0] = np.sqrt(ekf.P[6, 6])
    for i in range(1, N):
        dt = t[i] - t[i-1]
        wi = [w[0][i], w[1][i], w[2][i]]
        ai = [a[0][i], a[1][i], a[2][i]]
        ekf.predict(wi, dt)
        ekf.update(ai)
        
        q[i] = ekf.x[0:4]
        bx[i], by[i], bz[i] = ekf.x.flat[4:7]
        bx_err[i] = np.sqrt(ekf.P[4, 4])
        by_err[i] = np.sqrt(ekf.P[5, 5])
        bz_err[i] = np.sqrt(ekf.P[6, 6])

    return q, [bx, by, bz], [bx_err, by_err, bz_err]

def euler_degrees_to_quaternion(rpy):
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]
    N = len(roll)
    q = [None] * N

    for i in range(N):
        if roll[i] is not None:
            q[i] = quaternion_from_euler(
                np.radians(roll[i]),
                np.radians(pitch[i]),
                np.radians(yaw[i]))
    
    return q