from quaternion import Quaternion
import quaternion_utils as qu

import sys
from base import Gaussian, system
from ekf import EKF
import numpy as np
import matplotlib.pyplot as plt

g = 9.8

def get_F(x, w, dt):
    qw, qx, qy, qz = x.qw, x.qx, x.qy, x.qz
    bx, by, bz = 0, 0, 0
    wx, wy, wz = w.flat
    return np.array([
        [             1, dt*(-wx + bx)/2, dt*(-wy + by)/2, dt*(-wz + bz)/2,  dt*qx/2,  dt*qy/2,  dt*qz/2],
        [dt*(wx - bx)/2,               1, dt*( wz - bz)/2, dt*(-wy + by)/2, -dt*qw/2,  dt*qz/2, -dt*qy/2],
        [dt*(wy - by)/2, dt*(-wz + bz)/2,               1, dt*( wx - bx)/2, -dt*qz/2, -dt*qw/2,  dt*qx/2],
        [dt*(wz - bz)/2, dt*( wy - by)/2, dt*(-wx + bx)/2,               1,  dt*qy/2, -dt*qx/2, -dt*qw/2],
    ])

def get_W(x, dt):
    qw, qx, qy, qz = x.qw, x.qx, x.qy, x.qz
    return dt/2 * np.array([
        [-qx, -qy, -qz],
        [ qw, -qz,  qy],
        [ qz,  qw, -qx],
        [-qy,  qx,  qw],
    ])

def get_H(x):
   qw, qx, qy, qz = x.flat[0:4]
   return 2 * g * np.array([
        [ qy, -qz,  qw, -qx],
        [-qx, -qw, -qz, -qy],
        [-qw,  qx,  qy, -qz]
   ])

def f(x, w, dt):
    q = Quaternion(x.qw, x.qx, x.qy, x.qz)
    bx, by, bz = 0, 0, 0
    b = np.array([bx, by, bz])

    d_ang = (w - b) * dt
    dq = qu.quaternion_from_rotation_vector(d_ang)
    q = q.qmultiply(dq)
    q.normalizeq()

    return np.r_[q, b]

def h(x):
    q = Quaternion(x.qw, x.qx, x.qy, x.qz)
    R_from_body = q.quaternion_to_matrix()
    return R_from_body.T @ np.c_[[0, 0, -g]]