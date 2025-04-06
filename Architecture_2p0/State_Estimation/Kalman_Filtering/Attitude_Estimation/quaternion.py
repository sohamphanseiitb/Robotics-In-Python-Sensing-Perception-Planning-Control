"""
Referencing from: https://cookierobotics.com/073/
"""

import numpy as np

class Quaternion():

    def __init__(self, qw, qx, qy, qz):
        self.norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
        self.qw = qw/self.norm
        self.qx = qx/self.norm
        self.qy = qy/self.norm
        self.qz = qz/self.norm
    
    def quaternion_to_rotmat(self):
        r11 = self.qw**2 + self.qx**2 - self.qy**2 - self.qz**2
        r12 = 2 * (self.qx * self.qy - self.qw * self.qz)
        r13 = 2 * (self.qw * self.qy + self.qx * self.qz)
        r21 = 2 * (self.qx * self.qy + self.qw * self.qz)
        r22 = self.qw**2 - self.qx**2 + self.qy**2 - self.qz**2
        r23 = 2 * (self.qy * self.qz - self.qw * self.qx)
        r31 = 2 * (self.qx * self.qz - self.qw * self.qy)
        r32 = 2 * (self.qy * self.qz + self.qw * self.qx)
        r33 = self.qw**2 - self.qx**2 - self.qy**2 + self.qz**2 

        self.qrotmat = np.array([[r11, r12, r13], 
                         [r21, r22, r23], 
                         [r31, r32, r33]])
        return self.qrotmat
    
    def quaternion_to_euler(self):
        roll  = np.arctan2(2 * (self.qw * self.qx + self.qy * self.qz),
                        1 - 2 * (self.qx * self.qx + self.qy * self.qy))
        pitch = np.arcsin(2 * (self.qw * self.qy - self.qz * self.qx))
        yaw   = np.arctan2(2 * (self.qw * self.qz + self.qx * self.qy),
                        1 - 2 * (self.qy * self.qy + self.qz * self.qz))
        return roll, pitch, yaw

    def qmultiply(self, q2):
        rw = self.qw * q2.qw - self.qx * q2.qx - self.qy * q2.qy - self.qz * q2.qz
        rx = self.qw * q2.qx + self.qx * q2.qw + self.qy * q2.qz - self.qz * q2.qy
        ry = self.qw * q2.qy - self.qx * q2.qz + self.qy * q2.qw + self.qz * q2.qx
        rz = self.qw * q2.qz + self.qx * q2.qy - self.qy * q2.qx + self.qz * q2.qw
        return Quaternion(rw, rx, ry, rz)
    
    def normalizeq(self):
        self.norm = np.sqrt(self.qw**2 + self.qx**2 + self.qy**2 + self.qz**2)
        self.qw = self.qw/self.norm
        self.qx = self.qx/self.norm
        self.qy = self.qy/self.norm
        self.qz = self.qz/self.norm

    def invertq(self):
        return Quaternion(self.qw, -self.qx, -self.qy, -self.qz)
    
    def rotate_vector(self, v):

        qunit = Quaternion(1, 0, 0, 0)
        p = Quaternion(0, v[0], v[1], v[2])
        qup = qunit.qmultiply(p)
        qup = qup.qmultiply(qunit.invertq())

        return np.array([qup.qx, qup.qy, qup.qz])
    
    def quaternion_to_euler_degrees(self):

        r, p, y = self.quaternion_to_euler()
        r = np.degrees(r)
        p = np.degrees(p)
        y = np.degrees(y)
        
        return r, p, y
    
    def quater
    