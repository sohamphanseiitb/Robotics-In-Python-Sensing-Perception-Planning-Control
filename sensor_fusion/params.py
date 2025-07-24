"""
DOCSTRING:
This file stores all the constants and parameters needed for the simulation
"""

# Library Imports
import numpy as np

# Motion Simulation Params

dt = 0.1  # Time step for motion simulation: in seconds
t_start, t_end = 0, 20 # start and end times for motion simulation: in secondds

# function to generate a sample acceleration
def get_acceleration(t: float):
    if 0 < t < 5:
        return 0, 1, 0
    elif 5 < t < 10:
        return 1, 0, 0
    elif 10 < t < 15:
        return 0, -1, 0
    elif 15 < t < 20:
        return -1, 0, 0
    else:
        return 0, 0, 0

# LawnMower Field Params: Refer generalized LawnMower field diagram
l1f = 10
l2f = 1.428
l3f = 1
nf = 2
ntf = 50 # number of time steps between any 2 vertices

# LawnMower bot Params
bv = 1 # bot velocity: in m/s
bt0 = 0.0 # initial timestamp for bot's measuring scale
bx0 = 0.0 # initial bot's x position
by0 = 0.0 # initial bot's y position
bz0 = 0.0 # initial bot's z position

bvx0 = 0.0 # initial bot's x velocity
bvy0 = 0.0 # initial bot's y velocity
bvz0 = 0.0 # initial bot's z velocity

bth0 = 90.0 # initial heading angle for bot

# Sensor Params

## Sensor 1 Params:
s1xn = 0.1 # in metres
s1yn = 0.1 # in metres
s1zn = 0.1 # in metres
s1thn = 1.0 # in degrees
s1nm = 2 # noise multiplier
S1U = 2.5 # update rate in Hz
S1R = -1*np.eye(3) # transformation matrix from GCF -> S1 frame
S1T = np.array([-10, -10, -100]) # translation vector from GCF->S1
S1M = False # True means sensor is moving, False means fixed

## Sensor 2 Params
### no need to define noise parameters, since we already know S1 is twice as accurate, hence S2 will have twice ass noise parameters
S2U = 100 # update rate in Hz
S2R = np.eye(3) # transformation matrix from GCF->S2 frame: dummy value
S2T = np.zeros(3) # translation vector from GCF->S2 frame: dummy value
S2M = True # moving sensor: overrides S2R and S2T and computes these in while simulating

# Sensor Fusion Params
w1 = 2 # system 1 is twice as accurate, hence prioritize outputs from S1
w2 = 1 # weight for system 2 measurements