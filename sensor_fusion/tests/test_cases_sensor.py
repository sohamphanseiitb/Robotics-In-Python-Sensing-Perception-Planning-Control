"""
DOCSTRING:

For sensor:
- test if generated sensor trajectory is right
- test if GCF to sensor transformation matrix, and translation vector are correct
- test if sensor frame to GCF is right
- test for both moving and mounted sensor
"""   
from test_sensor import SensorTest

import numpy as np
import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from class_signatures import Sensor_Fusion, Sensor, LawnMowerField, LawnMowerRobot, DynamicsModel2D

# create a ground truth with bot
l1f, l2f, l3f, nf, ntf = 10, 1.428, 1, 2, 50
field = LawnMowerField(l1f, l2f, l3f, nf, ntf)

bth0 = 90.0 # initial heading angle for bot
bv, bt0, bx0, by0, bz0, bvx0, bvy0, bvz0, bth0 = 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 90.0
bot = LawnMowerRobot(bv, bt0, bx0, by0, bz0, bvx0, bvy0, bvz0, bth0)

# Create a trajectory for the bot to follow
from params import get_acceleration
t_start, t_end, dt = 0, 20, 0.1
timestamps = np.arange(t_start, t_end, dt)
acceleration_x = np.array([get_acceleration(ti)[0] for ti in timestamps])
acceleration_y = np.array([get_acceleration(ti)[1] for ti in timestamps])
acceleration_z = np.array([get_acceleration(ti)[2] for ti in timestamps])

# Initialize a dynamics model
model = DynamicsModel2D(dt, timestamps, bot.initial_cords, acceleration_x, acceleration_y, acceleration_z)

# Simulate motion and store trajectory
model.simulate_motion()
model_trajectory = {'time': model.time, 'x': model.motion_x, 'y': model.motion_y, 'z': model.motion_z, 'theta_z': model.heading}
bot.trajectory = model_trajectory

# Create fixed sensor
s1xn, s1yn, s1zn, s1thn, s1nm = 0.5, 0.5, 0.5, 0.5, 3
S1U, S1R, S1T = 10, -1*np.eye(3), np.array([-1, -1, -1])
S1noise = np.array([s1xn, s1yn, s1zn, s1thn])*s1nm
S1M = False
S1 = Sensor(S1U, S1R, S1T, S1noise, 'linear', "S1", S1M)
S1.simulate_sensor(bot) # sensor trajectory
S1.convert_sensor_to_gcf(bot)

# we check final position, and the matrices
expected_traj = np.array([bot.trajectory['x'][-1], bot.trajectory['y'][-1], bot.trajectory['z'][-1], bot.trajectory['theta_z'][100]])
S1_expected_results = [expected_traj, [], []]
sens_test1 = SensorTest(S1.converted_traj_gcf, S1.R_GCF_to_mounted_sensor, S1.T_GCF_mounted_sensor, S1_expected_results)