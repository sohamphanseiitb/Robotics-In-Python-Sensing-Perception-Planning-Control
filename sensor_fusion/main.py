"""
DOCSTRING: Main file for the project
Purpose: This file will integrate all the functions, and the aspects of the problem: 
    - ground truth generation
    - sensor measurement simulations
    - sensor fusion and error metrics

"""

# standard imports
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

# Custom imports
from dynamics import simulate_trajectory
from sensor_simulation import sensor_simulate
from class_signatures import *
from animation import animate
from params import *

# Main function

# Initialize the LawnMowerField
field = LawnMowerField(l1f, l2f, l3f, nf, ntf)

# Initialize the LawnMowerRobot
bot = LawnMowerRobot(bv, bt0, bx0, by0, bz0, bvx0, bvy0, bvz0, bth0)

# Create a trajectory for the bot to follow
timestamps = np.arange(t_start, t_end, dt)
acceleration_x = np.array([get_acceleration(ti)[0] for ti in timestamps])
acceleration_y = np.array([get_acceleration(ti)[1] for ti in timestamps])
acceleration_z = np.array([get_acceleration(ti)[2] for ti in timestamps])

# Initialize a dynamics model
model = DynamicsModel2D(dt, timestamps, bot.initial_cords, acceleration_x, acceleration_y, acceleration_z)

# Simulate motion and store trajectory
model.simulate_motion()
model_trajectory = {'time': model.time, 'x': model.motion_x, 'y': model.motion_y, 'z': model.motion_z, 'theta_z': model.heading}

# Generate ground truth trajectory -> store generated trajectory in bot.trajectory
bot.trajectory = model_trajectory   # use for human generated trajectory based on acceleration inputs
#bot.generate_trajectory(field) # use for parameterized serpentine trajeectory

# Initialize the Sensors
S1noise = np.array([s1xn, s1yn, s1zn, s1thn])*s1nm
S1 = Sensor(S1U, S1R, S1T, S1noise, 'linear', "S1", S1M)

S2noise = S1noise*2
S2 = Sensor(S2U, S2R, S2T, S2noise, 'linear', "S2", S2M)

# Simulate the sensors
S1.simulate_sensor(bot)
S2.simulate_sensor(bot)

# Convert the sensor trajectory back to GCF -> in order to compare with ground truth
S1.convert_sensor_to_gcf(bot)
S2.convert_sensor_to_gcf(bot)

# Plot and Compare
plotter = Plotter(bot.trajectory, S1.converted_traj_gcf, S1)
plotter.plot_and_compare()

# Plot and Compare
plotter = Plotter(bot.trajectory, S2.converted_traj_gcf, S2)
plotter.plot_and_compare()

# Animate the robot motion for ground truth and sensor data
#gt_anime_path = "D:\\Career\\Interviews\\GncCoding_Soham\\documentation\\bot_dynamics\\lawn_mover_animation_gcf.gif"
#sensor_anime_path_S1 = "D:\\Career\\Interviews\\GncCoding_Soham\\documentation\\bot_dynamics\\lawn_mover_animation_sensor_frame_S1.gif"
#sensor_anime_path_S2 = "D:\\Career\\Interviews\\GncCoding_Soham\\documentation\\bot_dynamics\\lawn_mover_animation_sensor_frame_S2.gif"

# Animate the ground truth : Initialize an Animator object
#gt_animator = Animator(bot.trajectory, gt_anime_path)
#gt_animator.animate_motion()

# Perform weighted fusion
SF = Sensor_Fusion(S1, S2, w1, w2, bot)
SF.fusion()
SF.error_metrics()

# Plot results
plt.figure(figsize=(10, 6))
plt.plot(bot.trajectory['x'], bot.trajectory['y'], label='Ground Truth', linestyle='dashed', color='black')
plt.scatter(S1.converted_traj_gcf['x'], S1.converted_traj_gcf['y'], label='Sensor 1', s=5, color='blue', alpha=0.6)
plt.scatter(S2.converted_traj_gcf['x'], S2.converted_traj_gcf['y'], label='Sensor 2', s=5, color='red', alpha=0.6)
plt.scatter(SF.fused_trajectory['x'], SF.fused_trajectory['y'], label='Fused Trajectory', s=5, color='orange', alpha=0.6)
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.legend()
plt.title("Sensor Fusion of Trajectory Data")
plt.grid()
plt.show()