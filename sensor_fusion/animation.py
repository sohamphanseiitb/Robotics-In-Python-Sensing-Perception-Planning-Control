"""
This Python script animates the bot motion along the generated trajectory, and provides the output in the form of a .gif file.
"""

# standard imports
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

# Custom imports
## Get the directory of the current script
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
module_dir = os.path.dirname(parent_dir)
sys.path.append(module_dir)

from dynamics import simulate_trajectory
from sensor_simulation import sensor_simulate

"""
DOCSTRING:
Function Name: animate
Function Inputs: trajectory
    1. trajectory: Dictionary containing the trajectory information
Purpose: This function animates the robot motion along the generated trajectory
Function Output: Gives back an animated .gif file of the robot motion along the serpentine trajectory"""

def animate(trajectory: dict, path: str):

    # unpack the trajectory dictionary
    time = trajectory['time']
    x = trajectory['x']
    y = trajectory['y']
    z = trajectory['z']
    theta_z = trajectory['theta_z']

    # Create figure for animation
    fig, ax = plt.subplots(figsize=(8, 6))  # Adjusted figure size
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Lawn Mower Path Animation")
    ax.grid(True)

    # Plot the full path
    ax.plot(x, y, "r--", alpha=0.5, label="Path")

    # Create robot representation as a rectangle
    robot_width = 0.5
    robot_length = 1.0  # Elongated for orientation visibility
    robot = patches.Rectangle((x[0] - robot_length / 2, y[0] - robot_width / 2), robot_length, robot_width, fc="blue", edgecolor="black", label="Robot")
    ax.add_patch(robot)

    # co-ordinates for text as the legend: current co-ordinates of the robot
    coord_text = ax.text(0.95, 0.95, f"Frame: 0\nX: {x[0]:.2f}\nY: {y[0]:.2f}\nTheta: {theta_z[0]:.2f}",
                    transform=ax.transAxes, fontsize=10, verticalalignment='top', horizontalalignment='right',
                    bbox=dict(facecolor='white', alpha=0.7, edgecolor='grey', boxstyle='round,pad=0.5'))

    # Update function for animation
    def update(frame):
        robot.set_xy((x[frame] - robot_length / 2, y[frame] - robot_width / 2))
        robot.angle = theta_z[frame]
        coord_text.set_text(f"Frame: {frame}\nX: {x[frame]:.2f}\nY: {y[frame]:.2f}\nZ: {z[frame]:.2f}\nTheta: {theta_z[frame]:.2f}")
        return robot, coord_text

    # Create animation
    ani = animation.FuncAnimation(fig, update, frames=len(x), interval=50, blit=True)

    # Save animation as GIF
    gif_path = path
    ani.save(gif_path, writer="pillow", fps=20)

    plt.show()