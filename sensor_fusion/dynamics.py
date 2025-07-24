"""
DOCSTRING:

Function Name: Simulates the serpentine robot dynamics
Function Inputs: Check the generalized bot dynamics figure
    1. l1: l(OA) = l(NP): in metres
    2. l2: l(AB) = l(CD) = l(EF) = l(GH) = l(IJ) = l(KL) ... = l(MN): in metres
    3. l3: l(OA) - l(BC) {assuming l(BC) = l(DE) = l(FG) = l(HI) = l(JK) = ... = l(LM)}: in metres
    4. n: Number of repeated passes : integer
    5. dt: Time step for simulation : integer
    6. initial_cords: Initial co-ordinates of the robot: tuple (t, x, y, z, theta_z) 
Function Outputs: trajectories of t, x_pos, y_pos, z_pos, theta_z

Function Author: Soham S. Phanse
"""

# Library Imports
import numpy as np

# Simulate a serpentine trajectory
def simulate_trajectory(l1: float, l2: float, l3: float, v: float, n: int, dt: int, initial_cords: np.array):

    # assign a variable to store trajectory sub-fields
    trajectory = {'time': [], 'x': [], 'y': [], 'z': [], 'theta_z': []}

    current_time, current_x, current_y, current_z, current_vx, current_vy, current_vz, current_theta_z = initial_cords

    """
    DOCSTRING:
    Function Name: add_point
    Function Inputs: t, x, y, theta
    Purpose: This function adds a point to the trajectory
    """
    def add_point(t, x, y, z, theta):
        trajectory['time'].append(t)
        trajectory['x'].append(x)
        trajectory['y'].append(y)
        trajectory['z'].append(z)
        trajectory['theta_z'].append(theta)

    """
    DOCSTRING:
    Function Name: straight
    Function Inputs: duration, end_x, end_y, end_theta_z
        1. duration: Time for which the robot moves in a straight line
        2. end_x: x-coordinate of the end position
        3. end_y: y-coordinate of the end position
        4. end_theta_z: Final orientation of the robot
    Purpose: This function moves the robot in a straight line from the current position to the end position for that particular duration
    """
    def straight(duration, end_x, end_y, end_z, end_theta_z):

        nonlocal current_time, current_x, current_y, current_z, current_theta_z
        start_x = current_x
        start_y = current_y
        start_z = current_z

        time_points = np.linspace(current_time, current_time + duration, dt)  # More frames for smoother motion

        for t in time_points:
            alpha = (t - current_time) / duration
            x = start_x + alpha * (end_x - start_x)
            y = start_y + alpha * (end_y - start_y)
            z = start_z + alpha * (end_z - start_z)
            add_point(t, x, y, z, current_theta_z)
        current_time += duration
        current_x, current_y, current_z, current_theta_z = end_x, end_y, end_z, end_theta_z

    """
    DOCSTRING:
    Function Name: rotate
    Function Inputs: duration, end_theta_z
        1. end_theta_z: Final orientation of the robot
    Purpose: the rotate function rotates the robot to the desired orientation
    """
    def rotate(end_theta_z):
        nonlocal current_time, current_theta_z
        current_theta_z = end_theta_z
        add_point(current_time, current_x, current_y, current_z, current_theta_z)

    # First pass
    add_point(current_time, current_x, current_y, current_z, current_theta_z)
    straight(l1/v, 0, l1, 0, 90)
    rotate(0)
    straight(l2/v, l2, l1, 0, 0)
    rotate(-90)
    straight((l1 - l3)/v, l2, l3, 0, -90)
    rotate(0)
    straight(l2/v, 2*l2, l3, 0, 0)
    rotate(90)

    # 'n' repeated passes
    for i in range(n):
        time_offset_npass = trajectory['time'][-1]
        x_offset_npass = trajectory['x'][-1]

        add_point(time_offset_npass, x_offset_npass, current_y, current_z, current_theta_z)
        current_time = time_offset_npass
        current_x = x_offset_npass

        straight((l1 - l3)/v, x_offset_npass, l1, 0, 90)
        rotate(0)
        straight(l2/v, x_offset_npass + l2, l1, 0, 0)
        rotate(-90)
        straight((l1 - l3)/v, x_offset_npass + l2, l3, 0, -90)
        rotate(0)
        straight(l2/v, x_offset_npass + 2*l2, l3, 0, 0)
        rotate(90)

    # Last pass
    time_offset_last_pass = trajectory['time'][-1]
    x_offset_last_pass = trajectory['x'][-1]

    add_point(time_offset_last_pass, x_offset_last_pass, current_y, current_z, current_theta_z)
    current_time = time_offset_last_pass
    current_x = x_offset_last_pass
    straight((l1 - l3)/v, x_offset_last_pass, l1, 0, 90)
    rotate(0)
    straight(l2/v, x_offset_last_pass + l2, l1, 0, 0)
    rotate(-90)
    straight((l1 - l3)/v, x_offset_last_pass + l2, 0, 0, -90)
    rotate(-180)
    straight(l1/v, 0, 0, 0, -180)
    rotate(90)

    return trajectory['time'], trajectory['x'], trajectory['y'], trajectory['z'], trajectory['theta_z']
