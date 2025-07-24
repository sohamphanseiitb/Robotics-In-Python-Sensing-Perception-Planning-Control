"""
Description:
This script will carry out 3 different sub-tasks required for sensor simulation:
1. Match the data rate from the ground truth trajectory to the sensor data rate with time interpolation
2. Incorporate noise models in the ground truth to emulate sensor measurements
3. Transform the ground truth to sensor frame of reference
"""

"""
DOCSTRING:
Function Name: sensor_simulation
Function Inputs: 
1. Ground Truth trajectory data: type: dictionary: ['time', 'x', 'y', 'theta_z']
2. Sensor data rate: type: float (in Hz)
3. Sensor transformation matrix with respect to the Global Co-ordinate frame: R: such that x_sensor = R * x_global (R: 3x3 matrix)
4. Sensor translation vector with respect to the Global Co-ordinate frame: T: such that x_sensor = x_global + T (T: 3x1 array)
5. Sensor noise model parameters: type: 3x1 array: [sigma_x, sigma_y, sigma_z, sigma_theta_z]
6. Robot velocity: type: float (in m/s)
7. Interpolation method: type: string: 'linear' or 'cubic': default value: 'linear'

Purpose:
1. This function will match the ground truth trajectory data rate to the sensor data rate with time interpolation
2. Incorporate noise models in the ground truth to emulate sensor measurements
3. Transform the ground truth to sensor frame of reference

Function Output: Sensor data with noise, matched data rate and transformed to sensor frame of reference:
type: dictionary: ['time', 'x', 'y', 'theta_z']
"""

# Import libraries
import numpy as np
from scipy.interpolate import interp1d

def sensor_simulate(trajectory: dict, data_rate: float, R: np.array, T: np.array, noise_params: np.array, v: float, interp_method: str, moving_sensor: bool):
    
    # Unpack the ground truth trajectory data
    time = trajectory['time']
    x = trajectory['x']
    y = trajectory['y']
    z = trajectory['z']
    theta_z = trajectory['theta_z']

    # 1. Sensor Data Rate Matching

    # convert data rate from hertz to seconds
    dt = 1/data_rate

    time_interp = np.arange(time[0], time[-1], dt)

    # create interpolation functions for x, y and theta_z
    x_interp = interp1d(time, x, kind = interp_method)
    y_interp = interp1d(time, y, kind = interp_method)
    z_interp = interp1d(time, z, kind = interp_method)
    theta_z_interp = interp1d(time, theta_z, kind = interp_method)

    # Perform the interpolation
    x_sensor = x_interp(time_interp)
    y_sensor = y_interp(time_interp)
    z_sensor = z_interp(time_interp)
    theta_z_sensor = theta_z_interp(time_interp)

    # populate interpolated trajectory in GCF in traj_interp dictionary
    traj_interp = {'time': time_interp, 'x': x_sensor, 'y': y_sensor, 'z': z_sensor, 'theta_z': theta_z_sensor}

    # 2. Shift the frames of reference from GCF to the sensor frame
    # form a new trajectory dictionary for sensor data
    traj_sensor = {'time': [], 'x': [], 'y': [], 'z': [], 'theta_z': []}

    # store these transformation matrices
    R_GCF_mounted_sensor = []
    T_GCF_mounted_sensor = []
    
    for i in range(len(time_interp)):

        if moving_sensor== False:
        
            # apply the rotation matrix to the ground truth and also apply translation
            sensor_vec = np.dot(R, np.array([x_sensor[i], y_sensor[i], z_sensor[i]]) + T) 

        else:
            # Compute the robot's heading angle at this time step
            theta = np.radians(theta_z_sensor[i])  # Convert degrees to radians

            # Compute the rotation matrix R (2D rotation about Z-axis for planar motion)
            R = np.array([
                            [np.sin(theta), -np.cos(theta), 0],
                            [np.cos(theta),  np.sin(theta),  0],
                            [0,             0,              1]  # Assuming no tilt
                        ])
            
            R_GCF_mounted_sensor.append(R)

            # Define the sensor offset in the robot frame (fixed w.r.t. robot)
            sensor_offset = np.array([0, 0, 0])  # dx, dy, dz: sensor's position in robot frame

            # Compute the translation vector T (sensor position in the global frame)
            T = np.dot(R, sensor_offset) + np.array([x_sensor[i], y_sensor[i], z_sensor[i]])

            T_GCF_mounted_sensor.append(T)

            # Transform the ground truth to the sensor frame
            sensor_vec = np.dot(R.T, np.array([x_sensor[i], y_sensor[i], z_sensor[i]]) - T)

        # store to traj_sensor dictionary
        traj_sensor['time'].append(time_interp[i])
        traj_sensor['x'].append(sensor_vec[0])
        traj_sensor['y'].append(sensor_vec[1])
        traj_sensor['z'].append(sensor_vec[2])

        # heading angle transformation
        # compute the velocity vector in the GCF
        ## 3D Conversion:
        vx = v * np.cos(np.radians(theta_z_sensor[i]))
        vy = v * np.sin(np.radians(theta_z_sensor[i]))
        vz = 0 # we know this, since the angle measured is in the xy plane only

        # transform velocity vector from the GCF to the sensor frame
        sensor_v = np.dot(R, np.array([vx, vy, vz]))
        theta_sensor = np.degrees(np.arctan2(sensor_v[1], sensor_v[0]))
        traj_sensor['theta_z'].append(theta_sensor)

    # 3. Incorporate noise models in the ground truth to emulate sensor measurements
    # unpack noise models data
    sigma_x, sigma_y, sigma_z, sigma_theta_z = noise_params

    # add noise to the sensor data
    traj_sensor['x'] = np.random.normal(traj_sensor['x'], sigma_x)
    traj_sensor['y'] = np.random.normal(traj_sensor['y'], sigma_y)
    traj_sensor['z'] = np.random.normal(traj_sensor['z'], sigma_z)
    traj_sensor['theta_z'] = np.random.normal(traj_sensor['theta_z'], sigma_theta_z)

    return traj_sensor, R_GCF_mounted_sensor, T_GCF_mounted_sensor