import numpy as np
import matplotlib.pyplot as plt
from dynamics import simulate_trajectory
from sensor_simulation import sensor_simulate
from animation import animate
from scipy.interpolate import interp1d

"""
DOCSTRING:
Class Name: LawnMowerField
Purpose: This class carries the parameters related to the LawnField
Class Inputs: Refer to : Generalized Lawn Field Parameters figure from DOC.md to know more about each
1. Field Dimensions: l1, l2, l3, n
2. Number of Time steps between any 2 vertices: dt

"""
class LawnMowerField():
    def __init__(self, l1: float, l2: float, l3: float, n: int, dt: int):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.n = n
        self.dt = dt


"""
DOCSTRING:
Class Name: LawnMowerRobot
Purpose: This class carries the parameters related to the LawnMowerRobot
Class Inputs:
1. Robot velocity: v   (m/s)
2. Initial co-ordinates: t0, x0, y0, z0, theta_z0
"""
class LawnMowerRobot():
    def __init__(self, v: float, t0: float, x0: float, y0: float, z0: float, vx0: float, vy0: float, vz0: float, theta_z0: float):
        self.initial_cords = [t0, x0, y0, z0, vx0, vy0, vz0, theta_z0]
        self.v = v


    # Add a method for ground truth trajectory generation
    def generate_trajectory(self, field: LawnMowerField):
        # Generate the trajectory
        time, x, y, z, theta_z = simulate_trajectory(field.l1, field.l2, field.l3, self.v, field.n, field.dt, self.initial_cords)
        self.trajectory = {'time': time, 'x': x, 'y': y, 'z': z, 'theta_z': theta_z}

"""
DOCSTRING:
Class Name: Sensor
Purpose: This class carries the parameters and measurement scripts related to the Sensor
Class Inputs:
1. Sensor data rate: data_rate
2. Sensor transformation matrix: R
3. Sensor translation vector: T
4. Sensor noise model parameters: noise_params
5. Interpolation method: interp_method
"""
class Sensor():
    def __init__(self, data_rate: float, R: np.array, T: np.array, noise_params: np.array, interp_method: str, name: str, moving_sensor: bool):
        self.data_rate = data_rate
        self.R = R
        self.T = T
        self.noise_params = noise_params
        self.interp_method = interp_method
        self.name = name
        self.moving_sensor = moving_sensor
    
    # Add a method for sensor simulation
    def simulate_sensor(self, robot: LawnMowerRobot):
            
        # Simulate the sensor
        self.sensor_trajectory, self.R_GCF_to_mounted_sensor, self.T_GCF_mounted_sensor = sensor_simulate(robot.trajectory, self.data_rate, self.R, self.T, self.noise_params, robot.v, self.interp_method, self.moving_sensor)
    
    # Add a method to convert from to GCF from Sensor Frame
    ## Takes in sensor trajectory and returns the GCF frame trajectory
    def convert_sensor_to_gcf(self, robot: LawnMowerRobot):

        # Unpack the sensor trajectory
        time = self.sensor_trajectory['time']
        x = self.sensor_trajectory['x']
        y = self.sensor_trajectory['y']
        z = self.sensor_trajectory['z']
        theta_z = self.sensor_trajectory['theta_z']

        if self.moving_sensor == False:

            # Form a new trajectory dictionary for GCF data
            self.converted_traj_gcf = {'time': [], 'x': [], 'y': [], 'z': [], 'theta_z': []}

            for i in range(len(time)):

                # Static sensor: Use precomputed values
                sensor_to_gcf_R = np.linalg.inv(self.R)
                sensor_to_gcf_T = -self.T

                # Apply rotation and translation to get GCF coordinates
                gcf_vec = np.dot(sensor_to_gcf_R, np.array([x[i], y[i], z[i]])) + sensor_to_gcf_T

                # Store in the trajectory dictionary
                self.converted_traj_gcf['time'].append(time[i])
                self.converted_traj_gcf['x'].append(gcf_vec[0])
                self.converted_traj_gcf['y'].append(gcf_vec[1])
                self.converted_traj_gcf['z'].append(gcf_vec[2])

                # Transform heading angle from the sensor frame to GCF
                vx = robot.v * np.cos(np.radians(theta_z[i]))
                vy = robot.v * np.sin(np.radians(theta_z[i]))

                # Transform velocity vector
                gcf_v = np.dot(sensor_to_gcf_R, np.array([vx, vy, 0]))
                theta_gcf = np.degrees(np.arctan2(gcf_v[1], gcf_v[0]))
                self.converted_traj_gcf['theta_z'].append(theta_gcf)
        
        else:

            # Form a new trajectory dictionary for GCF data
            self.converted_traj_gcf = {'time': [], 'x': [], 'y': [], 'z': [], 'theta_z': []}

            for i in range(len(time)):

                # Static sensor: Use precomputed values
                sensor_to_gcf_R = np.linalg.inv(self.R_GCF_to_mounted_sensor[i])
                sensor_to_gcf_T = self.T_GCF_mounted_sensor[i]

                # Apply rotation and translation to get GCF coordinates
                gcf_vec = np.dot(sensor_to_gcf_R, np.array([x[i], y[i], z[i]])) + sensor_to_gcf_T

                # Store in the trajectory dictionary
                self.converted_traj_gcf['time'].append(time[i])
                self.converted_traj_gcf['x'].append(gcf_vec[0])
                self.converted_traj_gcf['y'].append(gcf_vec[1])
                self.converted_traj_gcf['z'].append(gcf_vec[2])

                # Transform heading angle from the sensor frame to GCF
                vx = robot.v * np.cos(np.radians(theta_z[i]))
                vy = robot.v * np.sin(np.radians(theta_z[i]))

                # Transform velocity vector
                gcf_v = np.dot(sensor_to_gcf_R, np.array([vx, vy, 0]))
                theta_gcf = np.degrees(np.arctan2(gcf_v[1], gcf_v[0]))
                self.converted_traj_gcf['theta_z'].append(theta_gcf)
    
        # sensor trajectory: self.sensor_trajectory
        # GCF -> Sensor Transformation was as follows:
        # [xs, ys, zs] = R * [xg, yg, zg] + T
        # vxgcf = v * cos(theta_zg)
        # vygcf = v * sin(theta_zg)
        # vs = R * [vxgcf, vygcf, 0]
        # theta_s = atan2(vs[1], vs[0])

"""
DOCSTRING:
Class Name: Animator
Purpose: This class contains parameters and methods for animating the robot motion
Class Inputs:
1. trajectory: dictionary containing the trajectory information
2. path: path to save the animated gif
"""
class Animator():
    
    def __init__(self, trajectory: dict, path: str):
        self.trajectory = trajectory
        self.path = path
    
    def animate_motion(self):
        animate(self.trajectory, self.path)

"""
DOCSTRING:
Class Name: Plotter
Purpose: This class contains parameters and methods for plotting trajectories
Class Inputs:
1. trajectory: dictionary containing the trajectory information
2. sensor_trajectory: dictionary containing the sensor trajectory information
"""
class Plotter():

    def __init__(self, trajectory: dict, sensor_trajectory: dict, sensor: Sensor):
        self.trajectory = trajectory
        self.sensor_trajectory = sensor_trajectory
        self.sensor = sensor
    
    def plot_and_compare(self):

        fig, ax = plt.subplots(1, 3, figsize=(12, 6))  # Adjusted figure size, 
        fig.suptitle("Ground Truth vs Sensor Data %s" % self.sensor.name)
        
        ax[0].set_xlabel("X (m)")
        ax[0].set_ylabel("Y (m)")
        ax[0].grid(True)
        ax[0].plot(self.trajectory['x'], self.trajectory['y'], "r--", alpha=0.5, label="Ground Truth")
        ax[0].plot(self.sensor_trajectory['x'], self.sensor_trajectory['y'], "b--", alpha=0.5, label="Sensor Data %s" % self.sensor.name)
        ax[0].legend()
        
        ax[1].set_xlabel("Time (s)")
        ax[1].set_ylabel("Z (m)")
        ax[1].grid(True)
        ax[1].plot(self.trajectory['time'], self.trajectory['z'], "r--", alpha=0.5, label="Ground Truth")
        ax[1].plot(self.sensor_trajectory['time'], self.sensor_trajectory['z'], "b--", alpha=0.5, label="Sensor Data %s" % self.sensor.name)
        ax[1].legend()

        ax[2].set_xlabel("Time (s)")
        ax[2].set_ylabel("Heading Angle (rad)")
        ax[2].grid(True)
        ax[2].plot(self.trajectory['time'], self.trajectory['theta_z'], "r--", alpha=0.5, label="Ground Truth")
        ax[2].plot(self.sensor_trajectory['time'], self.sensor_trajectory['theta_z'], "b--", alpha=0.5, label="Sensor Data %s" % self.sensor.name)
        ax[2].legend()

        plt.show()

"""
DOCSTRING:
Class Name: DynamicsModel2D
Purpose: This class contains a simple 2D dynamics model which simulates a 2D planar motion of the LawnMower
Class Inputs:
1. timestamps: timestamp vector for the entire duration of the motion simulator
2. acceleration_x: acceleration in the x direction for the entire time duration
3. acceleration_y: acceleration in the y direction for the entire time duration
4. acceleration_z: acceleration in the z direction for the entire time duration 

Runs on a simple kinematic model assuming the lawnmower to be a point mass. 

Class Outputs:
1. motion_x: x co-ordinates during entire motion duration
2. motion_y: y co-ordinates during entire motion duration
3. motion_z: z co-ordinates during entire motion duration
4. velx: vx co-ordinates during entire motion duration
5. vely: vy co-ordinates during entire motion duration
6. velz: vz co-ordinates during entire motion duration
7. heading: heading angle during entire motion duration
"""
class DynamicsModel2D:
    def __init__(self, dt: float, timestamps: np.array, initial_cords: np.array, acceleration_x: np.array, acceleration_y: np.array, acceleration_z: np.array):
        
        # system states
        self.dt = dt
        self.x = initial_cords[0]
        self.y = initial_cords[1]
        self.z = initial_cords[2]
        self.vx = initial_cords[3]
        self.vy = initial_cords[4]
        self.vz = initial_cords[5]
        
        # system inputs
        self.time = timestamps
        self.accelx = acceleration_x
        self.accely = acceleration_y
        self.accelz = acceleration_z
        
        # system outputs
        self.motion_x = []
        self.motion_y = []
        self.motion_z = []
        self.heading = []

        self.velx = []
        self.vely = []
        self.velz = []

    def update(self, ax: float, ay: float, az: float):
        self.x = self.x + self.vx * self.dt + 0.5 * ax * self.dt**2
        self.y = self.y + self.vy * self.dt + 0.5 * ay * self.dt**2
        self.z = self.z + self.vz * self.dt + 0.5 * az * self.dt**2

        self.vx = self.vx + ax * self.dt
        self.vy = self.vy + ay * self.dt
        self.vz = self.vz + az * self.dt
        return self.x, self.y, self.z, self.vx, self.vy, self.vz

    def simulate_motion(self):
        for i in range(len(self.time)):
            self.next_state = self.update(self.accelx[i], self.accely[i], self.accelz[i])

            self.motion_x.append(self.next_state[0])
            self.motion_y.append(self.next_state[1])
            self.motion_z.append(self.next_state[2])
            # compute heading in 2D:
            self.heading.append(np.degrees(np.arctan2(self.vy, self.vx)))

            self.velx.append(self.next_state[3])
            self.vely.append(self.next_state[4])
            self.velz.append(self.next_state[5])

"""
DOCSTRING:
Class Name: Sensor_Fusion

Class Inputs:
1. Sensor1: Sensor() instance
2. Sensor2: Sensor() instance
3. w1: weight for Sensor1 measurements
4. w2: weight for Sensor2 measurements

Class Outputs: 
1. self.fused_trajectory: Dictionary containing fused x, y, z and theta_z
2. self.error_metrics(): Computes and returns error metrics (MSE, RMSE, MAE, Max Error)

"""
class Sensor_Fusion():

    def __init__(self, Sensor1: Sensor, Sensor2: Sensor, w1: float, w2: float, robot: LawnMowerRobot):

        self.S1 = Sensor1
        self.S2 = Sensor2
        self.common_timestamps = np.union1d(self.S1.converted_traj_gcf['time'], self.S2.converted_traj_gcf['time'])
        self.w1 = w1
        self.w2 = w2
        self.bot = robot
    
    def interpolate_sensor_data(self, Sensor_trajectory: dict):
        interp_funcs = {key: interp1d(Sensor_trajectory['time'], Sensor_trajectory[key], kind='linear', fill_value='extrapolate') 
                    for key in ['x', 'y', 'z', 'theta_z']}   
        interpolated_data = {key: interp_funcs[key](self.common_timestamps) for key in interp_funcs} 
        return interpolated_data
    
    def fusion(self):
        self.S1_interp = self.interpolate_sensor_data(self.S1.converted_traj_gcf)
        self.S2_interp = self.interpolate_sensor_data(self.S2.converted_traj_gcf)

        self.fused_trajectory = {key: (self.w1 * self.S1_interp[key] + self.w2 * self.S2_interp[key]) / (self.w1 + self.w2) for key in ['x', 'y', 'z', 'theta_z']}
        self.fused_trajectory['time'] = self.common_timestamps
    
    def error_metrics(self):
        # Interpolate ground truth onto the common timestamps
        gt_interp_funcs = {key: interp1d(self.bot.trajectory['time'], self.bot.trajectory[key], kind='linear', fill_value='extrapolate') 
                           for key in ['x', 'y', 'z', 'theta_z']}
        gt_interp = {key: gt_interp_funcs[key](self.common_timestamps) for key in gt_interp_funcs}

        # Compute error metrics
        errors = {key: self.fused_trajectory[key] - gt_interp[key] for key in ['x', 'y', 'z', 'theta_z']}

        mse = {key: np.mean(errors[key]**2) for key in errors}
        rmse = {key: np.sqrt(mse[key]) for key in mse}
        mae = {key: np.mean(np.abs(errors[key])) for key in errors}
        max_error = {key: np.max(np.abs(errors[key])) for key in errors}

        self.metrics = {
            'MSE': mse,
            'RMSE': rmse,
            'MAE': mae,
            'Max Error': max_error
        }

        return self.metrics