from base import Gaussian, system
from ekf import EKF
import numpy as np
import matplotlib.pyplot as plt

# User Section: Change Simulation parameters
# unchanging simulation parameters
dt, Nsims_ekf, g, qc_ekf = 1e-2, 500, 9.8, 0.1
ndim = 2
time_ekf = np.linspace(0, Nsims_ekf * dt, Nsims_ekf)
x0_ekf = np.array([1.50, 0])
obs_names_ekf = ['observed s1: angle', 'observed s2: angular rate']
state_names_ekf = ['state0', 'state1']
O_ekf = np.array([[1, 0]])                                              # observation matrix
mm_ekf = np.zeros(2)                                                    # process model noise mean
Q_ekf = np.array([[qc_ekf*dt**3/3, qc_ekf*dt**2/2], 
                  [qc_ekf*dt**2/2, qc_ekf*dt]])                         # process model noie covariance
om_ekf = 0.0                                                            # measurement model noise mean

# changing simulation parameters
deltas_ekf = 1
Rs_ekf = 0.1
colors = ['blue', 'green']

def motion_model_pendulum(state, params):
    g, dt = params[11], params[2]
    return np.array([state[0] + state[1]*dt, state[1] - g*np.sin(state[0])*dt])

def motion_model_Jacobian_pendulum(state, params):
    g, dt = params[11], params[2]
    return np.array([[1., dt], [-g * np.cos(state[0]) * dt, 1.]])

def observation_model_Jacobian_pendulum(state, params):
    g, dt = params[11], params[2]
    return np.array([[np.cos(state[0]), 0.]])

parameters = [time_ekf, Nsims_ekf, dt, deltas_ekf, x0_ekf, obs_names_ekf, O_ekf, mm_ekf, Q_ekf, om_ekf, Rs_ekf, g, state_names_ekf, ndim, deltas_ekf, Rs_ekf]

# Main Code: no need to touch
pendulum = system(parameters, motion_model_pendulum, motion_model_Jacobian_pendulum, observation_model_Jacobian_pendulum)
pendulum.generate_data()
pendulum.obsData
estimator = EKF(pendulum)
estimator.ekf()
estimator.plot(colors)
