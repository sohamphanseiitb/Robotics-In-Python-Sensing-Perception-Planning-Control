"""
DOCSTRING:
This file implements several base classes for ease of implementation

"""

# Define some data structures for better code syntax.
from dataclasses import dataclass
import numpy as np
from typing import Tuple, List, Optional

@dataclass
class Gaussian:
    mean: np.ndarray
    cov: np.ndarray

@dataclass
class Observations:
    times: np.ndarray
    obs_ind: np.ndarray  # index of times that are observed
    obs: np.ndarray
    names: List[str]

@dataclass
class KFTracker:
    means: np.ndarray
    covs: np.ndarray
    stds: np.ndarray


class system():

    """
    params: times, N, dt, obs_freq, x0, obs_names, O, mm, Q, om, R,   g, state_names, ndim, deltas_ekf, Rs_ekf
            0,     1,  2,        3,  4,         5, 6,  7, 8,  9, 10, 11,          12,    13,        14,     15, 
    """

    def __init__(self, params, motion_model, motion_model_J, observation_model_J):
        self.params = params
        self.motion = motion_model
        self.motionJ = motion_model_J
        self.observationJ = observation_model_J
        self.process_noise = Gaussian(self.params[7], np.sqrt(self.params[8]))
        self.measure_noise = Gaussian(self.params[9], np.sqrt(self.params[10]))
        self.prior = Gaussian(params[4], np.eye(params[13]))
    
    def propagate_motion(self, stateC, paramsC):
        return self.motion(stateC, paramsC)
    
    def propagate_model_J(self, stateC, paramsC):
        return self.motionJ(stateC, paramsC)

    def propagate_observation(self, stateC, paramsC):
        return self.observationJ(stateC, paramsC)
    
    def generate_data(self):
        self.state = np.zeros((self.params[1], 2))
        self.state[0, :] = self.params[4]

        self.obs_ind = np.arange(self.params[3], self.params[1], self.params[3])
        self.num_obs = self.obs_ind.shape[0]
        self.on_obs = 0
        self.yout = np.zeros((self.num_obs, self.params[6].shape[0]))

        for i in range(1, self.params[1]):
            self.state[i, :] = self.propagate_motion(self.state[i-1, :], self.params)
            if self.on_obs < self.num_obs and i == self.obs_ind[self.on_obs]:
                self.yout[self.on_obs, :] = np.dot(self.params[6], self.state[i, :]) + np.random.normal(self.params[9], np.sqrt(self.params[10]))
                self.on_obs += 1
        
        self.trueData = Observations(self.params[0], np.arange(self.params[1]), self.state, self.params[12])
        self.obsData = Observations(self.params[0], self.obs_ind, self.yout, self.params[5])