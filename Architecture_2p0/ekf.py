"""
DOCSTRING: 
This file implements an extended kalman filter class and its related metthods and functions:
Class Inputs:
Class Features:
"""

# Library Imports
from typing import Tuple, List, Optional
from dataclasses import dataclass
from functools import partial
import numpy as np
import matplotlib.pyplot as plt

from base import Gaussian, KFTracker, Observations, system

class EKF():

    def __init__(self, system):
        self.system = system
    
    def predict(self, X: Gaussian, xi: Gaussian):

        self.predicted_mean = self.system.motion(X.mean, self.system.params) + xi.mean
        self.J = self.system.motionJ(X.mean, self.system.params)
        self.predicted_cov = self.J @ X.cov @ self.J.T + xi.cov
        return Gaussian(self.predicted_mean, self.predicted_cov)
    
    def update(self, data: np.ndarray, X: Gaussian, eta: Gaussian):

        self.H = self.system.observationJ(X.mean, self.system.params)
        self.U = np.dot(X.cov, self.H.T)
        self.S = np.dot(self.H, self.U) + eta.cov
        self.mu = np.dot(self.H, X.mean) + eta.mean

        self.K  = np.linalg.solve(self.S, self.H@X.cov).T
        self.updated_mean = X.mean + self.K@np.atleast_1d(data - self.mu)
        self.updated_cov = X.cov - self.K@self.S@self.K.T

        return Gaussian(self.updated_mean, self.updated_cov)
    
    def ekf(self):

        self.n_steps = self.system.obsData.times.shape[0]
        self.d = self.system.prior.mean.shape[0]

        self.mean_store = np.zeros((self.n_steps, self.d))
        self.mean_store[0, :] = np.copy(self.system.prior.mean)

        self.cov_store = np.zeros((self.n_steps, self.d, self.d))
        self.cov_store[0, :, :] = np.copy(self.system.prior.cov)

        self.std_store = np.zeros((self.n_steps, self.d))
        self.std_store[0, :] = np.sqrt(np.diag(self.cov_store[0, :, :]))

        self.Xnext = self.system.prior
        self.on_obs = 0

        for i in range(1, self.n_steps):

            self.Xpred = self.predict(self.Xnext, self.system.process_noise)

            if self.on_obs < self.system.obsData.obs_ind.shape[0] and i == self.system.obsData.obs_ind[self.on_obs]:

                self.y = self.system.obsData.obs[self.on_obs]
                self.on_obs += 1

                self.Xup = self.update(self.y, self.Xpred, self.system.measure_noise)
                self.Xnext = self.Xup
            
            else:
                self.Xnext = self.Xpred
            
            self.mean_store[i, :] = np.copy(self.Xnext.mean)
            self.cov_store[i, : ,:] = np.copy(self.Xnext.cov)
            self.std_store[i, :] = np.sqrt(np.diag(self.cov_store[i, :, :]))
        
        self.tracker = KFTracker(self.mean_store, self.cov_store, self.std_store)
        self.RMSE = np.sqrt(np.mean((self.tracker.means - self.system.trueData.obs)**2))
    
    def plot(self, colors):

        plt.plot(self.system.trueData.times, self.system.trueData.obs[:, 0], color=colors[0], label='state0: angle')
        plt.plot(self.system.trueData.times, self.system.trueData.obs[:, 1], color=colors[1], label='state1: angular rate')

        plt.plot(self.system.obsData.times, self.tracker.means[:, 0], '--', color=colors[0], label='est-state0: angle')
        plt.plot(self.system.obsData.times, self.tracker.means[:, 1], '--', color=colors[1], label='est-state1: angular rate')
        plt.fill_between(self.system.obsData.times,
                                self.tracker.means[:, 0] - 2 * self.tracker.stds[:, 0],
                                self.tracker.means[:, 0] + 2 * self.tracker.stds[:, 0],
                                color=colors[0], alpha=0.3)
        plt.fill_between(self.system.obsData.times,
                                self.tracker.means[:, 1] - 2 * self.tracker.stds[:, 1],
                                self.tracker.means[:, 1] + 2 * self.tracker.stds[:, 1],
                                color=colors[1], alpha=0.3)

        plt.xlabel('Time', fontsize=10)
        plt.ylabel('State Estimate',fontsize=10)
        plt.title(f'Delta: {self.system.params[14]}, R: {self.system.params[15]} and RMSE error: {np.round(self.RMSE, 3)}', fontsize=10)


        for ii in range(self.system.obsData.obs.shape[1]):
            plt.scatter(self.system.obsData.times[self.system.obsData.obs_ind], self.system.obsData.obs,
                                color=colors[ii], alpha=0.4, label=self.system.obsData.names[0])

        plt.legend()
        plt.show()

    