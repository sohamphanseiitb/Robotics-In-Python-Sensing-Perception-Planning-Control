"""
This file will implement the Markov Reward Process (MDP) class.
"""

import numpy as np
from typing import Callable, Tuple

class MRP():

    """
    ns: number of states in the MRP
    allS: all states
    S: state transition function
    Sd: dictionary of states ['state name': state index]
    P: state transition matrix
    R: reward function (expected E[R_{t+1} | S_t = s])
    Rd: dictionary of rewards
    gamma: discount factor
    S0: initial state
    """
    def __init__(self, 
                 ns: int,
                 allS: list[int],
                 S: Callable[[np.ndarray, int, list[int]], int],
                 Sd: dict[str, int], 
                 P: np.ndarray, 
                 R: Callable[[int, np.ndarray, dict], float], 
                 Rd: dict[str, int],
                 gamma: float,
                 S0: int):

        self.ns, self.allS, self.S, self.Sd, \
        self.P, self.R, self.Rd, \
        self.gamma, self.S0 = ns, allS, S, Sd, P, R, Rd, gamma, S0

        self.SInvD = {v: k for k, v in self.Sd.items()}
        self.RInvD = {v: k for k, v in self.Rd.items()}
     
    """
    Inputs:
    1. ts: Number of time steps to simulate, an integer.
    """
    def simulateMRP(self, ts: int) -> Tuple[np.ndarray, np.ndarray]:

        # create arrays to store consecutive states
        states = np.zeros(ts, dtype=int)
        rewards = np.zeros(ts-1, dtype=float)

        states[0] = self.S0

        for i in range(1, ts):

            scurr = states[i-1] # current state
            states[i] = self.S(self.P, scurr, self.allS) # next state
            rewards[i-1] = self.Rd[self.SInvD[states[i]]]

        return states, rewards
    
      
    """
    Inputs:
    1. ts: Number of time steps to simulate, an integer.
    """
    def computeReturn(self, ts: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:

        states, rewards = self.simulateMRP(ts=ts)
        returns = np.zeros_like(rewards)
        returns[-1] = rewards[-1]
        for t in range(len(rewards) - 2, -1, -1):
            returns[t] = rewards[t] + self.gamma * returns[t+1]

        return states, rewards, returns



        