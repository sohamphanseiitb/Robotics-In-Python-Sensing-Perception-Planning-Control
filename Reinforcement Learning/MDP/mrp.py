"""
This file will implement the Markov Reward Process (MDP) class.
"""

import numpy as np
from typing import Callable, Tuple

class MRP():

    """
    ns: number of states in the MRP
    allStates: all states
    STF: state transition function
    StateDict: dictionary of states ['state name': state index]
    STM: state transition matrix
    ExpcRewardFunc: reward function (expected value of E[R_{t+1} | S_t = s])
    RewardDict: dictionary of rewards
    gamma: discount factor
    S0: initial state
    """
    def __init__(self, 
                 ns: int,
                 allStates: list[int],
                 STF: Callable[[np.ndarray, int, list[int]], int],
                 StateDict: dict[str, int], 
                 STM: np.ndarray, 
                 ExpcRewardFunc: Callable[[int, np.ndarray, dict], float], 
                 RewardDict: dict[str, float],
                 gamma: float):

        self.ns, self.allStates, self.STF, self.StateDict, \
        self.STM, self.ExpcRewardFunc, self.RewardDict, \
        self.gamma = ns, allStates, STF, StateDict, STM, ExpcRewardFunc, RewardDict, gamma

        self.SInvD = {v: k for k, v in self.StateDict.items()}
        self.RInvD = {v: k for k, v in self.RewardDict.items()}
     
    """
    Inputs:
    1. ts: Number of time steps to simulate, an integer.
    2. S0: int, initial state to start simulation from
    """
    def simulateMRP(self, 
                    ts: int, 
                    S0: int) -> Tuple[np.ndarray, np.ndarray]:

        # create arrays to store consecutive states
        states = np.zeros(ts, dtype=int)
        rewards = np.zeros(ts-1, dtype=float)

        states[0] = S0

        for i in range(1, ts):

            scurr = states[i-1] # current state
            states[i] = self.STF(self.STM, scurr, self.allStates) # next state
            rewards[i-1] = self.RewardDict[self.SInvD[states[i-1]]]

        return states, rewards
    
      
    """
    Inputs:
    1. ts: Number of time steps to simulate, an integer.
    2. S0: int, initial state to start the simulation from

    Return: G_t = R_{t+1} + gamma * R_{t+2} + ... + gamma^{n-1} * R_{t+n}
    G_0 = R_1 + gamma * R_2 + ... + gamma^{n-1} * R_{n} 
    G_1 = R_2 + gamma * R_3 + ... + gamma^{n-1} * R_{n+1}

    R_1 + gamma * G_1 = R_1 + gamma * R_2 + gamma^2 * R_3 + gamma^3 * R_4 + ... + gamma^{n} * R_{n+1}
                      = G_0 
    Outputs:
    1. State sequence
    2. reward sequence
    3. return sequence
    """
    def computeReturn(self, ts: int, S0: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:

        states, rewards = self.simulateMRP(ts=ts, S0=S0)
        returns = np.zeros_like(rewards)
        returns[-1] = rewards[-1] # return for the last step is the reward of the last step, because only 1 reward to sum, nothin to discount
        for t in range(len(rewards) - 2, -1, -1):
            returns[t] = rewards[t] + self.gamma * returns[t+1]

        return states, rewards, returns

    """
    First principles method:
    V(s) = E[G_t | S_t = s] = E[R_{t+1} + gamma * V(S_{t+1}) | S_t = s]
                            = E[R_{t+1} | S_t = s] + gamma * E[V(S_{t+1}) | S_t = s]
                            = ExpcRewardFunc(s) + gamma * sum_{s'} STM(s, s') * V(s')
    Inputs:
    1. N: number of times to simulate the MRP for each state, integer
    """
    def computeValueFunction(self, N: int = 100):

        # we need to compute value of each state, so we will have an array of size (ns, )
        self.V = np.zeros(self.ns)

        # for each state, we need to simulate the MRP, N times startgin from that state, and take the average return
        for each in self.allStates:

            returns = np.zeros(N) # for each state, we will carry out N simulations, and in each simulation we will save G_0
            for i in range(N):
                states, rewards, retSeq = self.computeReturn(ts=100, S0=each)
                returns[i] = retSeq[0]

            self.V[each] = np.mean(returns)
        