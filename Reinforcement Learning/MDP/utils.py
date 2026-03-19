"""
This file will contain the   state transition matrix, the reward model, and the states
"""

import numpy as np
"""
Inputs:
1. P: Transition probability matrix of size (n, n) where n is the number of states.
2. s: Current state (integer index).
3. all_s: List of all possible states.
"""
def stateTransition(P: np.ndarray, 
                    s: int, 
                    all_s: list[int]) -> int:

    # get transition probabilities for this state
    tP = P[s]

    # sample the next state, with tP probabilities
    next_s = np.random.choice(all_s, p=tP)

    return next_s

"""
Inputs:
1. s: current state (integer index).
2. P: Transition probability matrix of size (n, n) where n is the number of states.
3. R: Reward function, a dictionary mapping state index to reward value.
"""

def rewardFunction(s: int, P: np.ndarray, R: dict) -> float:
    nextStateProbs = P[s]
    # explicitly order rewards by state index
    rewards = np.array([R[i] for i in range(len(nextStateProbs))])
    return np.dot(nextStateProbs, rewards)

