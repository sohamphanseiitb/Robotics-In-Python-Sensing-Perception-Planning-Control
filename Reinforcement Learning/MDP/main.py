import utils
import mrp
import numpy as np
import matplotlib.pyplot as plt

# instantiate a class
states = {'Class1': 0, 'Class2': 1, 'Class3': 2, 'Pass': 3, 'Pub': 4, 'Facebook': 5, 'Sleep': 6}
rewardVals = {'Class1': -2, 'Class2': -2, 'Class3': -2, 'Pass': 10, 'Pub': 1, 'Facebook': -1, 'Sleep': 0}
allStates = list(states.values())
simTimeSteps = 100
NUM_STATES = 7
np.random.seed(2)
stateTransProbMat = np.array([[0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0],
              [0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 0.2],
              [0.0, 0.0, 0.0, 0.6, 0.4, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
              [0.2, 0.4, 0.4, 0.0, 0.0, 0.0, 0.0],
              [0.1, 0.0, 0.0, 0.0, 0.0, 0.9, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],]) 

MarkovRP = mrp.MRP(ns=NUM_STATES,
                   allStates=allStates,
                   STF=utils.stateTransition,
                   StateDict=states,
                   STM=stateTransProbMat,
                   ExpcRewardFunc=utils.rewardFunction,
                   RewardDict=rewardVals,
                   gamma=0.9)

# statesSeq, rewardsSeq, returnSeq = MarkovRP.computeReturn(ts=simTimeSteps, S0=states['Class1'])

# compute value function
MarkovRP.computeValueFunction(N=500)
MarkovRP.computeValueFunctionBellman()

print(dict(zip(states.keys(), MarkovRP.V)))
print("With Bellamn: ", dict(zip(states.keys(), MarkovRP.VBellman)))

# plt.plot(returnSeq)
# plt.show()