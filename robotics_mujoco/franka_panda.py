import time

import numpy as np
import robosuite as suite

# build the environment
env = suite.make(
    env_name="Lift",
    robots="Panda",
    has_renderer=True,
    has_offscreen_renderer=True,
)

obs = env.reset()
print("\n ----- Observation Ketys-----")
for k, v in obs.items():
    print(f" {k:30s} shape={np.shape(v)}")

# print action dimension
print("\n ----- Action Dimension -----")
print(env.action_dim)
print("\n ----- Action Bounds -----")
print(env.action_spec)

# Start a sim
for i in range(200):
    action = np.random.randn(env.action_dim) * 0.1  # sample random action
    obs, reward, done, info = env.step(action)
    env.render()
    time.sleep(2)
    if done:
        break