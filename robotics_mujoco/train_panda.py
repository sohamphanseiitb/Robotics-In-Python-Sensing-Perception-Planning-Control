import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback
import os

# Import the Franka Panda Environment: PandaGrasper Class
from franka_panda_grasp import PandaGrasper

#1. Create and wrap the environment
env = PandaGrasper()
env = Monitor(env) # records episode reward., length and other such important statistics

#2. Create the RL PPO agent
model = PPO(
    policy ="MlpPolicy", # multi-layer perceptron policy implementation inside Pytorch
    env=env,             # frank panda + cube environment
    verbose=1,
    learning_rate=3e-4,
    n_steps = 2048,      # how much data collected before doing policy update
    batch_size = 64,     # mini-batch size used for update
    n_epochs = 10,       # number of times we use the same batch
    gamma=0.99,          # discount factor
    ent_coef=0.01, # encourages some exploration
    tensorboard_log="robotics_mujoco\\panda_tensorboard",
    device="cpu"
)

#3. Callbacks (optional but very useful)
os.makedirs("models", exist_ok=True)
checkpoint_callback = CheckpointCallback(
    save_freq=10_000,
    save_path="robotics_mujoco\\franka_panda_models",
    name_prefix="panda_ppo"
)

#4 Train
print("Starting Model training: ")
model.learn(
    total_timesteps=5_000_000,
    callback=checkpoint_callback, 
    progress_bar=True
)

#5. Save final model
model.save("models/panda_ppo_final")
print("Training finished and model saved.")