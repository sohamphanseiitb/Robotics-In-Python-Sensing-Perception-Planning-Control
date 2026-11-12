from stable_baselines3 import PPO
from franka_panda_grasp import PandaGrasper
import time

# load the trained model checkpoint
model = PPO.load("models/panda_ppo_final")

# create environment with rendering
env = PandaGrasper(render_mode="human")

# run episodes
n_episodes = 5

for ep in range(n_episodes):
    obs, info = env.reset()
    total_reward = 0.0
    done = False
    step = 0

    print(f"\n=== Episode {ep+1} ===")

    while not done:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        env.render()

        total_reward += reward
        step += 1
        done = terminated or truncated

        time.sleep(0.1)

        if step %50 == 0:
            print(f"Step {step}: dist={info.get('distance', 0):.3f}, height={info.get('cube_height', 0):.3f}, success={info.get('success', False)}")

    
    print(f"Episode finished | Steps: {step} | Total reward: {total_reward:.1f} | Success: {info.get('success', False)} | Failure: {info.get('failure', False)}")

env.close()