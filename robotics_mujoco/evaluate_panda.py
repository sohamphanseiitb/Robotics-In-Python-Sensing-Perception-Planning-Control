import argparse
import csv
from pathlib import Path
from statistics import mean

import numpy as np
from stable_baselines3 import PPO

from franka_panda_grasp import PandaGrasper


def evaluate(env, model, seed, render):
    obs, _ = env.reset(seed=seed)
    cube_x, cube_y, cube_height = env.data.body(env.cube_body_id).xpos
    peak_height = cube_height
    zero_residual = np.zeros(env.action_space.shape, dtype=env.action_space.dtype)
    total_reward = 0.0
    steps = 0
    terminated = truncated = False

    while not (terminated or truncated):
        action = zero_residual if model is None else model.predict(obs, deterministic=True)[0]
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        peak_height = max(peak_height, info["cube_height"])
        steps += 1
        if render:
            env.render()

    return {
        "seed": seed,
        "cube_start_x": cube_x,
        "cube_start_y": cube_y,
        "return": total_reward,
        "steps": steps,
        "peak_cube_height": peak_height,
        "success": bool(info["success"]),
        "failure": bool(info["failure"]),
        "timeout": bool(truncated and not terminated),
    }


def main():
    parser = argparse.ArgumentParser(description="Compare zero residual and PPO on matching Panda resets.")
    parser.add_argument("--checkpoint", type=Path, action="append", help="PPO checkpoint. Repeat to compare models.")
    parser.add_argument("--baseline-only", action="store_true")
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--seed", type=int, default=10_000, help="First episode seed.")
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--csv", type=Path, help="Write one row per episode and policy.")
    args = parser.parse_args()
    if args.episodes < 1 or args.seed < 0:
        parser.error("--episodes must be positive and --seed must be nonnegative")
    if args.baseline_only and args.checkpoint:
        parser.error("--checkpoint cannot be combined with --baseline-only")

    checkpoints = args.checkpoint or [Path(__file__).with_name("panda_ppo_final.zip")]
    policies = [None] if args.baseline_only else [None, *checkpoints]
    env = PandaGrasper(render_mode="human" if args.render else None)
    rows = []
    try:
        for checkpoint in policies:
            label = "classical_only" if checkpoint is None else str(checkpoint)
            model = None if checkpoint is None else PPO.load(checkpoint, device="cpu")
            episodes = [
                evaluate(env, model, seed, args.render)
                for seed in range(args.seed, args.seed + args.episodes)
            ]
            rows.extend({"policy": label, **episode} for episode in episodes)
            successes = sum(row["success"] for row in episodes)
            failures = sum(row["failure"] for row in episodes)
            timeouts = sum(row["timeout"] for row in episodes)
            print(
                f"{label} | lifts {successes}/{args.episodes} | failures {failures} | timeouts {timeouts}"
                f" | mean return {mean(row['return'] for row in episodes):.1f}"
                f" | mean steps {mean(row['steps'] for row in episodes):.1f}"
                f" | mean peak height {mean(row['peak_cube_height'] for row in episodes):.3f} m",
                flush=True,
            )
    finally:
        env.close()

    if args.csv:
        with args.csv.open("w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


if __name__ == "__main__":
    main()
