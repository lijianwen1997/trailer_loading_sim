import gymnasium as gym
import numpy as np
from trailer_load_gym import TrailerLoadEnv  # Import your wrapper
import time
# Register the environment (optional)
gym.register(
    id='TrailerLoadEnv-v0',
    entry_point='trailer_load_gym:TrailerLoadEnv',
)

# Create and use the environment
env = TrailerLoadEnv(render_mode='human')  # or gym.make('WamvGazeboEnv-v0', render_mode='human') rgb_array

for episode in range(10):
    observation, info = env.reset()
    done = False
    episode_reward = 0

    while not done:
        # Example: simple forward motion with slight turn
        action = np.array([70, 70])  # left, right thrust
        observation, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated
        time.sleep(0.05)
        episode_reward += reward

    print(f"Episode {episode} reward: {episode_reward}")

env.close()