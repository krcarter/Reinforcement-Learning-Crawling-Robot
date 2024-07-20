import gym
import pybullet_envs
from stable_baselines3 import PPO

# Create the environment
env = gym.make('CrawlingRobot-v0')

# Create the PPO model
model = PPO('MlpPolicy', env, verbose=1)

# Train the model
model.learn(total_timesteps=10000)

# Save the model
model.save("models/ppo_robot")

# Close the environment
env.close()
