# evaluate.py
from stable_baselines3 import PPO
from pybullet_envs.robot_env import RobotEnv
import time

# Load the trained model
model = PPO.load("models/ppo_robot")

# Create the environment
env = RobotEnv("urdf/crawly.urdf")

# Test the trained model
obs = env.reset()
for _ in range(1000):
    #time.sleep(2)
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()
