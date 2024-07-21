from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from pybullet_envs.robot_env import RobotEnv


# Create the vectorized environment
env = make_vec_env(lambda: RobotEnv("urdf/crawly.urdf"), n_envs=1)

# Create the PPO model
model = PPO('MlpPolicy', env, verbose=1)

# Train the model
model.learn(total_timesteps=200000)

# Save the model
model.save("ppo_robot")

# Load the model (for later use)
model = PPO.load("ppo_robot")

# # Optionally, close the environment
# env.close()