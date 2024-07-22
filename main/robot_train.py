from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from pybullet_envs.robot_env import RobotEnv


# Create the vectorized environment
env = make_vec_env(lambda: RobotEnv("urdf/crawly.urdf"), n_envs=1)

# Create the PPO model
model = PPO('MlpPolicy', env, verbose=1)
# model = PPO('MlpPolicy', env, verbose=1, learning_rate=1e-4, n_steps=2048, batch_size=64)


# Train the model
#model.learn(total_timesteps = 10_000)
model.learn(total_timesteps = 1_000_000)
#model.learn(total_timesteps=10_000_000)

# Save the model
model.save("ppo_robot")

# Load the model (for later use)
model = PPO.load("ppo_robot")

# # Optionally, close the environment
# env.close()