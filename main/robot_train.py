from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from pybullet_envs.robot_env import RobotEnv


# Create the vectorized environment
env = make_vec_env(lambda: RobotEnv("urdf/crawly.urdf", show_training = True), n_envs=1)

# Create the PPO model
#model = PPO('MlpPolicy', env, verbose=1)
#model = PPO('MlpPolicy', env, verbose=1, learning_rate=5e-4, n_steps=1024, batch_size=64) #fast training
model = PPO('MlpPolicy', env, verbose=1, learning_rate=5e-5, n_steps=4096, batch_size=256) #slower but better


# Train the model
#model.learn(total_timesteps = 10_000)
#model.learn(total_timesteps = 100_000)
#model.learn(total_timesteps = 1_000_000)
#model.learn(total_timesteps=10_000_000)
model.learn(total_timesteps=20_000_000)

# Save the model
model.save("ppo_robot")

# Load the model (for later use)
model = PPO.load("ppo_robot")

# # Optionally, close the environment
# env.close()