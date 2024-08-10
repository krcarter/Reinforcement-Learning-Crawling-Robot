import argparse
from stable_baselines3 import PPO
from pybullet_envs.robot_env import RobotEnv
import time

def evaluate_model(model_path, render):
    # Load the trained model
    model = PPO.load(model_path)

    # Create the environment
    env = RobotEnv("urdf/crawly.urdf", render = render)

    # Test the trained model
    obs = env.reset()
    for _ in range(10000):
        #time.sleep(2)
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        env.render()

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description="Evaluate a trained PPO model")
    parser.add_argument("--render", type=bool, default=True, help="Render the environment")
    parser.add_argument("--model_path", type=str, required=True, help="Path to the trained model")

    #python robot_evaluate.py --render True --model_path models\ppo_robot_bound_best
    args = parser.parse_args()

    evaluate_model(args.model_path, args.render)