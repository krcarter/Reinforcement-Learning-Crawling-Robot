import argparse
from stable_baselines3 import PPO
from pybullet_envs.robot_env import RobotEnv
import numpy as np
import matplotlib.pyplot as plt

import time

def plot_joint_positions(time, trajectories):
    #Plot the trajectories
    plt.figure(figsize=(12, 8))
    num_joint = len(trajectories)
    print('num_joint: ', num_joint)
    for joint in range(num_joint):
        print(joint)
        plt.plot(time, trajectories[joint], label=f'Joint {joint+1}')

    plt.xlabel('Time [s]')
    plt.ylabel('Joint Angle [rad]')
    plt.title('Robot Joint trajectories')
    plt.legend()
    plt.grid(True)
    plt.show()

def evaluate_model(model_path, render):
    # Load the trained model
    model = PPO.load(model_path)

    # Create the environment
    env = RobotEnv("urdf/crawly.urdf", render = render)

    # Test the trained model
    obs = env.reset()

    joint_position_list = np.empty((8, 0))
    num_steps = 10000

    for _ in range(num_steps):
        #time.sleep(2)
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        joint_positions  = env.render()

        joint_positions_transposed = (np.array(joint_positions)).reshape((8,1))
        joint_position_list = np.append(joint_position_list, joint_positions_transposed, axis=1)


    time_list = np.linspace(0,num_steps,num_steps)
    plot_joint_positions(time_list, joint_position_list)

    np.savetxt('best_crawl.csv', joint_position_list, delimiter=',')


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description="Evaluate a trained PPO model")
    parser.add_argument("--render", type=bool, default=True, help="Render the environment")
    parser.add_argument("--model_path", type=str, required=True, help="Path to the trained model")

    #python robot_evaluate.py --render True --model_path models\ppo_robot_bound_best
    args = parser.parse_args()

    evaluate_model(args.model_path, args.render)