import gym
import pybullet_envs

def main():
    # Create the gym environment
    env = gym.make('MinitaurBulletEnv-v0')

    # Reset the environment to start
    observation = env.reset()

    env.render(mode='human')


    # Run the simulation for 1000 steps
    for _ in range(1000):
        # Sample a random action from the action space
        action = env.action_space.sample()

        print(type(action))
        print(action)

        # Take a step in the environment
        observation, reward, done, info = env.step(action)

        # Print observation, reward, and done flag
        print(f"Observation: {observation}, Reward: {reward}, Done: {done}")

        # If the episode is done, reset the environment
        if done:
            observation = env.reset()

    # Close the environment
    env.close()

if __name__ == "__main__":
    main()