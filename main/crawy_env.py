import gym
from gym import spaces
import pybullet as p
import pybullet_data
import numpy as np

class RobotEnv(gym.Env):
    def __init__(self, urdf_path):
        super(RobotEnv, self).__init__() # It ensures that the initialization code defined in the superclass (gym.Env) is run
        self.urdf_path = urdf_path
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        self.robot_id = p.loadURDF(self.urdf_path)
        
        self.num_joints = p.getNumJoints(self.robot_id)
        self.action_space = spaces.Box(low=-1, high=1, shape=(self.num_joints,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.num_joints*2,), dtype=np.float32)

    def reset(self):
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)
        self.robot_id = p.loadURDF(self.urdf_path)
        return self._get_observation()

    def step(self, action):
        # ToDo need to use setJointMotorControlArray instead
        for i in range(self.num_joints):
            p.setJointMotorControl2(self.robot_id, i, p.POSITION_CONTROL, targetPosition=action[i])
        
        p.stepSimulation()
        
        obs = self._get_observation()
        reward = self._compute_reward()
        done = self._is_done()
        
        return obs, reward, done, {}

    def _get_observation(self):
        joint_states = p.getJointStates(self.robot_id, range(self.num_joints))
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        return np.array(joint_positions + joint_velocities)

    def _compute_reward(self):
        # Get the linear velocity of the base link of the robot
        linear_velocity, _ = p.getBaseVelocity(self.robot_id)
        # Calculate the magnitude (absolute value) of the linear velocity
        velocity_magnitude = np.linalg.norm(linear_velocity)
        # The reward is the magnitude of the base link's velocity
        reward = velocity_magnitude
        return reward

    def _is_done(self):
        # Implement your termination condition here
        done = False
        return done

    # def render(self, mode='human'):
    #     pass

    def close(self):
        p.disconnect(self.physics_client)
