import gym
from gym import spaces
import pybullet as p
import pybullet_data
import numpy as np

class RobotEnv(gym.Env):
    def __init__(self, urdf_path):
        #print("HI I AM INTIALIZE:")
        super(RobotEnv, self).__init__() # It ensures that the initialization code defined in the superclass (gym.Env) is run
        self.urdf_path = urdf_path
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)


        # Set the path to PyBullet's data directory
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # Create a plane for the URDF to stand on
        plane_id = p.loadURDF("plane.urdf")
         #Set Friction Properties of Ground Plan
        p.changeDynamics(plane_id, -1, lateralFriction=1.0)


        # Load the URDF file
        basePosition = [0, 0, .2]
        baseOrientation = p.getQuaternionFromEuler([np.pi/2, 0, 0]) 
        flags = p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
        self.robot_id = p.loadURDF(self.urdf_path,
                            basePosition=basePosition,
                            baseOrientation=baseOrientation,
                            flags=flags)
        
        self.num_joints = p.getNumJoints(self.robot_id)
        self.joint_index_list = list(range(self.num_joints))

        initial_joint_angles = [np.pi/2, 0, -np.pi/2, 0, np.pi/2, 0, -np.pi/2, 0] # laying flat

        for joint in range(self.num_joints):
            #print("HI MY JOINTS ARE RESETTING")
            p.resetJointState(self.robot_id, joint,initial_joint_angles[joint])
            p.enableJointForceTorqueSensor(self.robot_id, joint, 1)
            #print('JointInfo' + str(joint) + ": ", p.getJointInfo(self.robot_id, joint))

        self.action_space = spaces.Box(low=-1, high=1, shape=(self.num_joints,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.num_joints*2,), dtype=np.float32)

    def reset(self):
        # Reset Function Example:
        # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/robot_bases.py
        # class URDFBasedRobot(XmlBasedRobot):
        # def reset
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)
        
        # Set the path to PyBullet's data directory
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # Create a plane for the URDF to stand on
        plane_id = p.loadURDF("plane.urdf")
         #Set Friction Properties of Ground Plan
        p.changeDynamics(plane_id, -1, lateralFriction=1.0)

        # Load the URDF file
        basePosition = [0, 0, .1]
        baseOrientation = p.getQuaternionFromEuler([np.pi/2, 0, 0]) 
        flags = p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
        self.robot_id = p.loadURDF(self.urdf_path,
                            basePosition=basePosition,
                            baseOrientation=baseOrientation,
                            flags=flags)
        #num_joint = p.getNumJoints(urdf_id)

        # Set the initial position and orientation of the base link from URDF
        # initial_position = [0, 0, 1] #x,y,z
        # initial_orientation = p.getQuaternionFromEuler([np.pi/2, 0, 0]) # XYZW - rpy?
        # p.resetBasePositionAndOrientation(self.urdf_path, initial_position, initial_orientation)
        

        # Set the intial joint angles

        initial_joint_angles = [np.pi/2, 0, -np.pi/2, 0, np.pi/2, 0, -np.pi/2, 0] # laying flat

        for joint in range(self.num_joints):
            p.resetJointState(self.robot_id, joint,initial_joint_angles[joint])
            p.enableJointForceTorqueSensor(self.robot_id, joint, 1)
            #print('JointInfo' + str(joint) + ": ", p.getJointInfo(self.robot_id, joint))


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
        # Example Get observation
        # (https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/robot_locomotors.py)
        # def calc_state(self):

        #print((self.num_joints))
        #print(self.joint_index_list)
        #print(p.getJointStates(self.robot_id, self.joint_index_list))
        #print(p.getJointStates(self.robot_id, [0,1]))
        #print("OBSERVATION")
        joint_states = p.getJointStates(self.robot_id, range(self.num_joints))
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]

        # ToDo 
        # Check if joints are at their limitis
        # Get body pose (position and orientation relative to world frame)
        # Get angle of the robot in comparison of the target
        # Get difference of the robot compared to target
        # Get robot body angular rotation

        return np.array(joint_positions + joint_velocities)

    def _compute_reward(self):
        # Get the linear velocity of the base link of the robot
        linear_velocity, _ = p.getBaseVelocity(self.robot_id)
        # Calculate the magnitude (absolute value) of the linear velocity
        velocity_magnitude = np.linalg.norm(linear_velocity)
        # The reward is the magnitude of the base link's velocity

        # ToDo
        # Positive Reward for walking to target (some object very far away)
        # Negative Reward for robot being on its side or back
        # Negative Reward for joints_at limit
        # Negative Reward for joint_collisions
        # Negative Reward for side to side motion
        # Negative Reward for body rotation


        # weights = [1]
        reward = velocity_magnitude
        print("REWARD:", reward)
        #reward = 0 
        return reward

    def _is_done(self):
        # Implement termination condition here
        # Orientation check on its side or back
        done = False
        return done

    def render(self, mode='human'):
        pass

    def close(self):
        p.disconnect(self.physics_client)
