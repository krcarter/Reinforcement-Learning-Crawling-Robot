import gym
from gym import spaces
import pybullet as p
import pybullet_data
import numpy as np
import time

class RobotEnv(gym.Env):
    def __init__(self, urdf_path, render = False, show_training = True):
        #print("HI I AM INTIALIZE:")
        super(RobotEnv, self).__init__() # It ensures that the initialization code defined in the superclass (gym.Env) is run
        self.urdf_path = urdf_path

        self._num_bullet_solver_iterations = 300
        self._is_render = render
        self._last_frame_time = 0.0
        self._time_step = 0.01

        if show_training == True or self.render==True:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT) 


        # Change the camera view
        self.camera_distance = 1.0  # Distance from the target position
        self.camera_yaw = 85        # Yaw angle in degrees
        self.camera_pitch = -35     # Pitch angle in degrees
        self.camera_target_position = [0, 0, 0]  # Target position [x, y, z]

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)

        p.setPhysicsEngineParameter(
          numSolverIterations=int(self._num_bullet_solver_iterations))
        p.setTimeStep(self._time_step)

        # Set the path to PyBullet's data directory
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # Create a plane for the URDF to stand on
        plane_id = p.loadURDF("plane.urdf")
         #Set Friction Properties of Ground Plan
        p.changeDynamics(plane_id, -1, lateralFriction=1.0)

        #self.reset()

        # Load the URDF file
        basePosition = [0, 0,.2]
        baseOrientation = p.getQuaternionFromEuler([np.pi/2, 0, np.pi/2]) 
        flags = p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
        self.robot_id = p.loadURDF(self.urdf_path,
                            basePosition=basePosition,
                            baseOrientation=baseOrientation,
                            flags=flags)
        
        self._last_base_position = [0, 0, 0]
        self._last_base_orientation = [0, 0, 0]
        self.base_position, self.base_orientation = p.getBasePositionAndOrientation(self.robot_id)
        self.num_joints = p.getNumJoints(self.robot_id)
        self.joint_index_list = list(range(self.num_joints))

        initial_joint_angles = [np.pi/2, 0, -np.pi/2, 0, np.pi/2, 0, -np.pi/2, 0] # laying flat

        for joint in range(self.num_joints):
            print("HI MY JOINTS ARE RESETTING")
            p.resetJointState(self.robot_id, joint,initial_joint_angles[joint])
            p.enableJointForceTorqueSensor(self.robot_id, joint, 1)
            print('JointInfo' + str(joint) + ": ", p.getJointInfo(self.robot_id, joint))
        
        print("ACTION SPACE SIZE: ", (self.num_joints,))
        print("OBSERVATION SPACE SIZE: ", (self.num_joints*2,))
        obs_dim = 2 * self.num_joints + 3 + 4 + 3 + 3 # Dimension of observation array

        # Crawl Gait 1
        action_low  = np.array([-60,   0,-60,   0, 90, 15, -90, -15]) * np.pi/180
        action_high = np.array([ 60, 120, 60, 120, 90, 15, -90, -15]) * np.pi/180

        # Bound Gait 1
        # action_low  = np.array([-np.pi/6, np.pi/4, 0, (-7/12)*np.pi,-np.pi/6, np.pi/4, -np.pi/4, (-7/12)*np.pi])
        # action_high = np.array([0, (7/12)*np.pi, np.pi/6, -np.pi/4,  np.pi/4, (7/12)*np.pi, np.pi/6, -np.pi/4])

        self.action_space = spaces.Box(action_low, action_high, shape=(self.num_joints,), dtype=np.float32)
        #self.action_space = spaces.Box(low=-np.pi, high=np.pi,  shape=(self.num_joints,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)

        self.current_step = 0  # Initialize the step counter

    def reset(self):
        # Reset Function Example:
        # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/robot_bases.py
        # class URDFBasedRobot(XmlBasedRobot):
        # def reset
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)


        p.setPhysicsEngineParameter(
          numSolverIterations=int(self._num_bullet_solver_iterations))
        p.setTimeStep(self._time_step)
        
        # Set the path to PyBullet's data directory
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # Create a plane for the URDF to stand on
        plane_id = p.loadURDF("plane.urdf")
        p.changeVisualShape(plane_id, -1, rgbaColor=[1, 1, 1, 0.9])
         #Set Friction Properties of Ground Plan
        p.changeDynamics(plane_id, -1, lateralFriction=1.0)

        # Load the URDF file
        basePosition = [0, 0, .2]
        baseOrientation = p.getQuaternionFromEuler([np.pi/2, 0, np.pi/2]) 
        flags = p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
        self.robot_id = p.loadURDF(self.urdf_path,
                            basePosition=basePosition,
                            baseOrientation=baseOrientation,
                            flags=flags)
        
        self._last_base_position = [0, 0, 0]
        self._last_base_orientation = [0, 0, 0]
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
            # print("HI MY JOINTS ARE RESETTING")
            # print('JointInfo' + str(joint) + ": ", p.getJointInfo(self.robot_id, joint))

        self.current_step = 0  # Reset the step counter at the beginning of each episode

        return self._get_observation()

    def step(self, action):
        # ToDo need to use setJointMotorControlArray instead
        #print("ACTION: ", action)
        #scaled_action = action * np.pi  # Scale action to a reasonable range

        if self._is_render:
            # Sleep, otherwise the computation takes less time than real time,
            # which will make the visualization like a fast-forward video.
            time_spent = time.time() - self._last_frame_time
            self._last_frame_time = time.time()
            self._action_repeat = 1
            time_to_sleep = self._action_repeat * self._time_step - time_spent
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)
        
        #   Camera

        # Camera Tracks the robot
        self.camera_yaw += .001
        self.camera_target_position  = self.base_position
        p.resetDebugVisualizerCamera(self.camera_distance, self.camera_yaw, self.camera_pitch, self.camera_target_position )


        # Commands
        for i in range(self.num_joints):
            p.setJointMotorControl2(self.robot_id, i, p.POSITION_CONTROL, targetPosition=action[i])
        
        p.stepSimulation()
        
        obs = self._get_observation()
        reward = self._compute_reward()
        #print("Fall Check: ", self._check_fall())
        done = self._is_done() or self._check_fall()

        self.current_step += 1  # Increment the step counter
        
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

        self.base_position, self.base_orientation = p.getBasePositionAndOrientation(self.robot_id)
        self.base_linear_velocity, self.base_angular_velocity = p.getBaseVelocity(self.robot_id)
    
        observation = np.concatenate([joint_positions, joint_velocities, self.base_position, self.base_orientation, self.base_linear_velocity, self.base_angular_velocity])
        # ToDo 
        # Check if joints are at their limitis
        # Get angle of the robot in comparison of the target
        # Get difference of the robot compared to target

        return observation
        #return np.array(joint_positions + joint_velocities)

    def _compute_reward(self):

        self._distance_weight = 1.0
        self._energy_weight = 0.005
        self._shake_weight = 0.005
        self._drift_weight = 0.005
        self._velocity_weight = 0.1
    
        # Get the linear velocity of the base link of the robot
        linear_velocity, angular_velocity = p.getBaseVelocity(self.robot_id)
        # Calculate the magnitude (absolute value) of the linear velocity

        # velocity_magnitude = np.linalg.norm(linear_velocity) # for bounding gait I used the x,y,z components of velocity
    
        velocity_magnitude = np.linalg.norm(linear_velocity[:2]) # Use only x and y components of the linear velocity
        #
        # The reward is the magnitude of the base link's velocity

        self.current_base_position, self.current_base_orientation = p.getBasePositionAndOrientation(self.robot_id)

        forward_reward = self.current_base_position[0] - self._last_base_position[0]
        drift_reward = -abs(self.current_base_position[1] - self._last_base_position[1]) # negative to reduce drift
        shake_reward = -abs(self.current_base_position[2] - self._last_base_position[2]) # negative to reduce shaking

        # print('Reward')
        # print(forward_reward, drift_reward, shake_reward)
        # print(type(forward_reward),type(drift_reward),type(shake_reward))

        self._last_base_position = self.current_base_position

        joint_states = p.getJointStates(self.robot_id, range(self.num_joints))
        joint_torques = [state[3] for state in joint_states]
        joint_motor_velocities = [state[1] for state in joint_states]
        
        energy_reward = abs(np.dot(joint_torques, joint_motor_velocities))  * self._time_step # Negative to penalize energy consumption

        # ToDo
        # Positive Reward for walking to target (some object very far away)
        # Negative Reward for robot being on its side or back
        # Negative Reward for joints_at limit
        # Negative Reward for joint_collisions
        # Negative Reward for side to side motion
        # Negative Reward for body rotation


        # weights = [1]
        # reward =  - velocity_magnitude 
        # Penalize falling 
        fall_weight = -1
        fall_penalty = fall_weight if self._check_fall() else 0

        # Penalize high angular velocities
        angular_penalty = -np.linalg.norm(angular_velocity)

        # Crawling

        reward = (self._velocity_weight*velocity_magnitude - self._energy_weight * energy_reward + self._shake_weight * shake_reward + fall_penalty)

        #No distance weight - Bounding

        # reward = (self._velocity_weight*velocity_magnitude - self._energy_weight * energy_reward +
        #       self._drift_weight * drift_reward + self._shake_weight * shake_reward)
        
        # Distance weights
        # reward = (self._velocity_weight*velocity_magnitude + self._distance_weight * forward_reward - self._energy_weight * energy_reward +
        #       self._drift_weight * drift_reward + self._shake_weight * shake_reward)


        #reward =  velocity_magnitude + fall_penalty + angular_penalty # Testing if negative velocity the robot should come to a stop from training
        #print("REWARD:", reward)
        #reward = 0 
        return reward

    def _is_done(self):
        # Implement termination condition here
        # Orientation check on its side or back
        done = False
        return done
    
    def _check_fall(self):
        # Get the orientation of the base of the robot
        orientation = p.getBasePositionAndOrientation(self.robot_id)[1]
        roll, pitch, yaw = p.getEulerFromQuaternion(orientation)

        # print("RPY: ", roll, pitch, yaw)
        
        # Check if the robot has fallen over (adjust the threshold as needed)
        # fallen when 0 > roll > pi or -pi/2 < pitch > pi/2 
        # if (np.pi/12) > roll or roll > ((11/12)*np.pi) or (-5/12)*np.pi > pitch  or pitch > (5/12)*np.pi:
        if roll < -np.pi/2 or roll > (3/2)*np.pi or pitch < -(1/2)*np.pi  or pitch > (1/2)*np.pi:
            return True
        return False


    def render(self, mode='human'):
        pass

    def close(self):
        p.disconnect(self.physics_client)
