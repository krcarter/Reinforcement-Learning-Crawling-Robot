import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import numpy as np
import time
import os


def fk(th1, th2):
    l1 = 0.1 # m
    l2 = 0.1 # m

    th1 = th1 - np.pi/2

    x = l1 * np.cos(th1) + (l1 + l2) * np.cos(th1 + th2)
    y = l2 * np.sin(th1) + (l1 + l2) * np.sin(th1 + th2)

    plt.figure(figsize=(10, 6))
    plt.plot(x, y, label='End-Effector Trajectory')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Forward Kinematics')
    plt.legend()
    plt.grid(True)
    plt.show()

    return (x,y)

def ik(x, y):
    l1 = 0.1 # m
    l2 = 0.1 # m

    th2 = np.arccos((x**2 + y**2 - l1**2 - l2**2)/(2*l1*l2))
    th1 = np.arctan(y/x) - np.tan( (l2*np.sin(th2))/ (l1 + l2*np.cos(th2)))

    return (th1, th2)


def plot_trajectory(time, trajectories):
    
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



def crawl_walk(time):
    #
    intiial_position = [np.pi/2, 0, -np.pi/2, 0, np.pi/2, 0, -np.pi/2, 0]
    # Define the parameters for the circular sweep
    sweep_duration = time  # duration of the sweep in seconds
    frequency = 1.0       # frequency of the sine wave (1 cycle per sweep_duration)

    # Define the simulation timestep
    timestep = 1.0 / 240.0

    # Generate time points
    num_steps = int(sweep_duration / timestep)
    time_points = np.linspace(0, sweep_duration, num_steps)

    # Generate initial empy list of trajectories
    # Repeat the values to fill an 8x100 array
    trajectories = np.tile(intiial_position, (num_steps, 1)).T

    # Generate the trajectory using a sine wave
    omega = 2 * np.pi * frequency
    Amp_fl1 = np.pi / 4
    phi1 = np.pi/4
    offset_fl1 = np.pi / 2 - np.pi / 6

    Amp_fl2 = np.pi / 2
    phi2 = np.pi/4
    offset_fl2 = 0

    front_links_L1 = Amp_fl1 *  np.sin(omega * time_points / sweep_duration + phi1) + offset_fl1 #Asin(2*pi*f*t) + Ao
    front_links_L2 = Amp_fl2 *  np.sin(omega * time_points / sweep_duration + phi2) + offset_fl2 #Asin(2*pi*f*t) + Ao


    trajectories[0][0:int(3*num_steps/2)] = front_links_L1[0:int(3*num_steps/2)]
    trajectories[2] = -1 * front_links_L1

    trajectories[1] = front_links_L2
    trajectories[3] = -1 * front_links_L2

    # plt.figure(figsize=(12, 8))
    # for joint in range(len(intiial_position)):
    #     plt.plot(time_points, trajectories[joint], label=f'Joint {joint}')

    # plt.xlabel('Time [s]')
    # plt.ylabel('Joint Angle [rad]')
    # plt.title('Crawling Trajectory')
    # plt.legend()
    # plt.grid(True)
    # plt.show()

    plot_trajectory(time_points,trajectories)
    (x,y) = fk(trajectories[0], trajectories[1])
    return trajectories

def load_and_visualize_urdf(urdf_path):
    # Connect to PyBullet
    width = 1600
    height = 900
    physicsClient = p.connect(p.GUI, options=f'--width={width} --height={height}')

    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

    # Set the path to PyBullet's data directory
    print("Pybullet Path: ", pybullet_data.getDataPath())
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Change the camera view
    camera_distance = 1.5  # Distance from the target position
    camera_yaw = 85        # Yaw angle in degrees
    camera_pitch = -35     # Pitch angle in degrees
    camera_target_position = [0, 0, 0]  # Target position [x, y, z]

    p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)

    # Set the gravity
    p.setGravity(0, 0, -9.81)

    # Create a plane for the URDF to stand on
    plane_id = p.loadURDF("plane.urdf")

    # Load the URDF file
    flags = p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
    urdf_id = p.loadURDF(urdf_path, flags=flags)
    #urdf_id = p.loadURDF(urdf_path)
    num_joint = p.getNumJoints(urdf_id)
    joint_index_list = list(range(num_joint))
    print(joint_index_list)

    ## INFO DEBUGGING ##
    '''
    for joint in range(num_joint):
        p.enableJointForceTorqueSensor(urdf_id, joint, 1)
        print('JointInfo' + str(joint) + ": ", p.getJointInfo(urdf_id, joint))
    print('Joint States')
    joint_state = p.getJointStates(urdf_id, joint_index_list)
    # position
     print(joint_state)
    '''

    # print('Joint States: ')
    # joint_state = p.getJointStates(urdf_id, joint_index_list)
    # print(joint_state)

    # print('Link States: ')
    # link_states = p.getLinkStates(urdf_id, joint_index_list)
    # print(link_states)
    
    # print('Base Velocity: ')
    # base_velocity = p.getBaseVelocity(urdf_id)
    # print(base_velocity)

    # print('Num Bodies: ')
    # print(p.getNumBodies())

    # print('Body Info: ')
    # print(p.getBodyInfo)


    # Commanding Joints
    # p.setJointMotorControlArray(
    #     bodyUniqueId = urdf_id, 
    #     jointIndices = joint_index_list,
    #     controlMode = p.POSITION_CONTROL,
    #     targetPositions = initial_joint_angles)

    ## INFO DEBUGGING End ##

    #print("JointInfo: ", p.getJointInfo(urdf_id))
    # Set the initial position and orientation of the URDF
    initial_position = [0, 0, 1] #x,y,z
    initial_orientation = p.getQuaternionFromEuler([np.pi/2, 0, 0]) # XYZW - rpy?
    p.resetBasePositionAndOrientation(urdf_id, initial_position, initial_orientation)

    print("NumJoints: ", p.getNumJoints(urdf_id))
    initial_joint_angles = [np.pi/2, 0, -np.pi/2, 0, np.pi/2, 0, -np.pi/2, 0] # laying flat
    #initial_joint_angles = [np.pi/4, -np.pi/2, -np.pi/4, np.pi/2, -np.pi/4, np.pi/2, np.pi/4, -np.pi/2] # standing up

    for joint in range(num_joint):
        p.resetJointState(urdf_id, joint,initial_joint_angles[joint])
        p.enableJointForceTorqueSensor(urdf_id, joint, 1)
        print('JointInfo' + str(joint) + ": ", p.getJointInfo(urdf_id, joint))


    #Generate Trajectory
    sweep_duration = 5.0 #seconds
    trajectory = crawl_walk(sweep_duration) # 8 x n array

    # Generate time points
    timestep = 1.0 / 240.0
    num_steps = int(sweep_duration / timestep)

    joint_index = 0

    # Run the simulation
    while True:
        start_time = time.time()
        current_time = 0

        while current_time < sweep_duration:
            current_time = time.time() - start_time
            step = int(current_time / timestep)
        
            if step >= num_steps:
                break
        
            target_position = trajectory[:, step]

            p.setJointMotorControlArray(
                bodyUniqueId = urdf_id, 
                jointIndices = joint_index_list,
                controlMode = p.POSITION_CONTROL,
                targetPositions = target_position)

            # # Apply the target joint position to the robot
            # p.setJointMotorControl2(
            #     bodyUniqueId=urdf_id,
            #     jointIndex=joint_index,
            #     controlMode=p.POSITION_CONTROL,
            #     targetPosition=target_position
            # )
      
            # Step the simulation
            p.stepSimulation()
            crawlyPos, crawlyOrn = p.getBasePositionAndOrientation(urdf_id)
            #print(crawlyPos,crawlyOrn)



    # Disconnect from PyBullet
    #p.disconnect() # won't reach here because while loop never breaks at the moment

# Path to your URDF file
urdf_path = "crawly/crawly.urdf"  # Change this to your URDF file path
load_and_visualize_urdf(urdf_path)
