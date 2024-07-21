import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import numpy as np
import time
import os

L1 = 0.1 # m
L2 = 0.1 # m

def fk(th1, th2):
    x = L1 * np.cos(th1) + (L1 + L2) * np.cos(th1 + th2)
    y = L2 * np.sin(th1) + (L1 + L2) * np.sin(th1 + th2)

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

    # Calculate the distance from the origin to the point (x, y)
    r = np.sqrt(x**2 + y**2)

    # Check if the point is reachable
    if r > (L1 + L2) or r < np.abs(L1 - L2):
        raise ValueError("The point (x, y) is not reachable with the given link lengths.")

    # Calculate the angle theta2 using the law of cosines
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    sin_theta2 = np.sqrt(1 - cos_theta2**2)  # sin(theta2) could be positive or negative

    theta2 = np.arctan2(sin_theta2, cos_theta2)

    # Calculate the angle theta1
    k1 = L1 + L2 * cos_theta2
    k2 = L2 * sin_theta2

    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
    
    return (theta1, theta2)

def robot_debug(urdf_id,joint_index_list):
    print('Joint States: ')
    joint_state = p.getJointStates(urdf_id, joint_index_list)
    print(joint_state)

    print('Link States: ')
    link_states = p.getLinkStates(urdf_id, joint_index_list)
    print(link_states)
    
    print('Base Velocity: ')
    base_velocity = p.getBaseVelocity(urdf_id)
    print(base_velocity)

    print('Num Bodies: ')
    print(p.getNumBodies())

    print('Body Info: ')
    print(p.getBodyInfo)

def plot_trajectory(time, trajectories):
    
    #Plot the trajectories
    plt.figure(figsize=(12, 8))
    num_joint = len(trajectories)
    #print('num_joint: ', num_joint)
    for joint in range(num_joint):
        #print(joint)
        plt.plot(time, trajectories[joint], label=f'Joint {joint+1}')

    plt.xlabel('Time [s]')
    plt.ylabel('Joint Angle [rad]')
    plt.title('Robot Joint trajectories')
    plt.legend()
    plt.grid(True)
    plt.show()

def crawl_walk(time):
    intiial_position = [-np.pi/12, np.pi/2, np.pi/12, -np.pi/2, np.pi/2, np.pi/12, -np.pi/2, -np.pi/12]

    # Define the parameters for the circular sweep
    sweep_duration = time  # duration of the sweep in seconds
    # frequency of the sine wave (ideally velocity control) 
    frequency = 10.0       # Remember servos have to be able to track this frequency

    # Define the simulation timestep
    timestep = 1.0 / 240.0

    # Generate time points
    num_steps = int(sweep_duration / timestep)
    time_points = np.linspace(0, sweep_duration, num_steps)

    # Generate initial empy list of trajectories
    # Repeat the values to fill an 8xsteps array
    trajectories = np.tile(intiial_position, (num_steps, 1)).T

    # Generate the trajectory using a sine wave
    omega = 2 * np.pi * frequency

    Amp  = [np.pi/12, np.pi/6, np.pi/12, np.pi/6, 0, 0, 0, 0] 
    phi  = [np.pi/4, np.pi/4, np.pi/4, np.pi/4, np.pi/2, np.pi/2, np.pi/2, np.pi/2]
    Amp0 = [-np.pi/12, (5/12)*np.pi, np.pi/12, -(5/12)*np.pi/2, 0, 0, 0, 0]
    thetas = np.tile(intiial_position, (num_steps, 1)).T

    for joint in range(len(intiial_position)):

        Amp_i = Amp[joint]
        phi_i = phi[joint]
        Amp0_i = Amp0[joint]

        thetas[joint] =  Amp_i * np.sin(omega * time_points / sweep_duration + phi_i) + Amp0_i #Asin(2*pi*f*t) + Ao


    # trajectories[0] = thetas[0]
    # trajectories[1] = thetas[1]
    # trajectories[2] = thetas[2]
    # trajectories[3] = thetas[3]

    plot_trajectory(time_points,trajectories)
    (x,y) = fk(trajectories[0], trajectories[1])
    (x1,y1) = fk(trajectories[4], trajectories[5])
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
    camera_distance = 1.0  # Distance from the target position
    camera_yaw = 85        # Yaw angle in degrees
    camera_pitch = -35     # Pitch angle in degrees
    camera_target_position = [0, 0, 0]  # Target position [x, y, z]

    p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)

    # Set the gravity
    p.setGravity(0, 0, -9.81)

    # Create a plane for the URDF to stand on
    plane_id = p.loadURDF("plane.urdf")

    #Set Friction Properties of Ground Plan
    p.changeDynamics(plane_id, -1, lateralFriction=1.0)

    # Load the URDF file
    basePosition = [0, 0, .3]
    baseOrientation = p.getQuaternionFromEuler([np.pi/2, 0, 0]) 

    flags = p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
    urdf_id = p.loadURDF(urdf_path, 
                        basePosition=basePosition,
                        baseOrientation=baseOrientation,
                        flags=flags)
    num_joint = p.getNumJoints(urdf_id)
    joint_index_list = list(range(num_joint))

    # robot_debug(urdf_id, joint_index_list) # print statements

    # Set the initial position and orientation of the URDF
    # initial_position = [0, 0, 1] #x,y,z
    # initial_orientation = p.getQuaternionFromEuler([np.pi/2, 0, 0]) # XYZW - rpy?
    # p.resetBasePositionAndOrientation(urdf_id, initial_position, initial_orientation)

    print("NumJoints: ", p.getNumJoints(urdf_id))
    initial_joint_angles = [np.pi/2, 0, -np.pi/2, 0, np.pi/2, 0, -np.pi/2, 0] # laying flat

    for joint in range(num_joint):
        p.resetJointState(urdf_id, joint,initial_joint_angles[joint])
        p.enableJointForceTorqueSensor(urdf_id, joint, 1)
        print('JointInfo' + str(joint) + ": ", p.getJointInfo(urdf_id, joint))

    print("Get joint state: ")
    print(p.getJointStates(urdf_id, joint_index_list))
    #Generate Trajectory
    sweep_duration = 5.0 #seconds
    trajectory = crawl_walk(sweep_duration) # 8 x n array

    # Generate time points
    timestep = 1.0 / 240.0
    num_steps = int(sweep_duration / timestep)


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
                targetPositions = target_position
                #maxVelocity=5
                )

            crawlyPos, crawlyOrn = p.getBasePositionAndOrientation(urdf_id)
            

            # Camera Tracks the robot
            camera_yaw += .001
            p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, crawlyPos)
            
            #print(crawlyPos,crawlyOrn)
            #       
            # Step the simulation
            p.stepSimulation()

    # Disconnect from PyBullet
    #p.disconnect() # won't reach here because while loop never breaks at the moment

# Path to your URDF file
#urdf_path = "crawly/crawly.urdf"  # Change this to your URDF file path
urdf_path = "urdf/crawly.urdf"
load_and_visualize_urdf(urdf_path)
