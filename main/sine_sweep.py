import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import numpy as np
import time
import os

current_path = os.path.dirname(os.path.realpath(__file__))
print("Current path:", current_path)

def sine_generator(time):
    # Define the parameters for the circular sweep
    sweep_duration = time  # duration of the sweep in seconds
    frequency = 1.0       # frequency of the sine wave (1 cycle per sweep_duration)

    # Define the simulation timestep
    timestep = 1.0 / 240.0

    # Generate time points
    num_steps = int(sweep_duration / timestep)
    time_points = np.linspace(0, sweep_duration, num_steps)

    # Generate the trajectory using a sine wave
    amplitude = np.pi / 2
    omega = 2 * np.pi * frequency
    offset = np.pi / 2
    trajectory = amplitude *  np.sin(omega * time_points / sweep_duration) + offset #Asin(2*pi*f*t)

    # Plot the trajectory
    # plt.figure(figsize=(10, 6))
    # plt.plot(time_points, trajectory, label='Joint 0')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Joint Angle [rad]')
    # plt.title('Circular Sweep Trajectory')
    # plt.legend()
    # plt.grid(True)
    # plt.show()

    return trajectory

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
    trajectory = sine_generator(sweep_duration)

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
        
            target_position = trajectory[step]
        
            # Apply the target joint position to the robot
            p.setJointMotorControl2(
                bodyUniqueId=urdf_id,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_position
            )

            # Apply the target joint position to the robot
            p.setJointMotorControl2(
                bodyUniqueId=urdf_id,
                jointIndex=2,
                controlMode=p.POSITION_CONTROL,
                targetPosition= -1*target_position
            )
        
            # Step the simulation
            p.stepSimulation()
            crawlyPos, crawlyOrn = p.getBasePositionAndOrientation(urdf_id)
            #print(crawlyPos,crawlyOrn)



    # Disconnect from PyBullet
    #p.disconnect() # won't reach here because while loop never breaks at the moment

# Path to your URDF file
urdf_path = "crawly/crawly.urdf"  # Change this to your URDF file path
load_and_visualize_urdf(urdf_path)
