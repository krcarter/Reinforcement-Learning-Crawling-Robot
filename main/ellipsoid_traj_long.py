import matplotlib.pyplot as plt
import matplotlib.cm as cm
import pybullet as p
import pybullet_data
import numpy as np
import time
import os

L1 = 0.1 # m
L2 = 0.1 # ma

def half_ellipse(a, b, origin=(0, 0), rotation_angle=0, num_pts =240):
    """
    Generates the points for a half ellipse.

    Parameters:
    a (float): Major axis length.
    b (float): Minor axis length.
    origin (tuple): (x, y) coordinates of the ellipse's center.
    rotation_angle (float): Angle to rotate the ellipse (in radians).
    
    Returns:
    x_rotated (ndarray): x coordinates of the half ellipse.
    y_rotated (ndarray): y coordinates of the half ellipse.
    """

    # Rotation matrix
    R = np.array([[np.cos(rotation_angle), -np.sin(rotation_angle)],
                  [np.sin(rotation_angle),  np.cos(rotation_angle)]])
    
    swing_speed = 0.30 # 0 to 1 Percentage Adjust to determine swing speed
    swing_pts = int(num_pts*(1-swing_speed))
    stance_pts = int(num_pts*(swing_speed))
    
    theta = np.linspace(0, np.pi, swing_pts) # only to np.pi because it is a half ellipse

    x = a * np.cos(theta)
    y = b * np.sin(theta)

    # Rotate and translate the points
    ellipse_points = np.array([x, y])

    rotated_points = R @ ellipse_points

    x_rotated, y_rotated = rotated_points[0] + origin[0], rotated_points[1] + origin[1]
    
    # Straight line trajectory
    xo = x_rotated[0]
    xf = x_rotated[-1]
    yo = y_rotated[0]
    yf = y_rotated[-1]

    xline = np.linspace(xo, xf, stance_pts)
    yline = np.linspace(yo, yf, stance_pts)

    # Originally was going to adjust x and y kinematics for right and left leg here
    # but decided to create another function, offset swings for that
    x_final_l = np.concatenate((x_rotated[::-1],xline))
    y_final_l = np.concatenate((y_rotated[::-1],yline))
    
    x_final_r = np.concatenate((x_rotated[::-1],xline))
    y_final_r = np.concatenate((y_rotated[::-1],yline))


    return (x_final_l, y_final_l, x_final_r, y_final_r)

def offset_swing(thetas,percent):
    
    thetas_length = len(thetas)
    start_index = int( (percent/100) * thetas_length)

    offset_thetas = np.concatenate((thetas[start_index:],thetas[:start_index]))

    return offset_thetas

def foot_trajectory(num_steps, percent_offset = 70):
    # Parameters
    # Hard coding the parameters of the ellipse trajectory
    # Come here to modify values if you want to change how the robot walks

    a1 = 0.10/2  # Major axis length
    b1 = 0.05  # Minor axis length

    rotation_adjustment = -5 * (np.pi/180) # tilts the leg walking up

    R = np.array([[np.cos(rotation_adjustment), -np.sin(rotation_adjustment)],
                  [np.sin(rotation_adjustment),  np.cos(rotation_adjustment)]])

    origin = np.array([0.15, 0.05])  # Origin of the ellipse

    origin = np.matmul(R,origin)

    rotation_angle = (1/2)*np.pi + rotation_adjustment   # Rotation angle in radians

    # Generate half ellipse points
    xl,yl,xr,yr = half_ellipse(a1, b1, origin, rotation_angle, num_steps)
    # plot_xy(xl,yl)
    # plot_xy(xr,yr)
    (theta0,theta1) =  trajectory_to_angles(xl,yl)
    (theta2,theta3) =  trajectory_to_angles(xr,yr)

    # Percent offset is between 0 to 100
    # This number adjust the starting point between left and right leg
    theta2 = offset_swing(theta2,percent_offset)
    theta3 = offset_swing(theta3,percent_offset)

    # Plots the Forward Kinematics
    (xl_c,yl_c) = fk(theta0, theta1)
    (xr_c,yr_c) = fk(-theta2, -theta3)

    
    return (theta0,theta1,-theta2,-theta3)

def fk(th1, th2):
    x = L1 * np.cos(th1) + (L2) * np.cos(th1 + th2)
    y = L2 * np.sin(th1) + (L2) * np.sin(th1 + th2)

    # Define the rotation matrix for 90 degrees counterclockwise
    R = np.array([[0, 1],
                  [-1, 0]])
    
    P = np.array([x, y])

    [xr,yr] = np.matmul(R,P)


    plt.figure(figsize=(10, 6))
    #plt.plot(x, y, label='Leg Rotated')
    # Normalize the path length for the colormap
    norm = plt.Normalize(0, 0.7*len(x))

    # Create a colormap
    colors = cm.viridis(norm(range(len(x))))

    # Plot the trajectory with a gradient color
    for i in range(len(x) - 1):
        plt.plot(x[i:i+2], y[i:i+2], color=colors[i], linewidth=5)
    #plt.plot(xr, yr, label='Leg Rotated')
    for i in range(len(xr) - 1):
        plt.plot(xr[i:i+2], yr[i:i+2], color=colors[i], linewidth=5)
    plt.scatter([x[0], x[-1]], [y[0], y[-1]], color='red')  # Plot endpoints for clarity
    plt.scatter([xr[0], xr[-1]], [yr[0], yr[-1]], color='red')  # Plot endpoints for clarity
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.axhline(0, color='black',linewidth=1.0)
    plt.axvline(0, color='black',linewidth=1.0)
    plt.ylim([-0.2,0.2])
    plt.xlim([-0.2,0.2])
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
    
    # th2 = np.arccos((x**2 + y**2 - L1**2 - L2**2)/(2*L1*L2))
    # th1 = np.arctan(y/x) - np.arctan( (L2*np.sin(th2))/ (L1 + L2*np.cos(th2)))

    # Calculate the angle theta2 using the law of cosines
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    sin_theta2 = np.sqrt(1 - cos_theta2**2)  # sin(theta2) could be positive or negative

    theta2 = np.arctan2(sin_theta2, cos_theta2)

    # Calculate the angle theta1
    k1 = L1 + L2 * cos_theta2
    k2 = L2 * sin_theta2

    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
    
    return (theta1, theta2)

def trajectory_to_angles(x,y):
    num_step = len(x)
    theta1_list = []
    theta2_list = []
    for point in range(num_step):
        x_curr = x[point]
        y_curr = y[point]
        theta1, theta2 = ik(x_curr, y_curr)
        theta1_list.append(theta1)
        theta2_list.append(theta2)

    return (np.array(theta1_list), np.array(theta2_list))

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

def plot_xy(x, y):
    """
    Plots the ellipse using the provided x and y coordinates.
    """
    plt.figure(figsize=(8, 6))
    #plt.plot(x, y, label="XY")
    # Normalize the path length for the colormap
    norm = plt.Normalize(0, 0.7*len(x))

    # Create a colormap
    colors = cm.viridis(norm(range(len(x))))

    # Plot the trajectory with a gradient color
    for i in range(len(x) - 1):
        plt.plot(x[i:i+2], y[i:i+2], color=colors[i], linewidth=2)

    plt.scatter([x[0], x[-1]], [y[0], y[-1]], color='red')  # Plot endpoints for clarity
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')

    plt.axhline(0, color='black',linewidth=0.5)
    plt.axvline(0, color='black',linewidth=0.5)
    plt.grid(color = 'gray', linestyle = '--', linewidth = 0.5)
    plt.legend()
    plt.ylim([-0.2,0.2])
    plt.xlim([-0.2,0.2])
    plt.title('XY Plot')
    #plt.axis('equal')
    plt.show()

def sim_to_real_text_file(trajectory, num_steps):
    #Simulation to Real Conversion
    degrees_trajectory = np.degrees(trajectory)
    rounded_trajectory = np.round(degrees_trajectory, 0)

    #np.savetxt('manual_walk_long.csv', rounded_trajectory, delimiter=',')

    # Sim to Real Coordinates transformations
    # This should definately be some type of rotation + translation, but couldn't figure it out
    # The Servo motors for physical robots are started at the 90 degree positon and range from 0 to 180 degrees
    # while in the simulation the robot's servos range from [-90 to 90]

    rounded_trajectory[0] = 90 - rounded_trajectory[0]
    rounded_trajectory[1] = rounded_trajectory[1]
    rounded_trajectory[2] = 90 - rounded_trajectory[2]
    rounded_trajectory[3] = 180 - (-1*rounded_trajectory[3])

    ### CSV Trajectory ###
    np.savetxt('manual_walk_long.csv', rounded_trajectory, delimiter=',')

    ### Text File ###

    # Open a text file to write the formatted arrays
    with open('arduino_arrays_long.txt', 'w') as file:
        for idx, row in enumerate(rounded_trajectory, start=1):
            # Format each value in the row no decimals
            formatted_values = ','.join(f"{val:.0f}" for val in row)
            
            # Create the Arduino array string
            #num_steps_string = str(num_steps)
            array_string = f"int row_{idx}[{num_steps}] = {{ {formatted_values} }};\n"
            
            # Write the array string to the file
            file.write(array_string)



def walk(time, timestep):
    #
    #initial_position = [0, np.pi/2, 0, -np.pi/2, np.pi/2, 0, -np.pi/2, 0] #laying flat
    #initial_position = np.array([ 0,  90,  0, -90, 0, 45,  0, -45]) * np.pi/180
    #initial_position = [0, np.pi/2, 0, -np.pi/2, 0, np.pi/2, 0, -np.pi/2] # back legs down
    initial_position = np.array([   0, 90, 0,  -90, 0,  95, 0, -95]) * np.pi/180 # back legs down but slightly up
    # Define the parameters for the circular sweep
    sweep_duration = time  # duration of the sweep in seconds
    frequency = 1.0       # frequency of the sine wave (1 cycle per sweep_duration)

    # Generate time points
    num_steps = int(sweep_duration / timestep)
    time_points = np.linspace(0, sweep_duration, num_steps)

    # Generate initial empy list of trajectories
    # Repeat the values to fill an 8x100 array
    trajectories = np.tile(initial_position, (num_steps, 1)).T

    (theta0,theta1,theta2,theta3) = foot_trajectory(num_steps)

    
    trajectories[0] = theta0
    trajectories[1] = theta1
    trajectories[2] = theta2
    trajectories[3] = theta3
    # trajectories[4] = theta4
    # trajectories[5] = theta5
    # trajectories[6] = -1*theta4
    # trajectories[7] = -1*theta5


    plot_trajectory(time_points,trajectories)

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

    # Load the URDF file
    flags = p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
    urdf_id = p.loadURDF(urdf_path, flags=flags)
    #urdf_id = p.loadURDF(urdf_path)
    num_joint = p.getNumJoints(urdf_id)
    joint_index_list = list(range(num_joint))
    print(joint_index_list)

    # Friction Parameters

    # # Set friction parameters
    # lateral_friction = 1.0  # Adjust as needed
    # rolling_friction = 0.01  # Adjust as needed
    # spinning_friction = 0.01  # Adjust as needed

    # # Apply the friction settings to each link of the robot
    # for link_id in range(p.getNumJoints(urdf_id)):
    #     p.changeDynamics(urdf_id, link_id,
    #                     frictionAnchor=True,
    #                     lateralFriction=lateral_friction,
    #                     # rollingFriction=rolling_friction,
    #                     # spinningFriction=spinning_friction
    #                     )

    # # Also, apply the friction settings to the robot's base (if it's part of the robot)
    # p.changeDynamics(urdf_id, -1,  # -1 refers to the base of the robot
    #                 frictionAnchor=True,
    #                 lateralFriction=lateral_friction,
    #                 # rollingFriction=rolling_friction,
    #                 # spinningFriction=spinning_friction
    #                 )

    # # Ensure the ground has friction as well
    # p.changeDynamics(urdf_id, -1,  # -1 refers to the ground
    #                 frictionAnchor=True,
    #                 lateralFriction=lateral_friction,
    #                 # rollingFriction=rolling_friction,
    #                 # spinningFriction=spinning_friction
    #                 )

    ## INFO DEBUGGING ##

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


    ## INFO DEBUGGING End ##

    # Set the initial position and orientation of the URDF
    initial_position = [0, 0, .2] #x,y,z
    initial_orientation = p.getQuaternionFromEuler([np.pi/2, 0, np.pi/2]) # XYZW - rpy? Robot laying flat
    #initial_orientation = p.getQuaternionFromEuler([np.pi/2, -np.pi/2, 0]) # Robot lying on its side
    #initial_orientation = p.getQuaternionFromEuler([np.pi/2, np.pi/2, 0]) # Robot lying on its other side
    p.resetBasePositionAndOrientation(urdf_id, initial_position, initial_orientation)

    print("NumJoints: ", p.getNumJoints(urdf_id))
    #initial_joint_angles = [np.pi/2, 0, -np.pi/2, 0, np.pi/2, 0, -np.pi/2, 0] # laying flat
    #initial_joint_angles = [0, np.pi/2, 0, -np.pi/2, 0, np.pi/2, 0, -np.pi/2] # back legs down
    initial_joint_angles = np.array([   0, 90, 0,  -90, 0,  100, 0, -100]) * np.pi/180 # back legs down but slightly up
    #initial_joint_angles = [np.pi/4, -np.pi/2, -np.pi/4, np.pi/2, -np.pi/4, np.pi/2, np.pi/4, -np.pi/2] # standing up

    for joint in range(num_joint):
        p.resetJointState(urdf_id, joint,initial_joint_angles[joint])
        p.enableJointForceTorqueSensor(urdf_id, joint, 1)
        print('JointInfo' + str(joint) + ": ", p.getJointInfo(urdf_id, joint))


    #Generate Trajectory
    sweep_duration = 1.0 #seconds
    # Generate time points
    timestep = 1.0 / 100.0
    num_steps = int(sweep_duration / timestep)

    joint_index = 0

    trajectory = walk(sweep_duration,timestep) # 8 x n array

    # Generate a text file that makes it simple to make an arduino array from
    sim_to_real_text_file(trajectory, num_steps)


    # crawlyPos, crawlyOrn = p.getBasePositionAndOrientation(urdf_id)
    # print('INITIAL ROBOT POSITION: ')
    # print(crawlyPos,crawlyOrn)
            
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
            
            # Doing some velocity and force limit test but it wasn't working
            #MG995 Servo Specs
            max_servo_velocity = 8.055 #rad/s
            max_servo_force = 1.177 #Nm
            # maxVelocity = max_servo_velocity,
            # force = max_servo_force

            p.setJointMotorControlArray(
                bodyUniqueId = urdf_id, 
                jointIndices = joint_index_list,
                controlMode = p.POSITION_CONTROL,
                targetPositions = target_position,
                #maxVelocity = max_servo_velocity,
                )
      
            # Step the simulation
            p.stepSimulation()

            # crawlyPos, crawlyOrn = p.getBasePositionAndOrientation(urdf_id)
            # print(crawlyPos,crawlyOrn)



    #Disconnect from PyBullet
    #p.disconnect() # won't reach here because while loop never breaks at the moment

# Path to your URDF file
urdf_path = "urdf/crawly.urdf"  # Change this to your URDF file path
load_and_visualize_urdf(urdf_path)
