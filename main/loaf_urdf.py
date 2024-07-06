import pybullet as p
import pybullet_data
import time
import os

current_path = os.path.dirname(os.path.realpath(__file__))
print("Current path:", current_path)

def load_and_visualize_urdf(urdf_path):
    # Connect to PyBullet
    physicsClient = p.connect(p.GUI)

    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

    # Set the path to PyBullet's data directory
    print("Pybullet Path: ", pybullet_data.getDataPath())
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Set the gravity
    p.setGravity(0, 0, -9.81)

    # Create a plane for the URDF to stand on
    plane_id = p.loadURDF("plane.urdf")

    # Load the URDF file
    urdf_id = p.loadURDF(urdf_path)
    num_joint = p.getNumJoints(urdf_id)

    ## INFO DEBUGGING ##

    # joint_order = [1,5,2,6,3,7,4,8]
    # Why is there a joint upper limit for the continous joints???????

    print("NumJoints: ", p.getNumJoints(urdf_id))
    initial_joint_angles = [0, 0, 0, 0, 0, 0, 0, 0]

    for joint in range(p.getNumJoints(urdf_id)):
        print('JointInfo' + str(joint) + ": ", p.getJointInfo(urdf_id, joint))
        #p.resetJointState(urdf_id, joint, initial_joint_angles[joint])

    ## INFO DEBUGGING End ##

    #print("JointInfo: ", p.getJointInfo(urdf_id))
    # Set the initial position and orientation of the URDF
    initial_position = [0, 0, 1] #x,y,z
    initial_orientation = p.getQuaternionFromEuler([0, 0, 0]) # XYZW - rpy?
    p.resetBasePositionAndOrientation(urdf_id, initial_position, initial_orientation)

    # Run the simulation
    while True:
        p.stepSimulation()
        time.sleep(1./240.)

        # LOGGGING #
        crawlyPos, crawlyOrn = p.getBasePositionAndOrientation(urdf_id)
        #print(crawlyPos,crawlyOrn)



    # Disconnect from PyBullet
    p.disconnect() # won't reach here because while loop never breaks at the moment

# Path to your URDF file
urdf_path = "crawly/crawly.urdf"  # Change this to your URDF file path
load_and_visualize_urdf(urdf_path)
