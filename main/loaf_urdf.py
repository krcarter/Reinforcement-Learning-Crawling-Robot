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

    # Load the URDF file
    urdf_id = p.loadURDF(urdf_path)

    # Set the gravity
    p.setGravity(0, 0, -9.81)

    # Create a plane for the URDF to stand on
    plane_id = p.loadURDF("plane.urdf")

    # Set the initial position and orientation of the URDF
    initial_position = [0, 0, 1] #x,y,z
    initial_orientation = p.getQuaternionFromEuler([0, 0, 0]) # rpy?
    p.resetBasePositionAndOrientation(urdf_id, initial_position, initial_orientation)

    # Run the simulation
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
        crawlyPos, crawlyOrn = p.getBasePositionAndOrientation(urdf_id)
        #print(crawlyPos,crawlyOrn)



    # Disconnect from PyBullet
    p.disconnect()

# Path to your URDF file
urdf_path = "crawly/crawly.urdf"  # Change this to your URDF file path
load_and_visualize_urdf(urdf_path)
