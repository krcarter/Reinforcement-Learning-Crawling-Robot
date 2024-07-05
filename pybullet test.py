import pybullet as p
import pybullet_data
import time

def test_pybullet():
    # Connect to PyBullet physics server
    physicsClient = p.connect(p.GUI)  # Use p.DIRECT for non-GUI version
    
    # Set the search path to find the URDF files
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Load the plane URDF
    planeId = p.loadURDF("plane.urdf")
    
    # Set gravity in the simulation
    p.setGravity(0, 0, -9.8)
    
    # Load the R2D2 robot URDF
    startPos = [0, 0, 1]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    robotId = p.loadURDF("r2d2.urdf", startPos, startOrientation)
    
    # Run the simulation for 1000 steps
    for i in range(1000):
        p.stepSimulation()
        time.sleep(1./240.)  # Slow down the simulation to real-time
        
        # Optionally, get the robot's position and orientation
        pos, orn = p.getBasePositionAndOrientation(robotId)
        print(f"Step {i}, Position: {pos}, Orientation: {orn}")
    
    # Disconnect from the physics server
    p.disconnect()

if __name__ == "__main__":
    test_pybullet()
