# Set friction parameters
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