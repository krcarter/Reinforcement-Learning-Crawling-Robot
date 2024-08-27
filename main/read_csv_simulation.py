import mplcursors
import numpy as np
import matplotlib.pyplot as plt


def plot_joint_positions(trajectories):
    # Step 2: Plot the data
    plt.figure(figsize=(10, 6))

    # Assuming each row corresponds to a different joint, plot each row
    for i in range(trajectories.shape[0]):
        plt.plot(trajectories[i, :], label=f'Joint {i+1}')

    # Step 3: Add labels and title
    plt.title('Joint Positions Over Time')
    plt.xlabel('Time Step')
    plt.ylabel('Joint Angle [rad]')
    plt.grid(True)
    plt.legend()

    mplcursors.cursor(hover=True).connect(
    "add", lambda sel: sel.annotation.set_text(f'x={sel.target[0]:.2f}, y={sel.target[1]:.2f}, i ')
)

    plt.show()

def arduino_text(trajectories):
    # Open a text file to write the formatted arrays

    num_steps = trajectories.shape[1]
    trajectories = np.degrees(trajectories)

    # Math to transform simulation angles to the robot arduino angles
    trajectories[0] = 90 - trajectories[0]
    trajectories[1] = trajectories[1]
    trajectories[2] = 90 - trajectories[2]
    trajectories[3] = 180 - (-1*trajectories[3])


    with open('sim_to_real_826.txt', 'w') as file:
        for idx, row in enumerate(trajectories, start=1):
            # Format each value in the row no decimals
            formatted_values = ','.join(f"{val:.0f}" for val in row)
            
            # Create the Arduino array string
            #num_steps_string = str(num_steps)
            array_string = f"int row_{idx}[{num_steps}] = {{ {formatted_values} }};\n"
            
            # Write the array string to the file
            file.write(array_string)

sim_trajectory = np.loadtxt('sim_trajectory_26.csv', delimiter=',')
# Define the range of timesteps you want to plot
start_timestep = 37
end_timestep = 87

# Step 2: Slice the data to get the desired range
sim_slice = sim_trajectory[:, start_timestep:end_timestep]

plot_joint_positions(sim_trajectory)

plot_joint_positions(sim_slice)

arduino_text(sim_slice)



