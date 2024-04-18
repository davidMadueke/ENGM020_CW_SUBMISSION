# Project Name: Quadcopter Simulation and Visualization

## Description:
This MATLAB project simulates the dynamics of a quadcopter flying in 3D space and visualizes its reference trajectory. The simulation is controlled by a main MATLAB script named `coursework1.m`, which interacts with two function files: `inputMatrix.m` to generate control inputs and `referenceTrajectory.m` to define the desired trajectory for the quadcopter.

## Files:
1. `coursework1.m`: This is the main MATLAB script where all the code is processed and data is visualized. It orchestrates the simulation, generates control inputs, computes the quadcopter dynamics, and visualizes the trajectory.

2. `inputMatrix.m`: This function file generates control inputs for the quadcopter simulation. It can be customized to define control signals based on desired maneuvers or control algorithms.

3. `referenceTrajectory.m`: This function file defines the reference trajectory (simulated flight) for the quadcopter. It specifies predefined waypoints and maneuvers such as takeoff, waypoint navigation, steady flight, and landing.

## Usage:
1. Ensure that all three MATLAB files (`coursework1.m`, `inputMatrix.m`, `referenceTrajectory.m`) are in the same directory.
2. Open MATLAB and run the `coursework1.m` script.
3. The script will execute the simulation, generate control inputs, compute the quadcopter dynamics, and visualize the trajectory in 3D space.

## Customization:
- Modify `inputMatrix.m` to customize control inputs based on specific requirements or control algorithms.
- Adjust parameters and maneuvers in `referenceTrajectory.m` to define different reference trajectories for the quadcopter.

## Dependencies:
- MATLAB R2018a or later versions.
- No additional toolboxes are required.

## Contributors:
- David Madueke

**Note:**
- This README provides an overview of the project and instructions for usage. For detailed code documentation, please refer to the comments within the MATLAB files.
