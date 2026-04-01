# Impedance Control of Dexterous Gripper


## 📖 Overview
This is a mini project of AAU ROB8 2026 student from group 860. This project carry out an Anisotropic Impedance Control framework for a UR5 manipulator in Gazebo Harmonic, transitioning from blind position control to a  interaction-based model. While presently validated through successful peg-in-hole simulations, the framework is theoretically designed to integrate Sequential Monte Carlo estimation for future high level state reasoning.


## 🛠️ Prerequisites
To run this project, you will need the following environment:

* **OS:** Ubuntu 24.04.4

* **ROS 2:** Jazzy Jalisco

* **Simulator:** Gazebo Harmonic


<img width="700" height="700" alt="env_with_arm" src="https://github.com/user-attachments/assets/52972fb6-0692-41f6-8e16-7616a2f8aca6" />


## ⚙️ Building the Workspace
Since this repository only contains the `src` folder, please follow these steps to build the workspace on your local machine:

1. **Create a workspace and clone the repository:**
   ```bash
   mkdir -p ~/peg_in_hole_ws/
   cd ~/peg_in_hole_ws/
   # Clone this repository
   Download and extract the src folder here

2. **Install Dependencies:**
   ```bash
   cd ~/simulation_ws
   rosdep update

3. **Build the project:**
   ```bash
   colcon build --symlink-install


## 🚀 Run the Simulation
Once built, you can launch the simulation using the following commands.

1. **Source the workspace:**
   ```bash
   cd ~/peg_in_hole_ws
   source install/setup.bash

2. **Launch the Gazebo world and robot:**
   ```bash
   ros2 launch peg_insertion sim_launch.py


## 📁 Project Structure
Here is a brief overview of what is included in the src package:

/launch: Contains the ROS 2 launch files to start the simulation.

/urdf: Contains the robot descriptions and XACRO files.

/worlds: Contains the custom Gazebo environment.

/config: Contains the controller for the arm and the hand

/meshes: Contains the 3D files for Shadow Hand

/peg_insertion: Contains the Impedance Controller Code
