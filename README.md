# Assignment2_rt_Part2: Turtle Simulation Project

## Overview:

This repository contains a ROS2 package `assignment2rt_part2` that allows you to control a robot in a simulation environment. The user can set the robot's linear and angular velocities through a terminal interface. The robot will move based on the given velocities.

### Features:

- User-friendly terminal interface for setting robot velocities.
- Controls both linear (X, Y, Z) and angular (Roll, Pitch, Yaw) velocities.
- Robot moves for exactly 5 seconds after the user inputs the desired velocities.
- Supports continuous user input for multiple commands.

### Prerequisites

1. **ROS2 Installed**:
   - Ensure you have ROS2 installed on your system (e.g., Humble, Foxy, Galactic, etc.).
   - If ROS2 is not installed.

2. **ROS2 Workspace**:
   - Make sure you have a ROS2 workspace set up. If not, create one:
     ```bash
     mkdir -p ~/my_ros2_ws/src
     cd ~/my_ros2_ws
     ```

---

## File structure:

├── assignment_2_part2 <br>
│   ├── setup.py       <br>         # Package setup file for ROS2
│   ├── package.xml     <br>        # ROS2 package configuration
│   ├── README.md        <br>       # Project documentation
│   └── assignment_2_part2  <br>    # Python package for the ROS2 node
│       ├── __init__.py   <br>
│       └── move_robot_node.py <br> # Main ROS2 node for robot control

     
## How to Make It Work

Follow these steps to clone the package, build it, and run the ROS2 node.


### Steps to Get Started

1. **Clone the Robot Environment Package**:
   - First, clone the `robot_urdf` package into your workspace. This package contains the robot's description and environment for Gazebo and RViz.
     ```bash
     cd ~/ros2_ws/src
     git clone https://github.com/CarmineD8/robot_urdf.git
     ```
   - Switch to the `ros2` branch:
     ```bash
     cd robot_urdf
     git checkout ros2
     ```

2. **Clone the Node Package**:
   - Next, clone the `assignment_2_part2` package into your workspace:
     ```bash
     git clone https://github.com/Mohamedags/assignment2rt_part2
     ```

3. **Build the Workspace**:
   - After cloning both packages, go to the root of your workspace and build it:
     ```bash
     cd ~/ros2_ws
     colcon build
     ```

4. **Source the Workspace**:
   - Source your workspace to use the built packages:
     ```bash
     source install/setup.bash
     ```

5. **Launch the Robot Environment**:
   - Launch the Gazebo simulation environment for the robot using the `robot_urdf` package:
     ```bash
     ros2 launch robot_urdf gazebo.launch.py
     ```

6. **Run the Node**:
   - In a new terminal (don’t forget to source the workspace again), run the control node:
     ```bash
     ros2 run assignment_2_part2 move_robot_node
     ```

---

### How It Works

1. **Set Velocities**:
   - After running the node, you’ll be prompted to enter the robot's linear and angular velocities.

   - Input 6 values:
     - **Linear Velocities**: X, Y, Z (in meters per second).
     - **Angular Velocities**: Roll, Pitch, Yaw (in radians per second).
     
2. **Set Velocities** Ask user if he wants to enter new velocities? (yes/no): yes -enter robot's velocities again-
   - After the robot stops, the program will ask: "Do you want to enter new velocities? (yes/no)"
   - Type yes to input new velocities and repeat the process.
   - Enter the robot's new linear and angular velocities as prompted.
   - Type no to exit the program.
   
   Example:
   ```bash
   Enter linear velocity X (m/s): 0.5
   Enter linear velocity Y (m/s): 0.0
   Enter linear velocity Z (m/s): 0.0
   Enter angular velocity Roll (rad/s): 0.0
   Enter angular velocity Pitch (rad/s): 0.0
   Enter angular velocity Yaw (rad/s): 0.3
   
- Illustration : 
![Example GIF](images/illustration_rt2_part2)

