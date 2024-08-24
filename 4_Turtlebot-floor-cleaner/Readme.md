# Turtlesim Trajectory Planner with ROS

## Introduction

This project demonstrates the control of a Turtlesim robot using ROS (Robot Operating System). The project includes basic motion control functionalities such as moving forward, rotating, navigating to a specific goal, and executing predefined cleaning patterns (grid cleaning and spiral cleaning). The code provides a foundation for understanding how to control a robot's movement using ROS topics, services, and messages.

## System Overview

The system consists of the following components:

- **Turtlesim**: A simple simulator for teaching ROS concepts.
- **ROS Nodes**: Nodes to control the movement of the Turtlesim robot by publishing velocity commands and subscribing to pose updates.
- **Motion Control Functions**: Functions to move the robot forward, rotate it to a specific angle, navigate to a goal position, and perform grid and spiral cleaning.

## Functional Modules

### 1. Pose Callback

The pose of the Turtlesim robot is continuously updated by subscribing to the `/turtle1/pose` topic. The `poseCallback` function captures the robot's current position and orientation (yaw).

- **`poseCallback(pose_message)`**: Updates global variables for the robot's `x`, `y`, and `yaw` (orientation) based on the incoming pose data.

### 2. Movement Functions

Several functions are defined to control the movement of the Turtlesim robot:

- **`move(speed, distance, is_forward)`**: Moves the robot a specified distance at a given speed. The `is_forward` parameter determines the direction of movement.
- **`rotate(angular_speed_degree, relative_angle_degree, clockwise)`**: Rotates the robot by a specified angle at a given angular speed. The `clockwise` parameter determines the direction of rotation.
- **`go_to_goal(x_goal, y_goal)`**: Moves the robot to a specified goal position (`x_goal`, `y_goal`) using proportional control for both linear and angular velocities.
- **`setDesiredOrientation(desired_angle_radians)`**: Rotates the robot to a specific orientation (in radians).
- **`gridClean()`**: Executes a grid cleaning pattern by moving the robot in a predefined path.
- **`spiralClean()`**: Executes a spiral cleaning pattern by gradually increasing the radius of the robot's path.

### 3. Cleaning Patterns

The robot can perform two types of cleaning patterns:

- **Grid Cleaning**: The robot moves in straight lines, covering the grid in a systematic pattern.

![Grid_clearning_algorithm](https://github.com/user-attachments/assets/eb738c81-3404-45f3-9adc-ea410b85fdd4)

- **Spiral Cleaning**: The robot moves in a spiral pattern, gradually increasing the radius as it moves outward.

![spiral_cleaning](https://github.com/user-attachments/assets/2a6aa7db-2ae1-489f-8020-3edd0550ec91)

## How to Run the Code

1. **Install ROS**: Ensure that ROS is installed and properly configured on your system.
2. **Launch Turtlesim**: Start the Turtlesim simulator by running the following command - rosrun turtlesim turtlesim_node
3. **Run Motion Control Node**: Execute the Python script to start controlling the Turtlesim robot - rosrun your_package_name turtlesim_motion_control.py
4. **Observe the Robot**: The robot will execute the predefined grid or spiral cleaning pattern. You can also modify the script to execute other movements or navigate to different goal positions.

## Conclusion
This project provides a practical introduction to controlling a simulated robot using ROS. By exploring the provided functions and modifying the parameters, users can gain a deeper understanding of motion control in robotic systems. The Turtlesim simulator serves as a simple yet effective tool for learning these concepts.
