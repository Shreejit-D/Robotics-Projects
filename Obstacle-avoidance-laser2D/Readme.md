# Obstacle Detection and Avoidance using ROS and LiDAR

## Introduction

This project implements a basic obstacle detection and avoidance system using a 2D LiDAR sensor and the Robot Operating System (ROS). The system reads laser scan data from the LiDAR sensor, processes it to determine the proximity of obstacles, and adjusts the robot's movement to avoid collisions.

## System Overview

The system consists of the following components:

- **LiDAR Sensor**: Provides 2D distance measurements (ranges) around the robot.
- **ROS Node**: A central control node that processes LiDAR data and makes decisions for robot navigation.
- **Movement Control**: Adjusts the robot's velocity and direction based on processed LiDAR data to avoid obstacles.

## Functional Modules

### 1. LiDAR Data Processing

The LiDAR sensor provides a set of range measurements representing distances from the sensor to the nearest obstacles at various angles. The following functions are implemented to process this data:

- **`min_range_index(ranges)`**: Finds the minimum distance (closest obstacle) and its corresponding angle.
- **`max_range_index(ranges)`**: Finds the maximum distance (farthest obstacle) and its corresponding angle.
- **`average_range(ranges)`**: Calculates the average distance of all valid measurements.
- **`average_between_indices(ranges, i, j)`**: Computes the average distance between specified indices.
- **`min_dist_in_view(ranges, angle)`**: Determines the minimum distance within a specified field of view.

### 2. Movement Control

The movement control module uses the processed LiDAR data to decide how the robot should move to avoid obstacles:

- **`move(avoid_obstacle)`**: Moves the robot forward until an obstacle is detected within a specified distance. Adjusts speed and direction based on the proximity of obstacles on the left and right sides.
- **`rotate(avoid_obstacle)`**: Rotates the robot until it finds an open space when an obstacle is detected directly ahead.

### 3. ROS Node Setup

The ROS node is responsible for subscribing to the LiDAR sensor data topic and controlling the robot's movement:

- **`scan_callback(scan_data)`**: Callback function that processes incoming LiDAR data.
- **`rospy.init_node('scan_node', anonymous=True)`**: Initializes the ROS node.
- **`rospy.Subscriber("scan", LaserScan, scan_callback)`**: Subscribes to the `/scan` topic to receive LiDAR data.

## How to Run the Code

1. **Install ROS**: Ensure that ROS is installed and properly configured on your system.
2. **Set Up LiDAR**: Connect the LiDAR sensor to your robot and ensure it publishes data to the `/scan` topic.
3. **Run the Node**: Execute the Python script using ROS to start the obstacle detection and avoidance system:

## Conclusion
This project provides a basic framework for obstacle detection and avoidance using ROS and a LiDAR sensor. It can be extended with more advanced features such as incorporating additional sensors, using more sophisticated algorithms for obstacle detection, or implementing more complex navigation strategies.
   rosrun your_package_name your_script_name.py
