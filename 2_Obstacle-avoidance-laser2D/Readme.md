# LaserScan Processing and Obstacle Avoidance for ROS

This project implements a ROS (Robot Operating System) node for processing laser scan data from a robot's LIDAR sensor to detect obstacles and control the robot's movement. The node subscribes to the `/scan` topic, processes the incoming laser scan data to identify the minimum and maximum distances to obstacles, and commands the robot to move or rotate based on this information.

## Features

- **LaserScan Data Processing**: The node calculates the minimum, maximum, and average distance values from the LIDAR scan data and identifies the field of view.
- **Obstacle Avoidance**: The robot adjusts its speed and direction based on the proximity of obstacles, ensuring safe navigation. The node includes two behaviors:
  - **Behavior 1**: Moves the robot forward with proportional speed control based on the distance to obstacles.
  - **Behavior 2**: Forces the robot to rotate until it finds open space when an obstacle is too close.
- **Field of View Calculation**: The field of view is calculated from the scan data, helping to determine the relevant scan angles for obstacle detection.
- **Modular Functions**: The project includes various helper functions to calculate minimum and maximum ranges, average values, and control movement based on these values.

## Code Overview

### Main Components

- **scan_callback(scan_data)**: Processes the LIDAR scan data to extract minimum, maximum, and average range values, and identifies the closest obstacles within a specified field of view.
- **move(avoid_obstacle)**: Controls the forward movement of the robot, with speed adjustments based on the distance to the nearest obstacle. If an obstacle is too close, it triggers the rotate behavior.
- **rotate(avoid_obstacle)**: Rotates the robot to find a clear path when an obstacle is detected within close range.

### Helper Functions

- **min_range_index(ranges)**: Finds the minimum range value and its index from the LIDAR scan data.
- **max_range_index(ranges)**: Finds the maximum range value and its index from the LIDAR scan data.
- **average_range(ranges)**: Calculates the average distance from the LIDAR scan data.
- **average_between_indices(ranges, i, j)**: Calculates the average distance between two specified indices in the LIDAR scan data.
- **min_dist_in_view(ranges, angle)**: Finds the minimum distance within a specified field of view.

## Running the Node

1. Initialize the ROS node with `rospy.init_node('scan_node', anonymous=True)`.
2. Subscribe to the `/scan` topic using `rospy.Subscriber("scan", LaserScan, scan_callback)`.
3. Call the `move(True)` and `rotate(True)` functions to begin obstacle avoidance behavior.
4. Use `rospy.spin()` to keep the node running.

## Dependencies

- Python
- ROS (Robot Operating System)
- sensor_msgs for LaserScan messages
- geometry_msgs for Twist messages

## Usage

To use this node, ensure you have a ROS environment set up with a LIDAR sensor publishing to the `/scan` topic. Run the script to start the obstacle avoidance behavior.
