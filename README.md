# Project Summaries

This repository contains five projects focused on various aspects of robotics, sensor fusion, visual odometry, and computer vision, primarily utilizing ROS (Robot Operating System), OpenCV, and the KITTI dataset. Below is a brief overview of each project:

## 1. Sensor Fusion and Dead Reckoning
This project implements sensor fusion using IMU (Inertial Measurement Unit) data to perform dead reckoning for estimating a robot's position. The focus is on yaw angle calculation using magnetometer and gyroscope data, followed by forward velocity estimation. A complementary filter is applied to merge these readings for improved accuracy, and the results are compared with GPS data to evaluate performance. The project highlights the challenges of sensor calibration and the importance of correcting sensor biases.

## 2. LaserScan Processing and Obstacle Avoidance for ROS
This project involves a ROS-based system that processes laser scan data from a LIDAR sensor to detect obstacles and control robot movement. The node subscribes to the `/scan` topic and processes the data to identify obstacles, enabling the robot to navigate safely. Key features include proportional speed control, obstacle avoidance behaviors, and field of view calculations, making it essential for robust and efficient robot navigation in dynamic environments.

## 3. Tennis Ball Detection using ROS and OpenCV
This project demonstrates real-time detection and tracking of a tennis ball in a video stream using ROS and OpenCV. It involves two main scripts: one for publishing video frames as ROS messages and another for processing these frames to detect the tennis ball. Techniques like color filtering and contour detection are employed to isolate and track the ball. The project serves as a practical example of integrating ROS with OpenCV for object detection tasks in robotics.

## 4. Turtlesim Trajectory Planner with ROS
This project provides a foundation for controlling a simulated Turtlesim robot using ROS. It includes basic motion control functionalities such as moving forward, rotating, navigating to a specific goal, and executing predefined cleaning patterns (grid and spiral cleaning). The project is designed to teach fundamental concepts of robot motion control using ROS topics, services, and messages, making it an ideal starting point for those new to ROS and robotic control.

## 5. Visual Odometry Using KITTI Dataset
This project focuses on visual odometry using the KITTI dataset to calculate vehicle poses through stereo matching techniques. It involves depth calculation using the SGBM algorithm, feature detection with SIFT, and pose estimation by analyzing transformations between consecutive frames. The project also integrates LIDAR corrections to improve accuracy and compares the results with ground truth data. The results demonstrate the potential and challenges of using visual odometry in autonomous driving, highlighting areas for further improvement such as advanced filtering techniques.

Each project in this repository offers a unique exploration into different aspects of robotics and computer vision, providing practical implementations and insights into these technologies.
