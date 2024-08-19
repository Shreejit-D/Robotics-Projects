# Quadrotor Simulation Projects

Welcome to the Quadrotor Simulation Projects repository! This repository contains multiple MATLAB-based projects that simulate and control quadrotors in different environments and dimensions. The projects included are:

<div align="center">
  <img src="https://cdn.dribbble.com/users/346181/screenshots/1771066/quadcopter.gif" width="300"/>
</div>

1. **Linear Quadrotor Simulation**
2. **Planar Quadrotor Simulation**
3. **3D Quadrotor Simulation**

## Overview of Quadrotors

A quadrotor, also known as a quadcopter, is a type of rotorcraft that uses four rotors to generate lift and control motion. Unlike traditional helicopters, which use variable pitch rotors, quadrotors achieve control by varying the speed of each rotor. This design provides stability and maneuverability, making quadrotors popular in both recreational and professional applications.

### Key Components of a Quadrotor

- **Rotors**: Four rotors arranged in a square configuration provide thrust. Two rotors rotate clockwise, and two rotate counterclockwise to balance the torque.
- **Frame**: The structure that connects the rotors and houses the electronics and sensors.
- **Inertial Measurement Unit (IMU)**: Sensors that provide orientation, acceleration, and angular velocity data.
- **Controller**: The system that adjusts rotor speeds to control the quadrotor’s position and orientation.

## Linearized Controllers

In the context of quadrotor control, a controller is an algorithm that adjusts the rotor speeds to achieve desired movements or maintain stability. Given the nonlinear dynamics of quadrotors, linearized controllers are often used for simplicity and efficiency, especially when the quadrotor is operating near a stable hover state.

### Proportional-Derivative (PD) Controller

The Proportional-Derivative (PD) controller is a common approach used to control the quadrotor's orientation (roll, pitch, yaw) and position. The PD controller works by:

- **Proportional Control (P)**: Correcting the error between the desired and actual state by applying a control force proportional to the error.
- **Derivative Control (D)**: Dampening the system by applying a control force proportional to the rate of change of the error.

By linearizing the nonlinear equations of motion around a nominal hover state, the PD controller can effectively stabilize the quadrotor and track desired trajectories with minimal computational complexity.

### Hover Control

Hover control is a specific application of the PD controller where the goal is to maintain the quadrotor in a fixed position. In this case, the desired position is constant, and the PD controller adjusts the rotor speeds to counteract disturbances like wind or slight changes in the quadrotor's position.

## Trajectory Generation

Trajectory generation refers to the process of planning a path for the quadrotor to follow. This path is usually specified as a sequence of waypoints that the quadrotor must navigate through.

### Polynomial Trajectory Generation

For smooth and continuous motion, trajectories are often represented as piecewise polynomials. In 3D quadrotor control, for example, a 7th-order polynomial is used to describe the path between each pair of waypoints. The polynomial coefficients are calculated to ensure that the quadrotor:

- Passes through all the waypoints.
- Starts and stops at rest.
- Maintains continuity in position, velocity, and acceleration between trajectory segments.

This method of trajectory generation ensures that the quadrotor can follow complex paths with precision while maintaining stability.

## Projects Overview

### 1. Linear Quadrotor Simulation

This project simulates the quadrotor's motion in a simplified linear environment. The focus is on understanding the basic dynamics of quadrotors and implementing a basic PD controller to maintain stability and follow simple trajectories.

### 2. Planar Quadrotor Simulation

In this project, the quadrotor is restricted to movement within the Y-Z plane. The PD controller is extended to control motion in this 2D environment. The project involves stabilizing the quadrotor and following a desired trajectory within the plane.

### 3. 3D Quadrotor Simulation

This project expands the control to a full 3D environment, where the quadrotor can move in all three spatial dimensions. The project involves implementing a PD controller that can stabilize the quadrotor in 3D space and follow complex 3D trajectories.

## Getting Started

Each project folder contains a detailed `README.md` file explaining how to set up and run the simulations. MATLAB is required to run the code, and each project includes sample trajectories and test scripts to evaluate your controller's performance.

## Contributing

Contributions to improve these projects are welcome! Feel free to submit a pull request or open an issue if you have suggestions for improvements or additional features.
