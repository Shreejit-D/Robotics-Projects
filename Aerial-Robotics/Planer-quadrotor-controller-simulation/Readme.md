# Planar Quadrotor Simulation in MATLAB

This project simulates a planar quadrotor in the Y-Z plane using MATLAB. The main focus is on implementing a Proportional-Derivative (PD) controller to control the motion of the quadrotor.

## Overview

The quadrotor is modeled in the Y-Z plane, with its orientation defined by a roll angle (φ). The dynamics of the system are governed by the following key equations:

### System Dynamics

The system's motion in the Y-Z plane is described by Newton's equations of motion:

m * r'' = m * [y'' z'']^T = [0 -mg]^T + [-u1 * sin(φ) u1 * cos(φ)]^T

The angular acceleration is given by:

Ixx * φ'' = u2

### Linearized System

To implement the PD controller, the dynamics are linearized around the hover configuration:

y'' = -g * φ
z'' = -g + (u1 / m)
φ'' = u2 / Ixx

### Controller Design

The PD controller is designed to minimize the position and velocity errors:

u1 = m * [g + zT'' + kv,z * (zT' - z') + kp,z * (zT - z)]
u2 = Ixx * [φT'' + kv,φ * (φT' - φ') + kp,φ * (φT - φ)]

### Hover Controller

For hovering, where the desired position is constant and the desired roll angle is zero:

u1 = m * [g - kv,z * z' + kp,z * (z0 - z)]
u2 = Ixx * [φT'' + kv,φ * (φT' - φ') + kp,φ * (φT - φ)]
φc = -1/g * [kv,y * (-y') + kp,y * (y0 - y)]

## Getting Started

### Prerequisites

- MATLAB installed on your system.

### Files Included

- `controller.m`: Implement the PD controller for the planar quadrotor.
- `evaluate.p`: Code to evaluate your controller.
- `runsim.m`: Test script for running the simulation.
- `simulation_2d.m`: Simulation code called by `runsim`.
- `submit.m`: Script for submitting and evaluating your solution.
- `trajectories/`: Example trajectories for testing the controller.
- `utils/`: Helper functions.

### Tasks

1. Implement the PD controller in `controller.m`.
2. Test the controller using `runsim.m`.
3. Adjust the controller gains to ensure the quadrotor follows the desired trajectory.

## Usage

To run the simulation and test your controller, execute the following command in MATLAB:

```matlab
runsim
