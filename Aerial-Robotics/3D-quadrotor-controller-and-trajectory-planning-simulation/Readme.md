# 3D Quadrotor Simulation in MATLAB

This project extends the previous 2D quadrotor control to a full 3D simulation using MATLAB. The goal is to implement a Proportional-Derivative (PD) controller to control the quadrotor's motion in 3D space and generate time-parameterized trajectories that navigate the quadrotor through specified waypoints.

## Overview

The quadrotor operates in a 3D environment, with its dynamics governed by several key equations. The simulation includes the effects of rotor forces, moments, and the quadrotor's rigid body dynamics.

### System Model

The system model for the 3D quadrotor includes:

1. **Motor Model**: Each rotor generates a force `Fi = kF * ωi^2` and a moment `Mi = kM * ωi^2`.
2. **Rigid Body Dynamics**: The quadrotor's orientation is described using Z-X-Y Euler angles (yaw, pitch, roll), and its dynamics are governed by Newton’s and Euler’s equations of motion.

### Key Equations

1. **Force Dynamics**:

      m * r'' = [0 0 -mg]^T + R * [0 0 (F1 + F2 + F3 + F4)]^T    where `u1 = Σ Fi` (sum of forces from all four rotors).

2. **Moment Dynamics**:

      I * [p' q' r']^T = [L(F2 - F4) L(F3 - F1) (M1 - M2 + M3 - M4)]^T - [p q r]^T × I * [p q r]^T

3. **Hover Controller**:
For maintaining a stationary position, the command accelerations are calculated using a PD controller:

      u1 = mg - m(kd3 * z' + kp3 * (z - z0))

4. **The desired roll and pitch angles are**:

     φ_des = 1/g * (r1''_des * sin(ψ_T) - r2''_des * cos(ψ_T))

     θ_des = 1/g * (r1''_des * cos(ψ_T) + r2''_des * sin(ψ_T))

4. **3D Trajectory Control**:
For trajectory following, the desired acceleration is calculated:

      (r'_T - r'_des) + kd * e_v + kp * e_p = 0

### Trajectory Generation

The trajectory is generated as a piecewise 7th-order polynomial, ensuring smooth transitions through all waypoints. The polynomial coefficients are determined by solving the following system of equations: 

A * α = b
α = A^(-1) * b

## Getting Started

### Prerequisites

- MATLAB installed on your system.

### Files Included

- `controller.m`: Implement the PD controller for the 3D quadrotor.
- `traj_generator.m`: Trajectory generator for passing through given waypoints.
- `runsim.m`: Test script for running the simulation.
- `simulation_3d.m`: Simulation code called by `runsim`.
- `evaluate.p`: Code to evaluate your controller.
- `submit.m`: Script for submission and evaluation.
- `traj_helix.p`: Pre-defined helical trajectory for testing.
- `traj_line.p`: Pre-defined line trajectory for testing.
- `utils/`: Helper functions for the project.

### Tasks

1. **Controller Implementation**: Complete the PD controller in `controller.m`.
2. **Trajectory Generation**: Implement `traj_generator.m` to generate smooth trajectories through waypoints.
3. **Testing**: Use `runsim.m` to test the controller with different trajectories.

## Usage

To run the simulation and test your controller, execute the following command in MATLAB:

```matlab
runsim
