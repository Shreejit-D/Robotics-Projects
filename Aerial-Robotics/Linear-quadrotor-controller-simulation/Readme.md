# Linear quadrotor PD Controller

## Introduction

This project involves developing and implementing a Proportional Derivative (PD) controller for a quadrotor. The goal is to control the height of the quadrotor by tuning the PD control gains. This exercise builds on the concepts learned in the lecture series, where you were introduced to quadrotor dynamics and control theory. The provided quadrotor simulator in MATLAB will be used to test and refine the PD controller.

## Quadrotor Simulator

The quadrotor's behavior is simulated using MATLAB’s ODE solver `ode45`. The simulation visualizes the state of the quadrotor at each time step using the `plot` and `plot3` functions. Before implementing the PD controller, it is recommended to run the provided simulation script (`runsim.m`). If the quadrotor falls from height 0, the simulator is functioning correctly, and you can proceed to develop the controller.

## PD Controller

The dynamic equation for the motion of the quadrotor in the z-direction is given by:

$$
\ddot{z} = \frac{u}{m} - g
$$

Where:
- `z''` is the acceleration in the z-direction,
- `u` is the control input (thrust),
- `m` is the mass of the quadrotor,
- `g` is the gravitational acceleration.

The PD controller for the height is defined as:

$$
u = m \left(\ddot{z}_{des} + K_p e + K_v \dot{e} + g\right)
$$

Where:
- `e = z_des - z` is the position error,
- `e' = z'_des - z'` is the velocity error,
- `Kp` is the proportional gain,
- `Kv` is the derivative gain.

## Tasks

1. **Implement the PD Controller**: Develop the PD controller in `controller.m` to control the height of the quadrotor.
2. **Tune the Gains**: Adjust the proportional gain (`Kp`) and derivative gain (`Kv`) until the quadrotor converges quickly and smoothly to the desired step response input.

## Files Included

- `controller.m`: Contains the PD controller implementation.
- `runsim.m`: Test script to simulate the quadrotor.
- `height_control.m`: Simulation code invoked by `runsim.m`.
- `submit.m`: Script for generating submission files.
- `evaluate.p`: Evaluation script used by `submit.m`.
- `fixed_set_point.m`: Script to test step response.
- `utils/`: Directory containing helper functions for the quadrotor simulator.

## Conclusion

This project provides hands-on experience with implementing a PD controller to manage the height of a quadrotor. By adjusting the gains appropriately, you can achieve a stable and responsive control system. This exercise also enhances your understanding of quadrotor dynamics and MATLAB-based simulation.
## Conclusion

This project provides hands-on experience with implementing a PD controller to manage the height of a quadrotor. By adjusting the gains appropriately, you can achieve a stable and responsive control system. This exercise also enhances your understanding of quadrotor dynamics and MATLAB-based simulation.
