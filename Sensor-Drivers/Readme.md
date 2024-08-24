## Overview

This repository contains a series of projects focused on analyzing data collected from GNSS (Global Navigation Satellite System) and IMU (Inertial Measurement Unit) sensors under various experimental conditions. The purpose of these projects is to evaluate the accuracy, stability, and error characteristics of the sensors, which are crucial for precise navigation and positioning in robotics and other applications.

### Included Projects:

1. **IMU Sensor Data Analysis**
   - **Objective**: To analyze the noise characteristics and error distribution of an IMU sensor under different conditions.
   - **Scenarios**:
     - *Individual Data*: Collected in a quiet environment to observe short-term sensor behavior.
     - *Group Data*: Collected over an extended period in a basement to analyze long-term noise in angular velocity and linear acceleration.
   - **Tools Used**: ROS2 for driver, Python for simulation and analysis.

2. **RTK Rover Position Analysis**
   - **Objective**: To assess the positional accuracy of a rover RTK using telemetry radio communications with a base RTK in both open and obstructed environments.
   - **Scenarios**:
     - *Stationary*: Analyzing the stability of the rover RTK when held in a fixed position.
     - *Moving*: Evaluating the positional accuracy while walking in a defined path.
   - **Tools Used**: ROS2 for driver, Python for simulation and analysis.

3. **GNSS Puck Data Collection and Analysis** 
   - **Objective**: To analyze GNSS data collected using a GPS puck in both stationary and moving scenarios to evaluate the stability and accuracy of the GPS readings.
   - **Scenarios**:
     - *Stationary*: Evaluating the stability of the GPS puck in a fixed position.
     - *Moving*: Assessing the accuracy of distance measurements during a straight-line walk.
   - **Tools Used**: ROS2 for driver, Python for simulation and analysis..


---
