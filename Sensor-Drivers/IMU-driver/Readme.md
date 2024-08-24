# IMU Sensor Data Analysis

## Project Overview

This project focuses on the analysis of data collected from an Inertial Measurement Unit (IMU) sensor in two different environments: a controlled quiet environment and a basement over an extended period. The goal is to evaluate the noise characteristics and errors associated with the IMU sensor.

### Sections:
1. **Individual Data Collection**: IMU data was collected in a quiet environment for 5 minutes.
2. **Group Data Collection**: IMU data was collected in a basement for 5 hours.

---

## Individual Data Collection

### Environment:
- **Location**: Garage (Quiet Place)
- **Duration**: 5 minutes

### Observations:
- The experiment was conducted in a garage with potential disturbances:
  - **Air Vent Fan**: Caused periodic fluctuations.
  - **Passing Cars**: Introduced additional noise.

### Analysis:
- **Fluctuations**: Notable in yaw angle (gradual increase over time), likely due to wire movement after initiating the recording.
- **Statistical Summary**: 
  - Yaw angle showed a high skew, indicating significant bias.
  - Other parameters exhibited expected ranges of fluctuations, with their mean values, skew, and standard deviation calculated.

### Distribution Analysis:
- **Histograms**: 
  - Normal distribution observed for most parameters except for roll, pitch, and yaw, which displayed random distributions.

---

## Group Data Collection

### Environment:
- **Location**: Basement
- **Duration**: 5 hours

### Noise Parameters Analyzed:
1. **Bias Instability (Pink Noise)**:
   - Measures drift over time at a constant temperature.
2. **Angle Random Walk Error (White Noise)**:
   - Drift due to noise when integrating angular rate signals.
3. **Rate Random Walk**:
   - Characterized by the red noise spectrum due to temperature fluctuations.
4. **Acceleration Dependency Error**:
   - Errors caused by varying accelerations.
5. **Sensor Non-Orthogonality Error**:
   - Errors due to non-perfect orthogonality of the gyroscopes and accelerometers.

### Analysis:
- **Allen Deviation Curve**: 
  - Used to extract noise parameters and differentiate slopes for various noise types.
  
- **Parameter Analysis**:
  - **Angular Velocity**: Assessed in X, Y, Z directions.
  - **Linear Acceleration**: Assessed in X, Y, Z directions.

### Simulation:
- **MATLAB Simulation**: 
  - Simulated error values (yellow dashed line) compared to actual data (blue line).
  - Differences attributed to the lack of temperature-related parameters in simulations.

### Key Findings:
- **N Values**:
  - Closest to datasheet values for Gyro data about Z and X axes.
- **B Values**:
  - Mostly consistent with datasheet requirements except for the Gyro about Y axis.

### Conclusion:
- Differences from datasheet values are attributed to non-ideal data collection conditions, unlike the ideal conditions in which the datasheet data was collected.
