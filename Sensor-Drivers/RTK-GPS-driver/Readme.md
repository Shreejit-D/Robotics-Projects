# RTK Rover Position Analysis using Telemetry Radio Communications

## Project Overview

This project aims to analyze the positional data of a rover RTK using telemetry radio communications with a base RTK. The experiment was conducted in two different environments to assess the impact of obstructions and movement on the accuracy of the readings.

### Experiment Locations:
1. **ISEC Top Roof Parking**: Minimal obstructions, providing a clear line of sight.
2. **Science Quad (in front of Hurtig Hall)**: Surrounded by buildings and trees, introducing potential obstructions and reflections.

### Data Collection Methods:
1. **Stationary**: The rover and base RTK were fixed at their positions for 10 minutes to observe time-based variations.
2. **Moving**: The rover RTK was moved in a rectangular pattern while the base RTK remained stationary, to observe deflections in the readings during movement.

---

## Open Space Reading

### Stationary Data

#### Statistical Analysis:
- **Objective**: To detect any deflections in easting, northing, or altitude readings over time.
- **Observations**:
  - No significant fluctuations in UTM northing and altitude.
  - Minor fluctuations detected in UTM easting.

#### Error Analysis:
- **Possible Causes**:
  - **Precision Limitations**: The use of `FLOAT32` may introduce errors due to limited significant digits.
  - **UTM Conversion**: Numerical computation methods used in the UTM library may have introduced small variations.

#### Conclusion:
- **Easting Error**: Mean error of ~0.00574 m (0.574 cm) and a range of 0.03125 m (3.125 cm).
- **Northing and Altitude**: No significant fluctuations observed.

### Moving Data

#### Analysis:
- **Path Analysis**: Plotted northing vs easting to visualize the rectangular walking pattern.
- **Error Calculation**: RMSE calculated by comparing actual data points to ideal straight lines.

#### Conclusion:
- **Positional Error**: Mean RMSE of 0.1945 m (19.45 cm) for northing and easting.
- **Altitude Error**: RMSE of 0.36 m (36 cm).

#### Error Sources:
- **General Errors**: Ionosphere density variations, atmospheric refraction, clock errors.
- **Experiment-specific Errors**:
  - **Multipath Reflection**: Potential reflections from steel frames in the vicinity.
  - **Human Error**: Assumptions of straight-line movement and flat terrain.

---

## Obstructed Space Reading

### Stationary Data

#### Statistical Analysis:
- **Objective**: To detect any deflections in readings over time in a more obstructed environment.
- **Observations**:
  - Altitude readings showed variations with a standard deviation of ~5 cm and a range of 30 cm.
  - Easting readings exhibited fluctuations over time.

#### Conclusion:
- **Easting Error**: Mean error of ~0.06462 m (6.46 cm), approximately 52% higher than in the open environment.
- **Altitude Error**: Observed random distribution with a significant range of variation.

### Moving Data

#### Analysis:
- **Path Analysis**: Similar method as in open space, plotting northing vs easting.
- **Error Calculation**: RMSE calculated based on deviations from ideal straight-line paths.

#### Conclusion:
- **Positional Error**: RMSE of 0.4303 m (43.03 cm) for northing and easting, 54.8% higher than in open space.
- **Altitude Error**: RMSE of 1.22 m.

#### Error Sources:
- **General Errors**: Similar to those identified in open space, with additional errors from obstructions.
- **Experiment-specific Errors**:
  - **Multipath Reflection**: Significant in a quad surrounded by buildings and trees.
  - **Human Error**: Assumptions of straight-line movement and flat terrain.

---

## How to Use This Repository

1. **Clone the repository**:
2. **Analyze the Data: Use the provided scripts to reproduce the analysis and visualize the data.**
3. **Explore the Results: Review the plots and statistical summaries to understand the impact of environment and movement on RTK readings.**
