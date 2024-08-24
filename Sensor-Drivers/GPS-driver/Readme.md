# GNSS Puck Data Collection and Analysis

## Objective

This project involves analyzing GNSS data collected using a GPS puck under two different scenarios:

<img src="https://qph.cf2.quoracdn.net/main-qimg-f554a5f5aca2948ba1d013695f8abadb" alt="Description of GIF" width="500" height="300">

A. **Stationary**: The GPS puck is held in a fixed position to check the stability and consistency of the position readings over time.

B. **Walking Straight for 100 Meters**: The GPS puck is carried while walking in a straight line for a recorded distance of approximately 100 meters.

All the analysis is conducted using the Matplotlib library in Python. The associated Jupyter notebook file is attached in the lab files.

## Analysis

### A. Stationary

In the stationary scenario, the GPS puck readings ideally should not fluctuate with time. The following key observations were made:

![image](https://github.com/user-attachments/assets/ea642d60-1e3d-4361-aa5f-2e1f68fc900e)

- **Plot Analysis**: Initial plots showed that the latitude, longitude, and altitude readings were fairly stable. However, minor variations were observed upon zooming into the data.
  
- **Statistical Summary**: A statistical analysis was performed to summarize the data and understand the distribution of errors:
  - **Northing Reading Error (Standard Deviation)**: 1.164 m
  - **Easting Reading Error (Standard Deviation)**: 1.116 m
  - **Altitude Error (Standard Deviation)**: 7.661 m
  
Since the skew values for all parameters are within the range (-1.5, 1.5), the errors are considered normally distributed. The standard deviation was used as the error estimate parameter, allowing us to compute confidence intervals for the data.

- **Altitude Analysis**: The altitude readings showed a sudden decrease after a certain time and then stabilized. This behavior was observed and plotted against time.

![image](https://github.com/user-attachments/assets/8174b4e8-9bbf-4869-bb50-e096f8608b8f)

### B. Moving

In the moving scenario, the puck was carried while walking straight for approximately 100 meters. The analysis focused on correlating Northing and Easting readings:

![image](https://github.com/user-attachments/assets/6967f36d-948a-4228-8997-51b176260dbd)

- **Plot Analysis**: The Northing and Easting readings were plotted, and an ideal straight line was overlaid on the data to represent the expected path. The root mean squared error (RMSE) between the data points and this line was calculated to estimate error:
  - **RMSE**: 3.545 m

- **Distance Estimation**: The distance traveled according to the GPS was compared to the actual distance traveled. The GPS estimated the distance to be 57.7% more than the actual distance, indicating an error in GPS distance measurement.

- **Altitude Analysis**: The altitude showed a decrease consistent with walking downhill, supporting the accuracy of the GPS readings during movement.

![image](https://github.com/user-attachments/assets/bf6e4be7-6ba7-42a3-b6b0-4341a9afbe3a)

## Sources of Error

Several sources of error were identified in both experiments:

1. **Experiment Conditions**: Environmental factors such as temperature (21°C) and wind, along with physical obstructions like trees and buildings, could have affected the readings.

2. **Human Error**: Minor movements of the puck during stationary readings and magnetic interference from nearby electronic devices (like a laptop) could introduce errors.

3. **Common GNSS Errors**:
   - **Ephemeris**: Errors in the satellite position.
   - **Multipath**: Interference caused by radio waves traveling different paths.
   - **Atmospheric Refraction**: Refraction of signals in the atmosphere.
   - **Clock Errors**: Timing errors in the satellite clocks.
   - **Satellite Position**: Inaccurate satellite positioning affecting the readings.

## Conclusion

This analysis highlights the importance of understanding and accounting for various sources of error when working with GNSS data. While stationary data provided a clearer understanding of the puck's accuracy, the moving data required more complex analysis to estimate and account for errors. Further improvements in data collection methods and error correction techniques could enhance the accuracy of such measurements.
