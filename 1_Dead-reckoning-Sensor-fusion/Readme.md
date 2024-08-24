# Sensor Fusion and Dead Reckoning

This project focuses on implementing sensor fusion and performing dead reckoning using IMU sensor data. The lab is divided into two main parts:

## Part I: Yaw Angle Calculation Using IMU Data

### A. Magnetometer Data Calibration

In this part, the yaw angle is calculated using the magnetometer and gyroscope readings from an IMU. The magnetometer data was calibrated by rotating the IMU sensor in a controlled environment to account for errors such as hard iron and soft iron distortions. The key steps include:

![image](https://github.com/user-attachments/assets/d3fd2cf0-f110-4968-b201-7ebf54f56b27)

- **Data Collection**: The IMU was rotated in a circle to gather magnetic field data.
- **Error Identification**: Two main distortion factors were identified: hard iron and soft iron.
- **Data Correction**: Calibration was performed by shifting and reshaping the magnetometer readings to correct the distortions.

Refer to the Jupyter notebook for detailed calculations.

### B. Yaw Angle Estimation

Yaw angles were calculated using both the magnetometer and gyroscope data:

1. **Magnetometer Yaw Calculation**: Yaw was calculated by finding the angle between the corrected magnetic flux in the X and Y directions.
2. **Gyroscope Yaw Calculation**: Yaw was derived by integrating the angular velocity in the Z direction.

The complementary filter was then applied to merge these readings to get a more accurate yaw estimate.

![image](https://github.com/user-attachments/assets/683d7217-b411-437a-a4f7-87c22b0b8350)

## Part II: Dead Reckoning and GPS Comparison

### A. Complementary Filter Implementation

The complementary filter was designed to combine the strengths of both the magnetometer and gyroscope:

![image](https://github.com/user-attachments/assets/e4c8c412-6d8a-47b4-9502-a4d7ec4f6ed8)

- **Magnetometer**: Good for long-term readings but prone to high-frequency fluctuations.
- **Gyroscope**: Good for short-term readings but subject to drift over time.

By filtering the yaw data through both high-pass and low-pass filters, a more accurate yaw angle was obtained.

### B. Forward Velocity Estimation

Forward velocity was estimated by integrating the forward acceleration from the IMU, after removing the gravity component. The velocity data was further processed to remove bias and correct for errors.

### C. Dead Reckoning

![image](https://github.com/user-attachments/assets/92db855e-5b98-4a0e-9086-fb87bfd21d4e)

Dead reckoning was performed by integrating the corrected velocity and angular velocity to calculate the distance covered and the path traversed by the IMU. This path was then compared with GPS data:

- **Mapping**: The IMU-generated map was scaled, rotated, and aligned with the GPS data to evaluate accuracy.
- **Error Analysis**: Discrepancies between the IMU and GPS data were identified and analyzed.

## Conclusion

The lab successfully demonstrates sensor fusion and dead reckoning using IMU data, with a focus on yaw angle estimation and forward velocity calculation. While the IMU data provided a good estimate of the path traversed, several factors, including sensor biases and errors introduced during integration, affected the accuracy of the dead reckoning results.

For detailed plots, calculations, and code, please refer to the Jupyter notebook included in this repository.

---
