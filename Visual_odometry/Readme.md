# Visual Odometry Using KITTI Dataset

This project focuses on performing visual odometry using the KITTI dataset. The goal is to calculate vehicle poses using stereo matching techniques, compare these poses with ground truth data, and estimate errors. Additionally, LIDAR corrections are integrated into the dataset to improve accuracy.

## Introduction

This project aims to perform visual odometry using the KITTI dataset by calculating vehicle poses through stereo matching and comparing these with ground truth data to estimate errors. The KITTI dataset was collected by Karlsruhe Institute of Technology using the autonomous driving platform Annieway, which includes stereo cameras (grayscale and RGB), Velodyne LIDAR, and an IMU/GPS pair.

The data was collected in Karlsruhe, Germany, and is divided into 22 sequences. For this project, the analysis is focused on the '02' dataset, which includes ground truth data.

## Coordinate Frame and Projection Matrices

To integrate the data from different sensors, all sensor readings must be transposed to a global coordinate frame, which is defined by the left grayscale camera in this project. This transformation is performed using projection matrices provided in the KITTI dataset.

![image](https://github.com/user-attachments/assets/d1f46098-3e1e-4635-9118-9f1967398921)

## The Approach

The visual odometry process involves calculating the changes in vehicle pose by analyzing the changes in feature positions between frames. Depth information is crucial for this process and is obtained through disparity matching using the SGBM algorithm. Key steps include:

1. **Depth Calculation**: Using SGBM to calculate disparity and infer depth.
2. **Feature Detection**: Utilizing the SIFT algorithm to identify scale-invariant features.
3. **Pose Estimation**: Estimating vehicle pose changes by analyzing the transformations of detected features between consecutive frames.

### SGBM (Semi Global Block Matching)

The SGBM algorithm is used to compute the disparity between left and right stereo camera images. This disparity is essential for depth estimation, which is a critical step in visual odometry. SGBM is an intensity-based approach that generates dense and smooth disparity maps, suitable for 3D reconstruction.

![image](https://github.com/user-attachments/assets/7036fb59-310c-430e-bc5e-d2ee3fa1ca71)

### SIFT (Scale Invariant Feature Transform)

SIFT is used for identifying and describing local features in images, which is crucial for matching features across frames in visual odometry. SIFT is robust to changes in image scale, rotation, and lighting conditions, making it ideal for detecting features in stereo images.

![image](https://github.com/user-attachments/assets/5be92051-8f6a-4eee-8f0a-86e97bcbc177)

## Results

### Section 1: Individual Frame Analysis

In this section, individual algorithms like SGBM and SIFT are applied to consecutive frames to evaluate their performance. Disparity is calculated between the first pair of stereo images, and depth is computed using stereo matching. The results are then compared with LIDAR data for visual error estimation.

![image](https://github.com/user-attachments/assets/5c99d35f-61d5-4f34-8e5b-156deafe3eb9)

### Section 2: Visual Odometry Fused with LIDAR

This section presents the results of pose estimation using two approaches:

1. **Pose Estimation without LIDAR Correction**: Pose is estimated using only stereo matching data.
2. **Pose Estimation with LIDAR Correction**: LIDAR data is integrated to refine pose estimates.

![image](https://github.com/user-attachments/assets/33d2d022-9776-4626-8c48-0ab4999b7440)

The integration of LIDAR data significantly improves pose estimation accuracy, though the error (~60 meters) indicates the need for more advanced filtering techniques such as Particle Filtering or Extended Kalman Filtering for real-world applications.

![Pose Estimation Comparison](path_to_pose_estimation_comparison_image)

## Conclusion

This project demonstrates a preliminary approach to visual odometry using classical methods of stereo matching, with LIDAR data used for corrections. The results indicate that while the approach provides a good estimate of vehicle localization, further improvements are needed for real-world applicability. Advanced filtering techniques and loop closure implementations could enhance the accuracy and reliability of the visual odometry system.
