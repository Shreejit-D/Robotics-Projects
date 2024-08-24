# Tennis Ball Detection using ROS and OpenCV

### Author: Shreejit Gajanan Deshmukh  
**Project**: Tennis Ball Detection using ROS and OpenCV  
**NUID**: 002767657  

---

## Project Overview

This project demonstrates the detection and tracking of a tennis ball in a video stream using ROS (Robot Operating System) and OpenCV. The project consists of two main scripts: one for publishing the video frames as ROS messages and another for processing these frames to detect and track the tennis ball.

![Ball_tracking_CV](https://github.com/user-attachments/assets/8249fa4d-95e2-4e4e-b9ef-999fbcb29be6)

### Components:
1. **Image Publisher**: Publishes video frames as ROS `Image` messages.
2. **Image Processor**: Subscribes to the ROS `Image` messages, processes the frames to detect the tennis ball, and visualizes the detection results.

---

## Project Structure

- **tennis_ball_publisher.py**: This script captures video frames from a specified video file and publishes them as ROS `Image` messages on the `tennis_ball_image` topic.

- **tennis_ball_listener.py**: This script subscribes to the `tennis_ball_image` topic, processes each frame to detect the tennis ball, and displays the detection results.

---

## How It Works

### 1. Image Publisher (`tennis_ball_publisher.py`):
- **Functionality**:
  - Initializes a ROS node named `tennis_ball_publisher`.
  - Captures frames from a video file using OpenCV.
  - Converts each frame to a ROS `Image` message using `CvBridge`.
  - Publishes the ROS `Image` messages to the `tennis_ball_image` topic.

- **Usage**:
  - The video file path is hardcoded in the script (`/home/shreejit/catkin_ws2/src/ros_essentials_cpp/src/topic03_perception/video/tennis-ball-video.mp4`).
  - The script will publish each frame until the video ends or until the script is interrupted.

### 2. Image Processor (`tennis_ball_listener.py`):
- **Functionality**:
  - Initializes a ROS node named `image_converter`.
  - Subscribes to the `tennis_ball_image` topic to receive video frames.
  - Converts the ROS `Image` messages back to OpenCV format using `CvBridge`.
  - Applies color filtering to isolate the yellow color of the tennis ball in the frame.
  - Detects contours in the filtered binary image to identify the tennis ball.
  - Draws the detected contours and marks the center of the tennis ball on the original frame.
  - Displays the processed frames with the detected tennis ball highlighted.

- **Usage**:
  - The script will continuously process and display the frames as long as it receives messages from the `tennis_ball_image` topic.

---

## Running the Project

### Prerequisites:
- ROS installed and configured.
- Python with OpenCV and ROS Python libraries (`rospy`, `cv_bridge`, etc.).
- A video file containing footage of a tennis ball (path specified in `tennis_ball_publisher.py`).

### Steps to Run:

1. **Launch the ROS Core**
2. **Run the Image Publisher**
3. **Run the Image listener**

 ## Conclusion
 This project effectively demonstrates how to use ROS and OpenCV together for real-time image processing and object detection. The techniques used here can be extended to other color-based object detection tasks in robotic applications.
