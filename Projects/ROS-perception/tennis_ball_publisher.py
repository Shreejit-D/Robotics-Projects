#!/usr/bin/env python

import rospy
import cv2
# from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()

def main(args):
    rospy.init_node('image_publisher', anonymous=True)
    image_pub = rospy.Publisher("tennis_ball_image",Image, queue_size=10)
    video_capture = cv2.VideoCapture('/home/shreejit/catkin_ws2/src/ros_essentials_cpp/src/topic03_perception/video/tennis-ball-video.mp4')
    i = 0
    while not rospy.is_shutdown():
        i += 1
        ret, frame = video_capture.read()
        try:
            image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
            print('frame ' + str(i) + ' is published\n')
        except CvBridgeError as e:
            print(e)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    main(sys.argv)