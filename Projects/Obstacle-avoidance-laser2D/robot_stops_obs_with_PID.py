#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import Twist
import time


min_distance = 9999
min_dist_l = 9999
min_dist_r = 9999

def scan_callback(scan_data):
    global min_distance
    global min_dist_l
    global min_dist_r

    print('length of range is ' + str(len(scan_data.ranges)))
    
    #Find minimum range
    min_value, min_index = min_range_index(scan_data.ranges)
    print ("\nthe minimum range value is: ", min_value)
    print ("the minimum range index is: ", min_index)

    min_distance, min_dist_l, min_dist_r = min_dist_in_view(scan_data.ranges,40)
    
    print ('\nthe minimum distance in fov is:', min_distance)

    print ('\nthe minimum distance in left', min_dist_l)

    print ('\nthe minimum distance in right', min_dist_r)
 
    # min_distance = min_dist_in_view(scan_data.ranges,40)
    # print ('\nthe minimum distance in fov is:', min_distance)
    
    max_value, max_index = max_range_index(scan_data.ranges)
    print ("\nthe maximum range value is: ", max_value)
    print ("the maximum range index is: ", max_index)

    average_value = average_range(scan_data.ranges)
    print ("\nthe average range value is: ", average_value)

    average2 = average_between_indices(scan_data.ranges, 1, 7)
    print ("\nthe average between 2 indices is: ", average2)

    print ("the field of view: ", field_of_view(scan_data))

def field_of_view(scan_data):
    return (scan_data.angle_max-scan_data.angle_min)*180.0/3.14

#function for finding minimum distance for given field of view
def min_dist_in_view(ranges, angle):
    ranges_r = ranges[0:angle-1]
    ranges_l = ranges[361-angle:359]
    ranges = ranges_l + ranges_r
    ranges = [x for x in ranges if not math.isnan(x)]
    ranges_l = [x for x in ranges_l if not math.isnan(x)]
    ranges_r = [x for x in ranges_r if not math.isnan(x)]
    if len(ranges) != 0:
        ran = (min(ranges))
        f = 1
    if len(ranges_l) != 0:
        ran_l = (min(ranges_l))
        f = 1
    if len(ranges_r) != 0:
        ran_r = (min(ranges_r))
        f = 1
    else:
        f = 0
    if f == 1:
        return (ran,ran_l,ran_r)
    else:
        return (0.1,0.1,0.1)
    

#find the max range and its index
def min_range_index(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    if (len(ranges)!=0):
        return (min(ranges), ranges.index(min(ranges)) )
    else:
        return 0.1

#find the max range 
def max_range_index(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    if (len(ranges)!=0):
        return (max(ranges), ranges.index(max(ranges)))
    else:
        return 4.0

#find the average range
def average_range(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    if (len(ranges)!=0):
        return ( sum(ranges) / float(len(ranges)) )
    else:
        return 0.0

def average_between_indices(ranges, i, j):
    ranges = [x for x in ranges if not math.isnan(x)]
    if (len(ranges)!=0): 
        slice_of_array = ranges[i: j+1]
        return ( sum(slice_of_array) / float(len(slice_of_array)) )
    else:
        return 0.0


def rotate(avoid_obstacle):
    global min_distance
    global min_dist_l
    global min_dist_r

    velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    cmd_vel = Twist()
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = 0.6
    loop_rate = rospy.Rate(10)
    if (avoid_obstacle):
        while (min_distance<0.8):
            velocity_publisher.publish(cmd_vel)
            loop_rate.sleep()

        #force the robot to step before avoiding obstacles
        cmd_vel.angular.z = 0.0
        velocity_publisher.publish(cmd_vel)
    else:
        velocity_publisher.publish(cmd_vel)
        loop_rate.sleep() 

def move(avoid_obstacle):
    #create a publisher to make the robot move
    global min_distance
    global min_dist_l
    global min_dist_r

    velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    cmd_vel = Twist()
    
    loop_rate = rospy.Rate(10)
    
    least_dist = 0.6

    while(True):
        if (avoid_obstacle):
            #Implementing proportional speed control
            # while (min_distance > least_dist):
            while(True):
                speed = min_distance-least_dist
                if speed > 0.26:
                    speed = 0.26
                if speed < 0:
                    speed = 0
                
                cmd_vel.linear.x = speed
                
                if (min_dist_r > min_dist_l):   #Obstacle in left
                    rad_vel = 1.8*(min_dist_r - least_dist)
                    if rad_vel > 1.8:
                        rad_vel = 1.8
                else:
                    rad_vel = -1.8*(min_dist_l - least_dist)
                    if rad_vel < -1.8:
                        rad_vel = -1.8
                cmd_vel.angular.z =rad_vel
                
                print('\n//Speed is '+str(speed)+'\n//angular speed is '+str(rad_vel))
                
                velocity_publisher.publish(cmd_vel)
                loop_rate.sleep()

            #behavior 2: force the robot rotate until it finds open space
            # cmd_vel.linear.x = 0.0
            # cmd_vel.angular.z = 1.82
            # while (min_distance < least_dist):
            #     velocity_publisher.publish(cmd_vel)
            #     loop_rate.sleep()

        else:
            velocity_publisher.publish(cmd_vel)
            loop_rate.sleep() 

if __name__ == '__main__':
    
    #init new a node and give it a name
    rospy.init_node('scan_node', anonymous=True)
    
    #subscribe to the topic /scan. 
    rospy.Subscriber("scan", LaserScan, scan_callback)

    time.sleep(2)
    move(True)
    
    rotate(True)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()