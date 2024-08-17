#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import time
import serial
import sys
import datetime
import utm
from math import sin, pi
from std_msgs.msg import Float32
from gps_rtk_driver.msg import gps_rtk_msg                                          

def gps_func(port_no):
    gps_publisher = rospy.Publisher('/gps',gps_rtk_msg, queue_size=10)          #created topic 'gps' with msg 'gps_msg' and publisher 'gps_publisher'
    rospy.init_node('gps_rtk_driver',anonymous=True)                            #initialized the node gps_rtk_driver

    port = serial.Serial(port_no, 57600, timeout=1.0)                        #setting baud_rate, timeout 

    gps_para = gps_rtk_msg()
    gps_para.Header.frame_id = 'GPS1_Frame'
    gps_para.Header.seq = 0

    while not rospy.is_shutdown():
        line = port.readline()

        if line == '':                                                      
            rospy.logerr('No output')

        else:
            if line.startswith(b'$GNGGA') or line.startswith(b'/r$GNGGA'):
                list_P = line.split(b',')                                    #Making list of our parameters
                print(list_P, len(list_P))

                if len(list_P) == 15:
                    time_s, latitude, latitude_dir, longitude, longitude_dir, altitude = list_P[1], list_P[2], list_P[3], list_P[4], list_P[5], float(list_P[9])
                    longitude = float(longitude[0:3]) + (float(longitude[3:])/60)
                    latitude = float(latitude[0:2]) + (float(latitude[2:])/60)
                    quality = int(list_P[6])
                    
                    latitude_dir = latitude_dir.decode("utf-8")
                    longitude_dir = longitude_dir.decode("utf-8")
                    
                    if latitude_dir == 'S':
                        latitude = -1 * latitude
                    if longitude_dir == 'W':
                        longitude = -1 * longitude                        
                    
                    today = datetime.date.today()
                    year, month, dt, hr, min, sec, decs = int(today.year), int(today.month), int(today.day), int(time_s[0:2]), int(time_s[2:4]), int(time_s[4:6]), int(time_s[7:9]) * (10 ** 6) * 0.864 / 100
                    decs = int(decs)
                    
                    epoch_time = datetime.datetime(year, month, dt, hr, min, sec, decs).timestamp()
                    gps_para.Header.stamp = rospy.Time(epoch_time)
                    gps_para.Header.seq += 1

                    utm_data = utm.from_latlon(latitude,longitude)          #Function returning converted data
                    gps_para.Latitude = latitude
                    gps_para.Longitude = longitude
                    gps_para.Altitude = altitude
                    gps_para.UTM_easting = utm_data[0]
                    gps_para.UTM_northing = utm_data[1]
                    gps_para.Zone = utm_data[2]
                    gps_para.Letter = utm_data[3]
                    gps_para.Quality = quality

                    print(gps_para)
                    print()

                    gps_publisher.publish(gps_para)

                else:
                    print('all required elements not found')

if __name__=='__main__':
    try:
        gps_func(sys.argv[1])               #function where we are passing the serial port id
    except rospy.ROSInterruptException:
        pass
