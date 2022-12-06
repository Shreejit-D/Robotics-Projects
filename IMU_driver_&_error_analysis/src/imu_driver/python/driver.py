import queue
import sys
import rospy
import sys
import serial
from math import *
from datetime import datetime
from imu_driver.msg import imu_msg

if __name__ == '__main__':
    port_imu = sys.argv[1]
    port = serial.Serial(port_imu,115200,timeout=1)
    port.write(b'$VNWRG,07,40')
    port.write(b'$VNWRG,06,14')

    rospy.init_node('driver',anonymous=True)
    imu_pub = rospy.Publisher('/imu',imu_msg, queue_size=10)

    imu_para = imu_msg()
    imu_para.Header.stamp = rospy.Time.now()
    imu_para.Header.frame_id = "IMU1_Frame"
    imu_para.Header.seq = 0

    while not rospy.is_shutdown():
        line = port.readline()

        try:
            if line.startswith(b'$VNYMR') or line.startswith(b'\r$VNYMR'):
                line_dec = line.decode("utf-8")
                imu_p_list = line_dec.split(',')
                if len(imu_p_list) == 13:
                    #All parameters 
                    yaw, roll, pitch, mag_x, mag_y, mag_z, acc_x, acc_y, acc_z, ang_x, ang_y = float(imu_p_list[1]), float(imu_p_list[2]), float(imu_p_list[3]), float(imu_p_list[4]), float(imu_p_list[5]), float(imu_p_list[6]), float(imu_p_list[7]), float(imu_p_list[8]), float(imu_p_list[9]), float(imu_p_list[10]), float(imu_p_list[11])
                    ang_z = float((imu_p_list[12].split('*'))[0])

                    #Quaternion conversion
                    cr = cos(radians(roll) * 0.5)
                    sr = sin(radians(roll) * 0.5)
                    cp = cos(radians(pitch) * 0.5)
                    sp = sin(radians(pitch) * 0.5)
                    cy = cos(radians(yaw) * 0.5)
                    sy = sin(radians(yaw) * 0.5)

                    qw = cr * cp * cy + sr * sp * sy
                    qx = sr * cp * cy - cr * sp * sy
                    qy = cr * sp * cy + sr * cp * sy
                    qz = cr * cp * sy - sr * sp * cy

                    #passing parameters
                    #Attitude
                    imu_para.IMU.orientation.x = float(qx)
                    imu_para.IMU.orientation.y = float(qy)
                    imu_para.IMU.orientation.z = float(qz)
                    imu_para.IMU.orientation.w = float(qw)
                    
                    #Magnetic field
                    imu_para.MagField.magnetic_field.x = float(mag_x)
                    imu_para.MagField.magnetic_field.y = float(mag_y)
                    imu_para.MagField.magnetic_field.z = float(mag_z)

                    #Acceleration 
                    imu_para.IMU.linear_acceleration.x = float(acc_x)
                    imu_para.IMU.linear_acceleration.y = float(acc_y)
                    imu_para.IMU.linear_acceleration.z = float(acc_z)

                    #Angular velocity
                    imu_para.IMU.angular_velocity.x = float(ang_x)
                    imu_para.IMU.angular_velocity.y = float(ang_y)
                    imu_para.IMU.angular_velocity.z = float(ang_z)

                    imu_para.raw_data = line_dec
            
                    imu_para.Header.seq += 1
                    imu_pub.publish(imu_para)

                    print(imu_para)
                    print()
                else:
                    print('13 elements not found')

        except rospy.ROSInterruptException:
            port.close()
            









