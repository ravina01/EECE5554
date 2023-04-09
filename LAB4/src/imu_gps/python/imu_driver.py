#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import numpy as np
import sys

from imu_driver.msg import Vectornav
from imu_driver.srv import convert_to_quaternion

def convert_to_quaternion_client(roll, pitch, yaw):
    rospy.wait_for_service("convert_to_quaternion")
    #try:
        #method to convert euler to quaternion
    conversion = rospy.ServiceProxy('convert_to_quaternion', convert_to_quaternion)
    response = conversion(roll, pitch, yaw)
    return (response.qx, response.qy, response.qz, response.qw)
    #except rospy.ServiceException as e:
        #print("Service Exception: %s" % e)

def imu_driver():
    pub = rospy.Publisher('imu', Vectornav, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    #rate = rospy.Rate(40)
    msg = Vectornav()

    args = rospy.myargv(argv = sys.argv)
    if len(args) != 2:
        print("error")
        sys.exit(1)

    connected_port = args[1]
    #connected_port = "/dev/ttyUSB3"
    serial_port = rospy.get_param('~port',connected_port)
    serial_baud = rospy.get_param('~baudrate',115200)

    ser = serial.Serial(serial_port, serial_baud, timeout = 3)
    #ser.write('$VNRRG,07,40*xx'.encode())
    #ser.write('$VWRRG,06,14*xx'.encode())
    ser.write(b"$VNWRG,07,40*xx")

    while not rospy.is_shutdown():
        recieve = ser.readline()
        #.decode('utf-8')
        recieve = str(recieve)
        #recieve = recieve.decode('utf-8')

        if "$VNYMR" in str(recieve):
            data = str(recieve).split(",")
            print(data)

            now = rospy.get_rostime()
            rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
            #now = datetime.now()
            #current_time = now.strftime("%H:%M:%S")
            #print("Current Time =", current_time)

            yaw = float(data[1])
            pitch = float(data[2])
            roll = float(data[3])
            magX = float(data[4])
            magY = float(data[5])
            magZ = float(data[6])
            accX = float(data[7])
            accY = float(data[8])
            accZ = float(data[9])
            gyroX = float(data[10])
            gyroY = float(data[11])
            gyroZ = float(data[12][0:9])

            
            #write a service instead to avoid code redunducy 
            #def orientation(roll, pitch, yaw):   #Convert an Euler angle to a quaternion.
            # qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            # qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            # qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            # qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            #return [qx, qy, qz, qw]

            # get_quaternion = rospy.ServiceProxy(service_name, GetQuaternion)
            # req = GetQuaternionRequest()
            # req.roll = roll  # Example input values
            # req.pitch = pitch0.
            
            # req.yaw = yaw
            #res = get_quaternion(req)
            (qx, qy, qz, qw) = convert_to_quaternion_client(roll, pitch, yaw)
            #print("service done")

            #msg.header.stamp = rospy.Time.from_sec(now)
            msg.header.stamp.secs = int(now.secs)
            msg.header.stamp.nsecs = int(now.nsecs)
            msg.header.frame_id = 'imu1_Frame'
            msg.imu.orientation.x = qx
            msg.imu.orientation.y = qy
            msg.imu.orientation.z = qz
            msg.imu.orientation.w = qw
            msg.imu.linear_acceleration.x = accX
            msg.imu.linear_acceleration.y = accY
            msg.imu.linear_acceleration.z = accZ
            msg.imu.angular_velocity.x = gyroX
            msg.imu.angular_velocity.y = gyroY
            msg.imu.angular_velocity.z = gyroZ
            msg.mag_field.magnetic_field.x = magX
            msg.mag_field.magnetic_field.y = magY
            msg.mag_field.magnetic_field.z = magZ

            pub.publish(msg)
            rospy.loginfo(msg)
            #rate.sleep()


if __name__ == '__main__':
    try:
        imu_driver()
    except rospy.ROSInterruptException:
        pass
