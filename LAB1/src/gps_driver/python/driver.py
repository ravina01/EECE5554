#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
from std_msgs.msg import *
from gps_driver.msg import gps_msg
import utm
import argparse
import sys


def driver():

    args = rospy.myargv(argv=sys.argv)
    if len(args) != 2:
        print(error)
        sys.exit(1)

    serial_port_arg = args[1]

    SENSOR_NAME = "gps"
    rospy.init_node('talker')

    # port = rospy.get_param('~port','/dev/pts/2') # hardcode serial port
    port = rospy.get_param('~port', serial_port_arg)
    baud = rospy.get_param('~baudrate', 4800)

    port = serial.Serial(port, baud, timeout=3.)
    rospy.logdebug("Using GPS sensor on port " +
                   serial_port_arg+" at "+str(baud))

    rospy.sleep(0.2)  # 5Hz sampling rate

    rospy.loginfo("Publishing GPS")

    pub = rospy.Publisher('/gps', gps_msg, queue_size=5)

    while not rospy.is_shutdown():
        data = str(port.readline())
        recievedData = str(data).split(',')
        if recievedData[0] == "b'$GPGGA":
            utc = float(recievedData[1])
            utc_hours = utc // 10000
            utc_minutes = (utc-(utc_hours*10000))//100
            utc_seconds = (utc - (utc_hours*10000) - (utc_minutes*100))
            total_utc_secs = (utc_hours*3600 + utc_minutes*60 + utc_seconds)
            total_utc_nsecs = int((total_utc_secs * (10**7)))

            latitude = float(recievedData[2])
            latitude_direction = recievedData[3]
            latitude_correction = 1
            if latitude_direction == "S":
                latitude_correction = -1

            latitude_degree = int(latitude/100)
            latitude_minutes = float(latitude) - (latitude_degree * 100)
            latitude_converted = latitude_correction * \
                float(latitude_degree + latitude_minutes/60)

            longitude = float(recievedData[4])
            longitude_direction = recievedData[5]
            longitude_correction = 1
            if longitude_direction == "W":
                longitude_correction = -1

            longitude_degree = int(longitude / 100)
            longitude_minutes = float(longitude) - (longitude_degree * 100)
            longitude_converted = longitude_correction * \
                float(longitude_degree + longitude_minutes/60)

            hdop = float(recievedData[8])
            altitude = float(recievedData[9])

            utm_lat_long = utm.from_latlon(
                latitude_converted, longitude_converted)

            msg = gps_msg()
            msg.Header.stamp.secs = int(total_utc_secs)
            msg.Header.stamp.nsecs = int(str(total_utc_nsecs)[:5])
            #print(f'UTM_East, UTM_north, Zone, Letter: {utm_lat_long}')
            msg.Header.frame_id = "GPS1_Frame"
            msg.Latitude = latitude_converted
            msg.Longitude = longitude_converted
            msg.Hdop = hdop
            msg.Altitude = altitude
            msg.UTM_easting = utm_lat_long[0]
            msg.UTM_northing = utm_lat_long[1]
            msg.Zone = utm_lat_long[2]
            msg.Letter = utm_lat_long[3]
            rospy.loginfo(msg)
            pub.publish(msg)

if __name__ == '__main__':
    try:
        driver()
        a = 1
    except rospy.ROSInterruptException:
        pass
