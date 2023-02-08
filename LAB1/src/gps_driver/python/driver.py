#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import utm
import sys
from gps_driver.msg import gps_msg
from gps_driver.msg import *

def driver():
    
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('gps', gps_msg, queue_size=10)
    
    args = rospy.myargv(argv = sys.argv)
    if len(args) != 2:
        print("error")
        sys.exit(1)

    connected_port = args[1]
    
    serial_port = rospy.get_param('~port', connected_port)
    serial_baud = rospy.get_param('~baudrate', 4800)
    
    port = serial.Serial(serial_port, serial_baud, timeout = 3)
    
    rospy.loginfo("Publishing parsed GPS data")

    while not rospy.is_shutdown():  
        	
        data = str(port.readline())
        data = data.lstrip("b'\\r$")
        data = data.lstrip("b'$")
        data = data.rstrip("\\n'")
        if data == '':
            rospy.logwarn("No GPS data found")
        else:
            recievedData = data.split(',')
            if recievedData[0] == "GPGGA":
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

                utc_data = recievedData[1]
                utc_data = utc_data[:2]+ " Hours " + utc_data[2:4] + " Minutes " + utc_data[4:] + " Seconds"
                utm_lat_long = utm.from_latlon(
                    latitude_converted, longitude_converted)
            
                    
                msg = gps_msg()
                msg.Header.stamp.secs = int(total_utc_secs)
                msg.Header.stamp.nsecs = int(str(total_utc_nsecs)[:5])
                #print(f'UTM_East, UTM_north, Zone, Letter: {utm_lat_long}')
                msg.Header.frame_id = "GPS1_Frame"
                msg.Latitude = latitude_converted
                msg.Longitude = longitude_converted
                msg.HDOP = hdop
                msg.Altitude = altitude
                msg.UTM_easting = utm_lat_long[0]
                msg.UTM_northing = utm_lat_long[1]
                msg.UTC = utc_data
                msg.Zone = utm_lat_long[2]
                msg.Letter = utm_lat_long[3]
                rospy.loginfo(msg)
                pub.publish(msg)
   

if __name__ == '__main__':

    try:
        driver()
    except rospy.ROSInterruptException:
        pass