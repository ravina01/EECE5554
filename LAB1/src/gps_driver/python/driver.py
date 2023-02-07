#!/usr/bin/env

import rospy
import serial
import utm
import sys
from gps_driver.msg import gps_msg
from gps_driver.msg import *

msg = gps_msg()
def driver():
    
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('gps', gps_msg, queue_size=10)
    
    args = rospy.myargv(argv = sys.argv)
    if len(args) != 2:
        print("error")
        sys.exit(1)

    connected_port = args[1]
    
    serial_port = rospy.get_param('~port', connected_port)
    #serial_port = rospy.get_param('~port', '/dev/pts/4')
    serial_baud = rospy.get_param('~baudrate', 4800)
    #rate = rospy.Rate(10)
    
    
    serialData = serial.Serial(serial_port, serial_baud, timeout = 3)
    
    while not rospy.is_shutdown():  
        	
        data = str(serialData.readline())
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
	      
	    
            msg.header.stamp.secs = int(total_utc_secs)
            msg.header.stamp.nsecs = int(str(total_utc_nsecs)[:5])
            #print(f'UTM_East, UTM_north, Zone, Letter: {utm_lat_long}')
            msg.header.frame_id = "GPS1_Frame"
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
            #rate.sleep()
	    

if __name__ == '__main__':

    try:
        driver()
    except rospy.ROSInterruptException:
        pass
