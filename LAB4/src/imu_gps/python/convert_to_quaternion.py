#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
from imu_driver.srv import convert_to_quaternion, convert_to_quaternionResponse 
import rospy
import numpy as np

def conversion(roll, pitch, yaw):
    """converts euler(inputs) to quaternion(outputs)"""
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)    
    return (qx, qy, qz, qw)
    
def handle_convert_to_quaternion(req):
    (qx, qy, qz, qw) = conversion(req.roll, req.pitch, req.yaw)
    
    return convert_to_quaternionResponse(qx, qy, qz, qw)

def convert_to_quaternion_server():
    rospy.init_node('convert_to_quaternion_server')
    s = rospy.Service('convert_to_quaternion', convert_to_quaternion, handle_convert_to_quaternion)
    rospy.spin()
 
if __name__ == "__main__":
    convert_to_quaternion_server()
