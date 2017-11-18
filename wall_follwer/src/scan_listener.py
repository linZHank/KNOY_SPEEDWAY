#!/usr/bin/env python
import rospy
import tf
import numpy as np
import math
import serial

import std_msgs
from std_msgs.msg import Header, Empty, String, Float64
from sensor_msgs.msg import LaserScan

# Enable serial 
console_ser = serial.Serial(port = "/dev/ttyACM0", baudrate=115200)

global err_pre, benchmark, K_p, K_d # error from previous step
err_pre = 0
benchmark = 0.5
K_p = -100
K_d = -4.5

def scanCallback(data):
    global err_pre, benchmark, K_p, K_d
    
    # print(data.angle_min)
    # print(data.angle_max)
    # print(data.angle_increment)
    # print(len(data.ranges))
    theta = math.radians(60) # between 0~70 deg
    r_n30 = data.ranges[420] # -30 degrees
    r_n90 = data.ranges[180] # -90 degrees
    r_p90 = data.ranges[900] # +90 degrees
    alpha = math.atan2((r_n30*math.cos(theta)-r_n90), (r_n30*math.sin(theta))) # car orientation
    
    dist2wall = r_n90 * math.cos(alpha)
    print("distance to wall = ", dist2wall)
    print("range @ +90 degrees: ", r_p90, "range @ -90 degrees: ", r_n90)
    
    # PD control
    err = benchmark - dist2wall
    derr = err_pre - err 
    V_theta = K_p*err + K_d*derr # control
    err_pre = err
    print("Control theta: ", V_theta)
    
    if V_theta < 0:
	ser_str = "A"+"%05d" %(int(V_theta*2048./50))+"%04d" %(int(20))
    else:
	ser_str = "A+"+"%04d" %(int(V_theta*2048./50))+"%04d" %(int(20))
    
    console_ser.write(ser_str)
     

    #return R_n90
# Execution function, initiating, triggrt callback function & keeps running
def scanListerner():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('laser_scan_listener', anonymous=True)
    # subscribe to /scan topic
    rospy.Subscriber('scan', LaserScan, scanCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    scanListerner()
