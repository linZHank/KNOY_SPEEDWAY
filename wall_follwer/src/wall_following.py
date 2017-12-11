#!/usr/bin/env python
from __future__ import print_function

import rospy
import tf
import numpy as np
import math
import serial

import std_msgs
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

# Enable serial 
console_ser = serial.Serial(port = "/dev/ttyACM0", baudrate=115200)

# Static parameters
BASE_LINE = 0.5 # distance to right wall, unit: m
KP = 100 # PID: K_p
KD = 4.5  # PID: k_d
PUBLISH_RATE = 150.0 # number of control commands to publish per second


class WallFollower():
    def __init__(self):
        self._ser_cmd_pub = rospy.Publisher("/serial_command", String, queue_size =1)
        self._sub = rospy.Subscriber("/scan", LaserScan, self.lidarCB, queue_size=1)


        # containers for laser scanner related data
        self.r_n30 = 0 # lidar range @ -30 degrees
        self.r_n90 = 0 # lidar range @-90 degrees
        self.r_p90 = 0 # lidar range @+90 degrees
        self._theta = math.radians(60) # between 0~70 degrees, used to compute car orientation
	self.dist2wall = 0.5
	self.V_theta = 0
	self.ser_str = "A+0000+0000"
	self.tmr = 0

        # containers for control related data
        self.err = 0
        self.err_pre = 0
	self.cmd = String()

    # Compute PID control according to tilted angle
    def lidarCB(self, data):
        # Compute car orientation
        self.r_n30 = data.ranges[420] 
        self.r_n90 = data.ranges[180] 
        self.r_p90 = data.ranges[900] 
        self.alpha = math.atan2((self.r_n30*math.cos(self._theta)-self.r_n90), \
                                (self.r_n30*math.sin(self._theta))) # car orientation
        self.dist2wall = self.r_n90 * math.cos(self.alpha) # distance to wall


        # PD control
        self.err = self.dist2wall - BASE_LINE 
        self.derr = self.err_pre - self.err 
        self.V_theta = KP*self.err + KD*self.derr # control
        self.err_pre = self.err
        print("Control theta: ", self.V_theta)

        if self.V_theta < 0:
            self.ser_str = "A"+"%05d" %(self.V_theta*2048/50)+"+0200"
        else:
            self.ser_str = "A+"+"%04d" %(self.V_theta*2048/50)+"+0200"


    def pub_ser_cmd(self):
        ''' Perform wall following '''
        rate = rospy.Rate(PUBLISH_RATE)

        print("Publishing serial command @ 150 Hz. Press Ctrl-C to stop...")
        while not rospy.is_shutdown():
            self._ser_cmd_pub.publish(self.ser_str)
            rospy.loginfo("serial command published %s", self.ser_str)
	    console_ser.write(self.ser_str)
	    print(self.tmr)
            rate.sleep()
	    self.tmr += 1
	
	# execute cmd

    # Callback function execute serial-port command 
    #def cmdCB(self, cmd):
     #   rospy.loginfo("serial command received %s", cmd.data)
      #  console_ser.write(cmd.data)

    #def sub_ser_cmd(self):
    #	self._ser_cmd_sub = rospy.Subscriber("/serial_command", String, cmdCB, queue_size=1)

if __name__ == '__main__':
    """Traxxas Rally Car Example: Wall Following

    Receive lidar scan message and ouput control velocities using PID control
    """
    
    print("Initializing node... ")
    rospy.init_node("wall_following")
    wall_follower = WallFollower()
    print("distance to wall = ", wall_follower.dist2wall)
    print("range @ +90 degrees: ", wall_follower.r_p90, "range @ -90 degrees: ", wall_follower.r_n90)
    # publish serial-port command string at 150 Hz frequency    
    wall_follower.pub_ser_cmd()
    #wall_follower.sub_ser_cmd()

    # subscribe to the high rate command
    # rospy.Subscriber("/serial_command", String, cmdCB, queue_size=1)


    rospy.spin()
    print("Done.")
