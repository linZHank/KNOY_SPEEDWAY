#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import numpy as np
import tf
import math
import serial

import pdb

# Load manually created waypoints file
def load_wpfile(filename):
    wp_array = np.zeros((15,3))
    i = 0
    with open(filename) as f:
        for line in f:
            wp_array[i][0] = line.split()[1] # X
            wp_array[i][1] = line.split()[4] # Y
            # wp_array[i][2] = line.split()[-2] # yaw seems not necessary
            i += 1
    return wp_array

# Enable serial 
console_ser = serial.Serial(port = "/dev/ttyACM0", baudrate=115200)
# Static parameters
KP_THETA = 10 # PID: k_d
KD_THETA = 4.5 # PID: k_d
KP_DIST = 500 # PID: K_p
KD_DIST = 100  # PID: k_d
PUBLISH_RATE = 150.0 # number of control commands to publish per second
WAYPOINTARR = load_wpfile('waypoints_15.txt') # load way points file, output (8,3) numpy array

class WaypointsDriver():
    '''Class defines rallycar maneuvering along a set of waypoints'''
    def __init__(self):
        self._ser_cmd_pub = rospy.Publisher("/serial_command", String, queue_size =1)
        self._sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped,
                                     self.amclCB)
        # parameters
        self.idx_target = 0 # target waypoint index
        self.amcl_pose = np.zeros(3)
        self.waypoints = WAYPOINTARR[:,0:2]
        #self.next_waypoint = WPARR[self.wp_idx]
        self.dist2wps = np.linalg.norm((self.amcl_pose[0:2] - self.waypoints), axis=1)
        self.V_dist = 0.
        self.V_theta = 0.
	self.serial_command = "A+0000+0000"
        # errors for PD control
        self.err_dist = 0
        self.prerr_dist = 0
        self.derr_dist = 0
        self.err_theta = 0
        self.prerr_theta = 0
        self.derr_theta = 0

    # Estimate rallycar's pose from amcl message
    def estimateCarPose(self, amcl_msg):
        ''' Convert read in amcl message data to form of [x, y, yaw] '''
        amcl_quaterion = (amcl_msg.pose.pose.orientation.x,
                    amcl_msg.pose.pose.orientation.y, amcl_msg.pose.pose.orientation.z,
                    amcl_msg.pose.pose.orientation.w) # amcl estimated quaternion
        amcl_euler = tf.transformations.euler_from_quaternion(amcl_quaterion) # euler
        self.amcl_pose[0] = amcl_msg.pose.pose.position.x
        self.amcl_pose[1] = amcl_msg.pose.pose.position.y
        self.amcl_pose[2] = math.degrees(amcl_euler[2]) # convert rad to degree
        #est_pose = self.amcl_pose
        #return est_pose

    def computeErrors(self,loadnew=0):
        ''' Compute distance error and angular error 
            between the car and the target waypoint
        '''
        # linear
        self.err_dist = np.linalg.norm(
                self.waypoints[self.idx_target]-self.amcl_pose[0:2]) # distance error
        if loadnew == 0:
            self.derr_dist = self.prerr_dist - self.err_dist # delta distance error
        elif loadnew == 1:
            self.derr_dist = 0 - self.err_dist # delta distance error
        # angular
        self.err_theta = \
        -np.arctan2(self.waypoints[self.idx_target,1] - self.amcl_pose[1], 
                self.waypoints[self.idx_target,0] - self.amcl_pose[0]) + self.amcl_pose[2] # orientation error, positive -> turn left
        if loadnew == 0:
            self.derr_theta = self.prerr_theta - self.err_theta # delta orientation error
        elif loadnew == 1:
            self.derr_theta = 0 - self.err_theta # delta orientation error

    def computeControl(self):
        ''' Compute PID control based on err and derr '''
        self.V_dist = KP_DIST*self.err_dist + KD_DIST*self.derr_dist
        # make sure V_dist in range
        if self.V_dist > 9999:
            self.V_dist = 2048
        elif self.V_dist < 0:
            self.V_dist = 0
        self.V_theta = KP_THETA*self.err_theta + KD_THETA*self.derr_theta
        # make sure V_theta in range
        if self.V_theta > 50:
            self.V_theta = 50
        elif self.V_theta < -50:
            self.V_theta = -50
        
    # Generate command to move car to next waypoint 
    def outOfRange(self):
        ''' Test if rallycar is 10 meters away from the nearest waypoint '''
        return self.dist2wps.min() > 10

    def computeDists(self):
        self.dist2wps = np.linalg.norm((self.amcl_pose[0:2] - self.waypoints), axis=1) 

    def generateCmd(self):
        ''' generate string command for serial port '''
        if self.V_theta < 0: # turn right
            self.serial_command = "A%05d+%04d" %(self.V_theta*2048/50, 200) # self.V_dist)
        else: # turn left
            self.serial_command = "A+%04d+%04d" %(self.V_theta*2048/50, 200) # self.V_dist)

    def amclCB(self, data):
        ''' Callback function implement program after received amcl message '''
        self.estimateCarPose(data) # estimate self.amcl_pose: [x, y, yaw]
        print("current car pose estimated: ", self.amcl_pose)
        print("next waypoint is: ", self.waypoints[self.idx_target], "waypoint index: ", self.idx_target+1)
        self.computeDists() # compute distance between car and each waypoint
        self.computeErrors(loadnew=0) # compute car-waypoint dispacement 
        print("err_dist = ", self.err_dist, "\terr_theta = ", self.err_theta)
        print("derr_dist = ", self.derr_dist, "\tderr_theta = ", self.derr_theta)
        # Decide car motion    
        if not self.outOfRange():
            if self.idx_target <= WAYPOINTARR.shape[0]-2:
                if self.err_dist > 0.5 and self.dist2wps[self.idx_target] < self.dist2wps[self.idx_target+1]:
                    self.computeControl() # compute V_dist and V_theta
                    self.generateCmd()
                else:
                    self.idx_target += 1
                    print("go to next waypoint: ", self.waypoints[self.idx_target], "waypoint index: ", self.idx_target+1)
                    self.computeErrors(loadnew=1) # compute error wrt new waypoint and set derr = 0
                    self.computeControl()
                    self.generateCmd()
            elif self.idx_target == WAYPOINTARR.shape[0]-1:
                if self.dist2wps[self.idx_target] > 0.5:
                    self.computeControl()
                    self.generateCmd()
                else:
                    self.serial_command = "A+0000+0000"
                    print("braking...")
        else:
            self.serial_command = "A+0000+0000"

        print("Serial command generated: ", self.serial_command)
        # update previous step error
        self.prerr_dist = self.err_dist
        self.prerr_theta = self.err_theta
        print("---")

    def pubSerCmd(self):
        ''' Perform wall following '''
        rate = rospy.Rate(PUBLISH_RATE)

        print("Publishing serial command @ 150 Hz. Press Ctrl-C to stop...")
        while not rospy.is_shutdown():
            self._ser_cmd_pub.publish(self.serial_command)
            # rospy.loginfo("serial command published %s", self.scrial_command)
            console_ser.write(self.serial_command)
            rate.sleep()

    def clean_shutdown(self):
        print("\n\nTurning off the car...")
        console_ser.write("A+0000+0000")
        console_ser.close()
        return True

def main():
    rospy.init_node('waypoints_driving')
    wp_driver = WaypointsDriver()
    rospy.on_shutdown(wp_driver.clean_shutdown)
    wp_driver.pubSerCmd() # publishing serial command at 150 Hz
    rospy.spin()

if __name__=='__main__':
    main()
