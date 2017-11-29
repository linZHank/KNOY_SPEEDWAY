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
    wp_list = [] # list store x and y coords of waypoints
    i = 0
    with open(filename) as f:
        for line in f:
            x = line.split()[1] # X
            y = line.split()[4] # Y
            wp_list.append([x, y]) # add waypoint to wp_list
            i += 1
    wp_array = np.array(wp_list)
    return wp_array

# Enable serial 
console_ser = serial.Serial(port = "/dev/ttyACM0", baudrate=115200)
# Static parameters
KP_THETA = 10 # PID: Kp governs turning angle 
KD_THETA = 4.5 # PID: Kd governs turning angle
KP_DIST = 500 # PID: Kp governs linear velocity
KD_DIST = 100  # PID: Kd governs linear velocity
PUBLISH_RATE = 150.0 # number of control commands to be published per second
WPS = load_wpfile('waypoints_1128.txt') # wapoints in numpy array

class WaypointsFollower():
    '''Class defines rallycar maneuvering along a set of waypoints'''
    def __init__(self):
        self._ser_cmd_pub = rospy.Publisher("/serial_command", String, queue_size =1)
        self._sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped,
                                     self.amclCB)
        # parameters
        self.wp_idx = 0
        self.current_pose = np.zeros(3)
        self.est_quaterion = (0.0, 0.0, 0.0, 0.0)
        self.est_euler = (0, 0, 0)
        self.next_waypoint = self.grid2point(GRID_WP[self.wp_idx]) # convert grid indices into point coordinates
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

    # Test if car is close to next waypoint enough
    def isNearby(self):
        nearNextWP = np.linalg.norm(self.next_waypoint[0:2]-self.current_pose[0:2]) < 0.25

        return np.linalg.norm(self.next_waypoint[0:2]-self.current_pose[0:2]) < 0.5 # euclidean distance, ignore oritation difference

    # Generate command to move car to next waypoint 
    def cmd2NextPoint(self):
        if self.V_theta < 0: # turn right
            self.serial_command = "A%05d+%04d" %(self.V_theta*2048/50, 200) # self.V_dist)
        else: # turn left
            self.serial_command = "A+%04d+%04d" %(self.V_theta*2048/50, 200) # self.V_dist)
        # return self.serial_command

    def grid2point(self, grid):
        x = (grid[1] - 1023) * 0.05 - 0.025
        y = (grid[0] - 1023) * 0.05 - 0.025
        wp = np.array([x, y])
        return wp

    def amclCB(self, data):
        # current pose [X, Y, yaw]
        self.est_quaterion = (data.pose.pose.orientation.x,
                    data.pose.pose.orientation.y, data.pose.pose.orientation.z,
                    data.pose.pose.orientation.w) # amcl estimated quaternion
        self.est_euler = tf.transformations.euler_from_quaternion(self.est_quaterion) # euler
        self.current_pose[0] = data.pose.pose.position.x
        self.current_pose[1] = data.pose.pose.position.y
        self.current_pose[2] = math.degrees(self.est_euler[2]) # convert rad to degree
        print("next waypoint: ", self.next_waypoint)
        print("current car pose: ", self.current_pose)
        # error
        self.err_dist = np.linalg.norm(self.next_waypoint[0:2]-self.current_pose[0:2]) # distance error
        self.derr_dist = self.prerr_dist - self.err_dist # delta distance error
        self.err_theta = \
        -np.arctan2(self.next_waypoint[1]-self.current_pose[1], 
                    self.next_waypoint[0]-self.current_pose[0]) + self.current_pose[2] # orientation error, positive -> turn left
        self.derr_theta = self.prerr_theta - self.err_theta # delta orientation error
        print("err_dist: ", self.err_dist, "derr_dist: ", self.derr_dist)
        print("err_theta: ", self.err_theta, "derr_theta: ", self.derr_theta)
        # move car
        if not self.isNearby():
            #PD control
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
            self.cmd2NextPoint() # command move car to next waypoint 
            print("Velocity control - speed: ", self.V_dist, "\nVellocity control - angular: ", self.V_theta)
            print("Serial control string: ", self.serial_command)
        elif self.isNearby() and not self.wp_idx == GRID_WP.shape[0]:
            self.wp_idx += 1
            self.next_waypoint = self.grid2point(GRID_WP[self.wp_idx]) # update next way point, get ready for next journey
            self.V_dist = 200
            self.err_theta = \
            -np.arctan2(self.next_waypoint[1]-self.current_pose[1], 
                        self.next_waypoint[0]-self.current_pose[0]) + self.current_pose[2] # orientat
            self.derr_theta = 0
            self.V_theta = KP_THETA*self.err_theta + KD_THETA*self.derr_theta
            self.cmd2NextPoint()
            print("load in next waypoint: ", self.next_waypoint)
        else:
            self.serial_command = "A+0000+0000" # generate command to stop at last waypoint

        # update previous step error
        self.prerr_dist = self.err_dist
        self.prerr_theta = self.err_theta


    def drive(self):
        ''' Control car @ 150 Hz '''
        rate = rospy.Rate(PUBLISH_RATE)
        print("Publishing serial command @ 150 Hz. Press Ctrl-C to stop...")

        while not rospy.is_shutdown():
            self.car_pose = self.updateCarPose() # update car pose from previous time step
            self.err_pos, self.err_ang, self.derr_pos, self.derr_ang = self.computeErrors() # compute errors
            self.V_gas, self.V_turn = self.computeControl() # compute control on gas paddle and wheel turning
            self.serial_command = self.generateCommand()
            
            self._ser_cmd_pub.publish(self.serial_command) # publish command
            # rospy.loginfo("serial command published %s", self.scrial_command)
            console_ser.write(self.serial_command) # send out command to serial console
            rate.sleep()

    def clean_shutdown(self):
        print("\n\nTurning off the car...")
        console_ser.write("A+0000+0000")
        console_ser.close()
        return True

def main():
    rospy.init_node('waypoints_driving')
    driver = WaypointsFollower()
    rospy.on_shutdown(driver.clean_shutdown)
    driver.drive() 
    rospy.spin()

if __name__=='__main__':
    main()
