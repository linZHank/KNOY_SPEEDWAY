#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import numpy as np
import tf
import math
import serial

import pdb

# Method load manually created waypoints file
def load_wpfile(filename):
    wp_list = [] # list store x and y coords of waypoints
    i = 0
    with open(filename) as f:
        for line in f:
            x = float(line.split()[1]) # X
            y = float(line.split()[4]) # Y
            wp_list.append([x, y]) # add waypoint to wp_list
            i += 1
    wp_array = np.array(wp_list)
    return wp_array 

def imuCalibration(console_ser):
    calix = []
    caliy = []
    lim = 0
    while lim < 100:
        if console_ser.inWaiting()>0:
            imu_cali = console_ser.read(22)
            print(imu_cali)
            calix.append(float(imu_cali[1:7]))
            caliy.append(float(imu_cali[7:13]))
            lim += 1
    acc_x_ref = sum(calix) / len(calix)
    acc_y_ref = 1.5 * sum(caliy) / len(caliy)
    print("\n---IMU calibrated---\n")
    return acc_x_ref, acc_y_ref
# Enable serial port 
console_ser = serial.Serial(port = "/dev/ttyACM0", baudrate=115200)
console_ser.close()
console_ser.open()
imuinfo = console_ser.read(62)
console_ser.write("IMU0")
console_ser.write("IMU1")
print(imuinfo)

# Static parameters
WHEELBASE = 0.335 # m
# ACC_X_REF, ACC_Y_REF = imuCalibration(console_ser)
ACC_X_REF = 0; ACC_Y_REF = 0. # no calibration
print("ACC_X_REF = ", ACC_X_REF, "ACC_Y_REF = ", ACC_Y_REF)
G = 16384 / 9.81 # gravitational acceleration in IMU reading
KP_THETA = 1.5 # PID: Kp governs steering angle 
KD_THETA = 7.5 # PID: Kd governs steering angle
KP_DIST = 360 # PID: Kp governs linear velocity
KD_DIST = 100  # PID: Kd governs linear velocity
PUBLISH_RATE = 150.0 # number of control commands to be published per second
DT = 1 / PUBLISH_RATE # time interval 
waypointsfile = './waypointsfiles/handmade_1210.txt'
WPS = load_wpfile(waypointsfile) # wapoints in numpy array

# Test if two points are close 
def isNearby(point1, point2, threshold = 1):
    ''' Test if two points are close to each other '''
    # np.linalg.norm(self.next_waypoint[0:2]-self.current_pose[0:2]) < 0.25
    flag = np.linalg.norm(point1-point2) < threshold
    return flag

class WaypointsFollower():
    '''Class defines rallycar maneuvering along a set of waypoints'''
    def __init__(self):
        # parameters
        self.wp_idx = 0 # waypoint index
        self.carpose = np.zeros(3) # [x, y, yaw]
        self.carpose_ref = np.zeros(3) # [x, y, yaw]
        self.acc_x = 0. # local accerleration regarding to car fixed CS
        self.acc_y = 0.
        self.x = 0 # states of the car in global CS
        self.y = 0
        self.phi = 0
        self.xdot = 0
        self.ydot = 0
        self.phidot = 0
        self.next_waypoint = WPS[0] #self.grid2point(GRID_WP[self.wp_idx]) # convert grid indices into point coordinates
        self.V_throttle = 0.
        self.V_steer = 0.
	self.serial_command = "A+0000+0000"
        self.imuread = "I+00000+00000+00000U"
        # errors for PD control
        self.err_dist = 0
        self.prerr_dist = 0.
        self.derr_dist = 0.
        self.err_ang = 0
        self.prerr_ang = 0.
        self.derr_ang = 0.
        # methods
        self._sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amclCB)
        self._ser_cmd_pub = rospy.Publisher('/serial_command', String, queue_size =1)

    def amclCB(self, data):
        ''' Callback function for amcl '''
        rospy.loginfo("\n~~~AMCL comes in !!!~~~\n")
        # current pose [X, Y, yaw]
        amcl_quaterion = (data.pose.pose.orientation.x,
                    data.pose.pose.orientation.y, data.pose.pose.orientation.z,
                    data.pose.pose.orientation.w) # amcl estimated quaternion
        amcl_euler = tf.transformations.euler_from_quaternion(amcl_quaterion) # euler
        self.carpose_ref[0] = data.pose.pose.position.x
        self.carpose_ref[1] = data.pose.pose.position.y
        self.carpose_ref[2] = math.degrees(amcl_euler[2]) # convert rad to degree
        rospy.loginfo("\n~~~AMCL reference set~~~\n")
        if isNearby(self.carpose_ref[:2], self.carpose[:2], threshold = 10):
            self.carpose = self.carpose_ref
            self.x = self.carpose[0]
            self.y = self.carpose[1]
            self.phi = self.carpose[2]
            rospy.loginfo("\n~~~Car pose updated by AMCL~~~\n")
        print("\n~~~Estimated car pose: ", self.carpose, "~~~\n")
        print("\n~~~Reference car pose by AMCL: ", self.carpose_ref, "~~~\n")
        print("\n~~~Driving towards waypoint %d: " % (self.wp_idx), self.next_waypoint, "~~~\n")

    def readIMU(self):
        if console_ser.inWaiting()>0:
            self.imuread = console_ser.read(22)
            print("IMU read: ", self.imuread)
            self.acc_x = (float(self.imuread[1:7]) - ACC_X_REF) / G
            if math.fabs(self.acc_x) < 0.05:
                self.acc_x = 0.
            self.acc_y = (float(self.imuread[7:13]) - ACC_Y_REF) / G
            if math.fabs(self.acc_y) < 0.25:
                self.acc_y = 0

    def updateCarPose(self, dt = 1 / PUBLISH_RATE):
        self.readIMU()
        # compute acceleration in global CS
        xdd = self.acc_x*math.cos(math.radians(self.phi)) - self.acc_y*math.sin(math.radians(self.phi))
        ydd = self.acc_x*math.sin(math.radians(self.phi)) + self.acc_y*math.cos(math.radians(self.phi))
        # compute car thrust speed
        # spd = self.xdot / math.cos(math.radians(self.phi)) # option(1)
        spd = self.acc_x*dt # option(2)
        # update car states
        self.phidot = math.degrees(spd / WHEELBASE * math.tan(math.radians(self.V_steer*50/2048))) # option(1)
        # self.phidot = math.degrees(V_throttle / WHEELBASE * math.tan(math.radians(self.V_steer))) # option(2)
        self.x += self.xdot * dt
        self.y += self.ydot * dt
        self.phi += self.phidot * dt
        self.xdot += xdd * dt
        self.ydot += ydd * dt
        # update car pose
        self.carpose =  np.array([self.x, self.y, self.phi])

    def computeErrors(self):
        ''' Compute error and change of error between car and next waypoint '''
        self.err_dist = np.linalg.norm(self.next_waypoint - self.carpose[:2]) # distance error
        self.derr_dist = self.err_dist - self.prerr_dist # delta distance error
        self.err_ang = self.phi - math.degrees(math.atan2(self.next_waypoint[1] - self.y,
                                     self.next_waypoint[0]-self.x)) #  orientation error, positive -> turn left
        self.derr_ang = self.err_ang - self.prerr_ang # delta orientation error
        self.prerr_dist = self.err_dist
        self.prerr_ang = self.err_ang

    def computeControl(self):
        ''' Compute V_steer and V_throttle using PD control
            V = Kp*err + Kd*derr '''
        if not isNearby(self.carpose[0:2], self.next_waypoint):
            self.computeErrors()
        else:
            if self.wp_idx < WPS.shape[0]-1:
                self.wp_idx += 1
                self.next_waypoint = WPS[self.wp_idx]
                print("---car gets close to the waypoint---\nload in next waypoint: ", self.next_waypoint, "----\n")
                self.computeErrors()
                self.derr_dist = 0
                self.derr_ang = 0
            else:
               self.clean_shutdown()
               print("Destination reached!\n---")
        self.V_throttle = KP_DIST*self.err_dist + KD_DIST*self.derr_dist
        self.V_steer = KP_THETA*self.err_ang + KD_THETA*self.derr_ang
        # make sure gas control in range
        if self.V_throttle > 360:
            self.V_throttle = 360
        elif self.V_throttle < 0:
            self.V_throttle = 0
        # make sure steering in range
        if self.V_steer > 50:
            self.V_steer = 50
        elif self.V_steer < -50:
            self.V_steer = -50
        self.generateCommand()

    def generateCommand(self):
        ''' Generate serial command according to V_steer and V_throttle'''
        if self.V_steer < 0: # turn right
            self.serial_command = "A%05d+%04d" %(self.V_steer*2048/50, self.V_throttle)
        else: # turn left
            self.serial_command = "A+%04d+%04d" %(self.V_steer*2048/50, self .V_throttle)

    def drive(self):
        ''' Control car @ 150 Hz '''
        rate = rospy.Rate(PUBLISH_RATE)
        print("Publishing serial command @ 150 Hz. Press Ctrl-C to stop...")
        counter = 0
        while not rospy.is_shutdown():
            self.computeControl() # compute control on gas paddle and wheel steering

            if counter % 5 == 0: # print estimated car pose and other debugging information @ frequency of 30 Hz
                rospy.loginfo(">>>>>>>>> Estimated Car States @ counter: %04d ---" % (counter))
                print("Driving towards the waypoint %d: " % (self.wp_idx), self.next_waypoint)
                print("Car pose, x: ", self.x, "y: ", self.y, "yaw: ", self.phi)
                print("Car pose changing rate, xdot: ", self.xdot, "ydot: ", self.ydot, "yawdot: ", self.phidot)
                print("Car longitudinal accelerations: ", self.acc_x, "Car transverse acceleration: ", self.acc_y)
                print("distance to next waypoint: ", self.err_dist, "derr_dist: ", self.derr_dist)
                print("angle to next waypoint: ", self.err_ang, "derr_ang: ", self.derr_ang)
                print("Speed control - throttle: ", self.V_throttle, "Steering wheel control - steer: ", self.V_steer)
                print("Serial control string: ", self.serial_command)
                print("----")

            self.updateCarPose() # update car pose from previous time step

            if counter % 5 == 0: # print updated car pose information
                print("Updated car states >>>>>>>>>")
                print("Car pose, x: ", self.x, "y: ", self.y, "yaw: ", self.phi)
                print("Car pose changing rate, xdot: ", self.xdot, "ydot: ", self.ydot, "yawdot: ", self.phidot)

            self._ser_cmd_pub.publish(self.serial_command) # publish command
            # rospy.loginfo("serial command published %s", self.scrial_command)
            console_ser.write(self.serial_command) # send out command to serial console
            rate.sleep()
            counter += 1

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
