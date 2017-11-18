#!/usr/bin/env python
'''Code for testing and viewing data in /amcl_pose'''
from __future__ import print_function
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped


class PoseReader():
    def __init__(self):
        self._sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,
                                     self.poseCB)
        self.x = 0.
        self.y = 0.
        self.z = 0.
    # read amcl pose callback 
    def poseCB(self, data):
        # read car pose
        self.position = data.pose.pose.position
        self.orientation = data.pose.pose.orientation
        self.x = self.position.x
        self.y = self.position.y
        self.z = self.position.z
        print('Estimated car position_x: ', self.x)
        print('Estimated car position_y: ', self.y)
        print('Estimated car position_z: ', self.z)
if __name__ == '__main__':
    """Read amcl estimated pose of rallycar"""

    print("Initializing node... ")
    rospy.init_node("pose_reading")
    pos_reader = PoseReader()
    rospy.spin()
    print("Done.")
