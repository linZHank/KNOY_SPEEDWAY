#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

MAP_W = 2048
MAP_H = 2048
def callback(data):
    global grid 
    grid = np.array(data.data).reshape((MAP_W,MAP_H))
    
    print(grid.dtype)

def map_listener():
    rospy.init_node('map_listener', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, callback)
    rospy.spin()

if __name__=='__main__':
    map_listener()
