#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

def callback(data, args):
	bc = tf2_ros.TransformBroadcaster()
	# bc = tf.TransformBroadcaster()
	t = TransformStamped()

	t.header.stamp = rospy.Time.now()
	t.header.frame_id = args[0]
	t.child_frame_id = args[1]
	t.transform.translation = data.pose.pose.position
	t.transform.rotation = data.pose.pose.orientation
#	print (data.pose.pose.position.x,data.pose.pose.position.y) # debug
	bc.sendTransform(t)

if __name__ == "__main__":
	rospy.init_node("odomtransformer")
	print("************* odom.py: establish odom to base transform **************")
	odomInput = rospy.get_param("/odomtransformer/odom_input")
	tfOutput  = rospy.get_param("/odomtransformer/tf_output")
	rospy.Subscriber(odomInput, Odometry, callback, (odomInput, tfOutput))
	rospy.spin()
