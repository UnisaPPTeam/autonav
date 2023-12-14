#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from autonav.msg import cordinate
#Node initialization
rospy.init_node('init_pose')
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
pub2 = rospy.Publisher('/robotinitialposition', cordinate, queue_size=1)

# Construct message with the initial position
init_msg = PoseWithCovarianceStamped()
init_msg.header.frame_id = "map"

# Get initial pose from Gazebo
odom_msg = rospy.wait_for_message('/odom', Odometry)
init_msg.pose.pose.position.x = odom_msg.pose.pose.position.x
init_msg.pose.pose.position.y = odom_msg.pose.pose.position.y
init_msg.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
init_msg.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
init_msg.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
init_msg.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w
initial_position = cordinate()
initial_position.x = odom_msg.pose.pose.position.x
initial_position.y = odom_msg.pose.pose.position.y
initial_position.z = 0
rospy.sleep(1)

rospy.loginfo("setting initial info")
pub.publish(init_msg)
rospy.loginfo("initial pose set")
rospy.loginfo("Sending initial position to distance server")
pub2.publish(initial_position)

