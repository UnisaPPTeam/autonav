#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def init_pose():
    #Node initialization
    rospy.init_node('init_pose')
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

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
    rospy.sleep(1)

    rospy.loginfo("setting initial info")
    pub.publish(init_msg)
    rospy.loginfo("initial pose set")
    # Save initial position to the parameter server
    initial_position = {'x':init_msg.pose.pose.position.x,'y':init_msg.pose.pose.position.y}
    rospy.set_param('initial_position',initial_position)
    
if __name__ == "__main__":
    init_pose()