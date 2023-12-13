#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from autonav.srv import *

def get_distance():
    rospy.wait_for_service("calculate_distance")
    try:
        distance = rospy.ServiceProxy('calculate_distance', calculate_distance)
        resp1 = distance(0, 0, 0 , 0)
        return resp1.distance
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

#Callback definition
def active_cb(extra):
    rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
    #rospy.loginfo("Current location: " + str(feedback))
    print(get_distance())
    return
    
def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")

rospy.init_node('goal_pose')

navclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
navclient.wait_for_server()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()

goal.target_pose.pose.position.x = 3.25
goal.target_pose.pose.position.y = 1.3
goal.target_pose.pose.position.z = 0.0
goal.target_pose.pose.orientation.x = 0.0
goal.target_pose.pose.orientation.y = 0.0
goal.target_pose.pose.orientation.z = 0.0
goal.target_pose.pose.orientation.w = 1

navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
finished = navclient.wait_for_result()

if not finished:
    rospy.logerr("Action server not available")
else:
    rospy.loginfo(navclient.get_result())



