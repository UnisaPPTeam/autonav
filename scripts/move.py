#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from autonav.srv import *
from autonav.msg import cordinate

x1 = 0.0
y1 = 0.0
z1 = 0.0 

def get_distance(feedback):
    rospy.wait_for_service("calculate_distance")
    try:
        print("distance calculation service init")
        distance = rospy.ServiceProxy('calculate_distance', calculate_distance)
        print("getting current node location")
        x2 = feedback.base_position.pose.position.x
        y2 = feedback.base_position.pose.position.y
        resp1 = distance(x1, y1, x2 , y2)
        return resp1.distance
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

#Callback definition
def active_cb(extra):
    rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
    #rospy.loginfo("Current location: " + str(feedback))
    print(f"the current robot distance from the starting point is{get_distance(feedback)}")
    return
    
def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")




def main():
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


    print("subscribing to robot initial position")
    # Subscribe to get the initial position
    data = rospy.wait_for_message('/ros', cordinate, timeout=5)
    print("Starting position recieved from init node")
    global x1, y1, z1
    x1 = data.x
    y1 = data.y
    z1 = data.z
    print(f"Starting position set:{x1},{y1},{z1}")
    navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
    finished = navclient.wait_for_result()

    if not finished:
        rospy.logerr("Action server not available")
    else:
        rospy.loginfo(navclient.get_result())

if __name__ == "__main__":
    main()



