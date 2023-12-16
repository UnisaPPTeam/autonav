#!/usr/bin/env python3

import sys
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from autonav.srv import *
from autonav.msg import coordinate

# Create a dictionary to hold all the rooms locations
rooms_coordinates = {
    "studio": {"x": 5.6, "y": -2.1, "z": 0.0},
    "living_room": {"x": -3.6, "y": 1.0, "z": 0.0},
    "bedroom": {"x": 3.25, "y": 1.3, "z": 0.0},
    "bathroom": {"x": 1.7, "y": 4.1, "z": 0.0},
}

x1 = 0.0
y1 = 0.0

def get_distance(feedback):
    rospy.wait_for_service("calculate_distance")
    try:
        distance = rospy.ServiceProxy('calculate_distance', calculate_distance)
        x2 = feedback.base_position.pose.position.x
        y2 = feedback.base_position.pose.position.y
        resp1 = distance(x1, y1, x2, y2)
        return resp1.distance
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

# Callback definition
def active_cb():
    rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
    print(f"the current robot distance from the starting point is {get_distance(feedback)}")
    position = coordinate()
    position.x = feedback.base_position.pose.position.x
    position.y = feedback.base_position.pose.position.y
    position.z = feedback.base_position.pose.position.z
    pub.publish(position)
    return

def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")

def navigate_to_room(room_name):
    rospy.init_node('goal_pose')

    navclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    navclient.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Utilizza il dizionario per ottenere le coordinate della stanza specificata
    room_coordinates = rooms_coordinates.get(room_name)
    if room_coordinates:
        goal.target_pose.pose.position.x = room_coordinates["x"]
        goal.target_pose.pose.position.y = room_coordinates["y"]
        goal.target_pose.pose.position.z = room_coordinates["z"]
    else:
        rospy.logerr(f"Room '{room_name}' not found in the coordinate dictionary.")
        return
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1

    print(f"getting the robot initial position")
    data = rospy.get_param('initial_position')
    global x1, y1
    x1 = data['x']
    y1 = data['y']
  
    print(f"Starting position set: {x1},{y1}")
    navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
    
    finished = navclient.wait_for_result(timeout=rospy.Duration(0.01))

    if not finished:
        rospy.logerr("Action server not available")
    else:
        rospy.loginfo(navclient.get_result())

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: ./script_name.py <room_name>")
        sys.exit(1)

    room_name = sys.argv[1]
    
    pub = rospy.Publisher('/robotinitialposition', coordinate, queue_size=30)
    
    navigate_to_room(room_name)
