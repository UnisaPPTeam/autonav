#!/usr/bin/env python3

import sys
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from autonav.srv import *
from autonav.msg import cordinate

# Definisci un dizionario per le coordinate delle stanze
stanze_coordinates = {
    "cucina": {"x": 4.5, "y": -2.55, "z": 0.0},
    "salone": {"x": -4.45, "y": -0.45, "z": 0.0},
    "stanza_da_letto": {"x": 3.25, "y": 1.3, "z": 0.0},
    "bagno": {"x": -0.15, "y": 2.9, "z": 0.0},
}

x1 = 0.0
y1 = 0.0

def get_distance(feedback):
    rospy.wait_for_service("calculate_distance")
    try:
        print("distance calculation service init")
        distance = rospy.ServiceProxy('calculate_distance', calculate_distance)
        print("getting current node location")
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
    room_coordinates = stanze_coordinates.get(room_name)
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
    print(f"Goal set: {goal}")
    navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
    finished = navclient.wait_for_result()

    if not finished:
        rospy.logerr("Action server not available")
    else:
        rospy.loginfo(navclient.get_result())

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: ./script_name.py <room_name>")
        sys.exit(1)

    room_name = sys.argv[1]
    navigate_to_room(room_name)