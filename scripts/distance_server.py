#!/usr/bin/env python

from math import  sqrt

from autonav.srv import calculate_distance, calculate_distanceResponse

import rospy

def get_distance(x1, y1, x2, y2):
    return sqrt((x1 - x2)**2+(y1 - y2)**2)

def handle_distance(req):
    return calculate_distanceResponse(get_distance(req.x1, req.y1, req.x2, req.y2))

def calculate_distance_server():
    rospy.init_node("distance")
    s = rospy.Service('calculate_distance', calculate_distance, handle_distance)
    print("Waiting for coordinates")
    rospy.spin()
    
if __name__ == "__main__":
    calculate_distance_server()