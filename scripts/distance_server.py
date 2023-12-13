#!/usr/bin/env python

from __future__ import print_function
from math import  sqrt

from autonav.srv import calculate_distance, calculate_distanceResponse

import rospy

def handle_distance(req):
    print(f"The distance is {sqrt((req.x1 - req.x2)**2+(req.y1 - req.y2)**2)}")
    return calculate_distanceResponse(sqrt((req.x1 - req.x2)**2+(req.y1 - req.y2)**2))

def calculate_distance_server():
    rospy.init_node("distance")
    s = rospy.Service('calculate_distance', calculate_distance, handle_distance)
    print("Waiting for coordinates")
    rospy.spin()
    
if __name__ == "__main__":
    calculate_distance_server()