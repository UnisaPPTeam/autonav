#!/usr/bin/env python3

import rospy
from autonav.msg import coordinate

def coordinates_callback(msg):
    # Callback chiamata quando vengono ricevute le coordinate del robot
    rospy.loginfo(f"Received robot coordinates: X={msg.x}, Y={msg.y}, Z={msg.z}")

if __name__ == "__main__":
    rospy.init_node('coordinate_subscriber_node')
    print("Waiting for coordinates")
    # Iscrizione al topic "robot_coordinates"
    rospy.Subscriber("/robotinitialposition", coordinate, coordinates_callback)

    rospy.spin()
    