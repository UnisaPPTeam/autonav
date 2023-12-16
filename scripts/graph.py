import rospy
from autonav.msg import cordinate

def graph():
    data = rospy.wait_for_message('/ros', cordinate)
    return
