#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from scara_kin.srv import Refpos, RefposResponse

def handle(refpos):

    pos1 = refpos.ref1
    pos2 = refpos.ref2
    pos3 = refpos.ref3

    return RefposResponse([pos1, pos2, pos3])

def rospos_server():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    rospy.init_node('rospos_server')

    L = rospy.Service('refposserver', Refpos, handle)
    print("Ready")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rospos_server()