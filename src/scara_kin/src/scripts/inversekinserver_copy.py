#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from numpy import cos
from numpy import sin
from math import pow, acos, atan2, cos
import math as m
from numpy import *
from std_msgs.msg import Float32MultiArray
from scara_kin.srv import Inversekin, InversekinResponse
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3


def handle(request):

    angles = Vector3()

    px = request.pose.position.x
    py = request.pose.position.y
    pz = request.pose.position.z

    l1 = 1.0
    l2 = 0.5
    l3 = 0.8

    #print(acos((pow(px, 2) + pow(py, 2) - pow(l2, 2) - pow(l3, 2))/2))
    angles.y = acos((pow(px, 2) + pow(py, 2) - pow(l2, 2) - pow(l3, 2))/(2 * l2 * l3))

    #c = arctan2(py, px)
    #d = arctan2(l2 * sin(angles.y))
    #e = l2 + l3 + cos(angles.y)

    angles.x = (atan2(py, px)) - atan2(l2 * sin(angles.y), l2 + l3 + cos(angles.y))

    angles.z = l1 - pz

    print (angles)

    return InversekinResponse(angles)


def inverse_server():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    rospy.init_node('inverse_server')

    L = rospy.Service('inversekinserver', Inversekin, handle)
    print("Ready")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    inverse_server()
