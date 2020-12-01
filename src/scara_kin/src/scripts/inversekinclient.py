#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from numpy import *
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, Point


def inversekinclient(pose):
    rospy.wait_for_service('Pose')
    try:
        Pose = rospy.ServiceProxy('inversekinserver', Inversekin)
        f = inversekin(pose)
        return f.joints
    except rospy.ServiceException as e:
        print ("Service Call Failed")