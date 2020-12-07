#!/usr/bin/env python
import rospy
import numpy as np
import sys
from scara_kin.srv import effjointvelocity
from gazebo_msgs.srv import GetJointProperties

#joint velocity calculation client
def task_space_velcal_client(lx,ly,lz,ox,oy,oz):
    rospy.wait_for_service('TaskSpace_velocities_cal')
    try:
        TaskSpace_velocity = rospy.ServiceProxy('TaskSpace_velocities_cal',TaskSpaceVelCal)
        resp = TaskSpace_velocity(lx,ly,lz,ox,oy,oz)
        return resp
        
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

if __main__ = "__main__":
    #get joint position:
    rospy.wait_for_service('/gazebo/get_joint_properties')
    get_joint_property = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)    #call ros service get_joint_properties
    request1 = get_joint_property('joint1')
    get_joint_property = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)
    request2 = get_joint_property('joint2')
    get_joint_property = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)
    request3 = get_joint_property('joint3')
    q1 = request1.position[0]
    q2 = request2.position[0]
    q3 = request3.position[0]

    #calculate joint velovity
    #user input velocity of end effector
    q1_dot = float(input('Input q1_dot: '))
    q2_dot = float(input('Input q2_dot: '))
    q3_dot = float(input('Input q3_dot: '))
    o_x = float(input('Input q1_dot: '))
    o_y = float(input('Input q2_dot: '))
    o_z = float(input('Input q3_dot: '))
    TaskSpace = task_space_velcal_client(q1_dot,q2_dot,q3_dot,o_x,o_y,o_z)
    print(TaskSpace.q_dot)
