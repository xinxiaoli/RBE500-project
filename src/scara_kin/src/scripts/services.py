#!/usr/bin/env python
import rospy
import numpy as np
import sys
from scara_kin.srv import effjointvelocity
from gazebo_msgs.srv import GetJointProperties

#joint velocity calculation client
def eff_Joint_velocity_calculation_client(a,b,c,vx,vy,vz,wx,wy,wz):
    rospy.wait_for_service('Joint_velocities_cal')
    try:
        Eff_joint_velocity = rospy.ServiceProxy('Joint_velocities_cal',effjointvelocity)
        resp = Eff_joint_velocity(a,b,c,vx,vy,vz,wx,wy,wz)
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
    v_x = float(input('Input q1_dot: '))
    v_y = float(input('Input q2_dot: '))
    v_z = float(input('Input q3_dot: '))
    w_x = float(input('Input q1_dot: '))
    w_y = float(input('Input q2_dot: '))
    w_z = float(input('Input q3_dot: '))
    Eff_Joint = eff_Joint_velocity_calculation_client(q1_dot,q2_dot,q3_dot,v_x,v_y,v_z,w_x,w_y,w_z)
    print(Eff_Joint.q_dot)
