#!/usr/bin/env python
import rospy
import numpy as np
import sys
from scara_kin.srv import *
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
from std_msgs.msg import Float64
import pid3



#joint velocity calculation client
def Joint_velocity_calculation_client(a,b,c,vx,vy,vz,wx,wy,wz):
    rospy.wait_for_service('Joint_velocities_cal')
    try:
        joint_velocity = rospy.ServiceProxy('Joint_velocities_cal',JointVelocityCal)
        resp1 = joint_velocity(a,b,c,vx,vy,vz,wx,wy,wz)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    

#joint velocity calculation client
def task_space_velcal_client(lx,ly,lz,ox,oy,oz):
    rospy.wait_for_service('TaskSpace_velocities_cal')
    try:
        TaskSpace_velocity = rospy.ServiceProxy('TaskSpace_velocities_cal',TaskSpaceVelCal)
        resp2 = TaskSpace_velocity(lx,ly,lz,ox,oy,oz)
        return resp2
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
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
    #velocity inpput
    y_velocity = float(input("velocity in positive y direction: "))
    vel = [0,y_velocity,0,0,0,0]
    #end_effector velocity
    ee_dot = np.zeros((3,1))
    q_dot = np.array([0,0,0]).reshape((3,1))
    #convert velocity into joint spaze
    input_q = [q1,q2,q3,vel[0],vel[1],vel[2],vel[3],vel[4],vel[5]]
    Joint_velocities = Joint_velocity_calculation_client(q1,q2,q3,vel[0],vel[1],vel[2],vel[3],vel[4],vel[5])
    rev_qdot = np.array(Joint_velocities.q_dot).reshape((3,1))
    #input controller
    pid = pid3.pid_class()
    #joint1 and 2 gain are 30 and 8, joint 3 gain are 375 and 25
    pid.set_gains(30, 8, 375, 25)



    
 
    #calculate joint velovity
    #user input velocity of end effector
    



