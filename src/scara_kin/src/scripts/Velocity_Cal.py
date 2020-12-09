#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from scara_kin.srv import JointVelocityCal,JointVelocityCalResponse,TaskSpaceVelCal,TaskSpaceVelCalResponse
from gazebo_msgs.srv import *

#This file calculate end effector velocity and joint velocity, contained with 2 service

def Jacobian_cal(a,b,c):
    #import joint position
    l1 = 1.0
    l2 = 0.5
    l3 = 0.8
    q1 = a
    q2 = b
    q3 = c
    alpha = [0.0,np.pi,0.0]
    theta = [q1,q2,0.0]
    # calcualte transfer matrix
    T0_1 = np.array([[np.cos(theta[0]), -np.sin(theta[0])*np.cos(alpha[0]),  np.sin(theta[0])*np.sin(alpha[0]), l2*np.cos(theta[0])],
                      [np.sin(theta[0]),  np.cos(theta[0])*np.cos(alpha[0]), -np.cos(theta[0])*np.sin(alpha[0]), l2*np.sin(theta[0])],
                      [               0.0,                   np.sin(alpha[0]),                   np.cos(alpha[0]),                  l1],
                      [               0.0,                                  0.0,                                  0.0,                     1.0]])
    T1_2 = np.array([[np.cos(theta[1]), -np.sin(theta[1])*np.cos(alpha[1]),  np.sin(theta[1])*np.sin(alpha[1]), l3*np.cos(theta[1])],
                      [np.sin(theta[1]),  np.cos(theta[1])*np.cos(alpha[1]), -np.cos(theta[1])*np.sin(alpha[1]), l3*np.sin(theta[1])],
                      [               0,                   np.sin(alpha[1]),                   np.cos(alpha[1]),                  0.0],
                      [               0.0,                                  0.0,                                  0.0,                     1.0]])
    
    T2_3 = np.array([[np.cos(theta[2]), -np.sin(theta[2])*np.cos(alpha[2]),  np.sin(theta[2])*np.sin(alpha[2]), 0*np.cos(theta[2])],
                      [np.sin(theta[2]),  np.cos(theta[2])*np.cos(alpha[2]), -np.cos(theta[2])*np.sin(alpha[2]), 0*np.sin(theta[2])],
                      [               0.0,                   np.sin(alpha[2]),                   np.cos(alpha[2]),                  q3],
                      [               0.0,                                  0.0,                                  0.0,                     1.0]])
    T0_2 = np.dot(T0_1,T1_2)
    T0_3 = np.dot(T0_2,T2_3)

    #Origins of each frame
    o_0 = np.array([0,0,0])
    o_1 = T0_1[0:3,3]
    o_2 = T0_2[0:3,3]
    o_3 = T0_3[0:3,3]
    #Z-axis
    z0_0 = np.array([0,0,1])
    z0_1 = T0_1[0:3,2]
    z0_2 = T0_2[0:3,2]
    # Jacobian matrix calculation
    J = np.zeros((6,3))
    J[0:3,0] = np.cross(z0_0,np.subtract(o_3,o_0))
    J[0:3,1] = np.cross(z0_1,np.subtract(o_3,o_1))
    J[0:3,2] = z0_2
    J[3:,0] = z0_0
    J[3:,1] = z0_1
    J[3:,2] = np.array([0,0,0])
  


    return J

#calculate joint velocities
def Joint_velocity_cal(obj):
    q1 = obj.a
    q2 = obj.b
    q3 = obj.c
    v_x = obj.vx
    v_y = obj.vy
    v_z = obj.vz
    w_x = obj.wx
    w_y = obj.wy
    w_z = obj.wz
    J = Jacobian_cal(q1,q2,q3)
    Jinv = np.linalg.pinv(J)
    ee_vel = np.array([v_x,v_y,v_z,w_x,w_y,w_z]).reshape((6,1))
    q_dot = np.matmul(Jinv,ee_vel)

    

    return JointVelocityCalResponse(q_dot)

#calculate end effector velocities
def taskspaceVel_Cal(obj1):
    J = Jacobian_cal([q1,q2,q3])
    ee_vel = np.array([obj1.ox,obj1.oy,obj1.oz]).reshape((3,1))
    q_dot = np.matmul(J,ee_vel)
    obj1.veloutput = q_dot

    return TaskSpaceVelCalResponse(obj1)


def Joint_velocity_server():
     s = rospy.Service('Joint_velocities_cal',JointVelocityCal,Joint_velocity_cal)


def TaskSpace_velocity_server():
     s = rospy.Service('TaskSpace_velocities_cal',TaskSpaceVelCal,taskspaceVel_Cal)


if __name__ == "__main__":
        rospy.init_node('velocity_calculation')
        Joint_velocity_server()
        TaskSpace_velocity_server()

        rospy.spin()    
    

