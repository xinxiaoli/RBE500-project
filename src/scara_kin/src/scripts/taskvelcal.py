#!/usr/bin/env python
import rospy
import numpy as np
import math as m
from std_msgs.msg import Float64
from scara_kin.srv import TaskSpaceVel,TaskSpaceVelResponse


def Jacobian_cal(q):
    #import joint position
    l1 = 1
    l2 = 0.5
    l3 = 0.8
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    # calcualte transfer matrix

    T0_1 = [[np.cos(q1), -np.sin(q1), 0, l2*np.cos(q1)],
            [np.sin(q1),  np.cos(q1), 0, l2*np.sin(q1)],
            [0, 0, 1, l1],
            [0, 0, 0, 1]]
    
    T1_2 = [[np.cos(q2) , np.sin(q2) , 0 , l3*np.cos(q2)],
            [np.sin(q2) , -np.cos(q2) , 0 ,  l3*np.sin(q2)],
            [0 , 0, -1 , 0],
            [0 , 0 , 0 , 1]]

    T2_3 = [[1 , 0 , 0 , 0],
            [0 , 1 , 0 , 0],
            [0 , 0 , 1 , q3],
            [0 , 0 , 0 , 1]]
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
    J[0:3,1] = np.cross(z1_0,np.subtract(o_3,o_1))
    J[0:3,2] = z0_2
    J[3:,0] = z0_0
    J[3:,1] = z1_0
    J[3:,2] = np.array([0,0,0])

    return J

#calculate joint velocities
def taskspaceVel_Cal(obj):
    J = Jacobian_cal([q1,q2,q3])
    ee_vel = np.array([obj.ox,obj.oy,obj.oz]).reshape((3,1))
    q_dot = np.matmul(J,ee_vel)
    obj.veloutput = q_dot

    return TaskSpaceVelResponse(obj)

def TaskSpace_velocity_server():
     s = rospy.Service('TaskSpace_velocities_cal',TaskSpaceVel,eff_joint_velocity)


if __name__ == "__main__":
        rospy.init_node('taskvelcal')
        TaskSpace_velocity_server()

        rospy.spin()    
    

