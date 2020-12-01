#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetJointProperties
from pyquaternion import Quaternion

def main():
    rospy.init_node('Forwardk', anonymous=True)
    rospy.wait_for_service('/gazebo/get_joint_properties')
    get_joint_property = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)    #call ros service get_joint_properties
    request1 = get_joint_property('joint1')
    get_joint_property = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)
    request2 = get_joint_property('joint2')
    get_joint_property = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)
    request3 = get_joint_property('joint3')
    link1 = 1
    link2 = 0.5
    link3 = 0.8
    l1 = link1
    l2 = link2
    l3 = link3
    q1 = request1.position[0]
    q2 = request2.position[0]
    q3 = request3.position[0]

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
    T0_3 = np.dot(T0_2,T2_3)            #calculate forward kinematics
    x = T0_3[0][3]
    y = T0_3[1][3]
    z = T0_3[2][3]
    pose = [x,y,z]
    T0_3 = np.array(T0_3)
    quat1 = Quaternion(matrix = T0_3)    #calculate quaternion
    i = quat1[0]
    j = quat1[1]
    k = quat1[2]
    w = quat1[3]
    quat = [i, j, k, w]                  #quaternion
    print("The Forward kinematics is: ", T0_3)
    print("The End effector is: ", pose)
    print("The quaternion is: ", quat1)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass
    