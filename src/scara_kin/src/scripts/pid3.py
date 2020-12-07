#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetJointProperties
from gazebo_msgs.srv import GetWorldProperties
from gazebo_msgs.srv import ApplyJointEffort
from gazebo_msgs.srv import JointRequest
from gazebo_msgs.srv import GetPhysicsProperties
#from scara_kin.msg import Output
#import time
#import math

#rosservice call /gazebo/get_joint_properties joint3
class pid_class():
    def __init__(self):
        rospy.wait_for_service('/gazebo/get_joint_properties')
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        rospy.wait_for_service('/gazebo/get_world_properties')
        rospy.wait_for_service('/gazebo/clear_joint_forces')
        self.get_joint_property = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)
        self.apply_joint_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort',ApplyJointEffort)
        self.get_world_property = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
        self.clear_effort = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)
        self.clear_effort('joint1')
        self.clear_effort('joint2')
        self.clear_effort('joint3')
        self.p_gain1 = 0
        self.p_gain2 = 0
        self.d_gain1 = 0
        self.d_gain2 = 0
        self.ref_pos1 = 0
        self.ref_pos2 = 0
        self.ref_pos3 = 0
        self.prev_err1 = 0
        self.prev_err2 = 0
        self.prev_err3 = 0
        self.prev_time = 10
    def set_ref_pos(self, refpos1, refpos2, refpos3):
        self.ref_pos1 = refpos1
        self.ref_pos2 = refpos2
        self.ref_pos3 = refpos3
    def get_time(self):
        temp = self.get_world_property()
        return temp.sim_time
    def get_pos(self):
        request1 = self.get_joint_property('joint1')
        request2 = self.get_joint_property('joint2')
        request3 = self.get_joint_property('joint3')
        req = [request1.position[0], request2.position[0], request3.position[0]]
        return req
    def set_gains(self, pg1, dg1, pg2, dg2): # first two gains are for joint1&2, last two for joint3
        self.p_gain1 = pg1
        self.d_gain1 = dg1
        self.p_gain2 = pg2
        self.d_gain2 = dg2
    def control(self):
        curr_pos = self.get_pos()
        self.curr_time = self.get_time()
        curr_err1 = self.ref_pos1 - curr_pos[0]
        curr_err2 = self.ref_pos2 - curr_pos[1]
        curr_err3 = self.ref_pos3 - curr_pos[2]
        p_term1 = self.p_gain1*curr_err1
        d_term1 = self.d_gain1*(curr_err1 - self.prev_err1)/(self.curr_time - self.prev_time)
        p_term2 = self.p_gain1*curr_err2
        d_term2 = self.d_gain1*(curr_err2 - self.prev_err2)/(self.curr_time - self.prev_time)
        p_term3 = self.p_gain2*curr_err3
        d_term3 = self.d_gain2*(curr_err3 - self.prev_err3)/(self.curr_time - self.prev_time)
        control_op1 = p_term1 + d_term1
        control_op2 = p_term2 + d_term2
        control_op3 = p_term3 + d_term3
        self.apply_joint_effort('joint1', control_op1, rospy.Time(0), rospy.Time(0,25000000))
        self.apply_joint_effort('joint2', control_op2, rospy.Time(0), rospy.Time(0,25000000))
        self.apply_joint_effort('joint3', control_op3, rospy.Time(0), rospy.Time(0,25000000))
        self.prev_err1 = curr_err1
        self.prev_err2 = curr_err2
        self.prev_err3 = curr_err3
        self.prev_time = self.curr_time

def main():
    rospy.init_node('pidnode', anonymous=True)
    pid = pid_class()
    pid.set_gains(30, 8, 375, 25)
    # For joint1&2 gains are 10, 1 and joint3 gains are 375, 25
    pid.set_ref_pos(4, 4, 0.25)
    while(1):
        pid.control()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass