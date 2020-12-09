#!/usr/bin/env python
import rospy
import csv
import numpy as np
from std_msgs.msg import Float32
from gazebo_msgs.srv import GetJointProperties
from gazebo_msgs.srv import GetWorldProperties
from gazebo_msgs.srv import ApplyJointEffort
from gazebo_msgs.srv import JointRequest
from gazebo_msgs.srv import GetPhysicsProperties
from scara_kin.srv import Refpos, RefposResponse
#from scara_kin.msg import Output
#import time
#import math

#rosservice call /gazebo/get_joint_properties joint3
class pid_class():
    def __init__(self):
        self.get_joint_property = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)
        self.apply_joint_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort',ApplyJointEffort)
        self.get_world_property = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
        self.clear_effort = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)
        self.p_gain = 0
        self.d_gain = 0
        self.ref_pos = 0
        self.prev_err = 0
        self.prev_time = 10

        self.np_array = np.array([])
        # with open('plotdata.csv') as csvfile:
        #     a = csv.reader(plotdata, delimiter=' ', quotechar='|')
    def set_ref_pos(self):
        refpos_proxy = rospy.ServiceProxy('refposserver', Refpos)
        self.ref_pos1 = refpos_proxy[0]
        self.ref_pos2 = refpos_proxy[1]
        self.ref_pos3 = refpos_proxy[2]
        print(refpos_proxy)

    def get_time(self):
        temp = self.get_world_property()
        return temp.sim_time
    def get_pos(self):
        request = self.get_joint_property('joint3')
        return request.position[0]
    def set_gains(self, pg, dg):
        self.p_gain = pg
        self.d_gain = dg
    def control(self):
        curr_pos = self.get_pos()
        print(curr_pos)
        self.curr_time = self.get_time()
        curr_err = self.ref_pos - curr_pos
        p_term = self.p_gain*curr_err
        d_term = self.d_gain*(curr_err - self.prev_err)/(self.curr_time - self.prev_time)
        control_op = p_term + d_term
        self.apply_joint_effort('joint3', control_op, rospy.Time(0), rospy.Time(0,10000000))
        #self.clear_effort('joint3')
        self.prev_err = curr_err
        self.prev_time = self.curr_time
        
        self.np_array = np.append(self.np_array, [self.curr_time, self.ref_pos, curr_pos], axis=0)
        self.np_array.tofile("plotdata.csv", sep=",")
        # print(self.np_array)
       # print("hi mom!")

        # for row in a:
        #     print('curr_pos, self.ref_pos, self.curr_time'.join(row))

    #def plot(self, curr_pos, curr_time, refpos):
     #   for row in a:
      #      print('curr_pos, self.ref_pos, self.curr_time'.join(row))

def main():
    rospy.init_node('pidnode', anonymous=True)
    rospy.wait_for_service('/gazebo/get_joint_properties')
    rospy.wait_for_service('/gazebo/apply_joint_effort')
    rospy.wait_for_service('/gazebo/get_world_properties')
    rospy.wait_for_service('/gazebo/clear_joint_forces')
    rospy.wait_for_service('refposserver')
    pid = pid_class()
    pid.set_gains(375, 25)
    pid.set_ref_pos()
    while not rospy.is_shutdown():
        pid.control()

if __name__ == '__main__':
    try:
        main()
    except rospy.ServiceException:
        print('RIP')
        pass