#!/usr/bin/env python
import rospy
import numpy as np
import sys
from scara_kin.srv import *
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
from std_msgs.msg import Float64
import pid

def handle_joint_ref(req):
    # Publish once with gazebo PID controller
    if req.joint_name == 'joint1':
        pub1.publish(req.ref)
        return SetJointRefResponse(True)
    elif req.joint_name == 'joint2':
        pub2.publish(req.ref)
        return SetJointRefResponse(True)
    elif req.joint_name == 'joint3':
        pub3.publish(req.ref)
        return SetJointRefResponse(True)

    else:
        rospy.loginfo("Input Joint Name Invalid")
        return SetJointRefResponse(False)


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

# sent effort to joint
def set_joint_effort(joint,effort,start,duration):
    rospy.wait_for_service('/gazebo/apply_joint_effort')
    try:
        set_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        success = set_effort(joint,effort,start,duration)
        return success
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

#clear effort in joint
def clear_joint_effort(joint):
    rospy.wait_for_service('/gazebo/clear_joint_forces')
    try:
        clear_effort = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)
        success = clear_effort(joint)
        return success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# get joint position function
def Get_joint_position():
    rospy.init_node('gazebo_get', anonymous=True)
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
    q = [q1,q2,q3]
    return(q)

def velocity_controller(ref_q_dot,q_dot,error_start,start_times,s_time):
    #define kp and kd
    duration = rospy.Time(1)
    kp = np.diag([500,500,500])
    kd = np.diag([1,1,1])
    time = rospy.get_time()
    error = ref_q_dot - q_dot
    e_dot = (error - error_start)/(time-start_time)
    effort = np.dot(kp,error)+np.dot(kd,e_dot)
    clear_joint_effort('joint1')
    clear_joint_effort('joint2')
    clear_joint_effort('joint3')
    set_joint_effort('joint1',effort[0],s_time,duration)
    set_joint_effort('joint2',effort[1],s_time,duration)
    set_joint_effort('joint3',effort[2],s_time,duration)

    return error


if __name__ == "__main__":
    #get joint position:
    q = Get_joint_position()
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    #velocity inpput
    y_velocity = float(input("velocity in positive y direction: "))
    #reference velocity vel (end effector)
    vel = [0,y_velocity,0,0,0,0]
    ee_dot = np.zeros((3,1))
    q_dot = np.array([0,0,0]).reshape((3,1))
    #convert reference velocity into joint spaze
    input_q = [q1,q2,q3,vel[0],vel[1],vel[2],vel[3],vel[4],vel[5]]
    Joint_velocities = Joint_velocity_calculation_client(q1,q2,q3,vel[0],vel[1],vel[2],vel[3],vel[4],vel[5])
    rev_qdot = np.array(Joint_velocities.q_dot).reshape((3,1))
    #input controller
    #pid = pid.pid_class()
    #joint1 and 2 gain are 30 and 8, joint 3 gain are 375 and 25
    #pid.set_gains(30, 8, 375, 25)

    ## controlling velocity
    start_time = rospy.get_time()
    start_time1 = start_time
    duration = rospy.Time(2)
    s_time = rospy.Time(0)
    q_start_position = np.array(q).reshape((3,1))
    #define the error from the beginning
    error_start = rev_qdot - q_dot
    #setting up controller
    time_bar = []
    q_previous = Get_joint_position()
    q_previous = np.array(q_previous).reshape((3,1))
    while error_start[0] > 0.000001 or error_start[1] > 0.0000001 or error_start[2] > 0.000001:
        q_position = Get_joint_position()
        q_position = np.array(q_position).reshape((3,1))
        time = rospy.get_time()
        q1 = q_position[0]
        q2 = q_position[1]
        q3 = q_position[2]
        Joint_velocities = Joint_velocity_calculation_client(q1,q2,q3,vel[0],vel[1],vel[2],vel[3],vel[4],vel[5])
        ref_q_dot = np.array(Joint_velocities.q_dot).reshape((3,1))
        q = q_position - q_start_position
        #actual velocity
        q_dot = np.array((q_position - q_previous))/(time-start_time)
        #PD controller
        e = velocity_controller(ref_q_dot,q_dot,error_start,start_time,s_time)
        #start_time = time - start_time1
        error_start = e
        q_previous = q_position 
        #output velocity ([vx,vy,vz,wx,wy,wz])
        v_actual = task_space_velcal_client(q1[0],q2[0],q3[0],q_dot[0],q_dot[1],q_dot[2])
        v_ref = task_space_velcal_client(q1[0],q2[0],q3[0],ref_q_dot[0],ref_q_dot[1],ref_q_dot[2])
        #actual velocity list
        actual_velocity.append(v_actual)
        #reference velocity list
        reference_velocity.append(v_ref)
        #time list
        Time.append(time)

        
        
with open('Actual_velocity', 'w') as f:
    for x in actual_velocity:
        f.write("%s\n" % x)
with open('Reference_velocity', 'w') as f:
    for x in reference_velocity:
        f.write("%s\n" % x)
with open('Time', 'w') as f:
    for x in Time:
        f.write("%s\n" % x)



        





        




    
 
    #calculate joint velovity
    #user input velocity of end effector
    









        





