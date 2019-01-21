#!/usr/bin/env python

import urx
import logging
import roslib; roslib.load_manifest('ur_driver')

import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi

import rospy
import sys
import copy
#import moveit_commander
#from copy import deepcopy
import geometry_msgs.msg
import moveit_msgs.msg

from std_msgs.msg import Header,Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from ur_driver import *
from ur_msgs.msg import *
import time
from ur_driver.io_interface import*


rob = urx.Robot("192.168.1.102")
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q0 =[-1.517266575490133, -0.5613916555987757, -2.267642323170797, -1.8395794073687952, 1.5515729188919067, 1.2095236778259277]
Q1 = [-1.2767632643329065, -1.4142497221576136, -2.135697666798727, -1.1884120146380823, 1.5516568422317505, 1.958543300628662]
Q2 =[-1.3004844824420374, -1.2923091093646448, -2.309441630040304, -1.0976837317096155, 1.579627275466919, 1.958543300628662]
Q3 =[-1.2767632643329065, -1.4142497221576136, -2.135697666798727, -1.1884120146380823, 1.5516568422317505, 1.958543300628662]
Q4=[0.07947114109992981, -1.4164212385760706, -2.1351340452777308, -1.164600674306051, 1.5516448020935059, 1.958591341972351]
Q5=[0.09197967499494553, -1.694958511983053, -2.3545873800860804, -0.6664398352252405, 1.5477979183197021, 1.9625786542892456]
Q6=[0.07947114109992981, -1.4164212385760706, -2.1351340452777308, -1.164600674306051, 1.5516448020935059, 1.958591341972351]
inp = False
client = None

from ur_msgs.srv import *
from ur_msgs.msg import *

FUN_SET_DIGITAL_OUT = 1
FUN_SET_FLAG = 2
FUN_SET_ANALOG_OUT = 3
FUN_SET_TOOL_VOLTAGE = 4

#Flag_States = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
Digital_Out_States = [0,0,0,0,0,0,0,0,0,0]  #8(controller)+2(tool)
Digital_In_States = [0,0,0,0,0,0,0,0,0,0]   #8(controller)+2(tool)
Analog_Out_States = [0,0]  #2(controller)
Analog_In_States = [0,0]   #2(controller)+0(tool)

ANALOG_TOLERANCE_VALUE = 0.01

def set_digital_out(pin, val):
    try:
        set_io(FUN_SET_DIGITAL_OUT, pin, val)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def callback(data):
    rospy.logerr("Flag_States are currently not supported")
    #for i in range(0,32):
        #del Flag_States[i]
        #Flag_States.insert(i, data.flag_states[i].state)
    for i in range(0,10):
        del Digital_Out_States[i]
        Digital_Out_States.insert(i, data.digital_out_states[i].state)
    for i in range(0,10):
        del Digital_In_States[i]
        Digital_In_States.insert(i, data.digital_in_states[i].state)
    for i in range(0,2):
        del Analog_Out_States[i]
        Analog_Out_States.insert(i, data.analog_out_states[i].state)
    rospy.logerr("ToolInput analog_in[2] and analog_in[3] currently not supported")
    for i in range(0,2):
        del Analog_In_States[i]
        Analog_In_States.insert(i, data.analog_in_states[i].state)

def get_states():
    rospy.init_node('UR_State_Getter')
    rospy.Subscriber("io_states", IOStates, callback)
    
def set_states():
    rospy.wait_for_service('/ur_driver/set_io')
    global set_io
    set_io = rospy.ServiceProxy('/ur_driver/set_io', SetIO)

def move_home():
    client.cancel_goal()
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        d = 2.0
        g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q0, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def move_repeated():
    client.cancel_goal()
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        d = 2.0
        g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q0, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def move_repeated1():
    client.cancel_goal()
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        d = 2.0
        g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q5, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q6, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def move_vision_place():
    client.cancel_goal()
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        d = 3.0
        g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]

        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2
	g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q4, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def cart():
    global x
    global y
    global z
    global bakleeg
   
    pose = (rob.getl())
    print (rob.getl())
    pose1 = pose
    bakleeg = False
    if (x == 0.0):
	if (y == 0.0):
	    if (z == 0.0 ):
		xas = 0 
		yas = 0 
		zas = 0
		bakleeg = True
		
    else:
        xas = x
        yas = -y
        zas = -z+0.451
	bakleeg = False
    v =0.07
    a = 0.1
    r =0.05

    pose1[0] += xas
    pose1[1] += yas
    pose1[2] += zas
    rob.movel(pose1, acc=a, vel=v, wait=False)
    while True:
        p = rob.getl(wait=True)
        if p[2] > pose[2] - 0.0005:
            break



def IOStates_callback(msg):
    global inp
    if (msg.digital_in_states[1].state == True):
        inp = True
        print inp
    if (msg.digital_in_states[0].state == True):
        inp = False
        print inp

def Location_callback(msg):
    global x
    global y
    global z
    global Found
    x = msg.data[0]
    y = msg.data[1]
    z = msg.data[2]   
    Found = True

def main():
    global inp
    global x
    global y
    global z
    global Found
    global client
    global begin
    global bakleeg

    try:
	get_states()
	set_states()
	#rospy.init_node("test_move", anonymous=True, disable_signals=True)
        rospy.Subscriber("/ur_driver/io_states", IOStates, IOStates_callback)
        rospy.Subscriber("/LocationArray", Float64MultiArray, Location_callback)
        pub = rospy.Publisher('StartVision', std_msgs.msg.String, queue_size=10)

        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        print "This program makes the robot move between the following three poses:"
        print str([Q0[i]*180./pi for i in xrange(0,6)])
        print str([Q1[i]*180./pi for i in xrange(0,6)])
        print str([Q2[i]*180./pi for i in xrange(0,6)])
        print str([Q3[i]*180./pi for i in xrange(0,6)])
        print str([Q4[i]*180./pi for i in xrange(0,6)])
        print str([Q5[i]*180./pi for i in xrange(0,6)])
        print str([Q6[i]*180./pi for i in xrange(0,6)])
        print "Please make sure that your robot can move freely between these poses before proceeding!"
        #inp = raw_input("Continue? y/n: ")[0]
        print inp		
        #rospy.spin()
	set_digital_out(0, False)
	set_digital_out(1, False)
	set_digital_out(2, True)
        while (True):
	    begin = True
	    Found = False

            if (inp == True):
                while (begin == True):
		    set_digital_out(0, True)
		    set_digital_out(2, False)
		    set_digital_out(1, False)
		    move_home()
		    print "1"                
		    pub.publish(std_msgs.msg.String("TRUE"))
		    
		    print "2"
		    while (Found == False):
			time.sleep(0.01)
		    move_repeated()
		    Found = False
		    cart()	

		    if (bakleeg == True):
			set_digital_out(0, False)
			set_digital_out(1, True)
			while (inp == True):
				time.sleep(0.01)
			set_digital_out(2, True)
			set_digital_out(1, False)			
			move_home()
			while (inp == False):
				time.sleep(0.01)
				
		    	break 
			
                    time.sleep(3)
                    move_vision_place()
                    #while (True):
			#time.sleep(5)
		    pub.publish(std_msgs.msg.String("TRUE"))
                    move_repeated1()
 		    
		    begin = False
		while (begin == False and inp == True):
		     set_digital_out(0, True)
		     set_digital_out(2, False)
		     set_digital_out(1, False)		     
		     move_home()
		     move_repeated()
		     while (Found == False):
			 time.sleep(0.01)	
		     Found = False                
		     cart()
		     if (bakleeg == True):
			 set_digital_out(0, False)
			 set_digital_out(1, True)
	      	    	 while (inp == True):
			 	 time.sleep(0.01)
			 set_digital_out(1, False)
		         set_digital_out(2, True)		    	 
			 move_home()
		    	 while (inp == False):
				 time.sleep(0.01)
			 begin = True
			 break
 		     time.sleep(4)
                     move_vision_place()
		     pub.publish(std_msgs.msg.String("TRUE"))
		     move_repeated1()
      	      	set_digital_out(0, False)
	      	set_digital_out(1, False)
	      	set_digital_out(2, True)

    except KeyboardInterrupt:
    #rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()



