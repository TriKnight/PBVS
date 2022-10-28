#!/usr/bin/env python3

########### Resolve Rate Controller testing on UR Robot ###################
# ---------------------------------------------------------------------
# This code will test the velocity controller of the UR Robot 
# To run the code please switch controller from position controller to velocity controller 
# The velocity controller message is:  /joint_group_vel_controller/command 
# Author: Tri Knight
# ---------------------------------------------------------------------
# Before running this code please run the switch controller bellow
# ---------------Disable the robot Ethernet/IP Adapter ------------------------
# In the PolyScope/Installation/Fieldbus/Ethernet-IP --> Disable
# ---------------Start the robot --------------------------------------
# roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.1.101 
# --------------- Change to veocity controller ----------------------------------------
# rosservice call /controller_manager/switch_controller "start_controllers: ['joint_group_vel_controller']
# stop_controllers: ['scaled_pos_joint_traj_controller']
# strictness: 2
# start_asap: false
# timeout: 0.0"
# ---------------------------------------------------------------------
#import package Robotics ToolBox
import roboticstoolbox as rp
from spatialmath import *
# spatialgeometry is a utility package for dealing with geometric objects
import spatialgeometry as sg
# the Python math library
import math
#import package ROS
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
import time 
from sensor_msgs.msg import JointState
import threading
import math

# Define the joint name of the UR10e robot.
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
class RRMC_control_UR():
    def __init__(self):
        #Start moving the robot
        rospy.init_node('RMMC_Control_UR', anonymous=True)
        self.lock = threading.Lock()
        self.name = []
        self.position = []
        self.lastposition = []
        self.velocity = []
        self.lastvelocity = []
        self.effort = []
        self.thread1 = threading.Thread(target=self.joint_states_listener)
        self.thread1.start() 
        self.pose_np =[]
        # self.thread2 = threading.Thread(target=self.RRMC_control)
        # self.thread2.start() 
        self.ur10 = rp.models.UR10()
        
        
        ##ROS Publisher topic
        self.pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
        self.msg = Float64MultiArray()

        ## ROS Subscriber Topic
        self.sub =  rospy.Subscriber('joint_states', JointState, self.joint_states_callback)
        
        #Do some clean up on shutdown
        rospy.on_shutdown(self.clean_shutdown)
        ## ROS parameters
        self.rate = rospy.Rate(100) # Hz
        self.RRMC_control()
        rospy.spin()
        

        
        # do some cleanup on shutdown

        ##Parameter for listening the joint
         

    ## Subscriber Part
    #pretty-print list to string
    def pplist_deg(self,list):
        #return ' '.join(['%2.3f'%x for x in list])
        return ' '.join(['%2.3f'%math.degrees(x) for x in list])

    def pplist_rad(self,list):
        #return ' '.join(['%2.3f'%x for x in list])
        return ' '.join(['%2.3f'%x for x in list])

    def joint_states_callback(self, msg):
        self.lock.acquire()
        self.name = msg.name
        self.position = msg.position
        self.velocity = msg.velocity
        self.effort = msg.effort
        self.lock.release()
        ## print velocity
        # if(self.lastvelocity!=self.velocity):
        #     print("velocity in deg:", self.pplist(self.velocity))
        # self.lastvelocity = self.velocity
        ## Print position
        
        if(self.lastposition!=self.position):
            #easy to see result, print the presision 2 decimal, and suppress small result
            np.set_printoptions(precision=2, suppress=True)
            self.pose_np = self.convert_real_joint(
                           np.fromstring(self.pplist_rad(self.position), dtype=float, sep=' '))
        self.lastposition=self.position

    def convert_real_joint(self, old_pos):
        # From: <elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint>
        # Convert to: <shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint>
        newpose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        np_old_pose = np.array(old_pos)
        newpose[0] = np_old_pose[2]
        newpose[2] = np_old_pose[0]
        newpose[1] = np_old_pose[1]
        newpose[3] = np_old_pose[3]
        newpose[4] = np_old_pose[4]
        newpose[5] = np_old_pose[5]
        return newpose

    #thread function: listen for joint_states messages
    def joint_states_listener(self):
        while not rospy.is_shutdown():
            rospy.Subscriber('joint_states', JointState, self.joint_states_callback)
            rospy.spin()
        rospy.signal_shutdown("We are done here!")

    ## Publisher Part
    def publish_msg(self):
        while self.pub.get_num_connections() <1:
            rospy.loginfo("Waiting for connection to publisher...")
            time.sleep(1)
        rospy.loginfo("Published a message.")
        self.pub.publish(self.msg)
    
    def RRMC_control(self):
        ## Resolved Rate Motion Control
        # Our desired end-effector velocity
        # A constant -0.05 m/s along the z-axis
        ev = [0.05, -0.05, 0.00, 0.00, -0.05, 0.0]
        # Specify our desired end-effector velocity 0.1 rad/s
        # ev = [0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        while not rospy.is_shutdown():
            J = self.ur10.jacob0(self.pose_np)
            det_J = np.linalg.det(J)

            # Calculate the required joint velocities to achieve the desired end-effector velocity ev
            dq = np.linalg.pinv(J) @ ev
            # Send velocity msgs to the real UR Robot
            self.msg.data = dq
            self.publish_msg()

            # self.rate.sleep()
            # rospy.sleep(1) # Delay 1s

            #Print result  
            print('Sub joint pos: ',self.pose_np)
            print('Jacobian matrix:\n', J)
            #print('det of the Jacobian matrix:', det_J)  
            print('publish joint vel:', np.round(dq, 4)) 
        rospy.signal_shutdown("We are done here!")
    
    def clean_shutdown(self):
        # works better than the rospy.is_shutdown()
            rospy.loginfo("System is shutting down. Stopping robot...")
            self.msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.publish_msg()

if __name__ == '__main__':
    try:
        RRMC_control_UR()
    except rospy.ROSInterruptException:
        pass