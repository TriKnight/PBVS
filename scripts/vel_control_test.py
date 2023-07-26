#!/usr/bin/env python3

########### Velocity controller testing on UR Robot ###################
# ---------------------------------------------------------------------
# This code will test the velocity controller of the UR Robot 
# To run the code please switch controller from position controller to velocity controller 
# The velocity controller message is:  /joint_group_vel_controller/command 
# Author: Tri Knight
# ---------------------------------------------------------------------
# Before running this code please run the switch controller bellow
# ---------------------------------------------------------------------
# rosservice call /controller_manager/switch_controller "start_controllers: ['joint_group_vel_controller']
# stop_controllers: ['scaled_pos_joint_traj_controller']
# strictness: 2
# start_asap: false
# timeout: 0.0"
# ---------------------------------------------------------------------
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import time 

# Define the joint name of the UR10e robot.
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
class Vel_control_UR():
    def __init__(self):
         self.pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
         self.msg = Float64MultiArray()
         #Do some clean up on shutdown
         rospy.on_shutdown(self.clean_shutdown)

         #Start moving the robot
         rospy.init_node('vel_control_UR', anonymous=True)
         self.rate = rospy.Rate(10) # 10hz
         self.vel_control()
         rospy.spin()
         # do some cleanup on shutdown
         

    def publish_msg(self):
        while self.pub.get_num_connections() <1:
            rospy.loginfo("Waiting for connection to publisher...")
            time.sleep(1)
        rospy.loginfo("Published a message.")
        self.pub.publish(self.msg)
    
    def vel_control(self):
        while not rospy.is_shutdown():
            self.msg.data = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
            self.publish_msg()
            self.rate.sleep()
        rospy.signal_shutdown("We are done here!")
    
    def clean_shutdown(self):
        # works better than the rospy.is_shutdown()
            rospy.loginfo("System is shutting down. Stopping robot...")
            self.msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.publish_msg()

if __name__ == '__main__':
    try:
        Vel_control_UR()
    except rospy.ROSInterruptException:
        pass