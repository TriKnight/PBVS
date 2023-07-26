#!/usr/bin/env python3
# 1. roslaunch dense_grasp ur10e_bringup.launch robot_ip:=192.168.1.101
# 2. -------run service
# rosservice call /controller_manager/switch_controller "start_controllers: ['joint_group_vel_controller']
# stop_controllers: ['scaled_pos_joint_traj_controller']
# strictness: 2
# start_asap: false
# timeout: 0.0"
# 3. roslaunch dense_grasp ar_tracker_u10e.launch
# 4. rosrun dense_grasp PBS_AR.py
# 5. Set Speed on TeachPendant 20%
# 5. rosbag record joint_states ar_pose_marker joint_group_vel_controller/command tf 

from ar_track_alvar_msgs.msg import AlvarMarkers
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
from typing import Tuple
import spatialmath as sm

# Define the joint name of the UR10e robot.
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
class PBS_UR_AR():
    def __init__(self):
        #Start moving the robot
        rospy.init_node('PBS_UR_AR_tracking', anonymous=True)
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
        self.ar_pose =[0.0, 0.0, 0.0]

        # The offset in z axis from AR tag to UR end-effector
        self.offset_z = rospy.get_param("~offset_z", 0.7)
        # Set flag to indicate when the AR marker is visible
        self.target_visible = False
        # self.thread2 = threading.Thread(target=self.RRMC_control)
        # self.thread2.start() 
        self.ur10 = rp.models.UR10()
        
        ## ROS Subscriber Topic
        # Subscribe to the ar_pose_marker topic to get the image width and height
        self.ar_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.position_servoing)
        self.sub =  rospy.Subscriber('joint_states', JointState, self.joint_states_callback)

        ##ROS Publisher topic
        self.pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
        self.msg = Float64MultiArray()

        
        
        #Do some clean up on shutdown
        rospy.on_shutdown(self.clean_shutdown)
        ## ROS parameters
        self.rate = rospy.Rate(100) # Hz
        self.PBS_control()
        rospy.spin()

        

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

    def position_servoing(self, msg):
        # Pick off the first marker (in case there is more than one)
        try:
            marker = msg.markers[0]
            if not self.target_visible:
                rospy.loginfo("FOLLOWER is Tracking Target!")
            self.target_visible = True
        except:
            # If target is loar, stop the robot by slowing it
            self.ar_pose = [0.0, 0.0, 0.0]
            print("Not AR detect")
    
            if self.target_visible:
                rospy.loginfo("FOLLOWER LOST Target!")
                self.ar_pose = [0.0, 0.0, 0.0]
            self.target_visible = False
            
            return

        # Get the distance of the marker from the base
        ar_target_x = marker.pose.pose.position.x      
        # Get the displacement of the marker relative to the base
        ar_target_y = marker.pose.pose.position.y
        # get the displacement of the marker with location z
        ar_target_z = marker.pose.pose.position.z
        self.ar_pose = [ar_target_x -self.offset_z, ar_target_y, ar_target_z]
        # print("ar_x", ar_target_x)
        # print("ar_y", ar_target_y)
        # print("ar_z", ar_target_z +self.offset_z)

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
        #rospy.loginfo("Published a message.")
        self.pub.publish(self.msg)

    def angle_axis(self, T: np.ndarray, Td: np.ndarray) -> np.ndarray:
        """
        Returns the error vector between T and Td in angle-axis form.

        :param T: The current pose
        :param Tep: The desired pose

        :returns e: the error vector between T and Td
        """

        e = np.empty(6)

        # The position error
        e[:3] = Td[:3, -1] - T[:3, -1]

        R = Td[:3, :3] @ T[:3, :3].T

        li = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])

        if np.linalg.norm(li) < 1e-6:
            # If li is a zero vector (or very close to it)

            # diagonal matrix case
            if np.trace(R) > 0:
                # (1,1,1) case
                a = np.zeros((3,))
            else:
                a = np.pi / 2 * (np.diag(R) + 1)
        else:
            # non-diagonal matrix case
            ln = np.linalg.norm(li)
            a = math.atan2(ln, np.trace(R) - 1) * li / ln

        e[3:] = a
        return e
    
    def p_servo(self,
    Te: np.ndarray, Tep: np.ndarray, gain: np.ndarray, threshold: float = 0.1
) -> Tuple[np.ndarray, bool]:
        """
        Position-based servoing.

        Returns the end-effector velocity which will cause the robot to approach
        the desired pose.

        :param Te: The current pose of the end-effecor in the base frame.
        :type wTe: ndarray
        :param Tep: The desired pose of the end-effecor in the base frame.
        :type wTep: ndarray
        :param gain: The gain for the controller. A vector corresponding to each
            Cartesian axis.
        :type gain: array-like
        :param threshold: The threshold or tolerance of the final error between
            the robot's pose and desired pose
        :type threshold: float

        :returns v: The velocity of the end-effector which will casue the robot
            to approach Tep
        :rtype v: ndarray(6)
        :returns arrived: True if the robot is within the threshold of the final
            pose
        :rtype arrived: bool
        """

        # Calculate the pose error vector
        e = self.angle_axis(Te, Tep)

        # Construct our gain diagonal matrix
        k = np.diag(gain)

        # Calculate our desired end0effector velocity
        v = k @ e

        # Check if we have arrived
        arrived = True if np.sum(np.abs(e)) < threshold else False

        return v, arrived

    def PBS_control(self):
        ## Position Based Servoing
        #self.ar_pose = [0.0, 0.0, 0.6]
        # Specify the gain for the p_servo method
        # gain = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        #Tep = self.ur10.fkine(self.pose_np) * sm.SE3.Trans(0.0 , 0.0, -0.3) 
        # AR Convert
        # Tep = self.ur10.fkine(self.pose_np) * sm.SE3.Trans(self.ar_pose ) 
        #print(Tep)
        # Tep = Tep.A #Convert to numpy
        # Our desired end-effector velocity
        # A constant -0.05 m/s along the z-axis
        #ev = [0.05, -0.05, 0.00, 0.00, -0.05, 0.0]
        # Specify our desired end-effector velocity 0.1 rad/s
        # ev = [0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        #self.ar_pose = [0.0, 0.0, 0.0]
        
        while not rospy.is_shutdown():
            pos = self.ar_pose
            print("postion: ", pos)
            gain = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
            #Tep = self.ur10.fkine(self.pose_np) * sm.SE3.Trans(0.0 , 0.0, -0.3) 
            # AR Convert
            Tep = self.ur10.fkine(self.pose_np) * sm.SE3.Trans(pos) 
            #print(Tep)
            Tep = Tep.A #Convert to numpy
             # Work out the base frame manipulator Jacobian using the current robot configuration
            J = self.ur10.jacob0(self.pose_np)
            det_J = np.linalg.det(J)
            # The end-effector pose of the panda (using .A to get a numpy array instead of an SE3 object)
            Te = self.ur10.fkine(self.pose_np).A
            # use the pseudoinverse (the pinv method)
            J_pinv = np.linalg.pinv(J)
            # Calculate the required end-effector velocity and whether the robot has arrived
            ev, arrived = self.p_servo(Te, Tep, gain=gain, threshold=0.001)
            # Calculate the required joint velocities to achieve the desired end-effector velocity ev
            dq = J_pinv @ ev
            # Send velocity msgs to the real UR Robot
            self.msg.data = dq
            self.publish_msg()

        rospy.signal_shutdown("We are done here!")
    
    def clean_shutdown(self):
        # works better than the rospy.is_shutdown()
            rospy.loginfo("System is shutting down. Stopping robot...")
            self.msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.publish_msg()

if __name__ == '__main__':
    try:
        PBS_UR_AR()
    except rospy.ROSInterruptException:
        pass
