#!/usr/bin/env python3

########### Position Based Servoing Resolve Rate Controller testing on UR Robot ###################
# ---------------------------------------------------------------------
# This code will test the velocity controller of the UR Robot 
# To run the code please switch controller from position controller to velocity controller 
# The velocity controller message is:  /joint_group_vel_controller/command 
# Author: Tri Knight
# ---------------------------------------------------------------------
# Before running this code please run the switch controller bellow
# ---------------Disable the robot Ethernet/IP Adapter ------------------------
# In the PolyScope/Installation/Fieldbus/Ethernet-IP --> Disable
# ---------------1. Start the robot --------------------------------------
# roslaunch dense_grasp ur10e_bringup.launch robot_ip:=192.168.1.201
# --------------- 2. Change to veocity controller ----------------------------------------
# rosservice call /controller_manager/switch_controller "start_controllers: ['joint_group_vel_controller']
# stop_controllers: ['scaled_pos_joint_traj_controller']
# strictness: 2
# start_asap: false
# timeout: 0.0"
# ---------------------------------------------------------------------
# --------------- 3. View in RViz --------------------------------------
# roslaunch ur10e_rg2_description real_ur10_rviz.launch 

#import package Robotics ToolBox
import swift
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
import qpsolvers as qp
import cProfile
from spatialmath.pose3d import SE3
import math as m 
# Define the joint name of the UR10e robot.
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
class NEO_control_UR():
    def __init__(self):
        #Start moving the robot
        rospy.init_node('NEO_Control_UR', anonymous=True)
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
        self.rate = rospy.Rate(10) # Hz
        #Run NEO Simulation
        
        self.NEO_control()
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
    

    def NEO_control(self):
        #Open Simulation
        #------------------------------------------------
        # Launch the simulator Swift
        env = swift.Swift()
        env.launch()

     

        # Set joint angles to ready configuration
        self.ur10.q = [0.0, -1/2*m.pi, m.pi/2 , -m.pi/2, -m.pi/2, 0]

        # Number of joint in the panda which we are controlling
        n = 6
        # Collisions
        ## s1: Base plate
        s1 = sg.Cuboid(
            scale=[0.60, 1.2, 0.02],
            pose=sm.SE3(0.8, 0, 0.0))

        # s2, s3 boundary plate front 1 and 2 robot
        s2 = sg.Cuboid(
            scale=[0.60, 0.02, 0.4],
            pose=sm.SE3(0.8, 0.6, 0.2))

        s3 = sg.Cuboid(
            scale=[0.60, 0.02, 0.4],
            pose=sm.SE3(0.8, -0.6, 0.2))

        # s4, s5 boundary plate left and right robot
        s4 = sg.Cuboid(
            scale=[0.02, 1.2, 0.4],
            pose=sm.SE3(0.5, 0, 0.2))

        s5 = sg.Cuboid(
            scale=[0.02, 1.2, 0.4],
            pose=sm.SE3(1.1, 0, 0.2))

        ## s6: Center plate
        s6 = sg.Cuboid(
            scale=[0.60, 0.02, 0.32],
            pose=sm.SE3(0.8, 0, 0.2))

        ## s7: Cylinder 
        # s7 = sg.Cylinder(
        #     radius =0.02, length =0.6,
        #     pose=sm.SE3(0.8, 0, 0.28)*sm.SE3.RPY([0.0, m.pi/2, 0 ]))

        ## s7: Sphere
        s7 = sg.Sphere(radius =0.07,
        pose=sm.SE3(0.55, 0, 0.35))
        s7.v = [0.01, 0.00, 0.0, 0.1, 0.1, 0.1]

        collisions = [s7]

        # Make a target
        target_1 = sg.Sphere(radius =0.01, pose=sm.SE3(0.7, -0.3, 0.40))
        target_2 = sg.Sphere(radius =0.01, pose=sm.SE3(0.7, 0.3, 0.40))
        # Add the Panda and shapes to the simulator
        env.add(self.ur10)
        env.add(s1)
        env.add(s2)
        env.add(s3)
        env.add(s4)
        env.add(s5)
        #env.add(s6)
        env.add(s7)
        env.add(target_1)
        env.add(target_2)   
        # Set the desired end-effector pose to the location of target
        Home = self.ur10.fkine(self.pose_np)
        #Tep_1 = self.ur10.fkine(self.ur10.q)
        Tep_1 = self.ur10.fkine(self.pose_np)
        Tep_1.A[:3, 3] = target_1.T[:3, 3] # get only position x y z
      
        #Tep_2 = self.ur10.fkine(self.ur10.q)
        Tep_2 = self.ur10.fkine(self.pose_np)
        Tep_2.A[:3, 3] = target_2.T[:3, 3] # get only position x y z


        #Show axis
        # end-effector axes
        ee_axes = sg.Axes(0.1)
        # This pose can be either a spatialmat SE3 or a 4x4 ndarray
        #ee_axes.T = self.ur10.fkine(self.ur10.q)
        ee_axes.T = self.ur10.fkine(self.pose_np)
        # goal axes 1
        goal_axes_1 = sg.Axes(0.1)
        goal_axes_1.T = Tep_1
        # goal axes 1=2
        goal_axes_2 = sg.Axes(0.1)
        goal_axes_2.T = Tep_2
        # Add the axes to the environment
        env.add(ee_axes)
        env.add(goal_axes_1) 
        env.add(goal_axes_2) 
        #----------------------------------------
        # Number of joint in the panda which we are controlling
        n = 6
        # Specify the gain for the p_servo method
        gain = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        dt = 0.01

        while not rospy.is_shutdown():
            def move(Tep):
                arrived = False
                while not arrived:
                    # Work out the base frame manipulator Jacobian using the current robot configuration
                    # J = self.ur10.jacob0(self.pose_np)
                    # det_J = np.linalg.det(J)
                    # Real ROBOT: The end-effector pose of the panda (using .A to get a numpy array instead of an SE3 object)
                    Te = self.ur10.fkine(self.pose_np)
                    # Transform from the end-effector to desired pose
                    ## Simulation robot
                    # The pose of the UR10e's end-effector
                    #Te = self.ur10.fkine(self.ur10.q)
                    eTep = Te.inv() * Tep
                     # Spatial error
                    e = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi / 180]))
                    # Calulate the required end-effector spatial velocity for the robot
                    # to approach the goal. Gain is set to 1.0
                    # Specify the gain for the p_servo method
                    gain = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
                    # Calulate the required end-effector spatial velocity for the robot
                    # to approach the goal. Gain is set to 1.0
                    v, arrived = rp.p_servo(Te, Tep, gain=gain, threshold=0.01)
                    # Gain term (lambda) for control minimisation
                    Y = 0.01
                     # Quadratic component of objective function
                    Q = np.eye(n + 6)

                    # Joint velocity component of Q
                    Q[:n, :n] *= Y

                    # Slack component of Q
                    Q[n:, n:] = (1 / e) * np.eye(6)

                    # The equality contraints
                    # Aeq = np.c_[self.ur10.jacobe(self.ur10.q), np.eye(6)]
                    # beq = v.reshape((6,))
                    # Real Robot
                    Aeq = np.c_[self.ur10.jacobe(self.pose_np), np.eye(6)]
                    beq = v.reshape((6,))

                    # The inequality constraints for joint limit avoidance
                    Ain = np.zeros((n + 6, n + 6))
                    bin = np.zeros(n + 6)   

                    # The minimum angle (in radians) in which the joint is allowed to approach
                    # to its limit
                    ps = 0.2

                    # The influence angle (in radians) in which the velocity damper
                    # becomes active
                    pi = 0.8

                    # Form the joint limit velocity damper
                    Ain[:n, :n], bin[:n] = self.ur10.joint_velocity_damper(ps, pi, n)

                    # For each collision in the scene
                    for collision in collisions:

                        # Form the velocity damper inequality contraint for each collision
                        # object on the robot to the collision in the scene
                        c_Ain, c_bin = self.ur10.link_collision_damper(
                            collision,
                            #self.ur10.q[:n],
                            self.pose_np, 
                            0.8,
                            0.06,
                            1.5,
                            start=self.ur10.link_dict["shoulder_link"],
                            end=self.ur10.link_dict["ee_link"],
                        )

                        # If there are any parts of the robot within the influence distance
                        # to the collision in the scene
                        if c_Ain is not None and c_bin is not None:
                            c_Ain = np.c_[c_Ain, np.zeros((c_Ain.shape[0], 6))]

                            # Stack the inequality constraints
                            Ain = np.r_[Ain, c_Ain]
                            bin = np.r_[bin, c_bin]

                    # Linear component of objective function: the manipulability Jacobian
                    # c = np.r_[-self.ur10.jacobm(self.ur10.q).reshape((n,)), np.zeros(6)]

                    # Real Robot Linear component of objective function: the manipulability Jacobian
                    c = np.r_[-self.ur10.jacobm(self.pose_np).reshape((n,)), np.zeros(6)]

                    # The lower and upper bounds on the joint velocity and slack variable
                    UR10e_qdlim = [0.5, 0.5, 0.5,  0.5, 0.5,  0.5]
                    lb = -np.r_[UR10e_qdlim, 10 * np.ones(6)]
                    ub = np.r_[UR10e_qdlim, 10 * np.ones(6)]

                    # Solve for the joint velocities dq
                    qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver='osqp')

                    # Send velocity msgs to the real UR Robot
                    self.msg.data = qd[:n]
                    self.publish_msg()

                    # self.rate.sleep()
                    # rospy.sleep(1) # Delay 1s

                    #SImulation: Send the velocity signal to the simulation robot
                    self.ur10.q = self.pose_np
                    env.step(dt)
                    #Print result  
                    print('Sub joint pos: ',self.pose_np)

                    #print('det of the Jacobian matrix:', det_J)  
                    print('publish joint vel:', np.round(qd, 4)) 

            #Robot Move Tep_1
            move(Tep_1)
            #Robot Move Tep_2
            move(Tep_2)
            #Robot Move Tep_1
            move(Tep_1)
            #Robot Move Tep_2
            move(Tep_2)  
         
            rospy.signal_shutdown("We are done here!")
    def clean_shutdown(self):
        # works better than the rospy.is_shutdown()
            rospy.loginfo("System is shutting down. Stopping robot...")
            self.msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.publish_msg()

if __name__ == '__main__':
    try:
        NEO_control_UR()
    except rospy.ROSInterruptException:
        pass