#!/usr/bin/env python3

########### Basic Move UR Robot ###################
# ---------------------------------------------------------------------
# This code use Ros Action help the UR robot move to desirable goal.
# Author: Tri Knight
# ---------------------------------------------------------------------

import rospy
import numpy as np
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
#import Cartesian control


# Define the joint name of the UR10e robot.
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
# The scaled versions should be preferred over the non-scaled versions.
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller", # for the real-robot
    "scaled_vel_joint_traj_controller", # for the real-robot
    "pos_joint_traj_controller", # for the sim-robot
    "vel_joint_traj_controller", # for the sim-robot
]

class RobotTrajectory:
    """ This is test the joint robot trajectories in simulation and real robot"""
    
    def __init__(self):
        rospy.init_node("test_trajectories")
        # For simulation robot
        # self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[2]
        # For real robot 
        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]

    #This function is convert from deg to rad
    def deg2rad(self, deg):
        rad = np.radians(deg)
        return rad

    # send the joint trajectorier
    def send_joint_trajectory(self):
        """ Creates a trajectories and send it to the robot"""
        
        #this is for sim robot
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )
         # Wait for the server to start up and start listening for goals.
        trajectory_client.wait_for_server()
        #create and fill trajectory the goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        #Create the list of the joint angles in degree
        position_list = [[-73, -106, 99, -77, -87, 0]]
        position_list.append([-120, -106, 105, -88, -88, 0])
        position_list.append([-73, -106, 99, -77, -87, 0])
        position_list.append([-110, -90, 95, -95, -82, 0])
        position_list.append([-73, -106, 99, -77, -87, 0])
        position_list.append([-120, -106, 105, -88, -88, 0])
        #Convert the list of the joint angles to radian
        positon_list_rad = self.deg2rad(position_list)
        

        #Time to translation between each position in second
        duration_list = [4.0, 5.0, 7.0, 10.0, 13.0, 16.0] 
        for i, position in enumerate(positon_list_rad):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)
        
        rospy.logwarn("Robot begin moving..............")
        rospy.loginfo("The robot will move to the following waypoints: \n{}".format(position_list))
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()
        
        
        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))
    
    # Select the robot controller simulation or the real robot
    def select_controller(self):
        print("Please choose your controller real [0, 1] and simulation[2, 3] robot:")
        for (index, name) in enumerate(JOINT_TRAJECTORY_CONTROLLERS):
            print("{} (joint-based): {}".format(index, name))    
        choice = -1    
        #Choose the controller
        while choice < 0:
            input_str = input(
                "Please choose a controller by entering its number: "
            )
            try:
                choice = int(input_str)
            except ValueError:
                print("Input is not a valid number. Please try again.")   
        if choice < len(JOINT_TRAJECTORY_CONTROLLERS):
            self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[choice]
            return "joint_based"
    
if __name__ == "__main__":
    client = RobotTrajectory()
    controller_type = client.select_controller()
    if controller_type=="joint_based":
        client.send_joint_trajectory()
    else:
        print("Error please choose again")
    

