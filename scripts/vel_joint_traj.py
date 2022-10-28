#!/usr/bin/env python3

########### Basic Move UR Robot ###################
# ---------------------------------------------------------------------
# This code use Ros Action help the UR robot move to desirable velocity
# Author: Tri Knight
# ---------------------------------------------------------------------

import rospy
import numpy as np
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint



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

class RobotVelTrajectory:
    """ This is a test to move the robot with disirable velocity"""
    def __init__(self):
        rospy.init_node("velocity_trajectories")
        # For simulation robot
        # self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[2]
        # For real robot 
        # self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
     #This function is convert from deg to rad
    def deg2rad(self, deg):
        rad = np.radians(deg)
        return rad
   
    #Send velocity trajectory
    def send_vel_trajectory(self):
        """ Create the velocity trajectory and send to the robot"""
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
         )
        trajectory_client.wait_for_server()
        goal=FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES
        
        # Position list
        position_list = [-73, -106, 99, -77, -87, 0]
        position_list_rad = self.deg2rad(position_list)

        velocity_list =[0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
         #Time to translation between each position in second
        duration_list = [10.0]
            
        joint_traj1 = JointTrajectoryPoint()
        joint_traj1.positions= position_list_rad
        joint_traj1.velocities = velocity_list
        joint_traj1.time_from_start = rospy.Duration(10)
        goal.trajectory.points.append(joint_traj1)

        rospy.logwarn("Robot begin moving..............")
        rospy.loginfo("The robot will move to the following waypoints: \n{}".format(position_list))
        rospy.loginfo("The robot will move with the velocity: \n{}".format(velocity_list))
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
    client = RobotVelTrajectory()
    controller_type = client.select_controller()
    if controller_type=="joint_based":
        client.send_vel_trajectory()
    else:
        print("Error please choose again")
