#!/usr/bin/env python
import sys
from moveit_commander import move_group
from moveit_commander.exception import MoveItCommanderException
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math as m
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class MoveGroupPython(object):
    """ Mover Group Python"""
    def __init__(self):
        super(MoveGroupPython, self).__init__()
        #Begin code
        #Initialize "moveit_commander" and "rospy" node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python', anonymous= True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot= moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## so we set the group's name to "manipulator".
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
    
    # Convert Euler(rad) to Quaternion
    def eul2qua(self, yaw, pitch, roll):# yaw(Z), pitch(Y), roll(X)
        cy = m.cos(yaw*0.5)
        sy = m.sin(yaw*0.5)
        cp = m.cos(pitch*0.5)
        sp = m.sin(pitch*0.5)
        cr = m.cos(roll*0.5)
        sr = m.sin(roll*0.5)
        #Quadternion q:
        q=[0.0, 0.0 ,0.0 ,0.0]
        q[0]=cr * cp * cy + sr * sp * sy # qw
        q[1]=sr * cp * cy - cr * sp * sy # qx
        q[2]=cr * sp * cy + sr * cp * sy # qy
        q[3]=cr * cp * sy - sr * sp * cy # qz
        return q

    ## Using the this set_pose_goal with quaternion to avoid SINGULARITIES.
    def set_pose_goal(self, x, y, z, qx, qy, qz, qw):
        """ This code help robot moving to desirable point"""
        move_group =self.move_group
        move_group.set_pose_reference_frame("base_link_inertia")

        """Planing to a Pose Goal 1"""
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.x = qx
        pose_goal.orientation.y = qy 
        pose_goal.orientation.z = qz
        pose_goal.orientation.w = qw

        move_group.set_pose_target(pose_goal)
        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()
    
    ## Be careful with Euler, It's can meet the SINGULARITIES .
    def set_pose_euler(self, x, y, z, roll, pitch, yaw):
        """ This code help robot moving to desirable point"""
        move_group =self.move_group
        move_group.set_pose_reference_frame("base_link_inertia")

        """ Convert Euler to Quaternion"""
        #quaternion = self.eul2qua(yaw, pitch, roll)
        """ RealRobot Constrain: Rotate around x axis amount of pi """
        quaternion = self.eul2qua(yaw , pitch, roll+pi)
        """Planing to a Pose Goal 1"""
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.w = quaternion[0]
        pose_goal.orientation.x = quaternion[1]
        pose_goal.orientation.y = quaternion[2]
        pose_goal.orientation.z = quaternion[3]

        move_group.set_pose_target(pose_goal)
        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()
        

def main():
    try:
        for i in range (3):
            rospy.loginfo("Looping in:  %s", i)
            tutorial = MoveGroupPython()
            tutorial.set_pose_euler(0.3, 0.5, 0.6, pi/3, 0.0, 0.0)
            rospy.loginfo("Waiting for next movement")
            rospy.sleep(3)
            tutorial.set_pose_euler(0.3, 0.5, 0.6, 0.0, pi/3, 0.0)
            rospy.loginfo("Waiting for next movement")
            rospy.sleep(3)
            tutorial.set_pose_euler(0.3, 0.5, 0.6, 0.0, 0.0, pi/2)
            rospy.loginfo("Waiting for next movement")
            rospy.sleep(3)
            tutorial.set_pose_goal(-0.3, 0.5, 0.6, 1, 0, 0, 0)
            rospy.loginfo("Waiting for next movement")
            rospy.sleep(3)
            tutorial.set_pose_goal(0.2, 0.5, 0.6, 1, 0, 0, 0)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
        
if __name__ == '__main__':
  main()
