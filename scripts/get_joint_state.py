#!/usr/bin/env python
## This code using for subscibed the topic /joint_state and print these joint states

import rospy
from sensor_msgs.msg import JointState
import threading
import math
import numpy as np
import roboticstoolbox as rp
# joint of UR Robot following
# <elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint>
#holds the latest states obtained from joint_states messages
class LatestJointStates():

    def __init__(self):
        rospy.init_node('joint_states_listener')
        self.lock = threading.Lock()
        self.name = []
        self.position = []
        self.lastposition = []
        self.velocity = []
        self.lastvelocity = []
        self.effort = []
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.start()
        self.ur10 = rp.models.UR10()

    #pretty-print list to string
    def pplist_deg(self,list):
        #return ' '.join(['%2.3f'%x for x in list])
        return ' '.join(['%2.3f'%math.degrees(x) for x in list])
    
    def pplist_rad(self,list):
        #return ' '.join(['%2.3f'%x for x in list])
        return ' '.join(['%2.3f'%x for x in list])

    #thread function: listen for joint_states messages
    def joint_states_listener(self):
        while not rospy.is_shutdown():
            rospy.Subscriber('joint_states', JointState, self.joint_states_callback)
            rospy.spin()
        rospy.signal_shutdown("We are done here!")

    #callback function: when a joint_states message arrives, save the values
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
            np.set_printoptions(precision=2)
            pose_np = self.convert_real_joint(
                np.fromstring(self.pplist_rad(self.position), dtype=float, sep=' '))
            # Test Jacobian matrix in this pose
            #pose_np =[0, -1.57, 1.57, 0, 0, 0]
            print(self.convert_real_joint(pose_np))
            J = self.ur10.jacob0(pose_np)
            det_J = np.linalg.det(J)


            # self.rate.sleep()
            # rospy.sleep(1) # Delay 1s

            #Print result  
            np.set_printoptions(precision=2, suppress=True)
            print('Sub joint pos: ', pose_np)
            print('Jacobian matrix:\n', J)
            print('det of the Jacobian matrix:', det_J)  
            
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


#run the server
if __name__ == "__main__":
    try:
        LatestJointStates()
    except rospy.ROSInterruptException:
        pass
