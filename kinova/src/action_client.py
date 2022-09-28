#! /usr/bin/env python

import rospy 
import actionlib 
from kinova.msg import SequenceAction, SequenceGoal, SequenceResult, SequenceFeedback
import sys 

class KinovaActionClient():
    def __init__(self):
        rospy.init_node('kinova_pkg_action_nodes')
        self.client = actionlib.SimpleActionClient('/kinova_pkg_action',SequenceAction)
        client.wait_for_server()
        self.goal = SequenceGoal()


    def send_goals(self, vagon_type, name, gun_state, gripper_range): 
        goal.vagon_type = vagon_type
        goal.name = name 
        goal.gun_state = gun_state
        goal.gripper_range = gripper_range

        client.send_goal(goal)
        client.wait_for_result() 

        return client.get_result() 

if __name__ == '__main__': 
    kinova_action_client = KinovaActionClient()
    kinova_action_client.send_goals(1, "test", 0, 0.5)

