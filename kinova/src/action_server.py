#! /usr/bin/env python

import rospy 
import actionlib
from enum import Enum
from kinova.msg import SequenceAction, SequenceResult, SequenceFeedback
from control import KinovaControl

class VagonType(Enum): 
    AMPEER = 1 
    BMPEER = 2
    BDMPEER = 3 
    BMZ = 4

class GunStateType(Enum): 
    OPEN = 1
    CLOSE = 0
    
class KinovaActionServer(object):
    _feedback = SequenceFeedback()
    _result =  SequenceResult()

    def __init__(self): 
        self._as = actionlib.SimpleActionServer("kinova_pkg_action", SequenceAction ,self.execute_cb, False)
        self._as.start()
        self.kinova = KinovaControl()

    def execute_cb(self, goal):
        success = True
        count = 0

        self.vagon_type = goal.vagon_type #int
        self.gun_state = goal.gun_state #int
        self.name = goal.name #string 
        self.gripper_range = goal.gripper_range #int

        if self.vagon_type == VagonType.AMPEER.value: 
            ampeer_list = self.kinova.get_ampeer_list()
            for i in ampeer_list: 
                self.kinova.realize_arm_move(i)
                self._feedback.joint_info = self.kinova.get_current_joints()
                self._as.publish_feedback(self._feedback)
                count = count + 1
        
        if self.vagon_type == VagonType.BMPEER.value: 
            bmpeer_list = self.kinova.get_bmpeer_list()
            for i in bmpeer_list:
                self.kinova.realize_arm_move(i)
                self._feedback.joint_info = self.kinova.get_current_joints()
                self._as.publish_feedback(self._feedback)
                count = count + 1

        if self.vagon_type == VagonType.BDMPEER.value: 
            bdmpeer_list = self.kinova.get_bdmpeer_list()
            for i in bdmpeer_list:
                self.kinova.realize_arm_move(i)
                self._feedback.joint_info = self.kinova.get_current_joints()
                self._as.publish_feedback(self._feedback)
                count = count + 1

        if self.vagon_type == VagonType.BMZ.value: 
            bmz_list = self.kinova.get_bmz_list()
            for i in bmz_list:
                self.kinova.realize_arm_move(i)
                self._feedback.joint_info = self.kinova.get_current_joints()
                self._as.publish_feedback(self._feedback)
                count = count + 1                        

        if success: 
            self._result.count = count
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node("kinova_action_server")
    server = KinovaActionServer()
    rospy.spin()
