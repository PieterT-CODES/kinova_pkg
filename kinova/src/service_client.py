#! /usr/bin/env python

import rospy
from kinova.srv import sequence, sequenceRequest 
import sys

class KinovaServiceClient():
    def __init__(self):
        rospy.init_node('service_client')
        rospy.wait_for_service('/kinova_service_client')        
        self.service = rospy.ServiceProxy('/kinova_service_server', sequence)        
        self.object = sequenceRequest()

    def send_goal(self, vagon_type, name, gun_state, gripper_range):
        self.object.vagon_type = vagon_type #int 
        self.object.name = name #string
        self.object.gun_state = gun_state #int
        self.object.gripper_range = gripper_range #float

        rospy.loginfo(self.object.vagon_type)
        rospy.loginfo(self.object.name)
        rospy.loginfo(self.object.gun_state)
        rospy.loginfo(self.object.gripper_range)

        result = self.service(self.object)

if __name__ == "__main__":
    kinovaserviceclient = KinovaServiceClient()
    kinovaserviceclient.send_goal(1, "hello world", 1, 0.5)
