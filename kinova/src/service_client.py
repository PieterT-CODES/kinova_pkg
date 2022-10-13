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

    def send_goal(self, vagon_type, seat_sequence_type, index):
        self.object.vagon_type = vagon_type #int 
        self.object.seat_sequence_type = seat_sequence_type #string
        self.object.index = index #int

        rospy.loginfo(self.object.vagon_type)
        rospy.loginfo(self.object.seat_sequence_type)
        rospy.loginfo(self.object.index)
    
        result = self.service(self.object)

if __name__ == "__main__":
    kinovaserviceclient = KinovaServiceClient()
    kinovaserviceclient.send_goal(1, "middle", 4)
