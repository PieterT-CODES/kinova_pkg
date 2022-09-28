#! /usr/bin/env python

import rospy
from kinova.srv import sequence, sequenceResponse
from enum import Enum
from control import KinovaControl

class VagonType(Enum): 
    AMPEER = 1 
    BMPEER = 2
    BDMPEER = 3
    BMZ = 4
    UNIVERSAL = 5

class GunStateType(Enum):
    OPEN = 1
    CLOSE = 0    

class KinovaService():
    def __init__(self):
        self.kinova = KinovaControl()
        self.my_service = rospy.Service('/kinova_service_server', sequence , self.my_callback) 

    def my_callback(self, request):
        rospy.loginfo("Kinova service action has been called")
        self.vagon_type = request.vagon_type #int
        self.gun_state = request.gun_state #int
        self.name = request.name #string 
        self.gripper_range = request.gripper_range #int

        response = ''

        rospy.loginfo(request.vagon_type)
        rospy.loginfo(self.name)
        rospy.loginfo(self.gun_state)
        rospy.loginfo(self.gripper_range)
                
        if self.vagon_type == VagonType.AMPEER.value: 
            ampeer_list = self.kinova.get_ampeer_list()
            for i in ampeer_list: 
                self.kinova.realize_arm_move(i)
            response = "Ampeer kinova sequence done careffuly"
    
        
        if self.vagon_type == VagonType.BMPEER.value: 
            bmpeer_list = self.kinova.get_bmpeer_list()
            for i in bmpeer_list: 
                self.kinova.realize_arm_move(i)
            response = "Bmpeer kinova sequence done careffuly"


        if self.vagon_type == VagonType.BDMPEER.value: 
            bdmpeer_list = self.kinova.get_bdmpeer_list()
            for i in bdmpeer_list: 
                self.kinova.realize_arm_move(i)
            response = "Bdmpeer kinova sequence done careffuly"


        if self.vagon_type == VagonType.BMZ.value: 
            bmz_list = self.kinova.get_bmz_list()
            for i in bmz_list: 
                self.kinova.realize_arm_move(i)
            response = "Bmz kinova sequence done careffuly"

        if self.vagon_type == VagonType.UNIVERSAL.value:
            universal_list = self.kinova.get_universal_list()
            for i in universal_list: 
                self.kinova.realize_arm_move(i)
            response = "Universal kinova sequence done careffuly"
                
        return sequenceResponse(response) # EmptyResponse


if __name__ == "__main__":
    kinova_service = KinovaService()
    rospy.init_node('kinova_service_server_node') 
    rospy.spin()