#! /usr/bin/env python

import rospy
from kinova.srv import sequence, sequenceResponse
from enum import Enum
from control import KinovaControl
from arduino import ArduinoControl

class VagonType(Enum): 
    AMPEER = 1 
    BMPEER = 2
    BDMPEER = 3
    BMZ = 4
    UNIVERSAL = 5

class KinovaService():
    def __init__(self):
        self.kinova = KinovaControl()
        self.my_service = rospy.Service('/kinova_service_server', sequence , self.my_callback) 
        self.arduino_rele = ArduinoControl()

    def my_callback(self, request):
        rospy.loginfo("Kinova service action has been called with:")
        self.vagon_type = request.vagon_type #int
        self.seat_sequence_type = request.seat_sequence_type #string
        self.index = request.index #int
        response = ''

        rospy.loginfo("Vagon Type: ",self.vagon_type)
        rospy.loginfo("Seat Sequence Type: ",self.seat_sequence_type)
        rospy.loginfo("Index: ",self.index)
                
        if self.vagon_type == VagonType.AMPEER.value: 
            ampeer_list = self.kinova.get_ampeer_list(self.seat_sequence_type)
            for i in ampeer_list: 
                if i == 1:
                    self.arduino_rele.open()
                if i == (len(ampeer_list) - 1):
                    self.arduino_rele.close()    
                self.kinova.realize_arm_move(i)
            response = "Ampeer kinova sequence done careffuly"


        if self.vagon_type == VagonType.BMPEER.value: 
            bmpeer_list = self.kinova.get_bmpeer_list(self.seat_sequence_type)
            for i in bmpeer_list: 
                if i == 1:
                    self.arduino_rele.open()
                if i == (len(bmpeer_list) - 1):
                    self.arduino_rele.close()
                self.kinova.realize_arm_move(i)
            response = "Bmpeer kinova sequence done careffuly"

        if self.vagon_type == VagonType.BDMPEER.value: 
            bdmpeer_list = self.kinova.get_bdmpeer_list(self.seat_sequence_type)
            for i in bdmpeer_list:
                if i == 1:
                    self.arduino_rele.open()
                if i == (len(bdmpeer_list) - 1):
                    self.arduino_rele.close() 
                self.kinova.realize_arm_move(i)
            response = "Bdmpeer kinova sequence done careffuly"

        if self.vagon_type == VagonType.BMZ.value: 
            bmz_list = self.kinova.get_bmz_list(self.seat_sequence_type)
            for i in bmz_list:
                if i == 1:
                    self.arduino_rele.open()
                if i == (len(bmz_list) - 1):
                    self.arduino_rele.close() 
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