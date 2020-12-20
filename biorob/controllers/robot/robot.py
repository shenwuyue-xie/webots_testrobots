from controller import Robot
from controller import Connector
from controller import Motor
from controller import DistanceSensor
from controller import Device
import numpy as np
from deepbots.robots.controllers.robot_emitter_receiver_csv import \
    RobotEmitterReceiverCSV
import math

def normalize_to_range(value, min, max, newMin, newMax):
    value = float(value)
    min = float(min)
    max = float(max)
    newMin = float(newMin)
    newMax = float(newMax)
    return (newMax - newMin) / (max - min) * (value - max) + newMax


class TaskDecisionRobot(RobotEmitterReceiverCSV):



    def __init__(self):
        super(TaskDecisionRobot,self).__init__()
        self.name = self.robot.getName()
        self.setupsensors()
        self.setupmotors()

        self.timestep = int(self.robot.getBasicTimeStep())
        self.robot.batterySensorEnable(self.timestep)
        

    def setupsensors(self):
        self.distancesensors = []
        if self.name == "0":
            self.n_distancesensors = 7
            self.rearconnector = self.robot.getConnector("rear_connector")
            self.dsNames = ['ds' + str(i) for i in range(self.n_distancesensors)]
            for i in range(self.n_distancesensors):
                self.distancesensors.append(self.robot.getDistanceSensor(self.dsNames[i]))
                self.distancesensors[i].enable(self.timestep)
            
        else :
            self.n_distancesensors = 4
            self.frontconnector = self.robot.getConnector("front_connector")
            self.rearconnector = self.robot.getConnector("rear_connector")
            self.dsNames = ['ds' + str(i) for i in range(self.n_distancesensors)]
            for i in range(self.n_distancesensors):
                self.distancesensors.append(self.robot.getDistanceSensor(self.dsNames[i]))
                self.distancesensors[i].enable(self.timestep)
        



    def setupmotors(self):
        self.leftmotor= self.robot.getMotor('left_motor')
        self.rightmotor= self.robot.getMotor('right_motor')
        self.frontmotor = self.robot.getMotor('front_motor')
        self.rearmotor = self.robot.getMotor('rear_motor')
        self.leftmotor.setPosition(float('inf'))
        self.rightmotor.setPosition(float('inf'))
        self.leftmotor.setVelocity(0.0)
        self.rightmotor.setVelocity(0.0)
        self.motorSpeeds = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]




    def create_message(self):
        message = []
        for distancesensor in self.distancesensors:
            message.append(distancesensor.getValue())
        return message
        

    


    def use_message_data(self,message):

        basic = float(message[1])
        turning = float(message[0])       
        basic = basic * 4
        turning = turning * 2
        turning = np.clip(turning,-2,2)
        self.motorSpeeds[0]=basic + turning
        self.motorSpeeds[1]=basic - turning

        basic = float(message[3])
        turning = float(message[2])       
        basic = basic * 4
        turning  = turning * 2
        turning = np.clip(turning,-2,2)
        self.motorSpeeds[2]=basic + turning
        self.motorSpeeds[3]=basic - turning

        basic = float(message[5])
        turning = float(message[4])        
        basic = basic * 4
        turning  = turning * 2
        turning = np.clip(turning,-2,2)
        self.motorSpeeds[4]=basic + turning
        self.motorSpeeds[5]=basic - turning

        basic = float(message[7])
        turning = float(message[6])       
        basic = basic * 4
        turning  = turning * 2
        turning = np.clip(turning,-2,2)
        self.motorSpeeds[6]=basic + turning
        self.motorSpeeds[7]=basic - turning

        self.motorSpeeds = np.clip(self.motorSpeeds, -4, 4)
        if self.name == "0":
            self.leftmotor.setVelocity(self.motorSpeeds[0])
            self.rightmotor.setVelocity(self.motorSpeeds[1])
        if self.name == "1":
            self.leftmotor.setVelocity(self.motorSpeeds[2])
            self.rightmotor.setVelocity(self.motorSpeeds[3])
        if self.name == "2":
            self.leftmotor.setVelocity(self.motorSpeeds[4])
            self.rightmotor.setVelocity(self.motorSpeeds[5])
        if self.name == "3":
            self.leftmotor.setVelocity(self.motorSpeeds[6])
            self.rightmotor.setVelocity(self.motorSpeeds[7])


controller = TaskDecisionRobot()
controller.run()
        
        

                


    
        

        
    
