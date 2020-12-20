from controller import Robot
from controller import Connector
from controller import Motor
from controller import DistanceSensor
from controller import Device
from controller import PositionSensor
import numpy as np
from deepbots.robots.controllers.robot_emitter_receiver_csv import \
    RobotEmitterReceiverCSV
import math


class TaskDecisionRobot(RobotEmitterReceiverCSV):



    def __init__(self):
        super(TaskDecisionRobot,self).__init__()
        self.name = self.robot.getName()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.setupsensors()
        self.setupmotors()
        self.robot.batterySensorEnable(self.timestep)
        

        
    
    def normalize_to_range(self,value, min, max, newMin, newMax):
        value = float(value)
        min = float(min)
        max = float(max)
        newMin = float(newMin)
        newMax = float(newMax)
        return (newMax - newMin) / (max - min) * (value - max) + newMax

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
        self.leftmotor.setVelocity(5.0)
        self.rightmotor.setVelocity(5.0)
        self.rearpositionsensor = self.rearmotor.getPositionSensor()
        self.rearpositionsensor.enable(self.timestep)


    def create_message(self):
        message = []
        for distancesensor in self.distancesensors:
            message.append(distancesensor.getValue())
        message.append(self.robot.batterySensorGetValue())
        message.append(self.rearpositionsensor.getValue())
        return message
        

    


    def use_message_data(self,message):

        # message[0] = self.normalize_to_range(float(message[0]),-1,2,-5,5)
        # basicspeed = message[1]
        # message[1] = self.normalize_to_range(float(message[1]),-1,2,-2,2)
        # turing = message[1]
        # self.leftmotor.setVelocity(basicspeed + turing)
        # self.rightmotor.setVelocity*basicspeed - turing)

        

            

        for i in range(0,2):
            message[i] = self.normalize_to_range(float(message[i]),-0.4,1.4,-8,8)

        for i in range(12,24):
            # message[i] = float(message[i])
            # x = np.random.uniform(0,1,12)
            message[i] = self.normalize_to_range(float(message[i]),-0.2,1.2,0,1)
            if message [i] >= 0.05 and message[i] <= 0.95:
                message[i] = 0
            elif message[i] > 0.95 and message[i] < 1:
                message[i] = self.normalize_to_range(message[i],0.95,1,0,math.pi/2)
            elif message[i] > 0 and message[i] < 0.05:
                message[i] = self.normalize_to_range(message[i],0,0.05,-math.pi/2,0)

        self.leftmotor.setVelocity(message[int(self.name) * 2])
        self.rightmotor.setVelocity(message[int(self.name) * 2 + 1])
        self.frontmotor.setPosition(message[int(self.name) * 2 + 12])
        self.rearmotor.setPosition(message[int(self.name) * 2 + 13])
        
        



controller = TaskDecisionRobot()
controller.run()
        
        

                


    
        

        
    
