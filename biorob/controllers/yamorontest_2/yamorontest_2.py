from controller import Emitter
from controller import Receiver
from controller import Robot
from controller import Motor
from controller import DistanceSensor

from controller import Connector
import struct

"""如果每个模块都有emitter和receiver，supervisor也是一个emitter和一个receiver,测试机器人传送到supervisor的信号顺序和什么有关"""

"""经测试，和supervisor位置无关，只和模块顺序有关"""
class testrobot(Robot):

    def __init__(self):
        super(testrobot,self).__init__()
        self.robot_name = self.getName()
        self.emitter = self.getEmitter('emitter')
        self.receiver = self.getReceiver('receiver')
        self.leftmotor = self.getMotor('left_motor')
        self.rightmotor = self.getMotor('right_motor')
        self.frontmotor = self.getMotor('front_motor')
        self.rearmotor = self.getMotor('rear_motor')
        self.leftmotor.setPosition(float('inf'))
        self.rightmotor.setPosition(float('inf'))
        self.leftmotor.setVelocity(0.0)
        self.rightmotor.setVelocity(0.0)
        self.timestep = int(self.getBasicTimeStep())
        self.receiver.enable(self.timestep)
        self.setsensors()

    
    def setsensors(self):
        self.distancesensors = []
        if self.robot_name == "0":
            self.n_distancesensors = 7
            self.rearconnector = self.getConnector("rear_connector")
            self.dsNames = ['ds' + str(i) for i in range(self.n_distancesensors)]
            for i in range(self.n_distancesensors):
                self.distancesensors.append(self.getDistanceSensor(self.dsNames[i]))
                self.distancesensors[i].enable(self.timestep)
            
        else :
            self.n_distancesensors = 4
            self.frontconnector = self.getConnector("front_connector")
            self.rearconnector = self.getConnector("rear_connector")
            self.dsNames = ['ds' + str(i) for i in range(self.n_distancesensors)]
            for i in range(self.n_distancesensors):
                self.distancesensors.append(self.getDistanceSensor(self.dsNames[i]))
                self.distancesensors[i].enable(self.timestep)


    def run(self):
        while self.step(self.timestep) != -1:
            self.leftmotor.setVelocity(5)
            self.rightmotor.setVelocity(5)
            
            # message = []
            
            # for distancesensor in self.distancesensors:
            #     message.append(distancesensor.getValue())

            # testmessage = struct.pack("chd","a",message)
            # self.emitter.send(testmessage)
            self.message = self.robot_name.encode("utf-8")
            self.emitter.send(self.message)
            
  

            
            

            




controller = testrobot()
controller.run()




