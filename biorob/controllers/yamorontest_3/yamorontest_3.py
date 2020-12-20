from controller import Emitter
from controller import Receiver
from controller import Robot
from controller import Motor
from controller import DistanceSensor
import struct
from controller import Connector

"""如果每个模块都有emitter和receiver，supervisor也是一个emitter和一个receiver,测试机器人传送到supervisor的信号顺序和什么有关"""

"""经测试，和supervisor位置无关，只和模块顺序有关"""
"""经测试，一个step里面supervisor收到7次first的信号，剩下的robot,每个step收到其四次信号"""
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
        step = 0
        goal = 0
        while self.step(self.timestep) != -1:
            self.leftmotor.setVelocity(5)
            self.rightmotor.setVelocity(5)


            message = []
            
            for distancesensor in self.distancesensors:
                message.append(distancesensor.getValue())

            # testmessage = struct.pack("chd","a",message)


            string_message = ""
            string_message = ",".join(map(str,message))
            string_message = string_message.encode("utf-8")
            self.emitter.send(string_message)
            # self.emitter.send(testmessage)

            if self.receiver.getQueueLength() > 0:
                if self.robot_name == "1":
                    self.rearmotor.setPosition(1.5708)

                goal = 1
            if goal == 1:
                step = step +1
                print(step) 
                if step == 200:
                    if self.robot_name == "0":
                        self.rearmotor.setPosition(-1.5708)

                #     if self.robot_name == "1":
                #         self.rearconnector.unlock()
                #     if self.robot_name == "2":
                #         self.frontconnector.unlock()
                    # if self.robot_name == "2":
                    #     self.frontmotor.setPosition(-1.5708)
                    #     self.rearmotor.setPosition(1.5708)
                    # if self.robot_name == "3":
                    #     self.frontmotor.setPosition(-1.5708)


            # self.message = self.robot_name.encode("utf-8")
            # self.emitter.send(self.message)

controller = testrobot()
controller.run()




