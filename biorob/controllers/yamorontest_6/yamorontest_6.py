from controller import Supervisor
from controller import Robot
from controller import Device
from controller import Emitter
from controller import Receiver


class testsupervisor(Supervisor):
    def __init__(self,robot):
        super(testsupervisor,self).__init__()
        self.timestep = int(self.getBasicTimeStep())
        self.emitter = self.getEmitter('emitter')
        self.receiver = self.getReceiver('receiver')
        self.receiver.enable(self.timestep)
        self.robot_handles = []
        self.robot_list = robot
        self.setuprobots()
        self.endbattery = []

    def setuprobots(self):
        for defname in self.robot_list:
            self.robot_handles.append(self.getFromDef(defname))
    

        

    def run(self):
        step = 0
        previous_message = []
        messagesend = 1 
        floatmessage = []
        allmessage = []
        goal = 0
        while self.step(self.timestep) != -1:
            while self.receiver.getQueueLength() > 0:
                for i in range(4):
                    message = self.receiver.getData().decode('utf-8')
                    message = message.split(",")
                    for ms in message : 
                        ms = float(ms)
                        allmessage.append(ms)

                    self.receiver.nextPacket()
                if allmessage != '' and allmessage != previous_message:
                    previous_message = allmessage
                    print(allmessage)
                print("next step")

                
                if min(allmessage) < 300 and messagesend == 1:

                    self.emitter.send("action".encode("utf-8"))
                    messagesend = 0
                    goal = 1


                if min(allmessage) < 300 or goal == 1:
                    step = step +1
                    # print(step)





robot_defnames = ['robot_0','robot_1','robot_2','robot_3']


controller = testsupervisor(robot_defnames)
controller.run()


                


    
        

        
    
