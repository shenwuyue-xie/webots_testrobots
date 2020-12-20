from controller import Supervisor
from controller import Robot
from controller import Device
from controller import Emitter
from controller import Receiver


class testsupervisor(Supervisor):
    def __init__(self):
        super(testsupervisor,self).__init__()
        self.timestep = int(self.getBasicTimeStep())
        self.emitter = self.getEmitter('emitter')
        self.receiver = self.getReceiver('receiver')
        self.receiver.enable(self.timestep)

    def run(self):
        step = 0
        previous_message = []
        messagesend = 1 
        floatmessage = []
        allmessage = []
        goal = 0
        while self.step(self.timestep) != -1:
            while self.receiver.getQueueLength() > 0:
                message = self.receiver.getData().decode('utf-8')
                message = message.split(",")
                for ms in message : 
                    allmessage.append(message)
                if allmessage != '' and allmessage != previous_message:
                    previous_message = allmessage
                print(allmessage)
    
                for ms in allmessage:
                    for msy in ms :
                        msy = float(msy)
                        floatmessage.append(msy)
                allmessage = []
                # message = self.receiver.getData()


                print(floatmessage)
                print("next step")
                self.receiver.nextPacket()


                if min(floatmessage) < 300 and messagesend == 1:

                    self.emitter.send("action".encode("utf-8"))
                    messagesend = 0
                    goal = 1
                """在测试过程中发现，这个if直接运行了七次"""
                # if min(floatmessage) < 400 :
                #     step = step +1
                #     print(step)
                """这是用来测试步骤数的"""
                # if min(floatmessage) < 400 or goal == 1:
                #     step = step +1
                #     print(step)
                #     goal = 1


                floatmessage = []
                # print("next step")
                # self.receiver.nextPacket()


    


controller = testsupervisor()
controller.run()


                


    
        

        
    
