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

        while self.step(self.timestep) != -1:
            while self.receiver.getQueueLength() > 0:
                message = self.receiver.getData().decode("utf-8")
                
                # message = self.receiver.getData()
                # dataList = struct.unpack("chd",message)
                # print(dataList)
                
                print(message)
                print("next step")
                self.receiver.nextPacket()


    


controller = testsupervisor()
controller.run()

    



