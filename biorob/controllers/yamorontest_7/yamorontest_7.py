from controller import Supervisor
from controller import Robot

class resetsupervisor(Supervisor):
    def __init__(self,robot_names):

        super(resetsupervisor,self).__init__()

        self.timestep = int(self.getBasicTimeStep())
        self.robot_list = robot_names
        self.robot_handles = []
        self.emitter = self.getEmitter("emitter")
        self.receiver = self.getReceiver("receiver")
        self.steps = 1
        self.setrobots()

    def setrobots(self):

        for defname in self.robot_list:
            self.robot_handles.append(self.getFromDef(defname))



    def reset(self):
        print("reset simulation")
        self.resetrobots()
        self.steps = 0
        self.message=None
        
    def resetrobots(self):

        for robot in self.robot_handles:
            if robot is not None:
                robot.remove()
        rootNode = self.getRoot()
        childrenField = rootNode.getField('children')
        childrenField.importMFNode(-2,"firstrobot.wbo")
        childrenField.importMFNode(-2,"secondrobot.wbo")
        childrenField.importMFNode(-2,"thirdrobot.wbo")
        childrenField.importMFNode(-2,"lastrobot.wbo")

        robot_handles = []

        for defname in self.robot_list:
            robot_handles.append(self.getFromDef(defname))



        self.simulationResetPhysics()

    

    def get_reward(self):
        pass

    def run(self):
        while self.step(self.timestep) != -1:
            
            if self.steps % 1000 ==0:
                self.reset()
            self.steps += 1
        

            

robot_defnames = ['robot_0','robot_1','robot_2','robot_3']


controller = resetsupervisor(robot_defnames)
controller.run()