# """test controller."""

# from controller import Robot
# from controller import Supervisor


# class setenvironment(Supervisor):
#     def __init__(self):
#         super(setenvironment,self).__init__()
    
#     # def mdoify(self):
#     #     with open ('environment.wbo','w') as f:
#     #         lines = f.readlines()


#     def setenv(self):
#         rootNode = self.getRoot()
#         childrenField = rootNode.getField('children')
#         childrenField.importMFNode(-1,'robot_0.wbo')
#         childrenField.importMFNode(-1,'robot_1.wbo')

#     def run(self):
#         # self.mdoify()
#         self.setenv()

# controller = setenvironment()
# controller.run()

