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


def main():
	ou_noise=(mu=np.zeros(2))
	plt.figure('data')
	y=[]
	t=np.linspace(0,100,1000)
	for _ in t:
		y.append(ou_noise())
	plt.plot(t,y)
	plt.show()
 
if __name__=="__main__":
	main()









