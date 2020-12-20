import numpy 
import math
import os



# def normalize_to_range(value, min, max, newMin, newMax):
#     value = float(value)
#     min = float(min)
#     max = float(max)
#     newMin = float(newMin)
#     newMax = float(newMax)
#     return (newMax - newMin) / (max - min) * (value - max) + newMax

# k = normalize_to_range(50,0,1000,0,1)
# print(k)

# x = [0.5 for i in range(12)]
# y = numpy.random.normal(12)

#     """ better function """
#     def robot_step(self,action):

#         flag_translation = False
#         flag_rotation = False

#         if  action[-1] > 0.8 and action[-1] <= 1 and self.robot_num < Max_robotnum:

#             last_translation = self.robot_handles[-1].getField('translation').getSFVec3f()
#             last_angle = self.robot_handles[-1].getField('rotation').getSFRotation()[3]
#             last_rotation = self.robot_handles[-1].getField('rotation').getSFRotation()

#             delta_z = 0.23 * math.cos(last_angle)
#             delta_x = 0.23 * math.sin(last_angle)
#             new_translation = []
#             new_translation.append(last_translation[0] - delta_x)
#             new_translation.append(last_translation[1])
#             new_translation.append(last_translation[2] - delta_z)

#             robot_children = self.robot_handles[-1].getField('children')
#             rearjoint_node = robot_children.getMFNode(4)
#             joint = rearjoint_node.getField('jointParameters')
#             joint = joint.getSFNode()
#             para = joint.getField('position')
#             hingeposition = para.getSFFloat()


            # if hingeposition > 0.8 or hingeposition < -0.8:
            #     delta = 0.03 - 0.03 * math.cos(hingeposition)
            #     delta_z = delta * math.cos(last_angle)
            #     delta_x = delta * math.sin(last_angle)
            #     new_translation[0] = new_translation[0] + delta_x
            #     new_translation[2] = new_translation[2] + delta_z

            # new_rotation = []
            # for i in range(4):
            #     new_rotation.append(last_rotation[i])

flag_translation = False
flag_rotation = False


with open ("Robot.wbo",'r+') as f:
    lines = f.readlines()
    for line in lines:
        if "translation" in line:
            if flag_translation == False:
                replace = "translation " + str(0) + " " + str(0) + " " + str(0)
                line = "\t" + replace +'\n'
                flag_translation = True
        if "rotation" in line:
            if flag_rotation == False:
                replace = "rotation " + str(0) + " " + str(0) + " " + str(0) + " " \
                    +str(0)  
                line = "\t" + replace +'\n'
                flag_rotation = True
        f.write(line)




            # rootNode = self.supervisor.getRoot()
            # childrenField = rootNode.getField('children')
            # childrenField.importMFNode(-1,importname)
            # defname = 'robot_' + str(self.robot_num)
            # self.robot_handles.append(self.supervisor.getFromDef(defname))
            # self.robot_num = self.robot_num + 1
            
        # elif action[-1] >0 and action[-1] <= 0.2 and self.robot_num >1:
        #     removerobot = self.robot_handles[-1]
        #     removerobot.remove()
        #     self.robot_num = self.robot_num - 1
        #     del(self.robot_handles[-1])

