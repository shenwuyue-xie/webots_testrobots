# """添加模块，应该有个计数，使得当前一直+-+-+-的情况受到惩罚"""
# """左右motor的速度设置应该尽可能使得大部分时候是直着走"""
#     # """ better function """
#     # def robot_step(self,action):

#     #     flag_translation = False
#     #     flag_rotation = False

#     #     if  action[-1] > 0.8 and action[-1] <= 1 and self.robot_num < Max_robotnum:

#     #         last_translation = self.robot_handles[-1].getField('translation').getSFVec3f()
#     #         last_angle = self.robot_handles[-1].getField('rotation').getSFRotation()[3]
#     #         last_rotation = self.robot_handles[-1].getField('rotation').getSFRotation()

#     #         delta_z = 0.23 * math.cos(last_angle)
#     #         delta_x = 0.23 * math.sin(last_angle)
#     #         new_translation = []
#     #         new_translation.append(last_translation[0] - delta_x)
#     #         new_translation.append(last_translation[1])
#     #         new_translation.append(last_translation[2] - delta_z)

#     #         new_rotation = []
#     #         for i in range(4):
#     #             new_rotation.append(last_rotation[i])

            
#     #         importname = "robot_" + str(self.robot_num) + '.wbo'
#     #         with open (importname,'w') as f:
#     #             lines = f.readlines()
#     #             for line in lines:
#     #                 if "translation" in line:
#     #                     if flag_translation == False:
#     #                         replace = "translation " + str(new_translation[0]) + " " + str(new_translation[1]) + " " + str(new_translation[2])
#     #                         line = "\t" + replace +'\n'
#     #                         flag_translation = True
#     #                         f.write(line)
#     #                 if "rotation" in line:
#     #                     if flag_rotation == False:
#     #                         replace = "rotation " + str(new_rotation[0]) + " " + str(new_rotation[1]) + " " + str(new_rotation[2]) + " " \
#     #                             +str(new_rotation[3])  
#     #                         line = "\t" + replace +'\n'
#     #                         flag_rotation = True
#     #                         f.write(line) 

#     #         rootNode = self.supervisor.getRoot()
#     #         childrenField = rootNode.getField('children')
#     #         childrenField.importMFNode(-1,importname)
#     #         defname = 'robot_' + str(self.robot_num)
#     #         self.robot_handles.append(self.supervisor.getFromDef(defname))
#     #         self.robot_num = self.robot_num + 1
            
#     #     elif action[-1] >0 and action[-1] <= 0.2 and self.robot_num >1:
#     #         removerobot = self.robot_handles[-1]
#     #         removerobot.remove()
#     #         self.robot_num = self.robot_num - 1
#     #         del(self.robot_handles[-1])

# def robot_step(self,action,ep):

#     # x = np.random.rand()
#     # e = 0.8 + ep * 0.2/10000 
#     # if x > e :
#     #     action[-1] = np.random.rand()

#     if action[-1] > 0.9 and action[-1] <= 1 and self.robot_num < Max_robotnum:

#         last_translation = self.robot_handles[-1].getField('translation').getSFVec3f()
#         last_angle = self.robot_handles[-1].getField('rotation').getSFRotation()[3]
#         last_rotation = self.robot_handles[-1].getField('rotation').getSFRotation()

#         delta_z = 0.23 * math.cos(last_angle)
#         delta_x = 0.23 * math.sin(last_angle)
#         new_translation = []
#         new_translation.append(last_translation[0] - delta_x)
#         new_translation.append(last_translation[1])
#         new_translation.append(last_translation[2] - delta_z)

#         rootNode = self.supervisor.getRoot()
#         childrenField = rootNode.getField('children')
#         robot_children = self.robot_handles[-1].getField('children')
#         rearjoint_node = robot_children.getMFNode(4)
#         joint = rearjoint_node.getField('jointParameters')
#         joint = joint.getSFNode()
#         para = joint.getField('position')
        
#         importname = "robot_" + str(self.robot_num) + '.wbo'
#         childrenField.importMFNode(-1,importname)

#         defname = 'robot_' + str(self.robot_num)
        
#         self.robot_handles.append(self.supervisor.getFromDef(defname))
#         self.robot_num = self.robot_num + 1
#         new_translation_field = self.robot_handles[-1].getField('translation')  
#         new_translation_field.setSFVec3f(new_translation)
#         new_rotation = []
#         for i in range(4):
#             new_rotation.append(last_rotation[i])
        
#         new_rotation_field = self.robot_handles[-1].getField('rotation')
#         new_rotation_field.setSFRotation(new_rotation)

#         # robot_children = self.robot_handles[-1].getField('children')
#         # rearjoint_node = robot_children.getMFNode(4)
#         # joint = rearjoint_node.getField('jointParameters')
#         # joint = joint.getSFNode()
#         # para = joint.getField('position')
#         # para.setSFFloat(-self.rearvalue[-1])
        
#         battery_remain = float(self.endbattery[self.robot_num - 1])
#         battery_field = self.robot_handles[-1].getField('battery')
#         battery_field.setMFFloat(0,battery_remain)
#         battery_field.setMFFloat(1,self.startbattery)
        
#     elif action[-1] >0 and action[-1] <= 0.1 and self.robot_num >1:
#         removerobot = self.robot_handles[-1]
#         removerobot.remove()
#         self.robot_num = self.robot_num - 1
#         del(self.robot_handles[-1])
        
        

