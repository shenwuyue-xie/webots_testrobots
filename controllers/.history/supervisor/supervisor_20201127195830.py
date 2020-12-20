import math

import numpy as np
from numpy import random
from numpy.core.fromnumeric import size
from numpy.lib.function_base import meshgrid

import utilities as utils
from deepbots.supervisor.controllers.supervisor_emitter_receiver import \
    SupervisorCSV
# # from deepbots.supervisor.wrappers.tensorboard_wrapper import TensorboardLogger
from tensorboardX import SummaryWriter

from models.networks import TD3
from controller import Keyboard
import os

Max_robotnum = 6
OBSERVATION_SPACE =  (Max_robotnum-1) * 4 + 7 + 9 * Max_robotnum    
ACTION_SPACE = Max_robotnum * 2 + 3
MAX_DSNUM = (Max_robotnum-1) * 4 + 7

DIST_SENSORS_MM = {'min': 0, 'max': 1000}
XPOSITION = {'min':-2, 'max':2}
YPOSITION = {'min':-1.5 , 'max':1.5}
ZPOSITION = {'min': -1, 'max' : 8}
MAX_DISTANCE = {'min':0, 'max':10}
MAX_ANGLE = {'min':-math.pi, 'max':math.pi}


# import ptvsd
# print("waiting for debugger attach")
# ptvsd.enable_attach(address=("127.0.0.1",7788))
# ptvsd.wait_for_attach()



class TaskDecisionSupervisor(SupervisorCSV):
    def __init__(self,robot,observation_space,log_dir,v_action,v_observation,v_reward,windows=[10,100,200]):
        super(TaskDecisionSupervisor,self).__init__()
        self.timestep = int(self.supervisor.getBasicTimeStep())
        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)
        self.emitter = self.supervisor.getEmitter('emitter')
        self.receiver = self.supervisor.getReceiver('receiver')
        self.robot_list = robot
        self.robot_handles = []
        self.observation = [0 for i in range(observation_space)]
        self.findThreshold = 0.2
        self.steps = 0
        self.steps_threshold = 6000    
        self.endbattery = [50000 for i in range(Max_robotnum)]
        self.final_distance = [50 for i in range(Max_robotnum)]
        self.final_target = self.supervisor.getFromDef('final_target')
        self.should_done = False
        self.startbattery = 50000
        self.setuprobots()


        self.step_cntr = 0
        self.step_global = 0
        self.step_reset = 0

        self.score = 0
        self.score_history = []

        self.v_action = v_action
        self.v_observation = v_observation
        self.v_reward = v_reward
        self.windows = windows

        self.file_writer = SummaryWriter(log_dir, flush_secs=30)

    def setuprobots(self):
        for defname in self.robot_list: 
            self.robot_handles.append(self.supervisor.getFromDef(defname))

    def handle_receiver(self):
        message = []
        for i in range(self.robot_num):
            if self.receiver.getQueueLength() > 0:
                string_message = self.receiver.getData().decode("utf-8")
                string_message = string_message.split(",")
                for ms in string_message:
                    message.append(ms)
                self.receiver.nextPacket()
        return message

    def get_observations(self):
        self.ds_values = []
        self.final_distance = [50 for i in range(Max_robotnum)]
        self.message = [1000 for i in range(MAX_DSNUM)]
        self.angles = []
        observation = []
        message = self.handle_receiver()
        self.angles = [0 for i in range(Max_robotnum)]
        

        if len(message) != 0:
            for i in range(len(message)):
                self.message[i] = float(message[i])
                self.ds_values.append(float(message[i]))
            for j in range(MAX_DSNUM):
                observation.append(utils.normalize_to_range(float(self.message[j]),DIST_SENSORS_MM['min'],DIST_SENSORS_MM['max'], 0, 1))

            for k in range(0,self.robot_num):
                robot_position = []
                robot_position = self.robot_handles[k].getPosition()
                robot_rotation = []
                robot_rotation = self.robot_handles[k].getOrientation()

                observation.append(utils.normalize_to_range(float(robot_position[0]),XPOSITION['min'],XPOSITION['max'],0,1))
                observation.append(utils.normalize_to_range(float(robot_position[1]),YPOSITION['min'],YPOSITION['max'],0,1))
                observation.append(utils.normalize_to_range(float(robot_position[2]),ZPOSITION['min'],ZPOSITION['max'],0,1))
                observation.append(utils.normalize_to_range(float(robot_rotation[0]),-1,1,0,1))
                observation.append(utils.normalize_to_range(float(robot_rotation[1]),-1,1,0,1))
                observation.append(utils.normalize_to_range(float(robot_rotation[2]),-1,1,0,1))
                observation.append(utils.normalize_to_range(float(robot_rotation[3]),-math.pi,math.pi,0,1))


                self.final_distance[k] = utils.get_distance_from_target(self.robot_handles[k],self.final_target)
                observation.append(utils.normalize_to_range(float(self.final_distance[k]),MAX_DISTANCE['min'],MAX_DISTANCE['max'],0,1))                

                self.angles[k] = utils.get_angle_from_target(self.robot_handles[k],self.final_target)
                observation.append(utils.normalize_to_range(float(self.angles[k]),MAX_ANGLE['min'],MAX_ANGLE['max'],0,1))


            for m in range(self.robot_num,Max_robotnum):
                for n in range(9):
                    observation.append(0.5)
            
        else :
            observation = [0 for i in range(OBSERVATION_SPACE)]
        
        self.observation = observation

        return self.observation



                # robot_children = self.robot_handles[k].getField('children')
                # frontjoint_node = robot_children.getMFNode(3)
                # frontjoint = frontjoint_node.getField('jointParameters')
                # frontjoint = frontjoint.getSFNode()
                # para = frontjoint.getField('position')
                # front_hingeposition = para.getSFFloat()
                # observation.append(utils.normalize_to_range(float(front_hingeposition),-math.pi/2,math.pi/2,0,1))

                # front_ep = frontjoint_node.getField('endPoint')
                # front_ep = front_ep.getSFNode()
                # frontrotation_field = front_ep.getField('rotation')
                # front_rotation = frontrotation_field.getSFRotation()
                # for f in range(3):
                #     observation.append(utils.normalize_to_range(float(front_rotation[f]),-1,1,0,1))
                # observation.append(utils.normalize_to_range(float(front_rotation[3]),-math.pi/2,math.pi/2,0,1))

                # robot_children = self.robot_handles[k].getField('children')
                # rearjoint_node = robot_children.getMFNode(4)
                # rearjoint = rearjoint_node.getField('jointParameters')
                # rearjoint = rearjoint.getSFNode()
                # para = rearjoint.getField('position')
                # rear_hingeposition = para.getSFFloat()
                # observation.append(utils.normalize_to_range(float(rear_hingeposition),-math.pi/2,math.pi/2,0,1))

                # rear_ep = rearjoint_node.getField('endPoint')
                # rear_ep = rear_ep.getSFNode()
                # rearrotation_field = rear_ep.getField('rotation')
                # rear_rotation = rearrotation_field.getSFRotation()
                # for r in range(3):
                #     observation.append(utils.normalize_to_range(float(rear_rotation[r]),-1,1,0,1))
                # observation.append(utils.normalize_to_range(float(rear_rotation[3]),-math.pi/2,math.pi/2,0,1))
            
            # final_position = []           
            # final_position = self.final_target.getPosition()
            # observation.append(utils.normalize_to_range(float(final_position[0]),XPOSITION['min'],XPOSITION['max'],0,1))
            # observation.append(utils.normalize_to_range(float(final_position[1]),YPOSITION['min'],YPOSITION['max'],0,1))
            # observation.append(utils.normalize_to_range(float(final_position[2]),ZPOSITION['min'],ZPOSITION['max'],0,1))
            # final_distance = []
            # for d in range(self.robot_num):
            #     final_distance.append(utils.get_distance_from_target(self.robot_handles[d],self.final_target))
            #     self.final_distance[d] = final_distance[d]


    def get_default_observation(self):
        self.observation = [0 for i in range(OBSERVATION_SPACE)]
        return self.observation

    def empty_queue(self):
        self.observation = [0 for i in range(OBSERVATION_SPACE)]
        # self.shockcount = 0
        self.overrangecount = 0
        # self.flagadd = False
        # self.flagreduce = False
        self.dscount = 0
        while self.supervisor.step(self.timestep) != -1:
            if self.receiver.getQueueLength() > 0:
                self.receiver.nextPacket()
            else:
                break
    
    def get_reward(self,action):
        if (self.observation == [0 for i in range(OBSERVATION_SPACE)] or len(self.observation) == 0 ) :
            return 0

        reward = 0

        translations  = []

        for i in range(len(self.robot_handles)):
            translation = self.robot_handles[i].getField('translation').getSFVec3f()
            translations.append(translation)

        if self.steps >= self.steps_threshold:
            return -20

        if np.min(self.ds_values) <= 50:
            reward = reward -2
            self.dscount = self.dscount + 1
            if self.dscount > 60:
                reward = reward -20
                self.should_done = True
            if self.dscount > 30:
                reward = reward - 5
        if np.min(self.ds_values) <= 150:
            reward = reward -1      

        for j in range(len(self.robot_handles)):
            if translations[j][2] <= ZPOSITION['min'] or translations[j][2] >= ZPOSITION['max']:
                reward = reward - 2
                self.overrangecount = self.overrangecount + 1
            if translations[j][0] <= XPOSITION['min'] or translations[j][0] >= ZPOSITION['max']:
                reward = reward - 2
                self.overrangecount = self.overrangecount + 1
        if self.overrangecount >40:
            reward = reward -20
            self.should_done = True  

        if min(self.final_distance) < self.findThreshold:
            reward  = reward + 100
            for m in range(Max_robotnum):
                consumption = self.startbattery - self.endbattery[m]
                reward  =  reward - float(consumption/self.startbattery) * 6
            return reward 
            
        else :
            reward = reward - float(min(self.final_distance))
            return reward 




        # """惩罚不停+-+-的行为 """
        # if action[-1] > 0.9 :
        #     if self.flagreduce == True:
        #         self.shockcount = self.shockcount + 1
        #     self.flagadd = True
        #     self.flagreduce = False
        
        # if action[-1] < 0.1:
        #     if self.flagadd == True:
        #         self.shockcount = self.shockcount + 1
        #     self.flagadd = False
        #     self.flagreduce =True
        
        # if action[-1] >=0.1 and action[-1] <=0.9:
        #     self.shockcount = self.shockcount - 1 
        #     self.flagadd = False
        #     self.flagreduce = False
        
        # if self.shockcount >= 8:
        #     reward = reward - 4
        
        # if self.shockcount >= 12:
        #     reward = reward - 8
        #     self.should_done = True


        # """如果ban的动作值有十个值出现在动作区域，不稳定给负的reward,训练到100代左右时，模块几乎不再动自己的前后motor"""
        # count = 0
        # for k in range(12,24):
        #     action[k] = utils.normalize_to_range(float(action[k]),-0.2,1.2,0,1)
        #     if action[k] > 0.95 or action[k] < 0.05:
        #         count = count + 1
        # if count > 9 :
        #     reward = reward - 2
    



        

    """something worse need to be modified"""
    """加机器人时还需要考虑rearmotor的位置，测试后发现是hingejoint的jointParameters域的position参数，需要找到这个参数"""
    """可以只改变相对应的hingejoint参数使两者结合，也可以改变模块位置和角度，但是改变模块位置和角度比较复杂"""
    # position = abs(get...)
    # 改变hingejoint,只需要改变front hingejoint的position参数
    # 改变模块位置和角度
    # deltax和deltaz可以根据position来计算，主要是rotation要更改，绕x轴旋转(1,0,0,rad)
    # 但是之前寻找模块的位置时已经修改过自己的rotation，所以不好更改,并且更改了rotation，translation也要更改，用这套体姿表征体系更改起来特别复杂
    # 另外，因为是往后加模块，所以除非尾巴上翘，否则都不能这样加（陷到地底下了）
    # 况且，即便尾巴上翘，可以直接加到后ban上，可能也会因为重力原因把整个构型掀翻
    # 综上所述，无论是可行性，还是稳定性原因，都建议只修改front_hingejoint的position值


    def robot_step(self,action):

        # x = np.random.rand()
        # e = 0.8 + ep * 0.2/10000 
        # if x > e :
        #     action[-1] = np.random.rand()

        if action[-1] > 0 and action[-1] <= 02 and self.robot_num < Max_robotnum:

            last_translation = self.robot_handles[-1].getField('translation').getSFVec3f()
            last_angle = self.robot_handles[-1].getField('rotation').getSFRotation()[3]
            last_rotation = self.robot_handles[-1].getField('rotation').getSFRotation()

            delta_z = 0.23 * math.cos(last_angle)
            delta_x = 0.23 * math.sin(last_angle)
            new_translation = []
            new_translation.append(last_translation[0] - delta_x)
            new_translation.append(last_translation[1])
            new_translation.append(last_translation[2] - delta_z)

            robot_children = self.robot_handles[-1].getField('children')
            rearjoint_node = robot_children.getMFNode(4)
            joint = rearjoint_node.getField('jointParameters')
            joint = joint.getSFNode()
            para = joint.getField('position')
            hingeposition = para.getSFFloat()


            # if hingeposition > 0.8 or hingeposition < -0.8:
            #     delta = 0.03 - 0.03 * math.cos(hingeposition)
            #     delta_z = delta * math.cos(last_angle)
            #     delta_x = delta * math.sin(last_angle)
            #     new_translation[0] = new_translation[0] + delta_x
            #     new_translation[2] = new_translation[2] + delta_z

            new_rotation = []
            for i in range(4):
                new_rotation.append(last_rotation[i])
            
            flag_translation = False
            flag_rotation = False
            flag_front = False
            flag_frontposition = False
            flag_frontrotation = False
            battery_remain = float(self.endbattery[self.robot_num])

            importname = "robot_" + str(self.robot_num) + '.wbo'
            new_file =[]
            with open(importname,'r') as f:
                lines = f.readlines()
                for line in lines:
                    if "translation" in line:
                        if flag_translation == False:
                            replace = "translation " + str(new_translation[0]) + " " + str(new_translation[1]) + " " + str(new_translation[2])
                            line = "\t" + replace +'\n'
                            flag_translation = True
                    if "rotation" in line:
                        if flag_rotation == False:
                            replace = "rotation " + str(new_rotation[0])  + " " + str(new_rotation[1]) + " " + str(new_rotation[2]) + " " \
                            +str(new_rotation[3])  
                            line = "\t" + replace +'\n'
                            flag_rotation = True
                    if 'front HingeJoint' in line:
                        flag_front = True
                    if 'position' in line:
                        if flag_front == True and flag_frontposition ==False: 
                            repalce = "position "+ str(-hingeposition)
                            line = "\t\t\t\t" + repalce + '\n'
                            flag_frontposition = True
                    if 'rotation' in line :
                        if flag_front == True and flag_frontrotation == False:
                            replace = "rotation " + str(1)+ ' ' + str(0)+ ' ' + str(0) + ' ' + str(-hingeposition)
                            line = "\t\t\t\t" + replace + '\n'
                            flag_frontrotation = True
                    if "50000" in line :
                        line = "\t\t" + str(battery_remain) + "," + " " + str(50000) + '\n'
                    new_file.append(line)
            with open(importname,'w') as f:
                for line in new_file:
                    f.write(line)
    

            rootNode = self.supervisor.getRoot()
            childrenField = rootNode.getField('children')
            childrenField.importMFNode(-1,importname)
            defname = 'robot_' + str(self.robot_num)
            self.robot_handles.append(self.supervisor.getFromDef(defname))
            self.robot_num = self.robot_num + 1

            # new_translation_field = self.robot_handles[-1].getField('translation')  
            # new_translation_field.setSFVec3f(new_translation)

            # new_rotation_field = self.robot_handles[-1].getField('rotation')
            # new_rotation_field.setSFRotation(new_rotation)

            # robot_children = self.robot_handles[-1].getField('children')
            # frontjoint_node = robot_children.getMFNode(3)
            # joint = frontjoint_node.getField('jointParameters')
            # joint = joint.getSFNode()
            # para = joint.getField('position')
            # para.setSFFloat(-hingeposition)
            # battery_remain = float(self.endbattery[self.robot_num - 1])
            # battery_field = self.robot_handles[-1].getField('battery')
            # battery_field.setMFFloat(0,battery_remain)
            # battery_field.setMFFloat(1,self.startbattery)
            
        elif action[-1] >=0.8 and action[-1] <1  and self.robot_num >1:

            battery_field = self.robot_handles[-1].getField('battery')
            battery_remain = battery_field.getMFFloat(0)
            self.endbattery[self.robot_num - 1] = battery_remain
            removerobot = self.robot_handles[-1]
            removerobot.remove()
            self.robot_num = self.robot_num - 1
            del(self.robot_handles[-1])

        
    def step(self,action):
        if self.supervisor.step(self.timestep) == -1:
            exit()
        self.handle_emitter(action)
        key = self.keyboard.getKey()

        observation  = self.get_observations()
        reward = self.get_reward(action)
        isdone = self.is_done()
        info = self.get_info()
        
        if key == Keyboard.CONTROL + ord("A"):
            print()
            print("Actions: ", action)
        if key == ord("R"):
            print()
            print("Rewards: ", reward)
        if key == Keyboard.CONTROL + ord("Y"):
            print()
            print("Observations: ", observation)
        if key == Keyboard.CONTROL + ord("M"):
            print()
            print("message", self.message)

        if (self.v_action > 1):
            self.file_writer.add_histogram(
                "Actions/Per Global Step",
                action,
                global_step=self.step_global)

        if (self.v_observation > 1):
            self.file_writer.add_histogram(
                "Observations/Per Global Step",
                observation,
                global_step=self.step_global)

        if (self.v_reward > 1):
            self.file_writer.add_scalar("Rewards/Per Global Step", reward,
                                        self.step_global)

        if (isdone):
            self.file_writer.add_scalar(
                "Is Done/Per Reset step",
                self.step_cntr,
                global_step=self.step_reset)

        self.file_writer.flush()

        self.score += reward

        self.step_cntr += 1
        self.step_global += 1

        return observation,reward,isdone,info


    def is_done(self):
        self.steps = self.steps +  1
        self.file_writer.flush()

        if min(self.final_distance) <= self.findThreshold:
            print("======== + Solved + ========")
            return True
        if self.steps >= self.steps_threshold or self.should_done:
            return True
    
        # rotation_field = self.robot_handles[0].getField('rotation').getSFRotation()

        # """需要计算出模块完全侧边倒的rotation是多少，遇到这种情况直接进行下一次迭代"""
        # # if rotation_field[0] < -0.4 and rotation_field[1] > 0.4 and rotation_field[2] > 0.4 and rotation_field[3] < -1.5708:
        # #     return True

        return False

    def reset(self):
        print("Reset simulation")
        self.respawnRobot()
        self.steps = 0
        self.should_done = False
        self.robot_num = 1
        """observation 源代码wrapper有问题"""
        self.score_history.append(self.score)
        if (self.v_reward > 0):
            self.file_writer.add_scalar(
                "Score/Per Reset", self.score, global_step=self.step_reset)

            for window in self.windows:
                if self.step_reset > window:
                    self.file_writer.add_scalar(
                        "Score/With Window {}".format(window),
                        np.average(self.score_history[-window:]),
                        global_step=self.step_reset - window)

        self.file_writer.flush()

        self.step_reset += 1
        self.step_cntr = 0
        self.score = 0
        return self.get_default_observation()

    def flush(self):
        if self._file_writer is not None:
            self._file_writer.flush()

    def close(self):
        if self._file_writer is not None:
            self._file_writer.close()

    def get_info(self):
        pass

    def respawnRobot(self):
        for robot in self.robot_handles:
                robot.remove()
        rootNode = self.supervisor.getRoot()
        childrenField = rootNode.getField('children')
        childrenField.importMFNode(-1,"robot_0.wbo")
        # childrenField.importMFNode(-1,"robot_1.wbo")
        # childrenField.importMFNode(-1,"robot_2.wbo")
        # childrenField.importMFNode(-1,"robot_3.wbo")
        # childrenField.importMFNode(-1,"robot_4.wbo")
        # childrenField.importMFNode(-1,"robot_5.wbo")


        self.robot_handles = []

        for defrobotname in self.robot_list:
            self.robot_handles.append(self.supervisor.getFromDef(defrobotname))

        self.final_target = self.supervisor.getFromDef('final_target')
        self.supervisor.simulationResetPhysics()
        self._last_message = None


robot_defnames = ['robot_0']

supervisor_env = TaskDecisionSupervisor(robot_defnames, observation_space=OBSERVATION_SPACE,log_dir="logs/results/ddpg", v_action=1,v_observation=1,v_reward=1,windows=[10,\
                                        10000, 2000])



agent = TD3(lr_actor=0.00025,
             lr_critic=0.0025,
             input_dims= OBSERVATION_SPACE,
             gamma=0.99,
             tau=0.001,
             env=supervisor_env,
             batch_size=512,
             layer1_size=400,
             layer2_size=300,
             layer3_size=200,
             layer4_size=400,
             layer5_size=300,
             layer6_size=200,  
             n_actions=ACTION_SPACE,
             load_models=False,
             save_dir='./models/saved/ddpg/')

score_history = []

np.random.seed(0)

for i in range(1, 20000):
    done = False
    score = 0
    obs = list(map(float, supervisor_env.reset()))
    supervisor_env.empty_queue()
    first_iter = True
    if i % 10000 == 0:
        print("================= TESTING =================")
        while not done:
            act = agent.choose_action_test(obs).tolist()
            supervisor_env.robot_step(act)
            new_state, _, done, _ = supervisor_env.step(act)
            obs = list(map(float, new_state))
    else:
        print("================= TRAINING =================")
        while not done:
            if (not first_iter):
                act = agent.choose_action_train(obs).tolist()
            else:
                first_iter = False

                act = [0,0]
                for k in range(0,13):
                    act.append(0.5)
 
            supervisor_env.robot_step(act)
            new_state, reward, done, info = supervisor_env.step(act)
            agent.remember(obs, act, reward, new_state, int(done))
            agent.learn()
            score += reward

            obs = list(map(float, new_state))

    score_history.append(score)
    print("===== Episode", i, "score %.2f" % score,
          "100 game average %.2f" % np.mean(score_history[-100:]))

    if i % 100 == 0:
        agent.save_models()