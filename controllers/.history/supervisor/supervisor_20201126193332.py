import math

import numpy as np
from numpy import random
from numpy.core.fromnumeric import size

import utilities as utils
from deepbots.supervisor.controllers.supervisor_emitter_receiver import \
    SupervisorCSV
from deepbots.supervisor.wrappers.tensorboard_wrapper import TensorboardLogger
from models.networks import TD3
from controller import Keyboard

smalltarget_num = 3
Max_robotnum = 6
MAX_DSNUM = (Max_robotnum-1) * 4 + 7
OBSERVATION_SPACE = (Max_robotnum-1) * 4 + 7 + smalltarget_num * 3 + 7
ACTION_SPACE = Max_robotnum * 4 + 1

# ANGLE_MM = {'min': -math.pi, 'max': math.pi}
DIST_SENSORS_MM = {'min': 0, 'max': 1000}
# EUCL_MM = {'min': 0, 'max': 30}
XPOSITION = {'min':-5, 'max':5}
YPOSITION = {'min':0 , 'max':1.5}
ZPOSITION = {'min': -5, 'max' : 5}

# import ptvsd
# print("waiting for debugger attach")
# ptvsd.enable_attach(address=("127.0.0.1",7788))
# ptvsd.wait_for_attach()



class TaskDecisionSupervisor(SupervisorCSV):
    def __init__(self,robot,target,observation_space):
        super(TaskDecisionSupervisor,self).__init__()
        self.timestep = int(self.supervisor.getBasicTimeStep())
        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)
        self.robot_list = robot
        self.target_list = target
        self.robot_handles = []
        self.target_handles = []
        self.emitter = self.supervisor.getEmitter('emitter')
        self.receiver = self.supervisor.getReceiver('receiver')
        self.observation = [0 for i in range(observation_space)]
        self.findThreshold = 0.2
        self.steps = 0
        self.steps_threshold = 10000    

        self.endbattery = [50000 for i in range(Max_robotnum)]
        self.ds_values = []
        self.distances = []
        self.angles = []
        self.final_distance = 100
        self.final_angle = 0

        
        self.target_signs = [1 for i in range(len(self.target_list))]
        self.final_target = self.supervisor.getFromDef('final_target')
        

        self.should_done = False
        self.startbattery = 50000
        self.small_done = False
        self.target_remain = len(self.target_list)

        self.setuprobots()
        self.setuptargets()
        



    def setuprobots(self):
        for defname in self.robot_list:
            self.robot_handles.append(self.supervisor.getFromDef(defname))
        


    def setuptargets(self):
        
        for defname in self.target_list:
            self.target_handles.append(self.supervisor.getFromDef(defname))
        

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
        self.distances = []
        self.angles = []
        self.final_distance = 100
        self.final_angle = 0
        self.message = [1000 for i in range(MAX_DSNUM)]
        observation = []
        message = self.handle_receiver()

        if len(message) != 0:
            for j in range(len(message)):
                self.message[j] = float(message[j])
                self.ds_values.append(float(message[j]))

            for k in range(MAX_DSNUM):
                observation.append(utils.normalize_to_range(float(self.message[k]),DIST_SENSORS_MM['min'],DIST_SENSORS_MM['max'], 0, 1))
            self.robot_position = []
            self.robot_position=self.robot_handles[0].getPosition()
            self.robot_rotation = []
            self.robot_rotation = self.robot_handles[0].getOrientation()
            observation.append(utils.normalize_to_range(float(self.robot_position[0]),XPOSITION['min'],XPOSITION['max'],0,1))
            observation.append(utils.normalize_to_range(float(self.robot_position[1]),YPOSITION['min'],YPOSITION['max'],0,1))
            observation.append(utils.normalize_to_range(float(self.robot_position[2]),ZPOSITION['min'],ZPOSITION['max'],0,1))
            observation.append(utils.normalize_to_range(float(self.robot_rotation[0]),-1,1,0,1))
            observation.append(utils.normalize_to_range(float(self.robot_rotation[1]),-1,1,0,1))
            observation.append(utils.normalize_to_range(float(self.robot_rotation[2]),-1,1,0,1))
            observation.append(utils.normalize_to_range(float(self.robot_rotation[3]),-math.pi,math.pi,0,1))
            
            for target in self.target_handles:
                target_position = []
                target_position = target.getPosition()
                observation.append(utils.normalize_to_range(float(target_position[0]),XPOSITION['min'],XPOSITION['max'],0,1))
                observation.append(utils.normalize_to_range(float(target_position[1]),YPOSITION['min'],YPOSITION['max'],0,1))
                observation.append(utils.normalize_to_range(float(target_position[2]),ZPOSITION['min'],ZPOSITION['max'],0,1))
                
                
            
            for target in self.target_handles :

                distanceFromTarget = utils.get_distance_from_target(self.robot_handles[0],target)
                self.distances.append(distanceFromTarget)


            final_distance = utils.get_distance_from_target(self.robot_handles[0],self.final_target)
            self.final_distance = final_distance

        else :
            observation = [0 for i in range(OBSERVATION_SPACE)]
        
        self.observation = observation

        return self.observation

    def empty_queue(self):
        self.message = [1000 for i in range(MAX_DSNUM)]
        self.observation = [0 for i in range(OBSERVATION_SPACE)]
        self.ds_values = []
        self.dscount = 0
        self.shockcount = 0
        self.overrangecount = 0
        self.flagadd = False
        self.flagrecude = False
        while self.supervisor.step(self.timestep) != -1:
            if self.receiver.getQueueLength() > 0:
                self.receiver.nextPacket()
            else:
                break
    
    def get_reward(self,action):
        if (self.observation == [0 for i in range(OBSERVATION_SPACE)] or len(self.ds_values) == 0 or len(self.message) == 0 \
        or len(self.observation) == 0 or self.message is None or self.message == [1000 for i in range(OBSERVATION_SPACE)]) :
            return 0

        distances = []
        reward = -1
        translations  = []

        for i in range(len(self.robot_handles)):
            translation = self.robot_handles[i].getField('translation').getSFVec3f()
            translations.append(translation)
        
        for j in range(len(self.robot_handles)):
            if translations[j][2] <= -5 or translations[j][2] >= 5:
                reward = reward - 2
            if translations[j][0] <= -5 or translations[j][0] >= 5:
                reward = reward - 2

        if self.steps >= self.steps_threshold:
            return -20

        if np.min(self.ds_values) <= 50:
            reward = reward -2
            self.dscount = self.dscount + 1
            # if self.dscount > 40:
            #     reward = reward -20 
                # self.should_done = True
        if np.min(self.ds_values) <= 150:
            reward = reward -1
        

        """惩罚不停+-+-的行为 """
        if action[-1] > 0.9 :
            if self.flagrecude == True:
                self.shockcount = self.shockcount + 1
            self.flagadd = True
            self.flagrecude = False
        
        if action[-1] < 0.1:
            if self.flagadd == True:
                self.shockcount = self.shockcount + 1
            self.flagadd = False
            self.flagrecude =True
        
        if action[-1] >=0.1 and action[-1] <=0.9:
            self.shockcount = self.shockcount - 1 
            self.flagadd = False
            self.flagrecude = False
        
        if self.shockcount >= 8:
            reward = reward - 4
        
        if self.shockcount >= 12:
            reward = reward - 8
            self.should_done = True
                


        """如果ban的动作值有九个值出现在动作区域，不稳定给负的reward"""
        count = 0
        for k in range(12,24):
            if action[k] > 0.95 or action[k] < 0.05:
                count = count + 1
        if count > 8 :
            reward = reward - 2
    



        if self.final_distance < self.findThreshold:
            reward  = +20
            for m in range(Max_robotnum):
                consumption = self.startbattery - self.endbattery[m]
                reward  =  reward - float(consumption/self.startbattery) * 6
            
            if self.small_done == True:
                return reward
            else:
                reward = reward - self.target_remain * 2
            return reward 
            

        else :
            if self.small_done == True:
                reward = reward + float(1/self.final_distance)
                return reward 
            else :
                for target in self.target_handles :
                    distanceFromTarget = utils.get_distance_from_target(self.robot_handles[0],target)
                    distances.append(distanceFromTarget)

                for sign in self.target_signs:

                    mindistance = min(distances)
                    minindex = distances.index(mindistance)

                    if self.target_signs[minindex] ==1:
                        reward = reward + float(1/mindistance)
                        return reward 
                    else :
                        distances[minindex]  =  distances[minindex] + 1000

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


    def robot_step(self,action,ep):
        # x = np.random.rand()
        # e = 0.8 + ep * 0.2/10000 
        # if x > e :
        #     action[-1] = np.random.rand()

        if action[-1] > 0.9 and action[-1] <= 1 and self.robot_num < Max_robotnum:


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

            rootNode = self.supervisor.getRoot()
            childrenField = rootNode.getField('children')
            importname = "robot_" + str(self.robot_num) + '.wbo'
            childrenField.importMFNode(-1,importname)

            defname = 'robot_' + str(self.robot_num)
            
            self.robot_handles.append(self.supervisor.getFromDef(defname))
            self.robot_num = self.robot_num + 1
            new_translation_field = self.robot_handles[-1].getField('translation')  
            new_translation_field.setSFVec3f(new_translation)
            new_rotation = []
            for i in range(4):
                new_rotation.append(last_rotation[i])
            
            new_rotation_field = self.robot_handles[-1].getField('rotation')
            new_rotation_field.setSFRotation(new_rotation)

            robot_children = self.robot_handles[-1].getField('children')
            rearjoint_node = robot_children.getMFNode(3)
            joint = rearjoint_node.getField('jointParameters')
            joint = joint.getSFNode()
            para = joint.getField('position')
            para.setSFFloat(-hingeposition)
            


            battery_remain = float(self.endbattery[self.robot_num - 1])
            battery_field = self.robot_handles[-1].getField('battery')
            battery_field.setMFFloat(0,battery_remain)
            battery_field.setMFFloat(1,self.startbattery)
            
        elif action[-1] >0 and action[-1] <= 0.1 and self.robot_num >1:

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
        key = self.keyboard.getKey()
        self.handle_emitter(action)

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
            print("Message :",self.message)

        return observation,reward,isdone,info


    def is_done(self):
        self.steps = self.steps +  1

        if self.small_done == False:
            for i in range(len(self.distances)) :
                if self.distances[i] <= self.findThreshold:
                    if self.target_signs[i] == 1:
                        self.target_signs[i] = 0
                        self.target_remain  = self.target_remain - 1
                        print(self.target_list[i] + ' has solved')

            if self.target_remain == 0:
                self.small_done = True


        if self.final_distance <= self.findThreshold:
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
        self.message = []
        self.small_done =False
        self.target_remain = len(self.target_list)
        self.robot_num = 2
        self.ds_values = []
        self.target_signs = [1 for i in range(len(self.target_list))]
        return self.observation

    def get_info(self):
        pass

    def respawnRobot(self):
        for robot in self.robot_handles:
            if robot is not None:
                robot.remove()
        rootNode = self.supervisor.getRoot()
        childrenField = rootNode.getField('children')
        childrenField.importMFNode(-1,"robot_0.wbo")
        childrenField.importMFNode(-1,"robot_1.wbo")


        self.robot_handles = []
        self.target_handles = []
        for defrobotname in self.robot_list:
            self.robot_handles.append(self.supervisor.getFromDef(defrobotname))
        for deftargetname in self.target_list:
            self.target_handles.append(self.supervisor.getFromDef(deftargetname))

        
        self.final_target = self.supervisor.getFromDef('final_target')
        self.supervisor.simulationResetPhysics()
        self._last_message = None


robot_defnames = ['robot_0','robot_1']
target_defnames = ['target_0','target_1','target_2']

supervisor_pre = TaskDecisionSupervisor(robot_defnames, target_defnames, observation_space=43)
supervisor_env = TensorboardLogger(supervisor_pre,log_dir="logs/results/ddpg", v_action=1,v_observation=1,v_reward=1,windows=[10, 10000, 2000])


agent = TD3(lr_actor=0.00025,
             lr_critic=0.0025,
             input_dims=43,
             gamma=0.99,
             tau=0.001,
             env=supervisor_env,
             batch_size=256,
             layer1_size=400,
             layer2_size=300,
             layer3_size=200,
             layer4_size=400,
             layer5_size=300,
             layer6_size=200,  
             n_actions=25,
             load_models=False,
             save_dir='./models/saved/ddpg/')

score_history = []

np.random.seed(0)

for i in range(1, 20000):
    done = False
    score = 0
    obs = list(map(float, supervisor_env.reset()))
    supervisor_pre.empty_queue()
    first_iter = True
    if i % 10000 == 0:
        print("================= TESTING =================")
        while not done:
            act = agent.choose_action_test(obs).tolist()
            supervisor_pre.robot_step(act,i)
            new_state, _, done, _ = supervisor_env.step(act)
            obs = list(map(float, new_state))
    else:
        print("================= TRAINING =================")
        while not done:
            if (not first_iter):
                act = agent.choose_action_train(obs).tolist()
            else:
                first_iter = False

                act = [0 for j in range(12)]
                for k in range(13):
                    act.append(0.5)
 
            supervisor_pre.robot_step(act,i)
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