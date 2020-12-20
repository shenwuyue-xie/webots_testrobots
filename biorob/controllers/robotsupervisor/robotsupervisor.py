import math

import numpy as np

import utilities as utils

from collections.abc import Iterable

from controller import Supervisor
from deepbots.supervisor.wrappers.keyboard_printer import KeyboardPrinter
from deepbots.supervisor.wrappers.tensorboard_wrapper import TensorboardLogger
from models.networks import DDPG



OBSERVATION_SPACE = 27
ACTION_SPACE = 8

ANGLE_MM = {'min': -math.pi, 'max': math.pi}
DIST_SENSORS_MM = {'min': 0, 'max': 1000}
EUCL_MM = {'min': 0, 'max': 30}




class TaskDecisionSupervisor(Supervisor):
    def __init__(self,robot,target,observation_space):
        super(TaskDecisionSupervisor,self).__init__()
        self.timestep = int(self.getBasicTimeStep())

        self.robot = self.getSelf()

        self.robot_name = self.getName()

        self.robot_list = robot
        self.robot_handles = []
        self.target_list = target
        self.target_handles = []
        
        self.emitter = self.getEmitter('emitter')
        self.receiver = self.getReceiver('receiver')
        self.receiver.enable(self.timestep)


        self.observation = [0 for i in range(observation_space)]
        self.findThreshold = 0.2
        self.steps = 0
        self.steps_threshold = 10000
        self.message = []
        self.target_signs = [1 for i in range(len(self.target_list))]
        self.final_target = self.getFromDef('final_target')
        self.should_done = False
        self.startbattery = 600000
        self.small_done = False
        self.target_remain = len(self.target_list)

        self.setupmotors()     
        self.setupsensors()
        
        for defname in self.target_list:
            self.target_handles.append(self.getFromDef(defname))
   
        for defname in self.robot_list:
            self.robot_handles.append(self.getFromDef(defname))


    def get_timestep(self):
        return int(self.getBasicTimeStep())

    def setupmotors(self):

        self.leftmotor= self.getMotor('left_motor')
        self.rightmotor= self.getMotor('right_motor')
        self.frontmotor = self.getMotor('front_motor')
        self.rearmotor = self.getMotor('rear_motor')
        self.leftmotor.setPosition(float('inf'))
        self.rightmotor.setPosition(float('inf'))
        self.leftmotor.setVelocity(0.0)
        self.rightmotor.setVelocity(0.0)
        self.motorSpeeds = [0.0 for i in range(0,8)]

    def setupsensors(self):
        self.distanceSensors = []

        if self.robot_name == "0":
            self.n_distanceSensors = 7
        else :
            self.n_distanceSensors = 4

        self.dsNames = ['ds' + str(i) for i in range(self.n_distanceSensors)]
        for i in range(self.n_distanceSensors):
            self.distanceSensors.append(self.getDistanceSensor(self.dsNames[i]))
            self.distanceSensors[i].enable(self.timestep)




    def apply_action(self,action):
        basic = float(action[1])
        turning = float(action[0])       
        basic = basic * 4
        turning = turning * 2
        turning = np.clip(turning,-2,2)
        self.motorSpeeds[0]=basic + turning
        self.motorSpeeds[1]=basic - turning

        basic = float(action[3])
        turning = float(action[2])       
        basic = basic * 4
        turning  = turning * 2
        turning = np.clip(turning,-2,2)
        self.motorSpeeds[2]=basic + turning
        self.motorSpeeds[3]=basic - turning

        basic = float(action[5])
        turning = float(action[4])        
        basic = basic * 4
        turning  = turning * 2
        turning = np.clip(turning,-2,2)
        self.motorSpeeds[4]=basic + turning
        self.motorSpeeds[5]=basic - turning

        basic = float(action[7])
        turning = float(action[6])       
        basic = basic * 4
        turning  = turning * 2
        turning = np.clip(turning,-2,2)
        self.motorSpeeds[6]=basic + turning
        self.motorSpeeds[7]=basic - turning

        self.motorSpeeds = np.clip(self.motorSpeeds, -4, 4)

        self.leftMotor0.setVelocity(self.motorSpeeds[0])
        self.rightmotor0.setVelocity(self.motorSpeeds[1])

        self.leftMotor1.setVelocity(self.motorSpeeds[2])
        self.rightmotor1.setVelocity(self.motorSpeeds[3])

        self.leftMotor2.setVelocity(self.motorSpeeds[4])
        self.rightmotor2.setVelocity(self.motorSpeeds[5])

        self.leftMotor3.setVelocity(self.motorSpeeds[6])
        self.rightmotor3.setVelocity(self.motorSpeeds[7])
    
    def step(self,action):
        if self.step(self.timestep) == -1:
            exit()

        self.apply_action(action)
        return (
            self.get_observations(),
            self.get_reward(action),
            self.is_done(),
            self.get_info(),
        )
    def get_info(self):
        pass
    
    def create_message(self):
        message = []
        message_0 = []
        message_1 = []
        message_2 = []
        message_3 = []
        
        if self.robot_name == "0":
            for i in range(self.n_distanceSensors):
                message_0.append(self.distanceSensors[i].getValue())
        if self.robot_name == "1":
            for i in range(self.n_distanceSensors):
                message_1.append(self.distanceSensors[i].getValue())
        if self.robot_name == "2":
            for i in range(self.n_distanceSensors):
                message_2.append(self.distanceSensors[i].getValue())
        if self.robot_name == "3":
            for i in range(self.n_distanceSensors):
                message_3.append(self.distanceSensors[i].getValue())

        message[0:7] = message_0
        message[7:11] = message_1
        message[11:15] = message_2
        message[15:19] = message_3
    
        return message
        


    def  get_observations(self):
        onobservation = []
        onmessage = self.create_message()
        if onmessage is not None :
            for i in range(len(onmessage)):
                self.message.append(float(onmessage[i]))
                onobservation.append(utils.normalize_to_range(float(onmessage[i]),DIST_SENSORS_MM['min'],DIST_SENSORS_MM['max'], 0, 1))
            for target in self.target_handles :

                distanceFromTarget = utils.get_distance_from_target(self.robot_handles[0],target)
                self.message.append(distanceFromTarget)
                distanceFromTarget = utils.normalize_to_range(distanceFromTarget, EUCL_MM['min'], EUCL_MM['max'], 0, 1)
                onobservation.append(distanceFromTarget)

                angleFromTarget = utils.get_angle_from_target(self.robot_handles[0],target,is_true_angle=True,is_abs=False)
                self.message.append(angleFromTarget)
                angleFromTarget = utils.normalize_to_range(angleFromTarget,ANGLE_MM['min'],ANGLE_MM['max'], 0, 1)
                onobservation.append(angleFromTarget)

            final_distance = utils.get_distance_from_target(self.robot_handles[0],self.final_target)
            self.message.append(final_distance)
            final_distance = utils.normalize_to_range(final_distance,EUCL_MM['min'], EUCL_MM['max'], 0, 1)
            onobservation.append(final_distance)

            final_angel = utils.get_angle_from_target(self.robot_handles[0],self.final_target,is_true_angle=True,is_abs=False)
            self.message.append(final_angel)
            final_angel = utils.normalize_to_range(final_angel,ANGLE_MM['min'],ANGLE_MM['max'], 0, 1)
            onobservation.append(final_angel)

        else :
            onobservation = [0 for i in range(OBSERVATION_SPACE)]
        
        self.observation = onobservation

        return self.observation


    
    """reward待定"""
    def get_reward(self,action):
        if (self.message is None or len(self.message) == 0 or self.observation is None):
            return 0

        Sensor_Number = 19
        ds_values = np.array(self.message[:Sensor_Number])
        distances = []
        reward = 0.0
        self.endbattery = []
        final_distance = utils.get_distance_from_target(self.robot_handles[0],self.final_target)

        if self.steps > self.steps_threshold:
            return -20

        if np.min(ds_values)<100:
            return -0.5


        if np.min(ds_values) < 50:
            self.should_done = True
            return -1
        """这里action和reward是怎么对应的不太清楚 """
        # if np.abs(action[1]) > 1.5 or np.abs(action[0]) > 1.5:
        #     if self.steps > 10:
        #         self.should_done = True
        #     return -1

        if final_distance < self.findThreshold:
            reward  = +20
            for robot in self.robot_handles:
                self.endbattery.append(robot.batterySensorGetValue())
            for i in range(len(self.robot_handles)):
                consumption=self.startbattery-self.endbattery[i]
                reward  =  reward - float(consumption/self.startbattery)
            if self.small_done == True:
                return reward
            else:
                reward = reward - self.target_remain * 2
                # for i in range(len(self.target_list)):
                #     if target_signs[i] == 1:
                #         reward -= 2
            return reward 
            

        else :
            if self.small_done == True:
                reward = float(0.5/final_distance)
                return reward 
            else :
                for target in self.target_handles :
                    distanceFromTarget = utils.get_distance_from_target(self.robot_handles[0],target)
                    distances.append(distanceFromTarget)

                for sign in self.target_signs:
                    if sign == 1:
                        mindistance = min(distances)
                        minindex = distances.index(mindistance)

                        if self.target_signs[minindex] ==1:
                            reward = float(0.5/mindistance)
                            return reward 
                        else :
                            distances[minindex]  =  distances[minindex] + 1000

    def is_done(self):
        self.steps =   self.steps +  1

        if self.small_done == False:

            distances = []

            for target in self.target_handles :
                distanceFromTarget = utils.get_distance_from_target(self.robot,target)
                distances.append(distanceFromTarget)


            findsign = False
            
            for i in range(len(distances)) :
                if distances[i] < self.findThreshold:
                    if self.target_signs[i] == 1:
                        self.target_signs[i] = 0
                        self.target_remain  = self.target_remain - 1
                        print(self.target_list[i] + 'has solved')
                else :
                    if findsign == False:
                        findsign = True

            if findsign == False:
                self.small_done = True


        final_distance = utils.get_distance_from_target(self.robot,self.final_target)

        if final_distance < self.findThreshold:
            print("======== + Solved + ========")
            return True
        if self.steps > self.steps_threshold or self.should_done:
            return True

        return False

    def reset(self):
        print("Reset simulation")
        self.respawnRobot()
        self.steps = 0
        self.should_done = False
        self.message = None
        self.small_done =False
        self.target_remain = len(self.target_list)
        return self.observation


    def get_info(self):
        pass

    def respawnRobot(self):
        for robot in self.robot_handles:
            if robot is not None:
                robot.remove()
        
        rootNode = self.getRoot()
        childrenField = rootNode.getField('children')

        childrenField.importMFNode(-1,"robot_0.wbo")
        childrenField.importMFNode(-1,"robot_1.wbo")
        childrenField.importMFNode(-1,"robot_2.wbo")
        childrenField.importMFNode(-1,"robot_3.wbo")

        robot_handles = []
        target_handles = []
        for defname in self.robot_list:
            robot_handles.append(self.getFromDef(defname))
        for defname in self.target_list:
            target_handles.append(self.getFromDef(defname))
        
        self.final_target = self.getFromDef('final_target')
        self.simulationReset()
        self._last_message = None


robot_defnames =['robot_0','robot_1','robot_2','robot_3']
target_defnames = ['target_0','target_1','target_2']

supervisor_pre = TaskDecisionSupervisor(robot_defnames, target_defnames, observation_space=27)
supervisor_env = KeyboardPrinter(supervisor_pre)
supervisor_env = TensorboardLogger(supervisor_env,log_dir="logs/results/ddpg", v_action=1,v_observation=1,v_reward=1,windows=[10, 100, 200])


agent = DDPG(lr_actor=0.00025,
             lr_critic=0.0025,
             input_dims=[27],
             gamma=0.99,
             tau=0.001,
             env=supervisor_env,
             batch_size=256,
             layer1_size=400,
             layer2_size=300,
             n_actions= 8 ,
             load_models=False,
             save_dir='./models/saved/ddpg/')

score_history = []

np.random.seed(0)

for i in range(1, 500):
    done = False
    score = 0
    obs = list(map(float, supervisor_env.reset()))
    first_iter = True
    if i % 250 == 0:
        print("================= TESTING =================")
        while not done:
            act = agent.choose_action_test(obs).tolist()
            new_state, _, done, _ = supervisor_env.step(act)
            obs = list(map(float, new_state))
    else:
        print("================= TRAINING =================")
        while not done:
            if (not first_iter):
                act = agent.choose_action_train(obs).tolist()
            else:
                first_iter = False
                act = [0, 0]

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
