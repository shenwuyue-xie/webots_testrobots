import os

import numpy as np
from numpy.testing._private.nosetester import NoseTester

import torch as T
from torch._C import dtype
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from models.noise_generator import OUActionNoise
from models.replay_buffer import ReplayBuffer



class CriticNetwork(nn.Module):
    def __init__(self,
                 lr,
                 input_dims,
                 fc1_dims,
                 fc2_dims,
                 fc3_dims,
                 fc4_dims,
                 fc5_dims,
                 fc6_dims,
                 n_actions,
                 name,
                 chkpt_dir='./models/saved/ddpg/'):
        super(CriticNetwork, self).__init__()

        self.input_dims = input_dims
        self.fc1_dims = fc1_dims
        self.fc2_dims = fc2_dims
        self.fc3_dims = fc3_dims
        self.fc4_dims = fc4_dims
        self.fc5_dims = fc5_dims
        self.fc6_dims = fc6_dims

        self.n_actions = n_actions
        self.checkpoint_file = os.path.join(chkpt_dir, name + '_ddpg')

        self.fc1 = nn.Linear(self.input_dims, self.fc1_dims)
        # self.bn1 = nn.LayerNorm(self.fc1_dims)
        self.fc2 = nn.Linear(self.fc1_dims, self.fc2_dims)
        # self.bn2 = nn.LayerNorm(self.fc2_dims)

        self.action_value_1 = nn.Linear(self.n_actions, fc2_dims)
        self.fc3 = nn.Linear(fc2_dims + fc2_dims, fc3_dims)
        self.q1 = nn.Linear(fc3_dims, 1)

        self.fc4 = nn.Linear(self.input_dims, self.fc4_dims)
        self.fc5 = nn.Linear(self.fc4_dims, self.fc5_dims)

        self.action_value_2 = nn.Linear(self.n_actions, fc5_dims)
        self.fc6 = nn.Linear(fc5_dims + fc5_dims, fc6_dims)
        self.q2 = nn.Linear(fc6_dims, 1)

        self.initialization()

        self.optimizer = optim.RMSprop(self.parameters(), lr=lr)

        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')

        self.to(self.device)

    def initialization(self):
        nn.init.xavier_uniform_(self.fc1.weight,
                                gain=nn.init.calculate_gain('leaky_relu'))

        nn.init.xavier_uniform_(self.fc2.weight,
                                gain=nn.init.calculate_gain('leaky_relu'))

        nn.init.xavier_uniform_(self.action_value_1.weight,
                                gain=nn.init.calculate_gain('leaky_relu'))

        nn.init.xavier_uniform_(self.fc3.weight,
                                gain=nn.init.calculate_gain('relu'))

        nn.init.xavier_uniform_(self.fc4.weight,
                                gain=nn.init.calculate_gain('leaky_relu'))

        nn.init.xavier_uniform_(self.fc5.weight,
                                gain=nn.init.calculate_gain('leaky_relu'))

        nn.init.xavier_uniform_(self.action_value_2.weight,
                                gain=nn.init.calculate_gain('leaky_relu'))

        nn.init.xavier_uniform_(self.fc6.weight,
                                gain=nn.init.calculate_gain('leaky_relu'))
        

    def forward(self, state, action):
        state_value_1 = self.fc1(state)
        state_value_1 = F.leaky_relu(state_value_1)
        # state_value = self.bn1(state_value)

        state_value_1 = self.fc2(state_value_1)
        state_value_1 = F.leaky_relu(state_value_1)
        # state_value = self.bn2(state_value)

        action_value_1 = self.action_value_1(action)
        action_value_1 = F.leaky_relu(action_value_1)

        # print(action_value_1.size(),state_value_1.size())

        state_action_value_1 = T.cat((action_value_1, state_value_1), dim=1)

        # print(state_action_value_1.size())

        state_action_value_1 = self.fc3(state_action_value_1)
        state_action_value_1 = F.relu(state_action_value_1)

        state_action_value_1 = self.q1(state_action_value_1)

        state_value_2 = self.fc4(state)
        state_value_2 = F.leaky_relu(state_value_2)
        # state_value = self.bn1(state_value)

        state_value_2 = self.fc5(state_value_2)
        state_value_2 = F.leaky_relu(state_value_2)
        # state_value = self.bn2(state_value)

        action_value_2 = self.action_value_2(action)
        action_value_2 = F.leaky_relu(action_value_2)

        state_action_value_2 = T.cat((action_value_2, state_value_2), dim=1)
        state_action_value_2 = self.fc6(state_action_value_2)
        state_action_value_2 = F.relu(state_action_value_2)

        state_action_value_2 = self.q2(state_action_value_2)

        return state_action_value_1,state_action_value_2
    
    def Q1(self,state,action):
        state_value_1 = self.fc1(state)
        state_value_1 = F.leaky_relu(state_value_1)

        state_value_1 = self.fc2(state_value_1)
        state_value_1 = F.leaky_relu(state_value_1)

        action_value_1 = self.action_value_1(action)
        action_value_1 = F.leaky_relu(action_value_1)

        state_action_value_1 = T.cat((action_value_1, state_value_1), dim=1)

        state_action_value_1 = self.fc3(state_action_value_1)
        state_action_value_1 = F.relu(state_action_value_1)

        state_action_value_1 = self.q1(state_action_value_1)

        return state_action_value_1        

    def save_checkpoint(self):
        print("...saving checkpoint....")
        T.save(self.state_dict(), self.checkpoint_file + '.pth')

    def load_checkpoint(self):
        print("..loading checkpoint...")
        self.load_state_dict(T.load(self.checkpoint_file + '.pth'))


class ActorNetwork(nn.Module):
    def __init__(self,
                 lr,
                 input_dims,
                 fc1_dims,
                 fc2_dims,
                 fc3_dims,
                 fc4_dims,
                 fc5_dims,
                 fc6_dims,
                 n_actions,
                 name,
                 chkpt_dir='./models/saved/ddpg/'):
        super(ActorNetwork, self).__init__()
        self.lr = lr
        self.input_dims = input_dims
        self.fc1_dims = fc1_dims
        self.fc2_dims = fc2_dims
        self.fc3_dims = fc3_dims
        self.fc4_dims = fc4_dims
        self.fc5_dims = fc5_dims
        self.fc6_dims = fc6_dims 

        self.n_actions = n_actions
        self.checkpoint_file = os.path.join(chkpt_dir, name + "_ddpg")

        self.fc1 = nn.Linear(self.input_dims, self.fc1_dims)
        # self.bn1 = nn.LayerNorm(self.fc1_dims)

        self.fc2 = nn.Linear(self.fc1_dims, self.fc2_dims)
        # self.bn2 = nn.LayerNorm(self.fc2_dims)

        self.fc3 = nn.Linear(self.fc2_dims, self.fc3_dims)
        # self.bn3 = nn.LayerNorm(self.fc3_dims)

        self.mu_1 = nn.Linear(self.fc3_dims, 1)

        self.fc4 = nn.Linear(self.input_dims, self.fc4_dims)

        self.fc5 = nn.Linear(self.fc4_dims, self.fc5_dims)

        self.fc6 = nn.Linear(self.fc5_dims, self.fc6_dims)

        self.mu_2 = nn.Linear(self.fc6_dims, 1)

        self.initialization()

        self.optimizer = optim.Adam(self.parameters(), lr=lr)
        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')
        self.to(self.device)

    def initialization(self):
        nn.init.xavier_uniform_(self.fc1.weight,
                                gain=nn.init.calculate_gain('leaky_relu'))

        nn.init.xavier_uniform_(self.fc2.weight,
                                gain=nn.init.calculate_gain('leaky_relu'))

        nn.init.xavier_uniform_(self.fc3.weight,
                                gain=nn.init.calculate_gain('leaky_relu'))

        nn.init.xavier_uniform_(self.mu_1.weight,
                                gain=nn.init.calculate_gain('tanh'))

        nn.init.xavier_uniform_(self.fc4.weight,
                                gain=nn.init.calculate_gain('leaky_relu'))

        nn.init.xavier_uniform_(self.fc5.weight,
                                gain=nn.init.calculate_gain('leaky_relu'))

        nn.init.xavier_uniform_(self.fc6.weight,
                                gain=nn.init.calculate_gain('leaky_relu'))

        nn.init.xavier_uniform_(self.mu_2.weight,
                                gain=nn.init.calculate_gain('tanh'))

    def forward(self, state):
        x = self.fc1(state)
        x = F.leaky_relu(x)
        # x = self.bn1(x)

        x = self.fc2(x)
        x = F.leaky_relu(x)
        # x = self.bn2(x)

        x = self.fc3(x)
        x = F.leaky_relu(x)
        # x = self.bn3(x)

        x = T.sigmoid(self.mu_1(x))

        y = self.fc4(state)
        y = F.leaky_relu(y)

        y = self.fc5(y)
        y = F.leaky_relu(y)
        
        y = self.fc6(y)
        y = F.leaky_relu(y)

        y = T.sigmoid(self.mu_2(y))

        # print(x.size())
        # print(y.size())

        # z = T.cat((x,y),dim = -1)    
        # print(z.size())
        return T.cat((x,y),dim = -1)



    def save_checkpoint(self):
        print("...saving checkpoint....")
        T.save(self.state_dict(), self.checkpoint_file + '.pth')

    def load_checkpoint(self):
        print("..loading checkpoint...")
        self.load_state_dict(T.load(self.checkpoint_file + '.pth'))


class TD3(object):
    def __init__(self,
                 lr_actor,
                 lr_critic,
                 input_dims,
                 tau,
                 env,
                 gamma=0.99,
                 n_actions=8,
                 max_size=1000000,
                 layer1_size=400,
                 layer2_size=300,
                 layer3_size=200,
                 layer4_size=400,
                 layer5_size=300,
                 layer6_size=200,
                 batch_size=64,
                 load_models=False,
                 save_dir='./models/saved/ddpg/'):
        self.gamma = gamma
        self.tau = tau
        self.batch_size = batch_size
        self.memory = ReplayBuffer(max_size, input_dims, n_actions)
        self.n_actions = n_actions
        self.policy_freq = 3
        self.total_it = 0

        if load_models:
            self.load_models(lr_critic, lr_actor, input_dims, layer1_size,
                             layer2_size, layer3_size,layer4_size,layer5_size,layer6_size,n_actions, save_dir)
        else:
            self.init_models(lr_critic, lr_actor, input_dims, layer1_size,
                             layer2_size, layer3_size,layer4_size,layer5_size,layer6_size,n_actions, save_dir)

        actor_params = self.actor.named_parameters()
        critic_params = self.critic.named_parameters()

        critic_state_dict = dict(critic_params)
        actor_state_dict = dict(actor_params)
                               
        self.target_critic.load_state_dict(critic_state_dict)
        self.target_actor.load_state_dict(actor_state_dict)


        self.noise_1 = OUActionNoise(mu=np.zeros(1),
                                   dt=1e-2,
                                   sigma=0.1
                                   # theta=0.15,
                                   )
        self.noise_2 = OUActionNoise(mu=np.zeros(1),
                                   dt=1e-2,
                                   sigma=0.1
                                   # theta=0.15,
                                   )
        # self.uniformnoise = np.random.uniform(-0.2,0.2,12)

    def choose_action_train(self, observation):
        if observation is not None:
            self.actor.eval()
            observation = T.tensor(observation,
                                   dtype=T.float).to(self.actor.device)
            mu = self.actor(observation).to(self.actor.device)
            noise_1 = T.tensor(self.noise_1(), dtype=T.float).to(self.actor.device).clamp(-0.05,0.05)
            noise_2 = T.tensor(self.noise_2(), dtype = T.float).to(self.actor.device).clamp(-0.05,0.05)
            # noise_3 = T.tensor(np.random.uniform(-0.1,0.1),dtype=T.float).to(self.actor.device)
            # print(noise_1)
            # print(noise_2)
            mu_1,mu_2 = T.split(mu,[1,1], dim = -1)
            mu_prime_1 = mu_1 + noise_1
            mu_prime_2 = mu_2 + noise_2
            # mu_prime_3 = mu_3 + noise_3
            mu_prime = T.cat((mu_prime_1,mu_prime_2),dim = -1)
            # mu_prime = T.cat((mu_prime,mu_prime_3),dim = -1)
            self.actor.train()
            return mu_prime.cpu().detach().numpy()
        return np.zeros((self.n_actions, ))

    def choose_action_test(self, observation):
        if observation is not None:
            self.actor.eval()
            observation = T.tensor(observation,
                                   dtype=T.float).to(self.actor.device)
            mu = self.target_actor(observation).to(self.target_actor.device)

            return mu.cpu().detach().numpy()
        return np.zeros((self.n_actions, ))

    def remember(self, state, action, reward, new_state, done):
        self.memory.store_transition(state, action, reward, new_state, done)

    def learn(self):
        self.total_it = self.total_it + 1
        if self.memory.mem_cntr < self.batch_size:
            return
        state, action, reward, new_state, done = \
                                                 self.memory.sample_buffer(self.batch_size)

        reward = T.tensor(reward, dtype=T.float).to(self.critic.device)
        done = T.tensor(done).to(self.critic.device)
        new_state = T.tensor(new_state, dtype=T.float).to(self.critic.device)
        action = T.tensor(action, dtype=T.float).to(self.critic.device)
        state = T.tensor(state, dtype=T.float).to(self.critic.device)



        self.target_actor.eval()
        self.target_critic.eval()
        self.critic.eval()
        with T.no_grad():

            noise_1 = T.tensor(self.noise_1(), dtype=T.float).to(self.actor.device)
            noise_2 = T.tensor(self.noise_2(), dtype=T.float).to(self.actor.device)
            # noise_3 = T.tensor(np.random.uniform(-0.1,0.1),dtype=T.float).to(self.actor.device)
            target_actions = self.target_actor.forward(new_state)


            mu_1,mu_2,mu_3 = T.split(target_actions,[1,1], dim = 1)

            mu_prime_1 = mu_1 + noise_1
            mu_prime_2 = mu_2 + noise_2
            # mu_prime_3 = mu_3 + noise_3
            # print(mu_prime_1.size())
            # print(mu_prime_2.size())

            mu_prime = T.cat((mu_prime_1,mu_prime_2),dim = 1)
            # print(mu_prime.size())
            target_actions = T.cat((mu_prime,mu_prime_3),dim = 1)
            # print(target_actions.size())
            target_Q1,target_Q2 = self.target_critic.forward(new_state,target_actions)
            target_Q = T.min(target_Q1,target_Q2)


        target = []
        for j in range(self.batch_size):
            target.append(reward[j] + self.gamma * target_Q[j] * done[j])

        target = T.tensor(target).to(self.critic.device)
        target = target.view(self.batch_size, 1)

        current_Q1,current_Q2 = self.critic(state,action)
        # print(self.actor.device)

        self.critic.train()
        self.critic.optimizer.zero_grad()
        critic_loss = F.mse_loss(current_Q1,target) + F.mse_loss(current_Q2,target)
        critic_loss.backward()
        self.critic.optimizer.step()
        self.critic.eval()

    #     self.update_network_parameters()

    # def work(self):
    #     self.target_actor.eval()
    #     self.target_critic.eval()

    # def update_network_parameters(self, tau=None):
    #     if tau is None:
    #         tau = self.tau
        tau = self.tau

        if self.total_it % self.policy_freq == 0:
            self.actor.optimizer.zero_grad()
            mu = self.actor.forward(state)
            self.actor.train()
            actor_loss = - self.critic.Q1(state, mu)
            actor_loss = T.mean(actor_loss)
            actor_loss.backward()
            self.actor.optimizer.step()

            actor_params = self.actor.named_parameters()
            critic_params = self.critic.named_parameters()

            target_actor_params = self.target_actor.named_parameters()
            target_critic_params = self.target_critic.named_parameters()

            critic_state_dict = dict(critic_params)
            actor_state_dict = dict(actor_params)

            target_critic_dict = dict(target_critic_params)
            target_actor_dict = dict(target_actor_params)

            for name in critic_state_dict:
                critic_state_dict[name] = tau*critic_state_dict[name].clone() +\
                                        (1-tau)*target_critic_dict[name].clone()

            self.target_critic.load_state_dict(critic_state_dict)

            for name in actor_state_dict:
                actor_state_dict[name] = tau*actor_state_dict[name].clone() +\
                                        (1-tau)*target_actor_dict[name].clone()

            self.target_actor.load_state_dict(actor_state_dict)

    def init_models(self, lr_critic, lr_actor, input_dims, layer1_size,
                    layer2_size, layer3_size,layer4_size,layer5_size,layer6_size, n_actions, save_dir):
        self.actor = ActorNetwork(lr_actor,
                                  input_dims,
                                  layer1_size,
                                  layer2_size,
                                  layer3_size,
                                  layer4_size,
                                  layer5_size,
                                  layer6_size,
                                  n_actions=n_actions,
                                  name="Actor",
                                  chkpt_dir=save_dir)

        self.target_actor = ActorNetwork(lr_actor,
                                         input_dims,
                                         layer1_size,
                                         layer2_size,
                                         layer3_size,
                                         layer4_size,
                                         layer5_size,
                                         layer6_size,
                                         n_actions=n_actions,
                                         name="TargetActor",
                                         chkpt_dir=save_dir)

        self.critic = CriticNetwork(lr_critic,
                                    input_dims,
                                    layer1_size,
                                    layer2_size,
                                    layer3_size,
                                    layer4_size,
                                    layer5_size,
                                    layer6_size,
                                    n_actions=n_actions,
                                    name="Critic",
                                    chkpt_dir=save_dir)

        self.target_critic = CriticNetwork(lr_critic,
                                           input_dims,
                                           layer1_size,
                                           layer2_size,
                                           layer3_size,
                                           layer4_size,
                                           layer5_size,
                                           layer6_size,
                                           n_actions=n_actions,
                                           name="TargetCritic",
                                           chkpt_dir=save_dir)

    def save_models(self):
        self.actor.save_checkpoint()
        self.critic.save_checkpoint()
        self.target_actor.save_checkpoint()
        self.target_critic.save_checkpoint()

    def load_models(self, lr_critic, lr_actor, input_dims, layer1_size,
                    layer2_size, layer3_size, layer4_size,layer5_size,layer6_size,n_actions, load_dir):

        self.init_models(lr_critic, lr_actor, input_dims, layer1_size,
                         layer2_size, layer3_size,layer4_size,layer5_size,layer6_size,n_actions, load_dir)

        self.actor.load_checkpoint()
        self.critic.load_checkpoint()
        self.target_actor.load_checkpoint()
        self.target_critic.load_checkpoint()
