import datetime
import math
import numpy as np
import matplotlib.pyplot as plt
import os
import random
import torch
import torch.nn.functional as F
from collections import namedtuple
from torch import nn
from torch.autograd import Variable

BATCH_SIZE = 128
Transition = namedtuple('Transition',
                        ('state', 'action', 'next_state', 'reward'))


class ReplayMemory(object):

    def __init__(self, capacity):
        self.capacity = capacity
        self.memory = []
        self.position = 0

    def push(self, *args):
        """Saves a transition."""
        if len(self.memory) < self.capacity:
            self.memory.append(None)
        self.memory[self.position] = Transition(*args)
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)


class DQN(nn.Module):

    def __init__(self,state_len,num_parti_accel,num_parti_angle):
        #Layers for the model
        super(DQN,self).__init__()
        self.layer_list = None
        self.last_layers_accel = None
        self.last_layers_angle = None
        self.initialiseLayers(state_len,num_parti_accel,num_parti_angle)


    def initialiseLayers(self,state_len,num_parti_accel,num_parti_angle):
        state_len = int(state_len)
        self.layer_list = []
        while state_len>max(20,num_parti_accel,num_parti_angle):
            self.layer_list.append(nn.Linear(state_len,int(state_len/2)))
            state_len= int(state_len/2)

        self.last_layers_accel = []
        self.last_layers_angle = []
        while state_len>max(num_parti_accel,num_parti_angle):
            self.last_layers_accel.append(nn.Linear(state_len,int(state_len/2)))
            self.last_layers_angle.append(nn.Linear(state_len,int(state_len/2)))
            state_len = int(state_len/2)
        self.last_layers_accel.append(nn.Linear(state_len,num_parti_accel))
        self.last_layers_angle.append(nn.Linear(state_len,num_parti_angle))

        self.layer_list = nn.ModuleList(self.layer_list)
        self.last_layers_accel = nn.ModuleList(self.last_layers_accel)
        self.last_layers_angle = nn.ModuleList(self.last_layers_angle)


    def forward(self,x):
        for layer in self.layer_list:
            x = layer(x)
            x = F.relu(x)
        y = x
        z = x
        for l_accel,l_angle in zip(self.last_layers_accel,self.last_layers_angle):
            y = l_accel(y)
            z = l_angle(z)
            if l_accel != self.last_layers_accel[-1]:
                y = F.relu(y)
                z = F.relu(z)

        #Returns Q-values for all the accelerations and all the angular accelerations
        return y,z


class RandomController():

    def __init__(self,accel_cats,angle_cats):
        self.model = -1 #Token value to make compatible with standard controller

        self.accel_ranges = accel_cats
        self.angle_ranges = angle_cats


    def selectAction(self,state):
        accel_cat = random.randint(0,len(self.accel_ranges)-1)
        angle_cat = random.randint(0,len(self.angle_ranges)-1)
        accel = np.random.uniform(self.accel_ranges[accel_cat][0],self.accel_ranges[accel_cat][1])
        angle = np.random.uniform(self.angle_ranges[angle_cat][0],self.angle_ranges[angle_cat][1])
        return accel,angle


    def train(self,state,next_state):
        pass

    def updateTarget(self):
        pass

    def recordModel(self):
        pass

    def plotResults(self):
        pass

class DQNController():
    def __init__(self,behaviour,accel_cats,angle_cats):
        self.gamma = .999
        self.eps_start = 0.9
        self.eps_end = .05
        self.eps_decay = 200

        self.num_steps = 0
        self.num_eps = 1

        self.accel = None
        self.angle = None

        self.accel_ranges = accel_cats
        self.angle_ranges = angle_cats

        self.model = None
        self.target_model = None
        self.behaviour = behaviour
        self.memory = ReplayMemory(10000)
        self.optimizer = None

        self.reward_sum = 0
        self.reward_list = []


    def initialiseModel(self,state_len):
        self.model = DQN(state_len,len(self.accel_ranges),len(self.angle_ranges))
        self.target_model = DQN(state_len,len(self.accel_ranges),len(self.angle_ranges))
        self.target_model.load_state_dict(self.model.state_dict())
        self.optimizer = torch.optim.RMSprop(self.model.parameters())


    def updateTarget(self):
        self.target_model.load_state_dict(self.model.state_dict())


    def recordModel(self):
        now = datetime.datetime.now()
        date = "{}-{}-{}".format(now.day,now.month,now.year)
        time = "{}-{}".format(now.hour,now.minute)
        if not os.path.exists("./driving_models/{}".format(date)):
            os.makedirs("./driving_models/{}".format(date))
        torch.save(self.model.state_dict(),"./driving_models/{}/{}-{}DQN.pt".format(date,time,self.behaviour))


    def restartModel(self):
        self.reward_list.append(self.reward_sum/self.num_steps)
        self.num_steps = 0
        self.num_eps += 1
        self.reward_sum = 0


    def storeResults(self):
        now = datetime.datetime.now()
        result_record = open("result-{}-{}-{}-{}.txt".format(self.behaviour,now.day,now.month,now.year),"w")
        for i in range(len(self.reward_list)):
            result_record.write("{}\t{}\n".format(i*10,self.reward_list[i]))
        result_record.close()
        title = "{}-DQN".format(self.behaviour.capitalize())
        rewardPlotter(title,self.reward_list,10)


    def selectAction(self,state):
        if not isinstance(state,torch.Tensor):
            state = torch.Tensor([state])
        sample = random.random()
        eps_threshold = self.eps_end + (self.eps_start - self.eps_end) * math.exp(-1.*self.num_steps/(self.num_eps*self.eps_decay))
        self.num_steps += 1
        if sample > eps_threshold:
            accel,angle = self.model(Variable(state, volatile=True))
            accel_cat = np.argmax(accel.data)
            angle_cat = np.argmax(angle.data)
        else:
            accel_cat = random.randint(0,len(self.accel_ranges)-1)
            angle_cat = random.randint(0,len(self.angle_ranges)-1)
        self.accel  = accel_cat
        self.angle = angle_cat
        accel = np.random.uniform(self.accel_ranges[accel_cat][0],self.accel_ranges[accel_cat][1])
        angle = np.random.uniform(self.angle_ranges[angle_cat][0],self.angle_ranges[angle_cat][1])
        self.num_steps += 1
        return accel,angle


    def computeReward(self,from_state,to_state):
        #States are unstructured, so in calculating the reward I have to make assumptions about
        # the locations of different values. Thus changes to state must be reflected here
        # Current version of state:
        # 0 v_par, 1 v_perp, 2 rel_heading
        # 3 time_on_objective, 4 dist_to_waypoint, 5 dist_to_right_side_of_road
        # 6 dist_to_left_side_of_road, 7 accel, 8 angle_accel,
        # 9 has_crashed, 10 is_on_road, 11 is_complete
        x,w = 0,0
        reward = 0
        reward += min(1,from_state[4]-to_state[4]) #Want this to be positive to be getting closer to destination
        reward += min(1,1.0/(abs(to_state[0]-30)))
        if self.behaviour is "safe":
            x = to_state[5]/to_state[6] #distanceLeft/distanceRight
            w = to_state[8] - from_state[8] #change in angular acceleration
            reward += min(1,w*(x-1/x)) #Rewards changes that tend x to 1 (i.e. the center of the road)
        elif self.behaviour is "aggro":
            if to_state[3] < 10: reward += 1
            else: reward += 1.0/to_state[3]
        if not to_state[10]: reward += -3
        if to_state[9]: reward += -3
        if to_state[11]: reward += 3
        #Reward is an integer
        return reward


    def train(self,state,next_state):
        #self.accel and self.angle are integers indicating chosen action and angle
        action = torch.Tensor([[self.accel,self.angle]])
        #Reward is an integer
        reward = self.computeReward(state,next_state)

        self.reward_sum += reward

        state = torch.Tensor([state])
        next_state = torch.Tensor([next_state])
        reward = torch.Tensor([reward])

        # Store the transition in memory
        self.memory.push(state, action, next_state, reward)

        # Perform one step of the optimization (on the target network)
        if self.num_steps%20 == 0:
            self.optimize_model()


    def optimize_model(self):
        if len(self.memory) < BATCH_SIZE:
            return
        #Transitions heire should be a list 
        transitions = self.memory.sample(BATCH_SIZE)
        # Transpose the batch (see http://stackoverflow.com/a/19343/3343043 for
        # detailed explanation).
        batch = Transition(*zip(*transitions))

        # We don't want to backprop through the expected action values and volatile
        # will save us on temporarily changing the model parameters'
        # requires_grad to False!
        non_final_next_states = Variable(torch.cat([s for s in batch.next_state
                                                   if s is not None]),volatile=True)
        state_batch = Variable(torch.cat(batch.state))
        action_batch = Variable(torch.cat(batch.action))
        reward_batch = Variable(torch.cat(batch.reward))

        # Compute Q(s_t, a) - the model computes Q(s_t), then we select the
        # columns of actions taken
        accel_q_values, angle_q_values  = self.model(state_batch)#.gather(1, action_batch)
        chosen_accels = accel_q_values.gather(1,action_batch[:,0].long().view(-1,1))
        chosen_angles = angle_q_values.gather(1,action_batch[:,1].long().view(-1,1))


        # Compute V(s_{t+1}) for all next states.
        accel_next_state_values = Variable(torch.zeros(BATCH_SIZE).type(torch.Tensor))
        angle_next_state_values = Variable(torch.zeros(BATCH_SIZE).type(torch.Tensor))
        #accel_next_state_values[non_final_mask],angle_next_state_values[non_final_mask] = self.model(non_final_next_states)
        accel_next_state_values,angle_next_state_values = self.target_model(non_final_next_states)
        accel_next_state_values = accel_next_state_values.max(1)[0]
        angle_next_state_values = angle_next_state_values.max(1)[0]
        # Now, we don't want to mess up the loss with a volatile flag, so let's
        # clear it. After this, we'll just end up with a Variable that has
        # requires_grad=False
        accel_next_state_values.volatile = False
        angle_next_state_values.volatile = False
        # Compute the expected Q values
        expected_accel_state_action_values = (accel_next_state_values * self.gamma) + reward_batch
        expected_angle_state_action_values = (angle_next_state_values * self.gamma) + reward_batch

        # Compute Huber loss
        accel_loss = F.smooth_l1_loss(chosen_accels, expected_accel_state_action_values)
        angle_loss = F.smooth_l1_loss(chosen_angles, expected_angle_state_action_values)

        # Optimize the model
        self.optimizer.zero_grad()
        accel_loss.backward()
        angle_loss.backward()
        for param in self.model.parameters():
            param.grad.data.clamp_(-1, 1)
        self.optimizer.step()


def rewardPlotter(title,reward_list,step_size):
    plt.figure(2)
    plt.clf()
    plt.title(title)
    plt.xlabel('Episode')
    plt.ylabel('Reward')
    plt.plot([x*step_size for x in range(len(reward_list))],reward_list)
    plt.show()
