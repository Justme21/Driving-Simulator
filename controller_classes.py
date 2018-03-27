import torch
from torch import nn
from torch.autograd import Variable


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


class DQN(nn.module):

    def __init__(self,state_len,num_parti_accel,num_parti_angle):
        self.layer_list = []
        while state_len>max(20,num_parti_accel,num_parti_angle):
            self.layer_list.append(nn.ReLU(nn.Linear(state_len,int(state_len/2))))
            state_len= int(state_len/2)

        self.last_layers_accel = []
        self.last_layers_angle = []
        while state_len>max(num_parti_accel,num_parti_angle):
            self.last_layers_accel.append(nn.ReLU(nn.Linear(state_len,state_len/2)))
            self.last_layers_angle.append(nn.ReLU(nn.Linear(state_len,state_len/2)))
            state_len = int(state_len/2)
        self.last_layers_accel.append(nn.Softmax(nn.Linear(state_len,num_parti_accel)))
        self.last_layers_angle.append(nn.Softmax(nn.Linear(state_len,num_parti_angle)))


    def forward(x):
        for layer in self.layer_list:
            x = layer(x)
        y = x
        z = x
        for l_accel,l_angle in zip(self.last_layers_accel,self.last_layers_angle):
            y = l_accel(y)
            z = l_angle(z)
        return y,z


class Controller():
    def __init__(self,state_len,num_partition_accel,num_partition_angle):
        self.batch_size = 128
        self.gamma = .999
        self.epsilon_start = 0.9
        self.epsilon_end = .05
        self.epsilon_decay = 200

        self.num_steps = 0

        self.memory = 
