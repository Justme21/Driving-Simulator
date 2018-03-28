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
        #Layers for the model
        self.layer_list = None
        self.last_layers_accel = None
        self.last_layers_angle = None
        self.initialiseLayers(state_len,num_parti_accel,num_parti_angle)


    def initialiseLayers(self,state_len,num_parti_accel,num_parti_angle):
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


    def optimize_model():
        if len(memory) < BATCH_SIZE:
            return
        transitions = memory.sample(BATCH_SIZE)
        # Transpose the batch (see http://stackoverflow.com/a/19343/3343043 for
        # detailed explanation).
        batch = Transition(*zip(*transitions))

        # Compute a mask of non-final states and concatenate the batch elements
        non_final_mask = ByteTensor(tuple(map(lambda s: s is not None,
                                              batch.next_state)))

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
        state_action_values = model(state_batch).gather(1, action_batch)

        # Compute V(s_{t+1}) for all next states.
        next_state_values = Variable(torch.zeros(BATCH_SIZE).type(Tensor))
        next_state_values[non_final_mask] = model(non_final_next_states).max(1)[0]
        # Now, we don't want to mess up the loss with a volatile flag, so let's
        # clear it. After this, we'll just end up with a Variable that has
        # requires_grad=False
        next_state_values.volatile = False
        # Compute the expected Q values
        expected_state_action_values = (next_state_values * GAMMA) + reward_batch

        # Compute Huber loss
        loss = F.smooth_l1_loss(state_action_values, expected_state_action_values)

        # Optimize the model
        optimizer.zero_grad()
        loss.backward()
        for param in model.parameters():
            param.grad.data.clamp_(-1, 1)
        optimizer.step()


class Controller():
    def __init__(self,state_len,num_partition_accel,num_partition_angle,behaviour="default"):
        self.batch_size = 128
        self.gamma = .999
        self.eps_start = 0.9
        self.eps_end = .05
        self.eps_decay = 200

        self.num_steps = 0
        self.accel_len = num_partition_accel
        self.angle_len = num_partition_accel

        self.model = DQN(state_len,self.accel_len,self.angle_len)
        self.behaviour = behaviour
        self.memory = memory = ReplayMemory(10000)


    def selectAction(self,state):
        if not isinstance(state,np.ndarray):
            state = np.asarray(state)
        sample = random.random()
        eps_threshold = self.eps_end + (self.eps_start - self.eps_end) * math.exp(-1.*self.num_steps/self.eps_decay)
        self.num_steps += 1
        if sample > eps_threshold:
            accel,angle = self.model(Variable(state, volatile=True))
            accel = np.argmax(accel.data)
            angle = np.argmax(angle.data)
        else:
            accel = random.randint(self.accel_len)
            angle = random.randint(self.angle_len)
        self.num_steps += 1
        self.accel = accel
        self.angle = angle
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
        return reward


    def plot_durations():
        plt.figure(2)
        plt.clf()
        durations_t = torch.FloatTensor(episode_durations)
        plt.title('Training...')
        plt.xlabel('Episode')
        plt.ylabel('Duration')
        plt.plot(durations_t.numpy())
        # Take 100 episode averages and plot them too
        if len(durations_t) >= 100:
            means = durations_t.unfold(0, 100, 1).mean(1).view(-1)
            means = torch.cat((torch.zeros(99), means))
            plt.plot(means.numpy())

        plt.pause(0.001)  # pause a bit so that plots are updated
        display.display(plt.gcf())


    def train(state,next_state):
        action = (self.accel,self.angle)
        reward = self.computeReward(state,next_state)
        reward = Tensor([reward])

        # Store the transition in memory
        memory.push(state, action, next_state, reward)

        # Perform one step of the optimization (on the target network)
        optimize_model()
                if done:
                    episode_durations.append(t + 1)
                    plot_durations()
                    break

        print('Complete')
        env.render(close=True)
        env.close()
        plt.ioff()
        plt.show()
