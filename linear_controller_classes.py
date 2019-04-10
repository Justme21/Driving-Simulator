import linear_controller_profiles as lcp
import math
import random

class DrivingController():

    def __init__(self,controller="standard",speed_limit=22.5,speed_limit_buffer=None,ego=None,other=None,timestep=.1,**kwargs):
        if speed_limit_buffer is None:
            speed_limit_buffer = .1*speed_limit

        #Parameters used to regerenerate/reinitialise the controller later
        self.controller_tag = controller
        self.initialisation_params = {'ego':ego,'other':other,'speed_limit':speed_limit,'speed_limit_buffer':speed_limit_buffer,'timestep':timestep}
        self.initialisation_params.update(kwargs)

        self.controller = self.getController(controller,**self.initialisation_params)
        self.ego = None
        self.other = None
        self.setup(ego=ego,other=other)

        self.log = []
        self.speed_limit = speed_limit+speed_limit_buffer

        self.state = []


    def getController(self,controller,**kwargs):
        controller_list = {"standard":lcp.StandardDrivingController,"follow":lcp.FollowController,"generator":lcp.DataGeneratorController,\
                "fast":lcp.GoFastController,"slow":lcp.GoSlowController,"overtake":lcp.OvertakeController,"trajectory":lcp.TrajectoryController\
                ,"random":lcp.RandomController}

        return controller_list[controller](**kwargs)


    def setup(self,ego=None,other=None,**kwargs):
        if ego is not None:
            self.ego = ego
            self.initialisation_params['ego'] = ego
        if other is not None:
            self.other = other
            self.initialisation_params['other'] = other

        if ego is not None:
            self.controller.setup(ego=ego,other=other,**kwargs)
            self.initialisation_params.update(kwargs)


    def getAccelRange(self,state):
        accel_range = self.controller.getAccelRange(state)
        angle_range = self.controller.getAngleRange(state)

        return accel_range,angle_range


    def extractFeatures(self,in_state):
        state = dict(in_state)

        if self.ego is not None:
            ego_vel = self.ego.v
            ego_accel = self.ego.accel
        else:
            ego_vel = 0
            ego_accel = 0

        if self.other is not None:
            other_v = self.other.v
            other_com = tuple([self.other.x_com,self.other.y_com])
            other_accel = self.other.accel
        else:
            other_v = 0
            other_com = tuple([0,0])
            other_accel = 0

        state["velocity"] = ego_vel
        state["other_v"] = other_v
        state["other_position"] = other_com
        state["acceleration"] = ego_accel
        state["other_accel"] = other_accel

        return state


    def inferFeatures(self,state):
        ego_com = state["position"]
        other_com = state["other_position"]

        state["del_d"] = distance(ego_com,other_com)
        state["del_v"] = state["other_v"] - state["velocity"]

        return state


    def defineState(self,in_state):
        state = self.extractFeatures(in_state)
        full_state = self.inferFeatures(state)

        #Note: This is here purely for debugging purposes
        self.state = full_state
        return full_state


    def chooseAction(self,state,accel_range,angle_range):
        accel,angle_accel = self.controller.selectAction(state,accel_range,angle_range)

        #HACKY: This needs to be removed. Inserted to facilitate first year review results
        next_vel = self.ego.state["velocity"] + self.ego.timestep*accel
        if accel_range == [None,None]:
            accel_range = list(accel_range)
            accel_range = self.controller.accel_range
        if next_vel<0:
            accel = max(accel_range[0],-self.ego.state["velocity"]/self.ego.timestep)
        elif next_vel>self.speed_limit:
            accel = min(accel_range[1],max(accel_range[0],(self.speed_limit-self.ego.state["velocity"])/self.ego.timestep))

        if accel>3.5:
            print("Accel too high")
            exit(-1)

        return (accel,angle_accel)


    def selectAction(self,in_state,accel_range,angle_range):
        state = self.defineState(in_state)
        action = self.chooseAction(dict(state),accel_range,angle_range)

        (accel,angle_accel) = action
        self.log.append([state,accel])
        return accel,angle_accel


    def reset(self):
        self.controller = self.getController(self.controller_tag,**self.initialisation_params)


    def getLog(self):
        return self.log


    def clearLog(self):
        self.log = []


    def testAction(self,accel,angle_accel,vel=None,heading=None):
        x_dot,y_dot,v_dot,head_dot = self.ego.simulateDynamics(accel,angle_accel,vel,heading)
        return x_dot,y_dot,v_dot,head_dot


    def paramCopy(self,target=None):
        dup_initialisation_params = dict(self.initialisation_params)
        dup_initialisation_params["ego"] = target #target is the vehicle the controller is being copied for
        return dup_initialisation_params


    def copy(self,**kwargs):
        dup_init_params = self.paramCopy(**kwargs)
        return DrivingController(controller=self.controller_tag,**dup_init_params)


class MultiDrivingController(DrivingController):
    """Variant on Driving Controller that has a choice of mulitple controllers
       Each time the controller is reset (each time the vehicle being controlled is reset) a new controller
       from which to choose actions is sampled. This is mainly useful for data generation to generate more diverse
       behaviours."""

    def __init__(self,controllers=["standard"],**kwargs):
        self.controllers = list(controllers)
        self.controller_tag = random.choice(self.controllers)
        kwargs.update({'controllers':controllers})
        super().__init__(controller=self.controller_tag,**kwargs)


    def reset(self):
        self.controller_tag = random.choice(self.controllers)
        super().reset()


    def copy(self,**kwargs):
        dup_init_params = super().paramCopy(**kwargs)
        return MultiDrivingController(**dup_init_params)


class DelayedDrivingController(DrivingController):

    def __init__(self,delay=0,timestep=.1,**kwargs):
        kwargs.update({'timestep':timestep,'delay':delay})
        super().__init__(**kwargs)

        self.action_list = [(0,0) for _ in range(int(delay/timestep))]


    def chooseAction(self,*args):
        action = super().chooseAction(*args)
        self.action_list.append(action)
        action = self.action_list.pop(0)

        return action


    def selectAction(self,in_state,*args):
        state = super().defineState(in_state)
        action = self.chooseAction(dict(state),*args)

        (accel,angle_accel) = action
        self.log.append([state,accel])
        return accel,angle_accel


    def reset(self):
        super().reset()
        self.action_list = [(0,0) for _ in range(len(self.action_list))]


    def copy(self,**kwargs):
        dup_init_params = super().paramCopy(**kwargs)
        return DelayedDrivingController(**dup_init_params)


class ProactiveDrivingController(DelayedDrivingController):

    def __init__(self,anticipation_time=0,awareness_coef=.1,non_ego_trajectory=None,timestep=.1,**kwargs):
        kwargs.update({'timestep':timestep,'anticipation_time':anticipation_time,'awareness_coef':awareness_coef,'non_ego_trajectroy':non_ego_trajectory})

        super().__init__(**kwargs)
        self.timestep = timestep
        self.awareness = awareness_coef
        self.anticipation_time = int(anticipation_time/timestep)

        if non_ego_trajectory is not None:
            self.other_trajectory = non_ego_trajectory
            self.traj_index = 0
        else:
            self.other_trajectory = None
            self.traj_index = None


    def setTargetTrajectory(self,trajectory):
        self.other_trajectory = trajectory
        self.traj_index = 0


    def chooseAction(self,state,accel_range,angle_range):
        tot_action = [0,0]
        determinant = 0
        #print("ActionList for ProactiveController is: {}".format(self.action_list))
        action_list = None #temporary copy of list
        #print("Proactive")
        if self.other_trajectory is not None:
            print("Index: {}\tState: {}".format(self.traj_index,state))

        for i in range(0,self.anticipation_time+1):
            action = super().chooseAction(state,accel_range,angle_range)
            #print("In ProactiveController in Loop ActionList is: {}".format(self.action_list))
            if i==0: action_list = list(self.action_list) #The main action_list only wants to store what you woul
            #print("I: {}\t State:{}\tAction: {}".format(i,state,action))
            tot_action = [x+(self.awareness**i)*y for x,y in zip(tot_action,action)]
            determinant += (self.awareness**i)

            if self.other_trajectory is None or self.traj_index+i>=len(self.other_trajectory): break
            else:
                traj_point = self.other_trajectory[self.traj_index+i]
                if (self.other_trajectory[self.traj_index][1]<0) != (traj_point[1]<0):# or self.other_trajectory[self.traj_index][1]>0 != traj_point[1]>0:
                    #This cripples the proactive controller so it cannot anticipate actions that are not consistent with the current trend
                    break
                #Placeholder value of 90 for heading since we know trajectory will be linear
                x_dot,y_dot,v_dot,head_dot = super().testAction(traj_point[1],0,state["other_v"],90)
                state["other_v"] = state["other_v"] + v_dot*self.timestep #we assume trajectory has same length timestep
                state["other_position"] = (state["other_position"][0]+x_dot*self.timestep,state["other_position"][1]+y_dot*self.timestep)
                state["other_accel"] = traj_point[1]

                x_dot,y_dot,v_dot,head_dot = super().testAction(action[0],action[1],state["velocity"],state["heading"])
                state["position"] = (state["position"][0]+x_dot*self.timestep,state["position"][1]+y_dot*self.timestep)
                state["velocity"] = state["velocity"]+v_dot*self.timestep
                state["heading"] = state["heading"]+head_dot*self.timestep
                state["acceleration"] = action[0]
                state = super().inferFeatures(state)

        tot_action = [x/determinant for x in tot_action]
        self.action_list = action_list
        if self.other_trajectory is not None:
            print("Action is: {}\n".format(tot_action))
            self.traj_index += 1
        return (tot_action[0],tot_action[1])


    def selectAction(self,in_state,accel_range,angle_range):
        state = super().defineState(in_state)

        action = self.chooseAction(dict(state),accel_range,angle_range)
        (accel,angle_accel) = action

        self.log.append([state,accel])
        return accel,angle_accel


    def reset(self):
        super().reset()
        if self.traj_index is not None:
            self.traj_index = 0


    def copy(self,**kwargs):
        dup_init_params = super().paramCopy(**kwargs)
        return ProactiveDrivingController(controller=self.controller_tag,**dup_init_params)


def distance(pt1,pt2):
    return math.sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2)
