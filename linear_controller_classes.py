import datetime
import math
import numpy as np
import random

class DrivingController():

    def __init__(self,controller="standard",speed_limit=22.5,speed_limit_buffer=None,ego=None,other=None,timestep=.1,reaction_time=0,**kwargs):
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

        self.action_list = [0 for _ in range(int(reaction_time/timestep))]


    def getController(self,controller,**kwargs):
        controller_list = {"standard":StandardDrivingController,"follow":FollowController,"generator":DataGeneratorController,\
                "fast":GoFastController,"slow":GoSlowController,"overtake": OvertakeController,"random":RandomController}

        return controller_list[controller](**kwargs)


    def setup(self,ego=None,other=None,**kwargs):
        if ego is not None:
            self.ego = ego
            self.initialisation_params['ego'] = ego
        if other is not None:
            self.other = other
            self.initialisation_params['other'] = other

        if ego is not None:
            if isinstance(self.controller,FollowController):
                self.controller.setLeaderAndFollower(leader=other,follower=ego,**kwargs)
            else:
                self.controller.setup(ego=ego,other=other)


    def getAccelRange(self,state):
        accel_range = self.controller.getAccelRange(state)
        angle_range = self.controller.getAngleRange(state)

        return accel_range,angle_range


    def defineState(self,in_state):
        full_state = dict(in_state)

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

        del_v = other_v - ego_vel

        ego_com = full_state["position"]
        dist = distance(ego_com,other_com)
        del_d = dist

        full_state["ego_v"] = ego_vel
        full_state["other_v"] = other_v
        full_state["other_position"] = other_com
        full_state["del_v"] = del_v
        full_state["del_d"] = del_d
        full_state["other_accel"] = other_accel
        full_state["ego_accel"] = ego_accel

        #NOTe: This is here purely for debuggin purposes
        self.state = full_state

        return full_state


    def selectAction(self,in_state,accel_range,angle_range):
        full_state = self.defineState(in_state)
        accel,angle_accel = self.controller.selectAction(full_state,accel_range,angle_range)

        #print("Full State: {}\nAccel:{}\n\n".format(full_state,accel))

        #HACKY: This needs to be removed. Inserted to facilitate first year review results
        next_vel = self.ego.state["velocity"] + self.ego.timestep*accel
        if accel_range == [None,None]:
            accel_range = list(accel_range)
            accel_range = self.controller.accel_range
        if next_vel<0:
            accel = max(accel_range[0],-self.ego.state["velocity"]/self.ego.timestep)
        elif next_vel>self.speed_limit:
            accel = min(accel_range[1],max(accel_range[0],(self.speed_limit-self.ego.state["velocity"])/self.ego.timestep))

        #if accel==0.0 and self.controller_tag == "follow":
        #    print("Found a 0 in general controller")
        #    print("Next vel is :{}\tAccel_Range is: {}\tSpeed_limit is: {}".format(next_vel,accel_range,self.speed_limit))
        #    exit(-1)

        self.log.append([full_state,accel])

        #To incorporate reaction time while still preserving what the controller "wanted" to do
        self.action_list.append(accel)
        accel = self.action_list.pop(0)

        return accel,angle_accel


    def reset(self):
        self.controller = self.getController(self.controller_tag,**self.initialisation_params)
        self.action_list = [0 for _ in self.action_list]


    def getLog(self):
        return self.log


    def clearLog(self):
        self.log = []


class StandardDrivingController():
    def __init__(self,ego=None,other=None,accel_range=[-5,5],accel_jerk=1,angle_range=[0,0],angle_jerk=5,speed_limit=5,speed_limit_buffer=5,**kwargs):
        self.speed_limit = speed_limit
        self.speed_limit_buffer = speed_limit_buffer

        self.accel_range = accel_range
        self.angle_range = angle_range

        self.ego = ego

        self.accel_up = True
        self.up_coef = .5
        self.down_coef = .5

        self.accel_jerk = accel_jerk
        self.angle_jerk = angle_jerk


    def setup(self,ego=None,**kwargs):
        if ego is not None:
            self.ego = ego


    def getAccelRange(self,state):
        accel_range = list(self.accel_range)
        ego_vel = state["velocity"]

        if ego_vel>=self.speed_limit+self.speed_limit_buffer:
            a_range = [self.down_coef*accel_range[0],self.up_coef*min(accel_range[1],0)]

        elif ego_vel <= self.speed_limit-self.speed_limit_buffer:
            a_range = [self.down_coef*max(accel_range[0],0),self.up_coef*accel_range[1]]
        else:
            small_accel_mag =  min(abs(accel_range[0]),abs(accel_range[1]))
            a_range = [self.down_coef*max(accel_range[0],-.7*small_accel_mag),self.up_coef*min(accel_range[1],.7*small_accel_mag)]

        prev_accel = state["acceleration"]
        a_range = [max(a_range[0],prev_accel-self.accel_jerk),min(a_range[1],prev_accel+self.accel_jerk)]

        return a_range


    def getAngleRange(self,state):
        angle_range = list(self.angle_range)
        prev_turn_angle = state["turn_angle"]

        angle_range = [max(angle_range[0],prev_turn_angle-self.angle_jerk),min(angle_range[1],prev_turn_angle+self.angle_jerk)]
        return angle_range


    def selectAction(self,state,lim_accel_range,lim_angle_range):
        accel_range = list(self.accel_range)
        angle_range = list(self.angle_range)

        ego_vel = state["velocity"]

        switch_val = False

        if self.accel_up:
            #if random.random()<(ego_vel/(self.speed_limit-self.speed_limit_buffer))-1:
            if random.random()<(ego_vel-(self.speed_limit-self.speed_limit_buffer))/(2*self.speed_limit_buffer):
                self.accel_up = False
                self.up_coef = 0
                switch_val = True
        else:
            #if random.random()>ego_vel/(self.speed_limit+self.speed_limit_buffer):
            if random.random()>(ego_vel-(self.speed_limit-self.speed_limit_buffer))/(2*self.speed_limit_buffer):
                self.accel_up = True
                self.down_coef = 0
                switch_val = True

        if self.accel_up:
            self.up_coef += .5
            self.down_coef += .1
        else:
            self.up_coef += .1
            self.down_coef += .5


        if ego_vel<self.speed_limit-self.speed_limit_buffer: self.down_coef = 0
        elif ego_vel>self.speed_limit+self.speed_limit_buffer: self.up_coef = 0

        self.up_coef = min(1,self.up_coef)
        self.down_coef = min(1,self.down_coef)

        accel_range = self.getAccelRange(state)
        if lim_accel_range[0] is not None and lim_accel_range[0]>accel_range[0]: accel_range[0] = lim_accel_range[0]
        if lim_accel_range[1] is not None and lim_accel_range[1]<accel_range[1]: accel_range[1] = lim_accel_range[1]
        accel = np.random.uniform(accel_range[0],accel_range[1])

        angle_range = self.getAngleRange(state)
        if lim_angle_range[0] is not None and lim_angle_range[0]>angle_range[0]: angle_range[0] = lim_angle_range[0]
        if lim_accel_range[1] is not None and lim_angle_range[1]<angle_range[1]: angle_range[1] = lim_angle_range[1]
        angle = np.random.uniform(angle_range[0],angle_range[1])

        return accel,angle


class FollowController():
    """Basic ACC controller taken from the method depicted in 'Stop and Go Cruise Control'"""
    # Values used in First Year Review; speed_limit=22.22, damping=95.8, stiff=1.88
    # Values used for experiments without basis in literature: speed_limit=22.22, damping=.275, stiff=.32
    def __init__(self,time_radius=1.5,damping=95.8,stiff=15,timestep=.1,accel_jerk=10,r=-1,ego=None,other=None,accel_range=[-5,5],angle_range=[0,0],speed_limit=5,**kwargs):
        self.jerk = accel_jerk #Value of jerk does not affect acceleration chosen somehow
        self.speed_limit = speed_limit
        self.timestep = timestep
        self.t_dist = time_radius

        self.radius = r

        #These are default values that will be overwritten when setLeaderAndFollower is called
        self.leader = None
        self.follower = None
        if other is not None: self.setLeaderAndFollower(leader=other,follower=ego,r=r)

        self.k_v = damping
        self.k_d = stiff

        self.accel_range = accel_range
        self.angle_range = angle_range


    def setLeaderAndFollower(self,leader,follower,r=0,**kwargs):
        self.leader = leader
        self.follower = follower #Follower is the vehicle being controlled
        if r==0:
            r = self.radius
        self.radius = r+(self.leader.length+self.follower.length)/2


    def selectAction(self,state,lim_accel_range,*args):
        if self.follower.v>self.speed_limit:
            #NOTE: This fires in some unnecessary cases, such as when both vehicles are going well under the speed limit
            #      This can be easily resolved, by the current setup apppears to work so I will leave it for another time
            #print("LINEAR_CONTROLLER_CLASSES: Speed Limit is:{}\t DEL_V changed from {} to {}".format(self.speed_limit,state["del_v"],self.speed_limit-self.follower.v))
            del_v = self.speed_limit-state["ego_v"]
            lead_accel = 0
        else:
            del_v = state["del_v"]
            lead_accel = self.leader.accel

        d_des = self.radius + state["ego_v"]*self.t_dist
        del_d = state["del_d"] - d_des

        a_z = 0 #Disturbance acceleration caused by environment

        coef = (1/(1+self.k_v*self.t_dist))
        t1 = coef*(self.k_v*del_v + self.k_v*self.k_d*del_d + lead_accel) - a_z
        t2 = self.t_dist*(coef)*self.jerk

        accel = t1-t2

        #Ensure acceleration is within the range of permitted accelerations
        accel_range = self.accel_range
        if lim_accel_range[0] is not None and lim_accel_range[0]>accel_range[0]: accel_range[0] = lim_accel_range[0]
        if lim_accel_range[1] is not None and lim_accel_range[1]<accel_range[1]: accel_range[1] = lim_accel_range[1]

        accel = min(accel_range[1],max(accel_range[0],accel))
        return accel,0


class DataGeneratorController():
    def __init__(self,time_range=[1,10],ego=None,other=None,accel_range=[-5,5],accel_jerk=1,angle_range=[0,0],speed_limit=0,speed_limit_buffer=0,**kwargs):
        self.fast_controller = GoFastController(speed_limit=speed_limit+speed_limit_buffer,accel_range=accel_range,accel_jerk=accel_jerk,ego=ego,**kwargs)
        self.slow_controller = GoSlowController(speed_limit=speed_limit+speed_limit_buffer,accel_range=accel_range,accel_jerk=accel_jerk,ego=ego,**kwargs)
        if ego is not None and ego.state["velocity"] > speed_limit/2:
            self.controller = self.slow_controller
        else:
            self.controller = self.fast_controller

        self.ego = ego
        self.other = other

        self.accel_range = accel_range
        self.angle_range = angle_range

        self.timestep = None

        t_0 = max(abs(accel_range[0]),abs(accel_range[1]))/accel_jerk
        t11 = -accel_range[0] + math.sqrt(accel_range[0]**2 + 4*accel_jerk*speed_limit)/(2*accel_jerk)
        t12 = -accel_range[0] - math.sqrt(accel_range[0]**2 + 4*accel_jerk*speed_limit)/(2*accel_jerk)
        t21 = -accel_range[1] + math.sqrt(accel_range[1]**2 + 4*accel_jerk*speed_limit)/(2*accel_jerk)
        t22 = -accel_range[1] - math.sqrt(accel_range[1]**2 + 4*accel_jerk*speed_limit)/(2*accel_jerk)

        t_max = max(t_0,t11,t12,t21,t22,time_range[1])
        #print("LINEAR_CONTROLLER_CLASSES: time_range: {}".format([min(time_range[0],math.ceil(.2*t_max)),t_max]))

        self.fast_time_range = [min(time_range[0],math.ceil(.2*t_max)),.5*t_max]
        self.slow_time_range = [min(time_range[0],math.ceil(.2*t_max)),.5*t_max]
        self.stop_time_range = [0,1.5]
        self.high_speed_time_range = [0,1]
        self.time = random.uniform(self.fast_time_range[0],self.fast_time_range[1])

        self.speed_limit = speed_limit
        self.speed_limit_buffer = speed_limit_buffer

        if ego is not None:
            self.timestep = ego.timestep


    def setup(self,ego=None,other=None,**kwargs):
        if ego is not None:
            self.ego = ego
        if other is not None:
            self.other = other

        self.fast_controller.setup(ego=ego,other=other)
        self.slow_controller.setup(ego=ego,other=other)

        if self.ego is not None and ego.state["velocity"]>self.speed_limit/2:
            self.controller = self.slow_controller
        else:
            self.controller = self.fast_controller

        self.timestep = ego.timestep


    def selectAction(self,state,lim_accel_range,lim_angle_range):
        #next_vel = state["velocity"]+self.ego.timestep*self.controller.accel
        if state["velocity"]<.5:
            if self.controller is self.slow_controller and self.time>self.stop_time_range[1]:
                self.time = random.uniform(self.stop_time_range[0],self.stop_time_range[1])

        elif state["velocity"]>=self.speed_limit:
            if self.controller is self.fast_controller and self.time>self.high_speed_time_range[1]:
                self.time = random.uniform(self.high_speed_time_range[0],self.high_speed_time_range[1])

        if self.time <= 0:
            if self.controller is self.slow_controller:
                self.time=random.uniform(self.fast_time_range[0],self.fast_time_range[1])
                self.controller = self.fast_controller
            elif self.controller is self.fast_controller:
                self.time = random.uniform(self.slow_time_range[0],self.slow_time_range[1])
                self.controller = self.slow_controller
        self.time -= self.timestep

        return self.controller.selectAction(state,lim_accel_range,lim_angle_range)


class OvertakeController():
    def __init__(self,ego=None,other=None,accel_range=[-5,5],angle_range=[0,0],speed_limit=0,speed_limit_buffer=0,**kwargs):
        self.accel_range = accel_range
        self.angle_range = angle_range

        self.ego = ego
        self.other = other

        self.speed_limit = speed_limit
        self.speed_limit_buffer = speed_limit_buffer


    def setup(self,ego=None,other=None,**kwargs):
        if ego is not None:
            self.ego = ego
        if other is not None:
            self.other = other


    def selectAction(self,state,lim_accel_range,lim_angle_range):
        accel_range = list(self.accel_range)
        angle_range = list(self.angle_range)
        del_v = state["del_v"]
        if del_v>0 or self.other.y_com-self.other.length/2<self.ego.y_com: #Need positive accel
            accel_range = [max(0,accel_range[0]),accel_range[1]]
        elif del_v<0 and self.ego.v>self.speed_limit+self.speed_limit_buffer:
            accel_range = [accel_range[0],min(0,accel_range[1])] #breaking speed limit and going faster than other guy
        else:
            min_mag = min(abs(accel_range[0]),abs(accel_range[1]))
            accel_range = [-.1*min_mag,.1*min_mag] #Maintain speed

        accel_range = self.getAccelRange(state)
        if lim_accel_range[0] is not None and lim_accel_range[0]>accel_range[0]: accel_range[0] = lim_accel_range[0]
        if lim_accel_range[1] is not None and lim_accel_range[1]<accel_range[1]: accel_range[1] = lim_accel_range[1]
        accel = np.random.uniform(accel_range[0],accel_range[1])

        return accel,0


class GoFastController():
    def __init__(self,speed_limit=0,accel_range=[-5,5],accel_jerk=10,ego=None,**kwargs):
        self.speed_limit = speed_limit
        self.accel = None
        self.accel_limit = accel_range[1]
        self.accel_jerk = accel_jerk

        self.ego = ego


    def setup(self,ego=None,other=None,**kwargs):
        if ego is not None:
            self.ego = ego


    def selectAction(self,state,lim_accel_range,*args):
        prev_accel = state["acceleration"]
        self.accel = prev_accel + self.accel_jerk*self.ego.timestep
        if self.accel > self.accel_limit: self.accel = self.accel_limit

        if state["velocity"]+self.ego.timestep*self.accel>self.speed_limit:
            accel = 0
        else:
            if lim_accel_range[1] is not None and lim_accel_range[1]<self.accel:
                accel = lim_accel_range[1]
            else:
                accel = self.accel
        return accel,0


#    def selectAction(self,state,lim_accel_range,*args):
#        #prev_accel = state["acceleration"]
#        if state["velocity"]+self.ego.timestep*self.accel>self.speed_limit:
#            accel = 0
#        else:
#            if lim_accel_range[1] is not None and lim_accel_range[1]<self.accel:
#                accel = lim_accel_range[1]
#            else:
#                accel = self.accel
#        return accel,0
#

class GoSlowController():
    def __init__(self,accel_range=[-5,5],accel_jerk=10,ego=None,**kwargs):
        self.accel = None
        self.accel_limit = accel_range[0]
        self.accel_jerk = accel_jerk

        self.ego = ego


    def setup(self,ego=None,other=None,**kwargs):
        if ego is not None:
            self.ego = ego


    def selectAction(self,state,lim_accel_range,*args):
        prev_accel = state["acceleration"]
        self.accel = prev_accel - self.accel_jerk*self.ego.timestep
        if self.accel < self.accel_limit: self.accel = self.accel_limit

        if state["velocity"]+self.ego.timestep*self.accel<0:
            accel = 0
        else:
            if lim_accel_range[0] is not None and lim_accel_range[0]>self.accel:
                accel = lim_accel_range[0]
            else:
                accel = self.accel
        return accel,0


#    def selectAction(self,state,lim_accel_range,*args):
#        if state["velocity"]+self.ego.timestep*self.accel<0:
#            accel = 0
#        else:
#            if lim_accel_range[0] is not None and lim_accel_range[0]>self.accel:
#                accel = lim_accel_range[0]
#            else:
#                accel = self.accel
#        return accel,0


class RandomController():
    def __init__(self,speed_limit=0,speed_limit_buffer=0,accel_range=[-5,5],angle_range=[0,0],**kwargs):
        self.accel_range = accel_range
        self.angle_range = angle_range


    def selectAction(self,state,lim_accel_range,lim_angle_range):
        accel_range = list(self.accel_range)
        angle_range = list(self.angle_range)

        if lim_accel_range[0] is not None and lim_accel_range[0]>accel_range[0]: accel_range[0] = lim_accel_range[0]
        if lim_accel_range[1] is not None and lim_accel_range[1]<accel_range[1]: accel_range[1] = lim_accel_range[1]

        if lim_angle_range[0] is not None and lim_angle_range[0]>angle_range[0]: angle_range[0] = lim_angle_range[0]
        if lim_accel_range[1] is not None and lim_angle_range[1]<angle_range[1]: angle_range[1] = lim_angle_range[1]

        accel = np.random.uniform(accel_range[0],accel_range[1])
        angle = np.random.uniform(angle_range[0],angle_range[1])
        return accel,angle

#def distance(pt1,pt2):
#    """Revised distance function that captures when an NE has passed E (del_d negative)"""
#    return (pt1[1]-pt2[1])

def distance(pt1,pt2):
    return math.sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2)
