# -*- coding: utf-8 -*-
import math
import matplotlib.pyplot as plt
import numpy as np
import pygame
import random
import sys

class StandardDrivingController():
    def __init__(self,ego=None,accel_range=[-5,5],accel_jerk=1,yaw_rate_range=[0,0],yaw_rate_jerk=5,speed_limit=5,speed_limit_buffer=5,**kwargs):
        self.speed_limit = speed_limit
        self.speed_limit_buffer = speed_limit_buffer

        self.accel_range = accel_range
        self.yaw_rate_range = yaw_rate_range

        self.ego = ego

        self.accel_up = True
        self.up_coef = .5
        self.down_coef = .5

        self.accel_jerk = accel_jerk
        self.yaw_rate_jerk = yaw_rate_jerk


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
        yaw_rate_range = list(self.yaw_rate_range)
        prev_yaw_rate = state["yaw_rate"]

        yaw_rate_range = [max(yaw_rate_range[0],prev_yaw_rate-self.yaw_rate_jerk),min(yaw_rate_range[1],prev_yaw_rate+self.yaw_rate_jerk)]
        return yaw_rate_range


    def selectAction(self,state,lim_accel_range,lim_yaw_rate_range):
        accel_range = list(self.accel_range)
        yaw_rate_range = list(self.yaw_rate_range)

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

        yaw_rate_range = self.getAngleRange(state)
        if lim_yaw_rate_range[0] is not None and lim_yaw_rate_range[0]>yaw_rate_range[0]: yaw_rate_range[0] = lim_yaw_rate_range[0]
        if lim_accel_range[1] is not None and lim_yaw_rate_range[1]<yaw_rate_range[1]: yaw_rate_range[1] = lim_yaw_rate_range[1]
        yaw_rate = np.random.uniform(yaw_rate_range[0],yaw_rate_range[1])

        return accel,yaw_rate


class TrajectoryController():
    def __init__(self,ego=None,traj_func=None,accel_range=None,yaw_rate_range=None,auto_start_traj=True,start_coef=1,**kwargs):
        self.ego = ego
        self.accel_range = accel_range

        self.traj_generator = traj_func
        kwargs.update({"ego":ego,"accel_range":accel_range,"yaw_rate_range":yaw_rate_range})
        self.generator_params = kwargs

        self.auto_start_trajectory = auto_start_traj
        self.start_coef = start_coef
        self.backup_controller = DataGeneratorController(**kwargs)

        if self.ego is None:
            self.trajectory = []
        else:
            self.trajectory = None
           # self.trajectory = self.traj_generator(**self.generator_params)
        self.index = -1


    def setup(self,ego=None,**kwargs):
        if ego is not None:
            self.ego = ego
            self.generator_params.update({"ego":ego})
            self.backup_controller.setup(ego=ego,**kwargs)

        #If ego has not been loaded onto a simulator when the controller is initialised then there will be no destination
        # or waypoints set, so not trajectory can be constructed
        if self.ego.destination is not None:
            self.trajectory = self.traj_generator(**self.generator_params)
            self.index = 0


    def selectAction(self,state,lim_accel_range,lim_yaw_rate_range):
        if self.trajectory is None:
            #It is a bit hacky to have this here, but generating trajectories whenever
            # vehicle was reset created lots of problems (i.e. short track => infinite loop
            # trying to create trajectory.
            # SelectAction only fires if there is an expectation that a trajectory can be 
            # constructed
            self.trajectory = self.traj_generator(**self.generator_params)
            if self.auto_start_trajectory: self.index = 0
            else:
                self.backup_controller.setup()
                self.index = -1

        if self.index == -1:
            if random.random()<self.start_coef: self.index = 0
            else: accel,yaw_rate = self.backup_controller.selectAction(state,lim_accel_range,lim_yaw_rate_range)

        if self.index != -1:
            if self.index<len(self.trajectory):
                accel,yaw_rate = self.trajectory[self.index][1]
            else:
                if self.index == len(self.trajectory):
                    self.backup_controller.setup()
                accel,yaw_rate = self.backup_controller.selectAction(state,lim_accel_range,lim_yaw_rate_range)
            self.index += 1
        return accel,yaw_rate


class FollowController():
    """Basic ACC controller taken from the method depicted in 'Stop and Go Cruise Control'"""
    # Values used in First Year Review; speed_limit=22.22, damping=95.8, stiff=1.88
    # Values used for experiments without basis in literature; speed_limit=22.22, damping=.275, stiff=.32
    #def __init__(self,time_headway=1.5,damping=95.8,stiff=15,timestep=.1,accel_jerk=10,r=-1,ego=None,other=None,accel_range=[-5,5],yaw_rate_range=[0,0],speed_limit=5,**kwargs):
    def __init__(self,time_headway=1.5,damping=95.8,stiff=15,timestep=.1,accel_jerk=10,r=-1,ego=None,other=None,accel_range=[-5,5],yaw_rate_range=[0,0],speed_limit=5,safe_dist_bounds=None,**kwargs):
        import sys
        sys.path.insert(0,"../responsibility_experiments")
        import risk_and_responsibility_tools as rnr
        self.jerk = accel_jerk #Value of jerk does not affect acceleration chosen somehow
        self.speed_limit = speed_limit
        self.timestep = timestep
        self.t_dist = time_headway

        self.radius = r
        self.safe_dist_bounds = safe_dist_bounds

        #These are default values that will be overwritten when setLeaderAndFollower is called
        self.leader = None
        self.follower = None
        if other is not None: self.setLeaderAndFollower(leader=other,follower=ego,r=r)

        self.k_v = damping
        self.k_d = stiff

        self.accel_range = accel_range
        self.yaw_rate_range = yaw_rate_range


    def simulateController(self,sim):
        dists = []
        for _ in range(20):
            sim.reinitialise()
            coef = random.uniform(0.1,1)
            traj_type = random.choice(("safe","bangBang"))
            traj_with_index = rnr.makeTrajectory(self.leader,traj_type,self.accel_range,self.speed_limit,coef)
            sim.reinitialise()
            move_dict = {self.leader:[(x[1],0) for x in traj_with_index[0]]}
            i = 0
            while rnr.canGo([self.leader,self.follower]):
                sim.singleStep(move_dict=move_dict,index=i)
                del_d = abs(rnr.computeDistance((self.leader.x_com,self.leader.y_com),(self.follower.x_com,self.follower.y_com)))
                #dists.append(math.sqrt((del_d-self.radius)**2 + (del_d-3*self.radius)**2))
                dists.append(del_d-self.radius)
                i += 1

            while rnr.canGo([self.follower]) and self.follower.v>.5:
                sim.singleStep([self.follower])
                del_d = abs(rnr.computeDistance((self.leader.x_com,self.leader.y_com),(self.follower.x_com,self.follower.y_com)))
                #dists.append((del_d-self.radius)**2 + (del_d-3*self.radius)**2)
                dists.append(del_d-self.radius)

        return dists


    def objective(self,params,sim=None):
        self.k_v = params[0]
        self.k_d = params[1]
        dists = self.simulateController(sim)
        plt.plot(dists)
        plt.show()
        exit(-1)


    def setup(self,ego=None,other=None,r=0,**kwargs):
        self.setLeaderAndFollower(leader=other,follower=ego,r=r,**kwargs)
        #sim = rnr.initialiseSimulator([self.leader,self.follower],self.speed_limit,True,[self.speed_limit,self.speed_limit],vehicle_spacing=2*self.follower.length,data_generation=True)
        #self.objective((self.k_v,self.k_d),sim)


    def setLeaderAndFollower(self,leader,follower,r=0,**kwargs):
        self.leader = leader
        self.follower = follower #Follower is the vehicle being controlled
        if r==0:
            r = self.radius
        self.radius = r+(self.leader.length+self.follower.length)/2
        if self.safe_dist_bounds is None:
            self.safe_dist_bounds = (self.radius,3*self.radius)


    def selectAction(self,state,lim_accel_range,*args):
        if self.follower.v>self.speed_limit:
            #NOTE: This fires in some unnecessary cases, such as when both vehicles are going well under the speed limit
            #      This can be easily resolved, by the current setup apppears to work so I will leave it for another time
            #print("LINEAR_CONTROLLER_CLASSES: Speed Limit is:{}\t DEL_V changed from {} to {}".format(self.speed_limit,state["del_v"],self.speed_limit-self.follower.v))
            del_v = self.speed_limit-state["velocity"]
            lead_accel = 0
        else:
            del_v = state["del_v"]
            lead_accel = self.leader.accel

        d_des = self.radius + state["velocity"]*self.t_dist
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


class IntelligentDrivingController():
    def __init__(self,ego=None,other=None,speed_limit=33.33,headway=1.6,accel_exponent=4,s0=2,b=None,accel_range=[-3,3],**kwargs):
        self.speed_limit = speed_limit #Also desired velocity
        self.accel_range = accel_range
        self.headway = headway #How much time you want there to be between cars
        self.accel_exponent = accel_exponent #exponent of the velocity term (v/v0)
        self.s0 = s0 #minimum distance you want there to be between vehicles
        self.b = b #Comfortable deceleration rate

        self.leader = None
        self.follower = None
        if other is not None: self.setLeaderAndFollower(leader=other,follower=ego)


    def setup(self,ego=None,other=None,**kwargs):
        self.setLeaderAndFollower(leader=other,follower=ego,**kwargs)


    def setLeaderAndFollower(self,leader,follower,**kwargs):
        self.leader = leader
        self.follower = follower


    def selectAction(self,state,lim_accel_range,*args):
        #Ensure acceleration is within the range of permitted accelerations
        accel_range = self.accel_range
        if lim_accel_range[0] is not None and lim_accel_range[0]>accel_range[0]: accel_range[0] = lim_accel_range[0]
        if lim_accel_range[1] is not None and lim_accel_range[1]<accel_range[1]: accel_range[1] = lim_accel_range[1]

        #Presumes left to right orientation
        ego_pos = state["position"] #presumed follower
        other_pos = state["other_position"] #presumed leader
        del_d = state["other_position"][0]-state["position"][0]

        v = state["velocity"]
        del_v = state["del_v"]
        #del_d = state["dell_d"] #This is euclidean  distance, which is orientation agnostic, but also not suitable here
        del_d -= self.follower.length
        if self.b is None: b = accel_range[0]
        else: b = self.b
        a_max = accel_range[1]
        #If there's no leader then this controller just drives to speed limit
        if self.leader is None:
            s_star = 0
        else:
            s_star = self.s0 + self.headway*v + (v*del_v)/(2*math.sqrt(a_max*abs(b)))

        accel = a_max*(1-((v/self.speed_limit)**self.accel_exponent) - (s_star/del_d)**2)

        return accel,0


class DataGeneratorController():
    def __init__(self,time_range=[1,10],ego=None,other=None,accel_range=[-5,5],accel_jerk=1,yaw_rate_range=[0,0],speed_limit=0,speed_limit_buffer=0,**kwargs):
        self.fast_controller = GoFastController(speed_limit=speed_limit+speed_limit_buffer,accel_range=accel_range,accel_jerk=accel_jerk,ego=ego,**kwargs)
        self.slow_controller = GoSlowController(speed_limit=speed_limit+speed_limit_buffer,accel_range=accel_range,accel_jerk=accel_jerk,ego=ego,**kwargs)

        self.controller = None

        self.ego = ego
        self.other = other

        self.accel_range = accel_range
        self.yaw_rate_range = yaw_rate_range

        self.timestep = None

        #t_0 = max(abs(accel_range[0]),abs(accel_range[1]))/accel_jerk
        #t11 = -accel_range[0] + math.sqrt(accel_range[0]**2 + 4*accel_jerk*speed_limit)/(2*accel_jerk)
        #t12 = -accel_range[0] - math.sqrt(accel_range[0]**2 + 4*accel_jerk*speed_limit)/(2*accel_jerk)
        #t21 = -accel_range[1] + math.sqrt(accel_range[1]**2 + 4*accel_jerk*speed_limit)/(2*accel_jerk)
        #t22 = -accel_range[1] - math.sqrt(accel_range[1]**2 + 4*accel_jerk*speed_limit)/(2*accel_jerk)

        #t_max = max(t_0,t11,t12,t21,t22,time_range[1])
        #print("LINEAR_CONTROLLER_CLASSES: time_range: {}".format([min(time_range[0],math.ceil(.2*t_max)),t_max]))

        #self.fast_time_range = [min(time_range[0],math.ceil(t_max)),min(t_max,time_range[1])]
        #self.slow_time_range = [min(time_range[0],math.ceil(t_max)),min(t_max,time_range[1])]
        ta = (accel_range[1]-accel_range[0])/accel_jerk
        self.fast_time_range = [time_range[0],max(time_range[1],ta)]
        self.slow_time_range = [time_range[0],max(time_range[1],ta)]
        self.stop_time_range = [0,2]
        self.high_speed_time_range = [0,2]
        #self.time = random.uniform(self.fast_time_range[0],self.fast_time_range[1])
        self.time = None

        self.speed_limit = speed_limit
        self.speed_limit_buffer = speed_limit_buffer

        if ego is not None:
            self.timestep = ego.timestep


    def setup(self,ego=None,other=None,**kwargs):
        if ego is not None:
            self.ego = ego
            self.timestep = self.ego.timestep
        if other is not None:
            self.other = other

        self.fast_controller.setup(ego=ego,other=other)
        self.slow_controller.setup(ego=ego,other=other)


    def selectAction(self,state,lim_accel_range,lim_yaw_rate_range):
        if self.controller is None:
            if state["velocity"]>self.speed_limit/2:
                self.controller = self.slow_controller
                self.time = random.uniform(self.slow_time_range[0],self.slow_time_range[1])
            else:
                self.controller = self.fast_controller
                self.time = random.uniform(self.fast_time_range[0],self.fast_time_range[1])

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

        return self.controller.selectAction(state,lim_accel_range,lim_yaw_rate_range)


class OvertakeController():
    def __init__(self,ego=None,other=None,accel_range=[-5,5],yaw_rate_range=[0,0],speed_limit=0,speed_limit_buffer=0,**kwargs):
        self.accel_range = accel_range
        self.yaw_rate_range = yaw_rate_range

        self.ego = ego
        self.other = other

        self.speed_limit = speed_limit
        self.speed_limit_buffer = speed_limit_buffer


    def setup(self,ego=None,other=None,**kwargs):
        if ego is not None:
            self.ego = ego
        if other is not None:
            self.other = other


    def selectAction(self,state,lim_accel_range,lim_yaw_rate_range):
        accel_range = list(self.accel_range)
        yaw_rate_range = list(self.yaw_rate_range)
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


class RandomController():
    def __init__(self,speed_limit=0,speed_limit_buffer=0,accel_range=[-5,5],yaw_rate_range=[0,0],**kwargs):
        self.accel_range = accel_range
        self.yaw_rate_range = yaw_rate_range


    def setup(self,**kwargs):
        pass


    def selectAction(self,state,lim_accel_range,lim_yaw_rate_range):
        accel_range = list(self.accel_range)
        yaw_rate_range = list(self.yaw_rate_range)

        if lim_accel_range[0] is not None and lim_accel_range[0]>accel_range[0]: accel_range[0] = lim_accel_range[0]
        if lim_accel_range[1] is not None and lim_accel_range[1]<accel_range[1]: accel_range[1] = lim_accel_range[1]

        if lim_yaw_rate_range[0] is not None and lim_yaw_rate_range[0]>yaw_rate_range[0]: yaw_rate_range[0] = lim_yaw_rate_range[0]
        if lim_accel_range[1] is not None and lim_yaw_rate_range[1]<yaw_rate_range[1]: yaw_rate_range[1] = lim_yaw_rate_range[1]

        accel = np.random.uniform(accel_range[0],accel_range[1])
        yaw_rate = np.random.uniform(yaw_rate_range[0],yaw_rate_range[1])
        return accel,yaw_rate


class UnbiasedRandomController():
    """Random controller that samples actions in such a way that the expected value of the magnitudes of the acceleration and yaw rate is 0
       i.e. if the magnitude of the lower bound of the range is greater than the magnitude of the upper bound (and they have opposite sign)
       then we increase the probability of sampling a value greater than 0 to capture this."""
    def __init__(self,speed_limit=0,speed_limit_buffer=0,accel_range=[-5,5],yaw_rate_range=[0,0],**kwargs):
        self.accel_range = accel_range
        self.yaw_rate_range = yaw_rate_range


    def setup(self,**kwargs):
        pass


    def selectAction(self,state,lim_accel_range,lim_yaw_rate_range):
        accel_range = list(self.accel_range)
        yaw_rate_range = list(self.yaw_rate_range)

        if lim_accel_range[0] is not None and lim_accel_range[0]>accel_range[0]: accel_range[0] = lim_accel_range[0]
        if lim_accel_range[1] is not None and lim_accel_range[1]<accel_range[1]: accel_range[1] = lim_accel_range[1]

        if lim_yaw_rate_range[0] is not None and lim_yaw_rate_range[0]>yaw_rate_range[0]: yaw_rate_range[0] = lim_yaw_rate_range[0]
        if lim_accel_range[1] is not None and lim_yaw_rate_range[1]<yaw_rate_range[1]: yaw_rate_range[1] = lim_yaw_rate_range[1]

        if accel_range[0]!=0 and accel_range[1]/accel_range[0] <0: #range contains both positive and negative values
            if random.random()<abs(accel_range[0])/(abs(accel_range[1]-accel_range[0])):
                accel = np.random.uniform(0,accel_range[1])
            else:
                accel = np.random.uniform(accel_range[0],0)
        else:
            accel = np.random.uniform(accel_range[0],accel_range[1])

        if yaw_rate_range[0]!=0 and yaw_rate_range[1]/yaw_rate_range[0]<0:
            if random.random()<abs(yaw_rate_range[0])/(abs(yaw_rate_range[1]-yaw_rate_range[0])):
                yaw_rate = np.random.uniform(0,yaw_rate_range[1])
            else:
                yaw_rate = np.random.uniform(yaw_rate_range[0],0)
        else:
            yaw_rate = np.random.uniform(yaw_rate_range[0],yaw_rate_range[1])

        return accel,yaw_rate


class ManualController():
    def __init__(self,ego=None,accel_range=[-5,5],accel_jerk=1,yaw_rate_range=[-30,30],yaw_rate_jerk=5,**kwargs):
        self.accel_range = accel_range
        self.accel_jerk = accel_jerk
        self.yaw_rate_range = yaw_rate_range
        self.yaw_rate_jerk = yaw_rate_jerk

        self.ego = ego
        #print("Manual Controller Initialised")


    def setup(self,ego=None,**kwargs):
        if ego is not None:
            self.ego = ego


    def selectAction(self,state,*args):
        accel,yaw_rate = state["acceleration"],state["yaw_rate"]
        dt = self.ego.timestep
        keys = pygame.key.get_pressed()

        #Negate an action if the corresponding key is not still being pressed
        if (not keys[pygame.K_LEFT]) and yaw_rate>0: yaw_rate = 0
        if (not keys[pygame.K_RIGHT]) and yaw_rate<0: yaw_rate = 0
        if (not keys[pygame.K_UP]) and accel>0: accel = 0
        if (not keys[pygame.K_DOWN]) and accel<0: accel = 0

        if keys[pygame.K_LEFT]:
            #print("Left Key Pressed, increasing yaw_rate")
            yaw_rate = min(self.yaw_rate_range[1],yaw_rate+self.yaw_rate_jerk*dt)
        if keys[pygame.K_RIGHT]:
            #print("Right Key Pressed, decreasing yaw_rate")
            yaw_rate = max(self.yaw_rate_range[0],yaw_rate-self.yaw_rate_jerk*dt)
        if keys[pygame.K_UP]:
            #print("Up key pressed, increasing Acceleration")
            accel = min(self.accel_range[1],accel+self.accel_jerk*dt)
        if keys[pygame.K_DOWN]:
            #print("Down Key pressed, decreasing Acceleration")
            accel = max(self.accel_range[0],accel-self.accel_jerk*dt)

        #print("Action: Accel: {}\tYaw: {}".format(accel,yaw_rate))
        return accel,yaw_rate


class ConstantVelocityController():
    def __init__(self,**kwargs):
        self.accel_range = [0,0]
        self.accel_jerk = 0
        self.yaw_rate_range = [0,0]
        self.yaw_rate_jerk = 0


    def setup(self,**kwargs):
        pass


    def selectAction(self,*args):
        return 0,0
