import math
import matplotlib.pyplot as plt
import numpy as np
import random
import sys

sys.path.insert(0,"../responsibility_experiments")
import rnr_exp1 as rnr

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
    #def __init__(self,time_headway=1.5,damping=95.8,stiff=15,timestep=.1,accel_jerk=10,r=-1,ego=None,other=None,accel_range=[-5,5],angle_range=[0,0],speed_limit=5,**kwargs):
    def __init__(self,time_headway=1.5,damping=95.8,stiff=15,timestep=.1,accel_jerk=10,r=-1,ego=None,other=None,accel_range=[-5,5],angle_range=[0,0],speed_limit=5,safe_dist_bounds=None,**kwargs):
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
        self.angle_range = angle_range


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

        self.fast_time_range = [min(time_range[0],math.ceil(.2*t_max)),min(.75*t_max,time_range[1])]
        self.slow_time_range = [min(time_range[0],math.ceil(.2*t_max)),min(.75*t_max,time_range[1])]
        self.stop_time_range = [0,.5]
        self.high_speed_time_range = [0,2]
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
