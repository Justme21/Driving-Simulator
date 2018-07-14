import datetime
import math
import numpy as np
import random

class FollowController():
    """Basic ACC controller taken from the method depicted in 'Stop and Go Cruise Control'"""
    # damping = .5, stiff=.15
    def __init__(self,follow_radius,accel_range,angle_range,time_radius=1.5,damping=5,stiff=5,timestep=.1,jerk=20):
        #These are unused but must be passed to match with Random Controller
        self.accel_range = accel_range
        self.angle_range = angle_range

        self.jerk = jerk
        self.timestep = timestep
        self.radius = follow_radius
        self.t_dist = time_radius

        self.k_v = damping
        self.k_d = stiff

        self.log = []


    def setLeaderAndFollower(self,leader,follower):
        self.leader = leader
        self.follower = follower #Follower is the vehicle being controlled
        self.radius += (self.leader.length+self.follower.length)/2


    def selectAction(self,state):
        log = {}

        ego_vel = state["velocity"]
        del_v = self.leader.v - ego_vel

        ego_com = state["position"]
        lead_com = (self.leader.x_com,self.leader.y_com)
        dist = distance(ego_com,lead_com)
        del_d = dist - self.radius

        lead_accel = self.leader.accel
        a_z = 0 #Disturbance acceleration caused by environment

        coef = (1/(1+self.k_v*self.t_dist))
        t1 = coef*(self.k_v*del_v + self.k_v*self.k_d*del_d + lead_accel) - a_z
        t2 = self.t_dist*(coef)*self.jerk

        accel = t1-t2
        #Ensure acceleration is within the range of permitted accelerations
        accel = min(self.accel_range[1],max(self.accel_range[0],accel))
        #if len(self.log)>0 and abs((accel-self.log[-1][1]))/self.timestep > self.jerk:
        #    if accel-self.log[-1][1]<0:
        #        accel = self.log[-1][1]-self.jerk*self.timestep
        #    else:
        #        accel = self.log[-1][1]+self.jerk*self.timestep

        log["ego_v"] = self.follower.v
        log["lead_v"] = self.leader.v
        log["del_v"] = del_v
        log["del_d"] = del_d
        log["lead_accel"] = self.leader.accel
        log["ego_accel"] = self.follower.accel

        self.log.append([log,accel])
        return accel,0


    def getLog(self):
        return self.log

    def clearLog(self):
        self.log = []


class DataGeneratorController():
    def __init__(self,accel_range,angle_range,time_range,*args):
        self.fast_controller = GoFasterController(accel_range,angle_range)
        self.slow_controller = GoSlowController(accel_range,angle_range)
        self.controller = self.fast_controller

        self.time_range = time_range
        self.time = random.randint(self.time_range[0],self.time_range[1])


    def selectAction(self,state):
        if self.time == 0 or (state["velocity"]<5 and self.controller is self.slow_controller):
            self.time=random.randint(self.time_range[0],self.time_range[1])
            if self.controller is self.slow_controller or state["velocity"]<5:
                self.controller = self.fast_controller
            else:
                self.controller = self.slow_controller
        self.time -= 1

        return self.controller.selectAction(state)



class GoFasterController():
    def __init__(self,accel_range,angle_range,*args):
        self.accel = accel_range[1]
        self.leader = None
        self.follower = None
        self.radius = None
        self.log = []


    def setLeaderAndFollower(self,v1,v2):
        self.leader = v1
        self.follower = v2
        self.radius = (v1.length+v2.length)/2


    def selectAction(self,state):
        if self.follower is not None:
            ego_vel = state["velocity"]
            del_v = self.leader.v - ego_vel

            ego_com = state["position"]
            lead_com = (self.leader.x_com,self.leader.y_com)
            dist = distance(ego_com,lead_com)
            del_d = dist - self.radius
            log = {}
            log["ego_v"] = self.follower.v
            log["lead_v"] = self.leader.v
            log["del_v"] = del_v
            log["del_d"] = del_d
            log["lead_accel"] = self.leader.accel
            log["ego_accel"] = self.follower.accel

            self.log.append([log,self.accel])
        return self.accel,0

    def getLog(self):
        return self.log

    def clearLog(self):
        self.log = []

class GoSlowController():
    def __init__(self,accel_range,angle_range,*args):
        self.accel = accel_range[0]

    def selectAction(self,state):
        return self.accel,0


class RandomController():

    def __init__(self,accel_range,angle_range,*args):
        self.accel_range = accel_range
        self.angle_range = angle_range

    def selectAction(self,state):
        #v = state["velocity"]
        #if v<10: 
        #    accel = np.random.uniform((self.accel_range[1]+self.accel_range[0])/2,self.accel_range[1])
        #else: accel = np.random.uniform(-1,1)
        #angle = 0
        v= state["velocity"]
        if v<=0 or random.random()>.35:
            accel = np.random.uniform(0,self.accel_range[1])
        else:
            accel = np.random.uniform(self.accel_range[0],0)

        angle = np.random.uniform(self.angle_range[0],self.angle_range[1])
        return accel,angle


def distance(pt1,pt2):
    return math.sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2)
