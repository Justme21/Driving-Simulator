import datetime
import math
import numpy as np
import random

class DrivingController():

    def __init__(self,controller="standard",limit=22.5,buffer=2.25,ego=None,other=None,accel_range=[-5,5],angle_range=[0,0],**kwargs):
        self.speed_limit = limit
        self.speed_limit_buffer = .1*limit

        self.accel_range = accel_range
        self.angle_range = angle_range

        self.controller = self.getController(controller,speed_limit=limit,speed_limit_buffer=self.speed_limit_buffer,**kwargs)
        self.other = other
        self.ego = ego

        self.log = []


    def getController(self,controller,**kwargs):
        controller_list = {"standard":StandardDrivingController,"follow":FollowController,"generator":DataGeneratorController,\
                "fast":GoFastController,"slow":GoSlowController,"safe":SafeController,"overtake": OvertakeController,"random":RandomController}

        return controller_list[controller](**kwargs)


    def setup(self,ego=None,other=None,**kwargs):
        if ego is not None:
            self.ego = ego
        if other is not None:
            self.other = other

        if isinstance(self.controller,FollowController):
            self.controller.setLeaderAndFollower(leader=other,follower=ego,**kwargs)
        elif isinstance(self.controller,SafeController):
            self.controller.buildSafeTraj(accel_range=self.accel_range,**kwargs)


    def selectAction(self,state):
        log = {}

        if self.ego is not None:
            ego_vel = self.ego.v
            ego_accel = self.ego.accel
        else:
            ego_vel = 0
            ego_accel = 0

        if self.other is not None:
            other_v = self.other.v
            other_com = (self.other.x_com,self.other.y_com)
            other_accel = self.other.accel
        else:
            other_v = 0
            other_com = (0,0)
            other_accel = 0


        del_v = other_v - ego_vel

        ego_com = state["position"]
        dist = distance(ego_com,other_com)
        del_d = dist

        log["ego_v"] = ego_vel
        log["lead_v"] = other_v
        log["del_v"] = del_v
        log["del_d"] = del_d
        log["lead_accel"] = other_accel
        log["ego_accel"] = ego_accel

        accel_range = self.accel_range
        angle_range = self.angle_range
        if isinstance(self.controller,OvertakeController):
            pass
        elif ego_vel>self.speed_limit+self.speed_limit_buffer:
            accel_range = [self.accel_range[0],min(self.accel_range[1],.5*self.accel_range[0])]
            #accel_range = [self.accel_range[0],min(self.accel_range[1],-self.speed_limit_buffer)]
        elif ego_vel<0:
            accel_range = [max(self.accel_range[0],self.speed_limit_buffer),self.accel_range[1]]

        accel,angle_accel = self.controller.selectAction(state,accel_range,angle_range)

        self.log.append([log,accel])
        return accel,angle_accel

    def getLog(self):
        return self.log

    def clearLog(self):
        self.log = []


class StandardDrivingController():
    def __init__(self,speed_limit=5,speed_limit_buffer=5,**kwargs):
        self.speed_limit = speed_limit
        self.speed_limit_buffer = speed_limit_buffer


    def selectAction(self,state,accel_range,angle_range):
        ego_vel = state["velocity"]
        if ego_vel>self.speed_limit+self.speed_limit_buffer:
            accel = np.random.uniform(accel_range[0],min(accel_range[1],0))
        elif ego_vel<self.speed_limit+self.speed_limit_buffer and ego_vel>self.speed_limit-self.speed_limit_buffer:
            small_accel_mag =  min(abs(accel_range[0]),abs(accel_range[1]))
            accel = np.random.uniform(max(accel_range[0],-small_accel_mag),min(accel_range[1],small_accel_mag))
        else:
            accel = np.random.uniform(max(accel_range[0],0),accel_range[1])

        return accel,0


class FollowController():
    """Basic ACC controller taken from the method depicted in 'Stop and Go Cruise Control'"""
    # damping = .5, stiff=.15
    def __init__(self,time_radius=1.5,damping=1,stiff=1,timestep=.1,jerk=10,r=-1,**kwargs):
        self.jerk = jerk
        self.timestep = timestep
        self.t_dist = time_radius

        self.radius = r

        self.k_v = damping
        self.k_d = stiff


    def setLeaderAndFollower(self,leader,follower,r=0,**kwargs):
        self.leader = leader
        self.follower = follower #Follower is the vehicle being controlled
        self.radius = r+(self.leader.length+self.follower.length)/2


    def selectAction(self,state,accel_range,angle_range):
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
        accel = min(accel_range[1],max(accel_range[0],accel))

        return accel,0


class DataGeneratorController():
    def __init__(self,time_range=[10,40],**kwargs):
        self.fast_controller = GoFastController(**kwargs)
        self.slow_controller = GoSlowController(**kwargs)
        self.controller = self.fast_controller

        self.time_range = time_range
        self.time = random.randint(self.time_range[0],self.time_range[1])


    def selectAction(self,state,accel_range,angle_range):
        if self.time == 0 or (state["velocity"]<1.5 and self.controller is self.slow_controller):
            self.time=random.randint(self.time_range[0],self.time_range[1])
            if self.controller is self.slow_controller or state["velocity"]<1.5:
                self.controller = self.fast_controller
            else:
                self.controller = self.slow_controller
        self.time -= 1

        return self.controller.selectAction(state,accel_range,angle_range)


class OvertakeController():
    def __init__(self,**kwargs):
        pass

    def selectAction(self,state,accel_range,angle_range):
        accel = 1.25*accel_range[1]
        return accel,0


class GoFastController():
    def __init__(self,**kwargs):
        pass


    def selectAction(self,state,accel_range,angle_range):
        accel = accel_range[1]
        return accel,0


class GoSlowController():
    def __init__(self,**kwargs):
        pass


    def selectAction(self,state,accel_range,angle_range):
        accel = accel_range[0]
        return accel,0


class SafeController():
    def __init__(self,**kwargs):
        self.traj = None
        self.traj_posit = None
        self.length = 0

        self.x_com,self.y_com,self.v,self.accel = None,None,None,None


    def buildSafeTraj(self,accel_range=[-10,10],distance=-1,start_posit=(0,0),dt=.1,**kwargs):
        a_up = accel_range[1]
        a_down = accel_range[0]
        t_down = int(math.sqrt(-distance/self.a_down))
        t_up = int(math.sqrt(2*(distance+.5*self.a_down*t_down**2)/self.a_up))
        max_v = -self.a_down*t_down
        print("T_DOWN: {}\tT_UP: {}\tMAX_V: {}".format(t_down,t_up,max_v))
        t = 0
        pos = list(start_posit)
        a = self.a_up
        v = 0
        self.traj = []
        self.traj.append(((start_posit,v),a))
        t= dt
        travel = 0
        while distance>0 and v>=0:
            if t>t_up:
                a = self.a_down
            elif v>max_v:
                a = 0
            travel = v*dt + .5*a*(dt**2)
            pos = [pos[0],pos[1]-travel]
            v += a*dt
            self.traj.append(((pos,v),a))
            t+=dt
            distance -= travel
            if distance<0:
                print("Distance was : {}\t Is now: {}\tTravel was: {}".format(distance+travel,distance, travel))
        self.traj_posit = 0
        print("This is the Safe Trajectory")
        for i,entry in enumerate(self.traj):
            print("{}: {}".format(i*dt,entry))


    def selectAction(self,**kwargs):
        if self.traj_posit<len(self.traj):
            traj_state = self.traj[self.traj_posit]
        else:
            traj_state = self.traj[-1]
        self.x_com = traj_state[0][0][0]
        self.y_com = traj_state[0][0][1]
        self.v = traj_state[0][1]
        self.accel = traj_state[1]
        self.traj_posit+=1


class RandomController():
    def __init__(self,**kwargs):
        pass


    def selectAction(self,state,accel_range,angle_range):
        v = state["velocity"]

        accel = np.random.uniform(self.accel_range[0],self.accel_range[1])
        angle = np.random.uniform(self.angle_range[0],self.angle_range[1])
        return accel,angle


def distance(pt1,pt2):
    return math.sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2)
